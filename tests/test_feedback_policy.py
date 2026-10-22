"""Regression checks for feedback-policy control flow, without RSS."""

import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

from knotlab import tip_feedback


SCRIPTS = Path(__file__).absolute().parents[1] / "graphs" / "overhand_feedback" / "scripts"


def _load_script(name, filename):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EXEC = _load_script("feedback_policy_exec_test", "ov_exec.py")
# tie_once uses a sibling import. Bind that import only while loading it, so
# unrelated policy tests cannot accidentally reuse a different ov_exec module.
with patch.dict(sys.modules, {"ov_exec": EXEC}), patch.object(sys, "path", list(sys.path)):
    POLICY = _load_script("feedback_policy_tie_test", "tie_once.py")


class TestFeedbackPolicy(unittest.TestCase):
    def test_lay_swaps_only_when_loop_other_hand_is_free(self):
        params = {"lay": {"height": 0.035, "retries": 1}}
        for phase, anchored, expected in (
            ("loop", False, True),
            ("loop", True, False),
            ("over", False, False),
            ("over", True, False),
        ):
            with self.subTest(phase=phase, anchored=anchored):
                hands = EXEC.Hands()
                if anchored:
                    hands.held[0] = {"s": 0.4}
                step = {"kind": "lay", "phase": phase, "arm_id": 1,
                        "s": 0.07, "xy": [0.45, 0.01]}
                result = {"ok": True, "arm_id": 0 if expected else 1,
                          "pinch": {"held": True}, "place": {"error": 0.0},
                          "carry": {"lag": 0.0}}
                with patch.object(EXEC, "lay", return_value=result) as lay:
                    out = EXEC.run_step(Mock(), step, params, hands, 0.76)
                self.assertEqual(lay.call_args.kwargs["allow_swap"], expected)
                self.assertEqual(out["actual_arm"], result["arm_id"])
                self.assertEqual(0 in hands.held, anchored)

    def test_phase_stops_immediately_after_required_step_fails(self):
        for kind in ("lay", "hold", "lift_hold", "aim_end", "push_under", "pull_through", "pull"):
            with self.subTest(kind=kind):
                steps = [{"k": 1, "phase": "thread", "kind": kind},
                         {"k": 2, "phase": "thread", "kind": "release"}]
                ctx = Mock()
                ctx.tool.return_value = {"layout": "unused"}
                rows = []
                with patch.object(POLICY.demo, "plan", return_value={"steps": steps}), \
                     patch.object(POLICY, "run_step", return_value={"ok": False}) as run_step:
                    _, results = POLICY._phase(ctx, "thread", {}, EXEC.Hands(), rows,
                                               state={"length": 0.76})
                run_step.assert_called_once()
                self.assertEqual(run_step.call_args.args[1]["k"], 1)
                self.assertEqual(len(results), 1)
                self.assertEqual(len(rows), 1)

    def test_replan_reads_current_state_and_replaces_remaining_hold(self):
        initial = {"length": 0.76, "generation": "initial"}
        current = {"length": 0.76, "generation": "after_crossing"}
        crossing = {"k": 4, "kind": "lay", "phase": "over", "replan_after": True}
        old_hold = {"k": 6, "kind": "hold", "phase": "over", "s": 0.44}
        new_hold = {"k": 6, "kind": "hold", "phase": "over", "s": 0.52}
        ctx = Mock()
        layout = {"layout": "episode"}
        ctx.tool.side_effect = lambda name: layout if name == "rope.layout" else current
        hands = EXEC.Hands()
        hands.held[0] = {"s": 0.4}
        executed = []

        def act(_ctx, step, _params, same_hands, _length):
            self.assertIs(same_hands, hands)
            executed.append(step)
            return {"ok": True}

        with patch.object(POLICY.demo, "plan", side_effect=[
            {"steps": [crossing, old_hold]},
            {"steps": [crossing, new_hold]},
        ]) as plan, patch.object(POLICY, "run_step", side_effect=act):
            POLICY._phase(ctx, "over", {}, hands, [], state=initial)

        self.assertEqual(executed, [crossing, new_hold])
        self.assertIs(plan.call_args_list[0].args[1], initial)
        self.assertIs(plan.call_args_list[1].args[1], current)
        ctx.tool.assert_any_call("rope.state")
        self.assertEqual(hands.held[0], {"s": 0.4})

    def test_tip_capture_keeps_wrist_and_world_offset_when_tangent_changes(self):
        ctx = _TipContext()
        cap = tip_feedback.capture(ctx, 1, s=0.0)
        offset = np.array([0.012, -0.008, -0.02])
        np.testing.assert_allclose(cap["offset"], offset)
        first = tip_feedback.retarget_pose(ctx, arm_id=1, s=0.0,
                                          target_xy=(0.48, 0.06), capture=cap, height=0.035)
        # Change only the free rope's tangent. It must not rotate a captured
        # world-space offset or silently turn the wrist while carrying.
        ctx.nodes[1:] = [[0.412, 0.042, 0.765], [0.412, 0.092, 0.765]]
        second = tip_feedback.retarget_pose(ctx, arm_id=1, s=0.0,
                                           target_xy=(0.48, 0.06), capture=cap, height=0.035)
        self.assertNotAlmostEqual(first["tip"]["heading_deg"], second["tip"]["heading_deg"])
        self.assertEqual(first["rotation"], ctx.rotation)
        self.assertEqual(second["rotation"], ctx.rotation)
        np.testing.assert_allclose(first["tcp"], [0.468, 0.068, 0.785])
        np.testing.assert_allclose(second["tcp"], first["tcp"])
        np.testing.assert_allclose(cap["offset"], offset)


class _TipContext:
    def __init__(self):
        self.nodes = np.array([[0.412, -0.008, 0.765], [0.462, -0.008, 0.765],
                               [0.512, -0.008, 0.765]])
        self.tcp = [0.4, 0.0, 0.785]
        self.rotation = {"w": 0.5, "x": 0.5, "y": 0.5, "z": 0.5}

    def tool(self, name, **kwargs):
        if name == "rope.point":
            if kwargs["s"] != 0.0:
                raise AssertionError("This fixture samples the captured endpoint")
            return {"position": self.nodes[0].tolist(), "heading_deg": 0.0}
        if name == "rope.state":
            arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.nodes, axis=0), axis=1))]
            return {"nodes": self.nodes.tolist(), "arc": arc.tolist()}
        if name == "robot.get_ee_pose":
            return {"pose": {"position": dict(zip(("x", "y", "z"), self.tcp)),
                             "rotation": dict(self.rotation)}}
        raise AssertionError(f"Unexpected tool call while wrist is locked: {name}")


if __name__ == "__main__":
    unittest.main()
