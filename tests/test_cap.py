import json
import tempfile
import unittest
from pathlib import Path

from knotlab.cap import CapRecorder, from_trace, replay, validate


class _Ctx:
    def __init__(self):
        self.progress = 0.0
        self.calls = []

    def tool(self, name, **kw):
        self.calls.append((name, kw))
        if name == "rope.state":
            return {"code": "", "target_code": "", "prefix": "", "progress": self.progress,
                    "tied": False, "length": 1.0, "crossings": []}
        if name == "robot.go_to_pose":
            self.progress = min(1.0, self.progress + 0.1)
            return {"outcome": "reached"}
        if name == "robot.set_grip":
            return {"outcome": "reached"}
        raise AssertionError(name)


class TestCap(unittest.TestCase):
    def test_record_save_and_replay_robot_only(self):
        ctx = _Ctx()
        rec = CapRecorder(ctx, "lift_probe", requires_hold=True, metadata={"source": "manual_rehearsal"})
        rec.run(lambda c: (c.tool("robot.set_grip", preset="open", arm_id=0),
                           c.tool("robot.go_to_pose", pose={"position": {"x": 0.5, "y": 0.0, "z": 0.8}}, arm_id=0)))
        cap = rec.as_dict()
        validate(cap)
        self.assertTrue(cap["audit"]["robot_only"])
        with tempfile.TemporaryDirectory() as d:
            p = rec.save(Path(d) / "lift.json")
            self.assertEqual(json.loads(p.read_text())["name"], "lift_probe")
        replay_ctx = _Ctx()
        report = replay(replay_ctx, cap)
        self.assertTrue(report["ok"])
        self.assertEqual([x[0] for x in replay_ctx.calls if x[0].startswith("robot.")],
                         ["robot.set_grip", "robot.go_to_pose"])

    def test_validate_rejects_state_mutation(self):
        with self.assertRaises(ValueError):
            validate({"cap_version": 1, "kind": "robot_action_cap", "calls": [
                {"tool": "rope.set_nodes", "args": {}, "result": {}}]})

    def test_replay_stops_on_unreached(self):
        ctx = _Ctx()
        rec = CapRecorder(ctx, "x")
        rec.run(lambda c: c.tool("robot.go_to_pose", pose={}, arm_id=0))
        cap = rec.as_dict()
        class Bad(_Ctx):
            def tool(self, name, **kw):
                if name == "robot.go_to_pose":
                    self.calls.append((name, kw))
                    return {"outcome": "timeout"}
                return super().tool(name, **kw)
        self.assertFalse(replay(Bad(), cap)["ok"])

    def test_from_trace_discards_perception_calls(self):
        with tempfile.TemporaryDirectory() as d:
            calls = Path(d) / "calls"
            for i, tool in enumerate(("point", "go_to_pose", "set_grip")):
                p = calls / f"{i:03d}_{tool}"
                p.mkdir(parents=True)
                (p / "request.json").write_text(json.dumps({"arm_id": 0}))
                (p / "response.json").write_text(json.dumps({"outcome": "reached"}))
            cap = from_trace(calls, name="trace_cap")
            self.assertEqual([c["tool"] for c in cap["calls"]], ["robot.go_to_pose", "robot.set_grip"])
            self.assertTrue(cap["audit"]["source_trace_untrusted"])


if __name__ == "__main__":
    unittest.main()
