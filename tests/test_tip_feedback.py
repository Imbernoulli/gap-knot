import json
import unittest

import numpy as np

from knotlab import tip_feedback


class _Ctx:
    def __init__(self):
        self.nodes = np.array([[0.0, 0.0, 0.75], [0.1, 0.0, 0.75], [0.2, 0.1, 0.75]], dtype=float)
        self.tcp = np.array([0.0, 0.0, 0.80], dtype=float)

    def tool(self, name, **kw):
        if name == "rope.state":
            arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.nodes, axis=0), axis=1))]
            return {"nodes": self.nodes.tolist(), "arc": arc.tolist()}
        if name == "rope.point":
            s = float(kw["s"])
            arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.nodes, axis=0), axis=1))]
            i = min(int(np.searchsorted(arc, s, side="right") - 1), len(self.nodes) - 2)
            u = (s - arc[i]) / max(arc[i + 1] - arc[i], 1e-9)
            p = self.nodes[i] + u * (self.nodes[i + 1] - self.nodes[i])
            t = self.nodes[min(i + 1, len(self.nodes) - 1)] - self.nodes[max(i, 0)]
            return {"position": p.tolist(), "heading_deg": float(np.degrees(np.arctan2(t[1], t[0])))}
        if name == "robot.get_ee_pose":
            return {"pose": {"position": dict(zip(("x", "y", "z"), self.tcp)), "rotation": {"w": 1, "x": 0, "y": 0, "z": 0}}}
        if name == "robot.grasp_frame":
            return {"rotation": {"w": 1, "x": 0, "y": 0, "z": 0}}
        if name == "robot.go_to_pose_cartesian":
            return {"outcome": "reached"}
        raise AssertionError(name)


class TestTipFeedback(unittest.TestCase):
    def test_endpoint_tangent_points_inward_for_both_ends(self):
        ctx = _Ctx()
        first = tip_feedback.tip_state(ctx, end_first=True)
        last = tip_feedback.tip_state(ctx, end_first=False)
        self.assertGreater(first["tangent"][0], 0.9)
        self.assertLess(last["tangent"][0], -0.5)

    def test_capture_and_tcp_target_use_measured_offset(self):
        ctx = _Ctx()
        cap = tip_feedback.capture(ctx, 1, s=0.0)
        np.testing.assert_allclose(cap["offset"], [-0.0, 0.0, -0.05])
        target = tip_feedback.tcp_for_tip((0.4, -0.2), cap, z=0.785)
        np.testing.assert_allclose(target, [0.4, -0.2, 0.785])

    def test_move_result_is_trace_serializable(self):
        ctx = _Ctx()
        cap = tip_feedback.capture(ctx, 1, s=0.0)
        result = tip_feedback.move_tip(ctx, arm_id=1, s=0.0, target_xy=(0.0, 0.0), capture=cap, height=0.035)
        json.dumps(result)


if __name__ == "__main__":
    unittest.main()
