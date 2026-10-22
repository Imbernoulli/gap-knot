"""The goal and robot perception must agree across world/bench coordinates."""
import unittest

import numpy as np

from knotlab import task


class TestTighteningScore(unittest.TestCase):
    def setUp(self):
        self.previous = task.CURRENT.copy()
        v = task.Variant(knot="overhand", board_x=-0.07, start_shape="loose_knot")
        self.loose = task.layout(v)["start"]
        task.CURRENT.clear()
        task.CURRENT.update(variant=v, L0=task.diagram_knot_length(self.loose))
        self.tight = self.loose.copy()
        self.tight[:, :2] *= 0.65

    def tearDown(self):
        task.CURRENT.clear()
        task.CURRENT.update(self.previous)

    def test_resting_tight_knot_counts_in_world_frame(self):
        self.assertTrue(task.score(self.tight)["tied"])
        self.assertEqual(task.world_progress(self.tight + task.BENCH_ORIGIN), 1.0)

    def test_airborne_tight_knot_does_not_count(self):
        airborne = self.tight + np.array([0, 0, 0.10])
        self.assertFalse(task.score(airborne)["tied"])
        self.assertLess(task.world_progress(airborne + task.BENCH_ORIGIN), 1.0)

    def test_loose_knot_does_not_count(self):
        self.assertFalse(task.score(self.loose)["tied"])
        self.assertLess(task.world_progress(self.loose + task.BENCH_ORIGIN), 1.0)

    def test_unknotted_rope_does_not_count(self):
        straight = self.tight.copy()
        straight[:, 0] = np.linspace(-0.3, 0.3, len(straight))
        straight[:, 1:] = [0, 0.005]
        self.assertLess(task.world_progress(straight + task.BENCH_ORIGIN), 1.0)

    def test_hands_must_be_open_and_withdrawn(self):
        nodes = self.tight + task.BENCH_ORIGIN
        far = [nodes[0] + [0, 0, 0.12], nodes[-1] + [0, 0, 0.12]]
        self.assertTrue(task.hands_clear(nodes, far, [0.095, 0.095]))
        self.assertFalse(task.hands_clear(nodes, far, [0.01, 0.095]))
        near = [nodes[0] + [0, 0, 0.02], far[1]]
        self.assertFalse(task.hands_clear(nodes, near, [0.095, 0.095]))


if __name__ == "__main__":
    unittest.main()
