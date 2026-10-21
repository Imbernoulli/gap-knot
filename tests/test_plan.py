"""The laying plan for the nominal double coin knot: every crossing made once, the fewest under steps."""

import unittest

import numpy as np

from knotlab import plan, task
from knotlab.diagram import arc_length


def nominal_layout() -> dict:
    lay = task.layout(task.NOMINAL)
    tgt = np.asarray(lay["target"]) + plan.BENCH_ORIGIN
    return {
        "target_nodes": tgt.tolist(),
        "target_arc": arc_length(tgt).tolist(),
        "pins": {k: (np.asarray(v) + plan.BENCH_ORIGIN).tolist() for k, v in lay["pins"].items()},
        "board_centre": (lay["board_centre"] + plan.BENCH_ORIGIN).tolist(),
    }


class TestLayPlan(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.layout = nominal_layout()
        cls.plan = plan.lay_plan(cls.layout)

    def test_every_crossing_is_made_exactly_once(self) -> None:
        made = [m["crossing"] for s in self.plan["steps"] for m in s["makes"]]
        self.assertEqual(sorted(made), list(range(8)))

    def test_two_under_steps_and_no_fewer_is_possible(self) -> None:
        self.assertEqual(len(self.plan["under_crossings"]), 2)
        threads = [s for s in self.plan["steps"] if s["kind"] == "thread"]
        self.assertEqual(len(threads), 2)
        for s in threads:
            self.assertIsNotNone(s["lift_s"])
            self.assertNotEqual(s["helper_arm"], s["arm_id"])

    def test_the_over_under_of_each_made_crossing_matches_the_target(self) -> None:
        locs, _ = plan.target_locations(self.layout["target_nodes"], self.layout["target_arc"])
        for s in self.plan["steps"]:
            for m in s["makes"]:
                mine = min((x for x in locs if x.crossing == m["crossing"]), key=lambda x: abs(x.s - s["s"]))
                self.assertEqual(m["working_over"], mine.over)

    def test_steps_walk_each_half_outward(self) -> None:
        for half in "AB":
            d = [s["d"] for s in self.plan["steps"] if s["half"] == half]
            self.assertEqual(d, sorted(d))
            self.assertAlmostEqual(max(d), self.plan["length"] / 2, delta=0.03)

    def test_knot_targets_are_in_reach(self) -> None:
        knot = [s for s in self.plan["steps"] if s["d"] < 0.55]
        self.assertTrue(all(s["reachable"] for s in knot), [s["xy"] for s in knot if not s["reachable"]])


if __name__ == "__main__":
    unittest.main()
