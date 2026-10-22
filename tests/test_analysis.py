"""Tightening moves material through crossings without changing topology."""
import unittest

from harness.analyze import diagnose, target_crossings


class TestTighteningDiagnosis(unittest.TestCase):
    def episode(self, height):
        variant = {"knot": "overhand", "board_x": -0.07, "start_shape": "loose_knot"}
        return {"id": "synthetic", "dir": "/nonexistent/knotlab-analysis-fixture", "rows": [], "events": [],
                "result": {"success": False, "variant": variant, "final": {
                    "code": "1U 2O 3U 1O 2U 3O", "prefix": 6, "tied": False,
                    "max_height": height, "x": [[0.30, 0.55, 0], [0.35, 0.60, 1], [0.40, 0.65, 0]]}}}

    def test_material_sliding_is_not_a_wrong_diagram(self):
        ep = self.episode(0.03)
        diag = diagnose(ep, target_crossings(variant=ep["result"]["variant"]))
        self.assertTrue(all(c["state"] == "OK" for c in diag["crossings"]))
        self.assertEqual(diag["extra_crossings"], 0)
        self.assertEqual(diag["cause"], "knot not drawn tight enough")

    def test_airborne_knot_has_a_distinct_failure(self):
        ep = self.episode(0.10)
        diag = diagnose(ep, target_crossings(variant=ep["result"]["variant"]))
        self.assertEqual(diag["cause"], "knot not returned to the bench")
