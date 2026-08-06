import unittest

from backtrack.geometry import distance
from backtrack.simulation import (MODE_ARRIVED, MODE_BACKTRACK, MODE_RECORDING,
                                  SimConfig, run_simulation)


class TestBacktrackEndToEnd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = run_simulation(SimConfig(heading_source="fusion", seed=7))

    def test_records_full_session(self):
        modes = {r["mode"] for r in self.result.records}
        self.assertIn(MODE_RECORDING, modes)
        self.assertIn(MODE_BACKTRACK, modes)
        self.assertEqual(self.result.records[-1]["mode"], MODE_ARRIVED)

    def test_link_drops_at_backtrack(self):
        outbound = [r for r in self.result.records if r["mode"] == MODE_RECORDING]
        backtrack = [r for r in self.result.records
                     if r["mode"] in (MODE_BACKTRACK, MODE_ARRIVED)]
        self.assertTrue(all(r["link_ok"] == 1 for r in outbound))
        self.assertTrue(all(r["link_ok"] == 0 for r in backtrack))

    def test_breadcrumb_trail_recorded(self):
        self.assertGreater(len(self.result.breadcrumbs), 100)

    def test_return_path_reverses_the_trail(self):
        # The reversed path starts at the outbound end and ends near home.
        start_of_return = self.result.return_path[0]
        end_of_return = self.result.return_path[-1]
        self.assertAlmostEqual(end_of_return[0], 0.0, delta=1.0)
        self.assertAlmostEqual(end_of_return[1], 0.0, delta=1.0)
        est_end = self.result.summary["est_outbound_end"]
        self.assertAlmostEqual(start_of_return[0], est_end[0], delta=1.0)
        self.assertAlmostEqual(start_of_return[1], est_end[1], delta=1.0)

    def test_backtrack_completes(self):
        self.assertTrue(self.result.summary["backtrack_completed"])

    def test_returns_home_within_acceptance(self):
        # TZ acceptance: <= 15 m final position error over a 500 m run (~3%).
        s = self.result.summary
        self.assertLessEqual(s["final_return_error_m"], 15.0)
        self.assertLessEqual(s["final_error_pct"], 5.0)

    def test_fusion_beats_odometry_only(self):
        odo = run_simulation(SimConfig(heading_source="odometry", seed=7))
        self.assertLess(self.result.summary["final_return_error_m"],
                        odo.summary["final_return_error_m"])


if __name__ == "__main__":
    unittest.main()
