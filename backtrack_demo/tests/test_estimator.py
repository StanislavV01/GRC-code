import math
import unittest

from backtrack.estimator import DeadReckoning
from backtrack.kinematics import DiffDriveModel


class TestDeadReckoning(unittest.TestCase):
    def setUp(self):
        self.model = DiffDriveModel()

    def _erpm(self, v_wheel):
        return self.model.erpm_from_wheel_speed(v_wheel)

    def test_clean_straight_line(self):
        # Perfect odometry + gyro -> estimate matches a clean integrator.
        est = DeadReckoning(self.model, heading_source="fusion")
        dt = 0.02
        erpm = self._erpm(1.0)
        for _ in range(500):  # 1 m/s for 10 s = 10 m
            est.update(dt, erpm, erpm, 0.0, 0.0, current_load=0.0)
        self.assertAlmostEqual(est.x, 10.0, places=1)
        self.assertAlmostEqual(est.y, 0.0, places=2)

    def test_fusion_heading_follows_gyro(self):
        est = DeadReckoning(self.model, heading_source="fusion", compass_gain=0.0)
        dt = 0.02
        erpm = self._erpm(1.0)
        gyro = 0.2
        for _ in range(500):  # integrate 0.2 rad/s for 10 s -> ~2 rad wrapped
            est.update(dt, erpm, erpm, gyro, 0.0, current_load=1.0)
        expected = math.atan2(math.sin(2.0), math.cos(2.0))
        self.assertAlmostEqual(est.heading, expected, places=3)

    def test_breadcrumbs_spaced_by_distance(self):
        est = DeadReckoning(self.model, heading_source="fusion",
                            breadcrumb_spacing=0.5)
        dt = 0.02
        erpm = self._erpm(1.0)
        for _ in range(500):  # travel 10 m
            est.update(dt, erpm, erpm, 0.0, 0.0)
        # ~10 m at 0.5 m spacing -> about 20 breadcrumbs beyond the origin.
        self.assertGreaterEqual(len(est.breadcrumbs), 19)
        self.assertLessEqual(len(est.breadcrumbs), 22)
        # Consecutive crumbs are roughly one spacing apart.
        b0, b1 = est.breadcrumbs[1], est.breadcrumbs[2]
        gap = math.hypot(b1.x - b0.x, b1.y - b0.y)
        self.assertAlmostEqual(gap, 0.5, delta=0.05)

    def test_odometry_heading_drifts_more_with_slip(self):
        # When wheel odometry over-reports rotation (the gyro disagrees),
        # the odometry-only estimator drifts further than the fusion one.
        fusion = DeadReckoning(self.model, heading_source="fusion")
        odo = DeadReckoning(self.model, heading_source="odometry")
        dt = 0.02
        # Command a turn via wheel difference, but the gyro sees a smaller,
        # truthful rotation (skid slip).
        vl, vr = 1.0, 1.4
        erpm_l, erpm_r = self._erpm(vl), self._erpm(vr)
        odo_omega = (vr - vl) / self.model.track_width
        true_omega = odo_omega * 0.85  # 15% slip
        for _ in range(300):
            fusion.update(dt, erpm_l, erpm_r, true_omega, 0.0, current_load=0.5)
            odo.update(dt, erpm_l, erpm_r, true_omega, 0.0, current_load=0.5)
        self.assertGreater(abs(odo.heading), abs(fusion.heading))


if __name__ == "__main__":
    unittest.main()
