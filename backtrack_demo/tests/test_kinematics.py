import math
import unittest

from backtrack.geometry import wrap_angle
from backtrack.kinematics import DiffDriveModel


class TestGeometry(unittest.TestCase):
    def test_wrap_into_range(self):
        # 3*pi and -3*pi both represent the same heading as +/-pi (cos == -1).
        self.assertAlmostEqual(math.cos(wrap_angle(3 * math.pi)), -1.0, places=6)
        self.assertAlmostEqual(math.cos(wrap_angle(-3 * math.pi)), -1.0, places=6)
        self.assertLessEqual(abs(wrap_angle(3 * math.pi)), math.pi + 1e-9)
        self.assertAlmostEqual(wrap_angle(0.5), 0.5, places=6)


class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.m = DiffDriveModel(track_width=0.5, wheel_radius=0.165)

    def test_twist_wheel_roundtrip(self):
        v, omega = 1.7, 0.4
        vl, vr = self.m.wheel_speeds_from_twist(v, omega)
        v2, omega2 = self.m.twist_from_wheel_speeds(vl, vr)
        self.assertAlmostEqual(v, v2, places=9)
        self.assertAlmostEqual(omega, omega2, places=9)

    def test_erpm_roundtrip(self):
        for speed in (0.0, 0.5, 2.0, -1.3):
            erpm = self.m.erpm_from_wheel_speed(speed)
            self.assertAlmostEqual(self.m.wheel_speed_from_erpm(erpm), speed,
                                   places=9)

    def test_straight_line(self):
        pose = (0.0, 0.0, 0.0)
        for _ in range(100):
            pose = self.m.step_pose(pose, 1.0, 0.0, 0.01)  # 1 m/s, 1 s total
        self.assertAlmostEqual(pose[0], 1.0, places=3)
        self.assertAlmostEqual(pose[1], 0.0, places=6)
        self.assertAlmostEqual(pose[2], 0.0, places=6)

    def test_pure_rotation_keeps_position(self):
        pose = (2.0, -1.0, 0.0)
        pose = self.m.step_pose(pose, 0.0, 1.0, 0.5)
        self.assertAlmostEqual(pose[0], 2.0, places=9)
        self.assertAlmostEqual(pose[1], -1.0, places=9)
        self.assertAlmostEqual(pose[2], 0.5, places=9)

    def test_full_circle_returns_to_start(self):
        pose = (0.0, 0.0, 0.0)
        v, omega = 1.0, 1.0  # radius 1 m
        steps = 1000
        dt = (2 * math.pi) / omega / steps
        for _ in range(steps):
            pose = self.m.step_pose(pose, v, omega, dt)
        self.assertAlmostEqual(pose[0], 0.0, places=3)
        self.assertAlmostEqual(pose[1], 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
