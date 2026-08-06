import math
import unittest

from backtrack.kinematics import DiffDriveModel
from backtrack.sensors import SensorModel, SensorParams


class TestSensors(unittest.TestCase):
    def setUp(self):
        self.model = DiffDriveModel()

    def test_erpm_reflects_wheel_speed(self):
        # No noise -> ERPM decodes back to the commanded wheel speed.
        params = SensorParams(erpm_noise=0.0, current_noise=0.0)
        sensor = SensorModel(self.model, params, seed=1)
        meas = sensor.sample(1.0, 1.5, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(
            self.model.wheel_speed_from_erpm(meas["erpm_left"]), 1.0, places=6)
        self.assertAlmostEqual(
            self.model.wheel_speed_from_erpm(meas["erpm_right"]), 1.5, places=6)

    def test_gyro_tracks_true_omega_on_average(self):
        params = SensorParams(gyro_noise=0.05, gyro_bias=0.0)
        sensor = SensorModel(self.model, params, seed=2)
        true_omega = 0.3
        vals = [sensor.sample(1.0, 1.0, true_omega, true_omega, 0.0)["gyro_z"]
                for _ in range(4000)]
        self.assertAlmostEqual(sum(vals) / len(vals), true_omega, places=2)

    def test_compass_disturbed_by_current(self):
        # With current load present, the mean compass heading is biased away
        # from truth -- exactly the 84V magnetometer problem from the TZ.
        params = SensorParams(compass_noise=0.0, compass_current_gain=0.18,
                              current_noise=0.0)
        sensor = SensorModel(self.model, params, seed=3)
        # Large turn command drives high current load.
        meas = sensor.sample(2.0, -2.0, 5.0, 0.0, 0.0)
        self.assertGreater(meas["current_load"], 0.0)
        self.assertGreater(abs(meas["compass_heading"]), 0.0)

    def test_current_increases_with_effort(self):
        params = SensorParams(current_noise=0.0)
        sensor = SensorModel(self.model, params, seed=4)
        idle = sensor.motor_current(0.0, 0.0, 0.0)
        driving = sensor.motor_current(2.0, 2.0, 0.0)
        turning = sensor.motor_current(2.0, -2.0, 4.0)
        self.assertLess(idle, driving)
        self.assertLess(driving, turning)


if __name__ == "__main__":
    unittest.main()
