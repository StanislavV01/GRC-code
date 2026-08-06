"""Synthetic on-board sensors.

Given the *commanded* wheel speeds and the *true* vehicle motion, produce the
raw measurements that the real hardware would log:

  * VESC / CAN bus : per-wheel ERPM + motor current
  * Gyroscope (IMU): yaw rate (sees true rotation, with bias + noise)
  * Magnetometer   : compass heading (true heading, disturbed by motor current)

The key modelled imperfection, straight from the TZ risk table, is that the
magnetometer is corrupted by the large currents of the 84V power stage, and
that skid-steer slip makes wheel odometry lie about rotation -- so heading must
lean on the gyro. These are exactly the errors the dead-reckoning estimator has
to survive on the way home.
"""

import random
from dataclasses import dataclass

from .geometry import clamp, wrap_angle


@dataclass
class SensorParams:
    erpm_noise: float = 4.0          # std dev on ERPM readings
    gyro_bias: float = 0.004         # constant gyro yaw-rate bias [rad/s]
    gyro_noise: float = 0.01         # std dev on gyro yaw rate [rad/s]
    compass_noise: float = 0.02      # std dev on compass heading [rad]
    compass_current_gain: float = 0.18   # heading error at full current load [rad]
    current_idle: float = 2.0        # baseline motor current [A]
    current_per_speed: float = 3.0   # A per (|v_l|+|v_r|) [A/(m/s)]
    current_per_turn: float = 12.0   # A per |omega| [A/(rad/s)]
    current_noise: float = 0.6       # std dev on current [A]
    current_max: float = 40.0        # full-scale current for load normalisation [A]


class SensorModel:
    """Stateless-ish sensor synthesiser driven by a seeded RNG."""

    def __init__(self, model, params=None, seed=0):
        self.model = model
        self.p = params or SensorParams()
        self.rng = random.Random(seed)

    def motor_current(self, v_left_cmd, v_right_cmd, omega_cmd):
        p = self.p
        load = (p.current_idle
                + p.current_per_speed * (abs(v_left_cmd) + abs(v_right_cmd))
                + p.current_per_turn * abs(omega_cmd))
        return max(0.0, load + self.rng.gauss(0.0, p.current_noise))

    def sample(self, v_left_cmd, v_right_cmd, omega_cmd, true_omega, true_heading):
        """Produce one synchronized sensor record.

        Returns a dict with erpm_left, erpm_right, motor_current,
        gyro_z, compass_heading, current_load (0..1).
        """
        p = self.p
        erpm_left = (self.model.erpm_from_wheel_speed(v_left_cmd)
                     + self.rng.gauss(0.0, p.erpm_noise))
        erpm_right = (self.model.erpm_from_wheel_speed(v_right_cmd)
                      + self.rng.gauss(0.0, p.erpm_noise))

        current = self.motor_current(v_left_cmd, v_right_cmd, omega_cmd)
        load = clamp(current / p.current_max, 0.0, 1.0)

        gyro_z = true_omega + p.gyro_bias + self.rng.gauss(0.0, p.gyro_noise)

        disturbance = p.compass_current_gain * load
        compass = wrap_angle(true_heading + disturbance
                             + self.rng.gauss(0.0, p.compass_noise))

        return {
            "erpm_left": erpm_left,
            "erpm_right": erpm_right,
            "motor_current": current,
            "gyro_z": gyro_z,
            "compass_heading": compass,
            "current_load": load,
        }
