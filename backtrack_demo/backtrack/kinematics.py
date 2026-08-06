"""Differential-drive (skid-steer) kinematics and VESC ERPM conversions.

Models the target platform from the TZ: a skid-steer UGV whose wheels are
driven by VESC-based ESCs (FSESC 75450). Wheel speed is observed on the CAN
bus as motor ERPM; this module converts between the unicycle twist (v, omega),
the per-wheel linear speeds, and ERPM.
"""

import math

from .geometry import wrap_angle


class DiffDriveModel:
    """Kinematic model of a skid-steer rover.

    track_width : distance between left/right wheel contact lines [m]
    wheel_radius: drive wheel radius [m]
    pole_pairs  : motor pole pairs (ERPM = mechanical_rpm * pole_pairs)
    gear_ratio  : motor revolutions per wheel revolution
    """

    def __init__(self, track_width=0.5, wheel_radius=0.165,
                 pole_pairs=15, gear_ratio=1.0):
        if track_width <= 0:
            raise ValueError("track_width must be positive")
        if wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")
        self.track_width = track_width
        self.wheel_radius = wheel_radius
        self.pole_pairs = pole_pairs
        self.gear_ratio = gear_ratio
        self._circumference = 2.0 * math.pi * wheel_radius

    # -- twist <-> wheel speeds -------------------------------------------
    def wheel_speeds_from_twist(self, v, omega):
        """(linear v, yaw omega) -> (left, right) wheel linear speed [m/s]."""
        half = omega * self.track_width / 2.0
        return v - half, v + half

    def twist_from_wheel_speeds(self, v_left, v_right):
        """(left, right) wheel linear speed -> (linear v, yaw omega)."""
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.track_width
        return v, omega

    # -- wheel speed <-> VESC ERPM ----------------------------------------
    def erpm_from_wheel_speed(self, v_wheel):
        """Wheel ground speed [m/s] -> motor electrical RPM."""
        wheel_rpm = v_wheel / self._circumference * 60.0
        return wheel_rpm * self.gear_ratio * self.pole_pairs

    def wheel_speed_from_erpm(self, erpm):
        """Motor electrical RPM -> wheel ground speed [m/s]."""
        wheel_rpm = erpm / self.pole_pairs / self.gear_ratio
        return wheel_rpm / 60.0 * self._circumference

    # -- pose integration --------------------------------------------------
    def step_pose(self, pose, v, omega, dt):
        """Exact arc integration of a unicycle pose.

        pose: (x, y, heading) -> returns new (x, y, heading).
        """
        x, y, theta = pose
        if abs(omega) < 1e-9:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
        else:
            radius = v / omega
            dtheta = omega * dt
            x += radius * (math.sin(theta + dtheta) - math.sin(theta))
            y -= radius * (math.cos(theta + dtheta) - math.cos(theta))
            theta = theta + dtheta
        return x, y, wrap_angle(theta)
