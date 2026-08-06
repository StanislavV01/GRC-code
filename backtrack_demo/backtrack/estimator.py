"""GNSS-denied dead-reckoning estimator (TZ FR-3 + breadcrumb recording FR-2).

Fuses wheel odometry (VESC ERPM) with the gyro and compass to integrate the
vehicle pose without GNSS, and records a breadcrumb trail at a fixed travelled
distance. This is the component that lets the vehicle know the route it took --
the trail is what the backtrack controller follows home.

Two heading sources are supported so the demo can contrast them:

  * 'fusion'   : gyro-integrated heading with a slip-aware compass correction.
                 The compass is trusted less while motor current is high, since
                 that is exactly when the 84V power stage corrupts it and when
                 skid-steer slip corrupts odometry-derived rotation.
  * 'odometry' : heading from wheel-speed difference only. Included to show why
                 the fusion path is necessary -- it drifts badly in turns.
"""

import math
from dataclasses import dataclass

from .geometry import angle_diff, wrap_angle


@dataclass
class Breadcrumb:
    x: float
    y: float
    heading: float
    cumulative_dist: float


class DeadReckoning:
    def __init__(self, model, heading_source="fusion",
                 breadcrumb_spacing=0.5, compass_gain=0.03,
                 init_pose=(0.0, 0.0, 0.0)):
        if heading_source not in ("fusion", "odometry"):
            raise ValueError("heading_source must be 'fusion' or 'odometry'")
        self.model = model
        self.heading_source = heading_source
        self.spacing = breadcrumb_spacing
        self.compass_gain = compass_gain

        self.x, self.y, self.heading = init_pose
        self.total_dist = 0.0
        self._dist_since_crumb = 0.0
        self.breadcrumbs = [Breadcrumb(self.x, self.y, self.heading, 0.0)]

    @property
    def pose(self):
        return (self.x, self.y, self.heading)

    def update(self, dt, erpm_left, erpm_right, gyro_z, compass_heading,
               current_load=0.0):
        """Advance the estimate by one tick and maybe drop a breadcrumb."""
        v_left = self.model.wheel_speed_from_erpm(erpm_left)
        v_right = self.model.wheel_speed_from_erpm(erpm_right)
        v, omega_odo = self.model.twist_from_wheel_speeds(v_left, v_right)

        if self.heading_source == "fusion":
            self.heading = wrap_angle(self.heading + gyro_z * dt)
            # Trust the compass less the harder the motors are working.
            trust = self.compass_gain * (1.0 - min(1.0, current_load))
            self.heading = wrap_angle(
                self.heading + trust * angle_diff(compass_heading, self.heading))
        else:
            self.heading = wrap_angle(self.heading + omega_odo * dt)

        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt

        step = abs(v * dt)
        self.total_dist += step
        self._dist_since_crumb += step
        if self._dist_since_crumb >= self.spacing:
            self._dist_since_crumb = 0.0
            self.breadcrumbs.append(
                Breadcrumb(self.x, self.y, self.heading, self.total_dist))

        return self.pose

    def trail_xy(self):
        """Breadcrumb trail as a list of (x, y) tuples, oldest first."""
        return [(b.x, b.y) for b in self.breadcrumbs]
