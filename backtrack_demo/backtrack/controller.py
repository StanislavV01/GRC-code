"""Backtrack path-following controller (TZ FR-4).

When all control channels are lost, the vehicle follows its recorded breadcrumb
trail *in reverse* back toward the start of recording (or a rally point) using
pure pursuit. It steers using only the dead-reckoned pose -- no GNSS, no comms.
"""

import math
from dataclasses import dataclass

from .geometry import angle_diff, clamp, distance


@dataclass
class PursuitConfig:
    cruise_speed: float = 1.5     # autonomous speed, below manual (FR-4) [m/s]
    lookahead: float = 1.5        # pure-pursuit lookahead distance [m]
    max_yaw_rate: float = 1.0     # steering clamp [rad/s]
    arrival_radius: float = 0.6   # consider target reached within this [m]
    turn_slowdown: float = 1.2    # speed reduction factor for sharp headings


class BacktrackController:
    """Follows a fixed path (list of (x, y)) from its first to last point."""

    def __init__(self, path, config=None):
        if len(path) < 2:
            raise ValueError("path needs at least two points")
        self.path = list(path)
        self.cfg = config or PursuitConfig()
        self._idx = 0          # index of the closest path point reached so far
        self.done = False

    @property
    def target(self):
        return self.path[-1]

    def _advance_closest(self, x, y):
        """Move the progress cursor forward to the nearest path point ahead."""
        best_i = self._idx
        best_d = distance(x, y, *self.path[self._idx])
        # Only ever scan forward so the controller cannot regress along the path.
        for i in range(self._idx + 1, len(self.path)):
            d = distance(x, y, *self.path[i])
            if d < best_d:
                best_d = d
                best_i = i
            # stop scanning once we are clearly past the local minimum
            if d > best_d + self.cfg.lookahead * 2:
                break
        self._idx = best_i

    def _lookahead_point(self, x, y):
        acc = 0.0
        prev = self.path[self._idx]
        for i in range(self._idx + 1, len(self.path)):
            cur = self.path[i]
            acc += distance(prev[0], prev[1], cur[0], cur[1])
            prev = cur
            if acc >= self.cfg.lookahead:
                return cur
        return self.path[-1]

    def compute(self, pose):
        """Given the dead-reckoned pose, return (v_cmd, omega_cmd, done)."""
        x, y, heading = pose
        if distance(x, y, *self.target) <= self.cfg.arrival_radius:
            self.done = True
            return 0.0, 0.0, True

        self._advance_closest(x, y)
        tx, ty = self._lookahead_point(x, y)

        alpha = angle_diff(math.atan2(ty - y, tx - x), heading)
        ld = self.cfg.lookahead
        curvature = 2.0 * math.sin(alpha) / ld
        omega = clamp(self.cfg.cruise_speed * curvature,
                      -self.cfg.max_yaw_rate, self.cfg.max_yaw_rate)

        # Slow down for sharp corrections so pure pursuit stays stable.
        v = self.cfg.cruise_speed / (1.0 + self.cfg.turn_slowdown * abs(alpha))
        return v, omega, False
