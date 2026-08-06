"""Small angle / vector helpers used across the backtrack demo."""

import math


def wrap_angle(theta):
    """Wrap an angle to the range (-pi, pi]."""
    return math.atan2(math.sin(theta), math.cos(theta))


def angle_diff(target, source):
    """Smallest signed difference target - source, wrapped to (-pi, pi]."""
    return wrap_angle(target - source)


def distance(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def clamp(value, low, high):
    return max(low, min(high, value))
