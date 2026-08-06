"""Outbound route definition for the ~300 m offroad demo.

The route is a list of maneuvers, each a constant-speed arc described by its
ground length and the total heading change over that length. This expands into
a per-tick stream of desired (v, omega) commands the simulation plays back on
the way out.
"""

from dataclasses import dataclass


@dataclass
class Maneuver:
    length: float       # ground distance of the segment [m]
    turn: float         # total heading change over the segment [rad]


# ~300 m wandering offroad track: straights joined by left/right turns.
import math  # noqa: E402

DEFAULT_ROUTE = [
    Maneuver(70.0, 0.0),
    Maneuver(25.0, +math.pi / 2),    # left 90
    Maneuver(55.0, 0.0),
    Maneuver(20.0, -math.pi / 2),    # right 90
    Maneuver(60.0, 0.0),
    Maneuver(30.0, +2.0 * math.pi / 3),  # left 120
    Maneuver(40.0, 0.0),
]


def total_length(route=DEFAULT_ROUTE):
    return sum(m.length for m in route)


def outbound_commands(dt, cruise_speed=2.0, route=DEFAULT_ROUTE):
    """Expand the route into a list of (v, omega) commands, one per tick."""
    commands = []
    for m in route:
        duration = m.length / cruise_speed
        steps = max(1, int(round(duration / dt)))
        omega = m.turn / (steps * dt)
        for _ in range(steps):
            commands.append((cruise_speed, omega))
    return commands
