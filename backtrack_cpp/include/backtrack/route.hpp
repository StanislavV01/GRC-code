#ifndef BACKTRACK_ROUTE_HPP
#define BACKTRACK_ROUTE_HPP

// Outbound route definition for the ~300 m offroad demo.
//
// The route is a list of constant-speed arcs, each described by its ground
// length and total heading change. It expands into a per-tick stream of desired
// (v, omega) commands the simulation plays back on the way out.

#include <vector>

#include "backtrack/types.hpp"

namespace backtrack {

struct Maneuver {
    double length;  // ground distance of the segment [m]
    double turn;    // total heading change over the segment [rad]
};

// ~300 m wandering offroad track: straights joined by left/right turns.
const std::vector<Maneuver>& default_route();

double total_length(const std::vector<Maneuver>& route = default_route());

// Expand the route into per-tick (v, omega) commands.
std::vector<Twist> outbound_commands(
    double dt, double cruise_speed = 2.0,
    const std::vector<Maneuver>& route = default_route());

}  // namespace backtrack

#endif  // BACKTRACK_ROUTE_HPP
