#include "backtrack/route.hpp"

#include <cmath>

namespace backtrack {

const std::vector<Maneuver>& default_route() {
    static const std::vector<Maneuver> route = {
        {70.0, 0.0},
        {25.0, +M_PI / 2.0},        // left 90
        {55.0, 0.0},
        {20.0, -M_PI / 2.0},        // right 90
        {60.0, 0.0},
        {30.0, +2.0 * M_PI / 3.0},  // left 120
        {40.0, 0.0},
    };
    return route;
}

double total_length(const std::vector<Maneuver>& route) {
    double sum = 0.0;
    for (const Maneuver& m : route) sum += m.length;
    return sum;
}

std::vector<Twist> outbound_commands(double dt, double cruise_speed,
                                     const std::vector<Maneuver>& route) {
    std::vector<Twist> commands;
    for (const Maneuver& m : route) {
        const double duration = m.length / cruise_speed;
        const int steps = std::max(1, static_cast<int>(std::lround(duration / dt)));
        const double omega = m.turn / (steps * dt);
        for (int i = 0; i < steps; ++i) {
            commands.push_back(Twist{cruise_speed, omega});
        }
    }
    return commands;
}

}  // namespace backtrack
