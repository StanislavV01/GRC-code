#ifndef BACKTRACK_CONTROLLER_HPP
#define BACKTRACK_CONTROLLER_HPP

// Backtrack path-following controller (TZ FR-4).
//
// When all control channels are lost, the vehicle follows its recorded
// breadcrumb trail *in reverse* back toward the start of recording (or a rally
// point) using pure pursuit. It steers using only the dead-reckoned pose --
// no GNSS, no comms.

#include <vector>

#include "backtrack/types.hpp"

namespace backtrack {

struct PursuitConfig {
    double cruise_speed = 1.5;    // autonomous speed, below manual (FR-4) [m/s]
    double lookahead = 1.5;       // pure-pursuit lookahead distance [m]
    double max_yaw_rate = 1.0;    // steering clamp [rad/s]
    double arrival_radius = 0.6;  // consider target reached within this [m]
    double turn_slowdown = 1.2;   // speed reduction factor for sharp headings
    // Reverse-only backtrack (FR-4): drive backwards without turning the hull
    // around -- the chassis keeps its outbound orientation, the rear leads.
    // For narrow lanes / preserving frontal orientation.
    bool reverse = false;
};

struct Command {
    double v{0.0};
    double omega{0.0};
    bool done{false};
};

class BacktrackController {
public:
    BacktrackController(std::vector<Point> path, const PursuitConfig& config);

    // Given the dead-reckoned pose, return (v, omega, done).
    Command compute(const Pose& pose);

    bool done() const { return done_; }
    const Point& target() const { return path_.back(); }

private:
    void advance_closest(double x, double y);
    Point lookahead_point() const;

    std::vector<Point> path_;
    PursuitConfig cfg_;
    std::size_t idx_{0};  // index of the closest path point reached so far
    bool done_{false};
};

}  // namespace backtrack

#endif  // BACKTRACK_CONTROLLER_HPP
