#include "backtrack/controller.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "backtrack/geometry.hpp"

namespace backtrack {

BacktrackController::BacktrackController(std::vector<Point> path,
                                         const PursuitConfig& config)
    : path_(std::move(path)), cfg_(config) {
    if (path_.size() < 2) {
        throw std::invalid_argument("path needs at least two points");
    }
}

void BacktrackController::advance_closest(double x, double y) {
    std::size_t best_i = idx_;
    double best_d = distance(x, y, path_[idx_].x, path_[idx_].y);
    // Only ever scan forward so the controller cannot regress along the path.
    for (std::size_t i = idx_ + 1; i < path_.size(); ++i) {
        const double d = distance(x, y, path_[i].x, path_[i].y);
        if (d < best_d) {
            best_d = d;
            best_i = i;
        }
        // Stop scanning once we are clearly past the local minimum.
        if (d > best_d + cfg_.lookahead * 2.0) break;
    }
    idx_ = best_i;
}

Point BacktrackController::lookahead_point() const {
    double acc = 0.0;
    Point prev = path_[idx_];
    for (std::size_t i = idx_ + 1; i < path_.size(); ++i) {
        const Point cur = path_[i];
        acc += distance(prev.x, prev.y, cur.x, cur.y);
        prev = cur;
        if (acc >= cfg_.lookahead) return cur;
    }
    return path_.back();
}

Command BacktrackController::compute(const Pose& pose) {
    const Point tgt = path_.back();
    if (distance(pose.x, pose.y, tgt.x, tgt.y) <= cfg_.arrival_radius) {
        done_ = true;
        return Command{0.0, 0.0, true};
    }

    advance_closest(pose.x, pose.y);
    const Point look = lookahead_point();

    // The leading direction is the hull nose when driving forward, or the rear
    // (heading + pi) when reversing without turning around. Pursuit geometry is
    // identical once measured from whichever end leads.
    const double leading_heading =
        cfg_.reverse ? wrap_angle(pose.heading + M_PI) : pose.heading;
    const double bearing = std::atan2(look.y - pose.y, look.x - pose.x);
    const double alpha = angle_diff(bearing, leading_heading);

    // Positive speed magnitude, slowed for sharp corrections; sign set by mode.
    const double speed =
        cfg_.cruise_speed / (1.0 + cfg_.turn_slowdown * std::abs(alpha));
    const double curvature = 2.0 * std::sin(alpha) / cfg_.lookahead;
    const double omega =
        clamp(speed * curvature, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
    const double v = cfg_.reverse ? -speed : speed;
    return Command{v, omega, false};
}

}  // namespace backtrack
