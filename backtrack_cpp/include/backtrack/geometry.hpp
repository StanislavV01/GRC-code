#ifndef BACKTRACK_GEOMETRY_HPP
#define BACKTRACK_GEOMETRY_HPP

// Small angle / vector helpers used across the backtrack core.
// Header-only: these are pure, cheap, and constexpr-friendly (F.4, F.8).

#include <cmath>

namespace backtrack {

// Wrap an angle to the range (-pi, pi].
inline double wrap_angle(double theta) {
    return std::atan2(std::sin(theta), std::cos(theta));
}

// Smallest signed difference target - source, wrapped to (-pi, pi].
inline double angle_diff(double target, double source) {
    return wrap_angle(target - source);
}

inline double distance(double ax, double ay, double bx, double by) {
    return std::hypot(bx - ax, by - ay);
}

constexpr double clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
}

}  // namespace backtrack

#endif  // BACKTRACK_GEOMETRY_HPP
