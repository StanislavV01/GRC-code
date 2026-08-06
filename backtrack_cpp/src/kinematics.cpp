#include "backtrack/kinematics.hpp"

#include <cmath>
#include <stdexcept>

#include "backtrack/geometry.hpp"

namespace backtrack {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kOmegaEps = 1e-9;
}  // namespace

DiffDriveModel::DiffDriveModel(double track_width, double wheel_radius,
                               double pole_pairs, double internal_gear,
                               double external_gear)
    : track_width_(track_width),
      wheel_radius_(wheel_radius),
      pole_pairs_(pole_pairs),
      total_gear_(internal_gear * external_gear),
      circumference_(kTwoPi * wheel_radius) {
    if (track_width <= 0.0) throw std::invalid_argument("track_width must be positive");
    if (wheel_radius <= 0.0) throw std::invalid_argument("wheel_radius must be positive");
    if (total_gear_ <= 0.0) throw std::invalid_argument("gear ratio must be positive");
}

WheelSpeeds DiffDriveModel::wheel_speeds_from_twist(double v, double omega) const {
    const double half = omega * track_width_ / 2.0;
    return WheelSpeeds{v - half, v + half};
}

Twist DiffDriveModel::twist_from_wheel_speeds(double v_left, double v_right) const {
    const double v = (v_left + v_right) / 2.0;
    const double omega = (v_right - v_left) / track_width_;
    return Twist{v, omega};
}

double DiffDriveModel::erpm_from_wheel_speed(double v_wheel) const {
    const double wheel_rpm = v_wheel / circumference_ * 60.0;
    return wheel_rpm * total_gear_ * pole_pairs_;
}

double DiffDriveModel::wheel_speed_from_erpm(double erpm) const {
    const double wheel_rpm = erpm / pole_pairs_ / total_gear_;
    return wheel_rpm / 60.0 * circumference_;
}

Pose DiffDriveModel::step_pose(const Pose& pose, double v, double omega,
                               double dt) const {
    double x = pose.x;
    double y = pose.y;
    double theta = pose.heading;
    if (std::abs(omega) < kOmegaEps) {
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
    } else {
        const double radius = v / omega;
        const double dtheta = omega * dt;
        x += radius * (std::sin(theta + dtheta) - std::sin(theta));
        y -= radius * (std::cos(theta + dtheta) - std::cos(theta));
        theta += dtheta;
    }
    return Pose{x, y, wrap_angle(theta)};
}

}  // namespace backtrack
