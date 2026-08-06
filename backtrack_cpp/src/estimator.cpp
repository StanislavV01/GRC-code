#include "backtrack/estimator.hpp"

#include <cmath>

#include "backtrack/geometry.hpp"

namespace backtrack {

DeadReckoning::DeadReckoning(const DiffDriveModel& model, EstimatorConfig config,
                             Pose init_pose)
    : model_(model), cfg_(config), pose_(init_pose) {
    breadcrumbs_.push_back(Breadcrumb{pose_.x, pose_.y, pose_.heading, 0.0});
}

bool DeadReckoning::detect_stationary(double v_odo, double gyro_z,
                                      double accel_x, double accel_y,
                                      double accel_z) const {
    if (!cfg_.zupt && !cfg_.calibrate_bias) return false;
    const double accel_mag = std::sqrt(accel_x * accel_x + accel_y * accel_y
                                       + accel_z * accel_z);
    return std::abs(v_odo) < cfg_.v_still
           && std::abs(gyro_z) < cfg_.gyro_still
           && std::abs(accel_mag - cfg_.gravity) < cfg_.accel_still;
}

Pose DeadReckoning::update(double dt, double erpm_left, double erpm_right,
                           double gyro_z, double accel_x, double accel_y,
                           double accel_z) {
    const double v_left = model_.wheel_speed_from_erpm(erpm_left);
    const double v_right = model_.wheel_speed_from_erpm(erpm_right);
    const Twist twist = model_.twist_from_wheel_speeds(v_left, v_right);

    const bool stationary =
        detect_stationary(twist.v, gyro_z, accel_x, accel_y, accel_z);
    last_stationary_ = stationary;

    // While still, the gyro reading IS the bias -- accumulate a running mean.
    if (cfg_.calibrate_bias && stationary) {
        bias_sum_ += gyro_z;
        ++bias_count_;
        gyro_bias_est_ = bias_sum_ / static_cast<double>(bias_count_);
    }

    // Heading: gyro-integrated with bias removed (the only honest source here),
    // or track-difference for the deliberately-naive baseline.
    if (cfg_.heading_source == HeadingSource::gyro) {
        const double corrected = gyro_z - (cfg_.calibrate_bias ? gyro_bias_est_ : 0.0);
        pose_.heading = wrap_angle(pose_.heading + corrected * dt);
    } else {
        pose_.heading = wrap_angle(pose_.heading + twist.omega * dt);
    }

    // ZUPT: hold position while provably still so drift cannot accumulate.
    const double v_eff = (stationary && cfg_.zupt) ? 0.0 : twist.v;
    pose_.x += v_eff * std::cos(pose_.heading) * dt;
    pose_.y += v_eff * std::sin(pose_.heading) * dt;

    const double step = std::abs(v_eff * dt);
    total_dist_ += step;
    dist_since_crumb_ += step;
    if (dist_since_crumb_ >= cfg_.breadcrumb_spacing) {
        dist_since_crumb_ = 0.0;
        breadcrumbs_.push_back(
            Breadcrumb{pose_.x, pose_.y, pose_.heading, total_dist_});
    }
    return pose_;
}

std::vector<Point> DeadReckoning::trail_xy() const {
    std::vector<Point> trail;
    trail.reserve(breadcrumbs_.size());
    for (const Breadcrumb& b : breadcrumbs_) {
        trail.push_back(Point{b.x, b.y});
    }
    return trail;
}

}  // namespace backtrack
