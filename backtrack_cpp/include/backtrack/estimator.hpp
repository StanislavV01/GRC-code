#ifndef BACKTRACK_ESTIMATOR_HPP
#define BACKTRACK_ESTIMATOR_HPP

// GNSS-denied, compass-free dead-reckoning estimator (TZ FR-3 + FR-2).
//
// Heading comes ONLY from the gyroscope -- there is no compass to bound it.
// A gyro has a constant bias, so an uncorrected heading drifts without limit.
// Two inertial techniques keep that drift usable:
//
//   * Gyro-bias calibration: while the vehicle is provably still, the mean of
//     the gyro reading IS the bias. We accumulate it and subtract it. The demo
//     prepends a stationary window so the bias is learned before driving, and
//     every later stop refines it.
//   * ZUPT (zero-velocity update): when track odometry, gyro, and accelerometer
//     all say "still", force velocity to zero so position drift stops growing
//     during halts.
//
// The accelerometer's job here is stationarity confirmation (specific force ~ g
// with no rotation). Heading (yaw) is unobservable from the accelerometer --
// gravity is vertical -- so it cannot correct course; only the gyro integrates
// heading. Tilt/slope projection on non-level ground is future work.
//
// HeadingSource lets the demo contrast the right source (gyro) against the
// naive one (track-speed difference), which lies badly under track slip.

#include <vector>

#include "backtrack/kinematics.hpp"
#include "backtrack/types.hpp"

namespace backtrack {

enum class HeadingSource { gyro, odometry };

struct EstimatorConfig {
    HeadingSource heading_source = HeadingSource::gyro;
    bool calibrate_bias = true;   // learn & subtract gyro bias when still
    bool zupt = true;             // zero velocity when still
    double breadcrumb_spacing = 0.5;  // [m]
    double gravity = 9.81;            // [m/s^2]
    double v_still = 0.05;            // odometry speed below this = candidate still
    double gyro_still = 0.03;         // |gyro| below this = no rotation [rad/s]
    double accel_still = 0.6;         // |accel|-g below this = no linear accel
};

struct Breadcrumb {
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double cumulative_dist{0.0};
};

class DeadReckoning {
public:
    DeadReckoning(const DiffDriveModel& model, EstimatorConfig config = {},
                  Pose init_pose = Pose{});

    // Advance the estimate by one tick from an IMU + odometry sample.
    // Returns the updated pose.
    Pose update(double dt, double erpm_left, double erpm_right, double gyro_z,
                double accel_x, double accel_y, double accel_z);

    Pose pose() const { return pose_; }
    double total_distance() const { return total_dist_; }
    double gyro_bias_estimate() const { return gyro_bias_est_; }
    bool last_stationary() const { return last_stationary_; }
    const std::vector<Breadcrumb>& breadcrumbs() const { return breadcrumbs_; }

    // Breadcrumb trail as (x, y) points, oldest first.
    std::vector<Point> trail_xy() const;

private:
    bool detect_stationary(double v_odo, double gyro_z, double accel_x,
                           double accel_y, double accel_z) const;

    const DiffDriveModel& model_;
    EstimatorConfig cfg_;

    Pose pose_;
    double total_dist_{0.0};
    double dist_since_crumb_{0.0};

    double gyro_bias_est_{0.0};
    double bias_sum_{0.0};
    long bias_count_{0};
    bool last_stationary_{false};

    std::vector<Breadcrumb> breadcrumbs_;
};

}  // namespace backtrack

#endif  // BACKTRACK_ESTIMATOR_HPP
