#ifndef BACKTRACK_SENSORS_HPP
#define BACKTRACK_SENSORS_HPP

// Synthetic on-board sensors for a tracked (skid-steer) UGV.
//
// The platform has NO compass and NO GNSS (GNSS jammed; the magnetometer is
// useless next to the 84V FSESC power stage). Only the inertial sensors and
// track odometry remain:
//   * VESC / CAN bus : per-track ERPM + motor current
//   * Gyroscope      : yaw rate (true rotation + constant bias + noise)
//   * Accelerometer  : 3-axis specific force (body frame), used for ZUPT
//                      (stationary detection) and gyro-bias calibration.
//
// On level ground the accelerometer reads ~ (a_forward, a_lateral, g). When the
// vehicle is truly still it reads ~ (0, 0, g): that, together with near-zero
// gyro and near-zero track speed, is the signature ZUPT keys on. (Tilt/slope
// projection on non-level ground is a future extension -- see README.)

#include "backtrack/kinematics.hpp"
#include "backtrack/random.hpp"

namespace backtrack {

// Speed-sensor resolution as counts per *motor mechanical* revolution.
//   * Hall only (QS138, single hall set): 6 commutation steps * 5 pole pairs.
//   * AS5047P 14-bit absolute encoder (FSESC 75450 supported): 2^14.
// At the ~1.5 m/s backtrack crawl the motor barely turns, so this resolution is
// the dominant odometry-distance error. 0 = ideal/continuous (no quantization).
constexpr double kHallCountsPerRev = 30.0;
constexpr double kEncoderCountsPerRev = 16384.0;

struct SensorParams {
    double odo_counts_per_motor_rev = kEncoderCountsPerRev;  // speed-sensor res
    double erpm_noise = 4.0;          // std dev on ERPM readings
    double gyro_bias = 0.004;         // cold-start gyro bias, removed by boot cal
    double gyro_noise = 0.003;        // in-run gyro noise [rad/s] (good MEMS).
                                      // Dominant accuracy knob with no compass:
                                      // 0.010=cheap ~30m worst, 0.003=good ~10m.
    double gravity = 9.81;            // g magnitude [m/s^2]
    double accel_noise = 0.15;        // std dev on each accel axis [m/s^2]
    double accel_bias_x = 0.05;       // constant accel bias, body x [m/s^2]
    double accel_bias_y = 0.03;       // constant accel bias, body y [m/s^2]
    double accel_bias_z = 0.04;       // constant accel bias, body z [m/s^2]
    double current_idle = 2.0;        // baseline motor current [A]
    double current_per_speed = 3.0;   // A per (|v_l|+|v_r|)
    double current_per_turn = 12.0;   // A per |omega|
    double current_noise = 0.6;       // std dev on current [A]
    double current_max = 40.0;        // full-scale current for load norm [A]
};

struct SensorSample {
    double erpm_left{0.0};
    double erpm_right{0.0};
    double motor_current{0.0};
    double gyro_z{0.0};      // measured yaw rate [rad/s]
    double accel_x{0.0};     // body forward specific force [m/s^2]
    double accel_y{0.0};     // body lateral specific force [m/s^2]
    double accel_z{0.0};     // body vertical specific force [m/s^2] (~g)
    double current_load{0.0};  // 0..1
};

class SensorModel {
public:
    SensorModel(const DiffDriveModel& model, const SensorParams& params,
                unsigned long seed);

    double motor_current(double v_left_cmd, double v_right_cmd, double omega_cmd);

    // VESC ERPM as actually measured: the true ERPM quantised to the speed
    // sensor's count resolution over one tick of length dt.
    double measured_erpm(double v_wheel_cmd, double dt);

    // Produce one synchronized sensor record. The caller supplies the tick dt
    // (for ERPM quantisation) and the true body-frame forward and lateral
    // accelerations (from the ground-truth motion).
    SensorSample sample(double dt, double v_left_cmd, double v_right_cmd,
                        double omega_cmd, double true_omega,
                        double true_forward_accel, double true_lateral_accel);

private:
    const DiffDriveModel& model_;
    SensorParams p_;
    GaussianSource rng_;
};

}  // namespace backtrack

#endif  // BACKTRACK_SENSORS_HPP
