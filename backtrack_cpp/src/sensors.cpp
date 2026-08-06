#include "backtrack/sensors.hpp"

#include <algorithm>
#include <cmath>

#include "backtrack/geometry.hpp"

namespace backtrack {

SensorModel::SensorModel(const DiffDriveModel& model, const SensorParams& params,
                         unsigned long seed)
    : model_(model), p_(params), rng_(seed) {}

double SensorModel::motor_current(double v_left_cmd, double v_right_cmd,
                                  double omega_cmd) {
    const double load = p_.current_idle
        + p_.current_per_speed * (std::abs(v_left_cmd) + std::abs(v_right_cmd))
        + p_.current_per_turn * std::abs(omega_cmd);
    return std::max(0.0, load + rng_.gauss(0.0, p_.current_noise));
}

double SensorModel::measured_erpm(double v_wheel_cmd, double dt) {
    const double true_erpm = model_.erpm_from_wheel_speed(v_wheel_cmd);
    if (p_.odo_counts_per_motor_rev > 0.0 && dt > 0.0) {
        // Counts the speed sensor accrues this tick, rounded to whole counts.
        const double mech_rev = true_erpm / model_.pole_pairs() / 60.0 * dt;
        const double counts = std::round(mech_rev * p_.odo_counts_per_motor_rev);
        const double meas_rev = counts / p_.odo_counts_per_motor_rev;
        const double quantised = meas_rev / dt * 60.0 * model_.pole_pairs();
        return quantised + rng_.gauss(0.0, p_.erpm_noise);
    }
    return true_erpm + rng_.gauss(0.0, p_.erpm_noise);
}

SensorSample SensorModel::sample(double dt, double v_left_cmd, double v_right_cmd,
                                 double omega_cmd, double true_omega,
                                 double true_forward_accel,
                                 double true_lateral_accel) {
    SensorSample s;
    s.erpm_left = measured_erpm(v_left_cmd, dt);
    s.erpm_right = measured_erpm(v_right_cmd, dt);

    s.motor_current = motor_current(v_left_cmd, v_right_cmd, omega_cmd);
    s.current_load = clamp(s.motor_current / p_.current_max, 0.0, 1.0);

    s.gyro_z = true_omega + p_.gyro_bias + rng_.gauss(0.0, p_.gyro_noise);

    // Level-ground specific force in the body frame: longitudinal + lateral
    // (centripetal) accelerations, with gravity on the vertical axis.
    s.accel_x = true_forward_accel + p_.accel_bias_x
                + rng_.gauss(0.0, p_.accel_noise);
    s.accel_y = true_lateral_accel + p_.accel_bias_y
                + rng_.gauss(0.0, p_.accel_noise);
    s.accel_z = p_.gravity + p_.accel_bias_z + rng_.gauss(0.0, p_.accel_noise);
    return s;
}

}  // namespace backtrack
