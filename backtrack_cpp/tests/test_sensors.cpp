#include <cmath>

#include "backtrack/kinematics.hpp"
#include "backtrack/sensors.hpp"
#include "test_harness.hpp"

using backtrack::DiffDriveModel;
using backtrack::SensorModel;
using backtrack::SensorParams;
using backtrack::SensorSample;

TEST(erpm_reflects_wheel_speed) {
    const DiffDriveModel model;
    SensorParams p;
    p.erpm_noise = 0.0;
    p.current_noise = 0.0;
    p.odo_counts_per_motor_rev = 0.0;  // ideal: test the kinematic conversion
    SensorModel sensor(model, p, 1);
    const SensorSample m = sensor.sample(0.02, 1.0, 1.5, 0.0, 0.0, 0.0, 0.0);
    CHECK_NEAR(model.wheel_speed_from_erpm(m.erpm_left), 1.0, 1e-6);
    CHECK_NEAR(model.wheel_speed_from_erpm(m.erpm_right), 1.5, 1e-6);
}

TEST(gyro_tracks_true_omega_plus_bias_on_average) {
    const DiffDriveModel model;
    SensorParams p;
    p.gyro_noise = 0.05;
    p.gyro_bias = 0.01;
    SensorModel sensor(model, p, 2);
    const double true_omega = 0.3;
    double sum = 0.0;
    const int n = 4000;
    for (int i = 0; i < n; ++i)
        sum += sensor.sample(0.02, 1.0, 1.0, true_omega, true_omega, 0.0, 0.0)
                   .gyro_z;
    // The gyro reads the true rate plus its constant bias.
    CHECK_NEAR(sum / n, true_omega + p.gyro_bias, 1e-2);
}

TEST(accel_z_reads_gravity_when_level) {
    const DiffDriveModel model;
    SensorParams p;
    p.accel_noise = 0.0;
    p.accel_bias_z = 0.0;
    SensorModel sensor(model, p, 3);
    const SensorSample m = sensor.sample(0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    CHECK_NEAR(m.accel_z, p.gravity, 1e-9);
    CHECK_NEAR(m.accel_x, p.accel_bias_x, 1e-9);
}

TEST(accel_x_reflects_forward_acceleration) {
    const DiffDriveModel model;
    SensorParams p;
    p.accel_noise = 0.0;
    p.accel_bias_x = 0.0;
    SensorModel sensor(model, p, 4);
    const SensorSample m =
        sensor.sample(0.02, 2.0, 2.0, 0.0, 0.0, /*fwd=*/1.5, 0.0);
    CHECK_NEAR(m.accel_x, 1.5, 1e-9);
}

TEST(hall_quantises_low_speed_erpm_coarser_than_encoder) {
    // At the ~1.5 m/s crawl the motor barely turns; Hall resolution makes the
    // decoded speed jump around, an encoder stays smooth.
    const DiffDriveModel model;
    const double dt = 0.02;
    const double v = 1.5;
    SensorParams hall_p;
    hall_p.erpm_noise = 0.0;
    hall_p.odo_counts_per_motor_rev = backtrack::kHallCountsPerRev;
    SensorParams enc_p;
    enc_p.erpm_noise = 0.0;
    enc_p.odo_counts_per_motor_rev = backtrack::kEncoderCountsPerRev;
    SensorModel hall(model, hall_p, 5);
    SensorModel enc(model, enc_p, 5);

    double hall_err = 0.0, enc_err = 0.0;
    const int n = 2000;
    for (int i = 0; i < n; ++i) {
        hall_err += std::fabs(
            model.wheel_speed_from_erpm(hall.measured_erpm(v, dt)) - v);
        enc_err += std::fabs(
            model.wheel_speed_from_erpm(enc.measured_erpm(v, dt)) - v);
    }
    CHECK_GT(hall_err / n, enc_err / n);
    CHECK_LT(enc_err / n, 0.02);   // encoder: sub-cm/s decoding error
    CHECK_GT(hall_err / n, 0.05);  // hall: coarse at crawl speed
}

TEST(current_increases_with_effort) {
    const DiffDriveModel model;
    SensorParams p;
    p.current_noise = 0.0;
    SensorModel sensor(model, p, 5);
    const double idle = sensor.motor_current(0.0, 0.0, 0.0);
    const double driving = sensor.motor_current(2.0, 2.0, 0.0);
    const double turning = sensor.motor_current(2.0, -2.0, 4.0);
    CHECK_LT(idle, driving);
    CHECK_LT(driving, turning);
}
