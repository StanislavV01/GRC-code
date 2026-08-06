#include <cmath>

#include "backtrack/estimator.hpp"
#include "backtrack/kinematics.hpp"
#include "test_harness.hpp"

using backtrack::DeadReckoning;
using backtrack::DiffDriveModel;
using backtrack::EstimatorConfig;
using backtrack::HeadingSource;

namespace {
double erpm_of(const DiffDriveModel& m, double v) {
    return m.erpm_from_wheel_speed(v);
}
constexpr double kG = 9.81;
}  // namespace

TEST(clean_straight_line) {
    const DiffDriveModel m;
    DeadReckoning est(m);  // gyro heading, calib + zupt default on
    const double dt = 0.02;
    const double e = erpm_of(m, 1.0);
    for (int i = 0; i < 500; ++i) est.update(dt, e, e, 0.0, 0.0, 0.0, kG);
    CHECK_NEAR(est.pose().x, 10.0, 0.1);
    CHECK_NEAR(est.pose().y, 0.0, 0.01);
}

TEST(gyro_heading_integrates) {
    const DiffDriveModel m;
    EstimatorConfig cfg;
    cfg.calibrate_bias = false;  // feed a clean gyro, no bias removal
    DeadReckoning est(m, cfg);
    const double dt = 0.02;
    const double e = erpm_of(m, 1.0);
    for (int i = 0; i < 500; ++i) est.update(dt, e, e, 0.2, 0.0, 0.0, kG);
    const double expected = std::atan2(std::sin(2.0), std::cos(2.0));
    CHECK_NEAR(est.pose().heading, expected, 1e-3);
}

TEST(bias_calibration_recovers_gyro_bias) {
    const DiffDriveModel m;
    DeadReckoning est(m);  // calibration on
    const double dt = 0.02;
    const double e_still = erpm_of(m, 0.0);
    const double bias = 0.004;
    // Sit still: gyro reads only the bias, accel reads g, odometry ~ 0.
    for (int i = 0; i < 500; ++i)
        est.update(dt, e_still, e_still, bias, 0.0, 0.0, kG);
    CHECK_NEAR(est.gyro_bias_estimate(), bias, 1e-6);
    CHECK_TRUE(est.last_stationary());
}

TEST(zupt_holds_position_while_still) {
    const DiffDriveModel m;
    DeadReckoning est(m);
    const double dt = 0.02;
    const double e_still = erpm_of(m, 0.0);
    // Even with a biased gyro, a stationary vehicle must not wander.
    for (int i = 0; i < 1000; ++i)
        est.update(dt, e_still, e_still, 0.004, 0.0, 0.0, kG);
    CHECK_NEAR(est.pose().x, 0.0, 1e-6);
    CHECK_NEAR(est.pose().y, 0.0, 1e-6);
}

TEST(calibration_beats_no_calibration_under_bias) {
    const DiffDriveModel m;
    EstimatorConfig with_cal;          // calibrate_bias = true
    EstimatorConfig without_cal;
    without_cal.calibrate_bias = false;
    DeadReckoning cal(m, with_cal);
    DeadReckoning raw(m, without_cal);
    const double dt = 0.02;
    const double e_still = erpm_of(m, 0.0);
    const double e_move = erpm_of(m, 1.0);
    const double bias = 0.01;
    // Calibration window: both sit still (only 'cal' learns the bias).
    for (int i = 0; i < 250; ++i) {
        cal.update(dt, e_still, e_still, bias, 0.0, 0.0, kG);
        raw.update(dt, e_still, e_still, bias, 0.0, 0.0, kG);
    }
    // Then drive straight for 20 s; the biased raw heading curls away.
    for (int i = 0; i < 1000; ++i) {
        cal.update(dt, e_move, e_move, bias, 0.0, 0.0, kG);
        raw.update(dt, e_move, e_move, bias, 0.0, 0.0, kG);
    }
    CHECK_LT(std::fabs(cal.pose().heading), std::fabs(raw.pose().heading));
}

TEST(breadcrumbs_spaced_by_distance) {
    const DiffDriveModel m;
    DeadReckoning est(m);
    const double dt = 0.02;
    const double e = erpm_of(m, 1.0);
    for (int i = 0; i < 500; ++i) est.update(dt, e, e, 0.0, 0.0, 0.0, kG);
    CHECK_GE(est.breadcrumbs().size(), 19);
    CHECK_LE(est.breadcrumbs().size(), 22);
    const auto& b0 = est.breadcrumbs()[1];
    const auto& b1 = est.breadcrumbs()[2];
    const double gap = std::hypot(b1.x - b0.x, b1.y - b0.y);
    CHECK_NEAR(gap, 0.5, 0.05);
}

TEST(odometry_heading_drifts_more_with_slip) {
    const DiffDriveModel m;
    EstimatorConfig gyro_cfg;  // heading from gyro
    EstimatorConfig odo_cfg;
    odo_cfg.heading_source = HeadingSource::odometry;
    DeadReckoning gyro(m, gyro_cfg);
    DeadReckoning odo(m, odo_cfg);
    const double dt = 0.02;
    const double el = erpm_of(m, 1.0);
    const double er = erpm_of(m, 1.4);
    const double odo_omega = (1.4 - 1.0) / m.track_width();
    const double true_omega = odo_omega * 0.85;  // 15% track slip
    // Keep the run short enough that both headings stay within (-pi, pi], so
    // the comparison is not confounded by angle wrapping.
    for (int i = 0; i < 100; ++i) {
        gyro.update(dt, el, er, true_omega, 0.0, 0.0, kG);
        odo.update(dt, el, er, true_omega, 0.0, 0.0, kG);
    }
    CHECK_GT(std::fabs(odo.pose().heading), std::fabs(gyro.pose().heading));
}
