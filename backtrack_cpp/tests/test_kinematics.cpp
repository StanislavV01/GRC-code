#include <cmath>

#include "backtrack/geometry.hpp"
#include "backtrack/kinematics.hpp"
#include "test_harness.hpp"

using backtrack::DiffDriveModel;
using backtrack::Pose;
using backtrack::Twist;
using backtrack::WheelSpeeds;
using backtrack::wrap_angle;

TEST(wrap_into_range) {
    CHECK_NEAR(std::cos(wrap_angle(3 * M_PI)), -1.0, 1e-6);
    CHECK_NEAR(std::cos(wrap_angle(-3 * M_PI)), -1.0, 1e-6);
    CHECK_LE(std::fabs(wrap_angle(3 * M_PI)), M_PI + 1e-9);
    CHECK_NEAR(wrap_angle(0.5), 0.5, 1e-6);
}

TEST(twist_wheel_roundtrip) {
    const DiffDriveModel m;
    const WheelSpeeds w = m.wheel_speeds_from_twist(1.7, 0.4);
    const Twist t = m.twist_from_wheel_speeds(w.left, w.right);
    CHECK_NEAR(t.v, 1.7, 1e-9);
    CHECK_NEAR(t.omega, 0.4, 1e-9);
}

TEST(erpm_roundtrip) {
    const DiffDriveModel m;
    for (double speed : {0.0, 0.5, 2.0, -1.3}) {
        const double erpm = m.erpm_from_wheel_speed(speed);
        CHECK_NEAR(m.wheel_speed_from_erpm(erpm), speed, 1e-9);
    }
}

TEST(straight_line) {
    const DiffDriveModel m;
    Pose pose;
    for (int i = 0; i < 100; ++i) pose = m.step_pose(pose, 1.0, 0.0, 0.01);
    CHECK_NEAR(pose.x, 1.0, 1e-3);
    CHECK_NEAR(pose.y, 0.0, 1e-6);
    CHECK_NEAR(pose.heading, 0.0, 1e-6);
}

TEST(pure_rotation_keeps_position) {
    const DiffDriveModel m;
    Pose pose{2.0, -1.0, 0.0};
    pose = m.step_pose(pose, 0.0, 1.0, 0.5);
    CHECK_NEAR(pose.x, 2.0, 1e-9);
    CHECK_NEAR(pose.y, -1.0, 1e-9);
    CHECK_NEAR(pose.heading, 0.5, 1e-9);
}

TEST(full_circle_returns_to_start) {
    const DiffDriveModel m;
    Pose pose;
    const double v = 1.0, omega = 1.0;  // radius 1 m
    const int steps = 1000;
    const double dt = (2 * M_PI) / omega / steps;
    for (int i = 0; i < steps; ++i) pose = m.step_pose(pose, v, omega, dt);
    CHECK_NEAR(pose.x, 0.0, 1e-3);
    CHECK_NEAR(pose.y, 0.0, 1e-3);
}
