#ifndef BACKTRACK_KINEMATICS_HPP
#define BACKTRACK_KINEMATICS_HPP

// Differential-drive (skid-steer) kinematics and VESC ERPM conversions for the
// real platform: tracked UGV, one QS138 70H V3 PMSM mid-drive per side driven
// by a FlipSky FSESC 75450.
//
// Verified hardware numbers baked in as defaults:
//   * QS138 70H V3: 5 pole pairs, internal reduction 1 : 2.35, Hall-sensored.
//   * Wheel speed is observed on the CAN bus as motor ERPM
//     (ERPM = mechanical_RPM * pole_pairs).
//
// Still TBD on the platform (measure, then set): the external reduction between
// the motor output and the track sprocket, and the sprocket effective radius.
// Defaults below (external_gear = 1.0, wheel_radius = 0.12 m) are the
// conservative placeholders used for analysis.

#include "backtrack/types.hpp"

namespace backtrack {

class DiffDriveModel {
public:
    DiffDriveModel(double track_width = 0.5, double wheel_radius = 0.12,
                   double pole_pairs = 5.0, double internal_gear = 2.35,
                   double external_gear = 1.0);

    // twist <-> wheel speeds
    WheelSpeeds wheel_speeds_from_twist(double v, double omega) const;
    Twist twist_from_wheel_speeds(double v_left, double v_right) const;

    // wheel (track-sprocket) speed <-> VESC ERPM
    double erpm_from_wheel_speed(double v_wheel) const;
    double wheel_speed_from_erpm(double erpm) const;

    // exact arc integration of a unicycle pose
    Pose step_pose(const Pose& pose, double v, double omega, double dt) const;

    double track_width() const { return track_width_; }
    double wheel_radius() const { return wheel_radius_; }
    double pole_pairs() const { return pole_pairs_; }
    double total_gear() const { return total_gear_; }  // motor revs per wheel rev

private:
    double track_width_;
    double wheel_radius_;
    double pole_pairs_;
    double total_gear_;     // internal_gear * external_gear
    double circumference_;  // 2*pi*wheel_radius
};

}  // namespace backtrack

#endif  // BACKTRACK_KINEMATICS_HPP
