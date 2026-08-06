#ifndef BACKTRACK_TYPES_HPP
#define BACKTRACK_TYPES_HPP

// Shared value types. Plain structs with independently-varying members (C.2).

namespace backtrack {

struct Pose {
    double x{0.0};
    double y{0.0};
    double heading{0.0};
};

struct Point {
    double x{0.0};
    double y{0.0};
};

struct Twist {
    double v{0.0};      // linear speed [m/s]
    double omega{0.0};  // yaw rate [rad/s]
};

struct WheelSpeeds {
    double left{0.0};   // [m/s]
    double right{0.0};  // [m/s]
};

}  // namespace backtrack

#endif  // BACKTRACK_TYPES_HPP
