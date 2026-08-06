#ifndef BACKTRACK_SIMULATION_HPP
#define BACKTRACK_SIMULATION_HPP

// End-to-end simulation: stationary calibration -> outbound drive -> link loss
// -> autonomous backtrack, for a compass-free tracked UGV.
//
// The loop maintains a hidden *ground truth* pose (with skid slip) and feeds
// only synthetic IMU + odometry measurements to the dead-reckoning estimator.
// The run opens with the vehicle standing still so the estimator can calibrate
// its gyro bias (no compass to lean on). When all comms drop, the controller
// steers the true vehicle home using only the estimated pose and the trail.

#include <vector>

#include "backtrack/controller.hpp"
#include "backtrack/estimator.hpp"
#include "backtrack/route.hpp"
#include "backtrack/sensors.hpp"
#include "backtrack/types.hpp"

namespace backtrack {

enum class Mode { calibrating, recording, failsafe_backtrack, arrived };

const char* to_string(Mode mode);

struct SimConfig {
    double dt = 0.02;
    double cruise_speed = 2.0;
    double slip_long = 0.02;          // fractional under-travel from track slip
    double slip_turn = 0.10;          // fractional under-rotation in skid turns
    double slip_noise = 0.01;         // per-tick random slip jitter
    double calibration_seconds = 15.0;  // boot gyro-bias window (cf. ArduPilot)
    // BACKTRACK_PARTIAL (FR-4): halt every N metres of return to re-run ZUPT
    // bias calibration (and, on the real vehicle, re-check comms). Bounds the
    // compass-free heading drift that would otherwise grow over the whole route.
    double backtrack_stop_interval_m = 75.0;  // 0 disables periodic stops
    double backtrack_stop_seconds = 2.5;
    unsigned long seed = 7;
    int max_backtrack_steps = 40000;
    EstimatorConfig estimator{};
    SensorParams sensor_params{};
    PursuitConfig pursuit{};
};

// One logged tick of the blackbox time-series.
struct Record {
    double t{0.0};
    Mode mode{Mode::recording};
    int link_ok{1};
    double erpm_left{0.0};
    double erpm_right{0.0};
    double motor_current{0.0};
    double gyro_z{0.0};
    double accel_x{0.0};
    double accel_y{0.0};
    double accel_z{0.0};
    double gyro_bias_est{0.0};
    int stationary{0};
    Pose est{};
    Pose gt{};
};

struct SimSummary {
    HeadingSource heading_source{HeadingSource::gyro};
    bool calibrate_bias{true};
    bool zupt{true};
    double true_gyro_bias{0.0};
    double final_gyro_bias_est{0.0};
    double outbound_distance_m{0.0};
    std::size_t ticks{0};
    std::size_t breadcrumbs{0};
    Pose gt_outbound_end{};
    Pose est_outbound_end{};
    double est_outbound_error_m{0.0};
    double est_heading_error_deg{0.0};
    double est_distance_error_m{0.0};   // |estimated travelled - true travelled|
    double mean_backtrack_erpm{0.0};    // avg |ERPM| while backtracking
    Pose gt_final{};
    double final_return_error_m{0.0};
    double final_error_pct{0.0};
    bool backtrack_completed{false};
};

struct SimResult {
    std::vector<Record> records;
    std::vector<Breadcrumb> breadcrumbs;
    std::vector<Point> return_path;
    SimSummary summary;
};

SimResult run_simulation(const SimConfig& config = SimConfig{},
                         const std::vector<Maneuver>& route = default_route());

}  // namespace backtrack

#endif  // BACKTRACK_SIMULATION_HPP
