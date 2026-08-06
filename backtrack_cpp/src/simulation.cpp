#include "backtrack/simulation.hpp"

#include <algorithm>
#include <cmath>

#include "backtrack/geometry.hpp"
#include "backtrack/kinematics.hpp"
#include "backtrack/log.hpp"
#include "backtrack/random.hpp"

namespace backtrack {

const char* to_string(Mode mode) {
    switch (mode) {
        case Mode::calibrating: return "CALIBRATING";
        case Mode::recording: return "RECORDING";
        case Mode::failsafe_backtrack: return "FAILSAFE_BACKTRACK";
        case Mode::arrived: return "ARRIVED";
    }
    return "UNKNOWN";
}

SimResult run_simulation(const SimConfig& cfg,
                         const std::vector<Maneuver>& route) {
    const DiffDriveModel model;
    SensorModel sensors(model, cfg.sensor_params, cfg.seed);
    DeadReckoning estimator(model, cfg.estimator);
    GaussianSource slip_rng(cfg.seed + 1000UL);

    Pose gt{};                 // ground-truth pose
    double prev_true_v = 0.0;  // for body-frame forward acceleration
    std::vector<Record> records;
    double t = 0.0;

    // Advance one tick: apply slip, synth IMU+odometry, update estimate, log.
    const auto drive = [&](double v_des, double omega_des, Mode mode,
                           bool link_ok) {
        const WheelSpeeds cmd = model.wheel_speeds_from_twist(v_des, omega_des);

        // Ground truth: tracks spin as commanded but the chassis slips.
        const double jitter = slip_rng.gauss(0.0, cfg.slip_noise);
        const double true_v = v_des * (1.0 - cfg.slip_long + jitter);
        const double true_omega = omega_des * (1.0 - cfg.slip_turn);
        gt = model.step_pose(gt, true_v, true_omega, cfg.dt);

        // True body-frame accelerations the IMU would feel on level ground.
        const double fwd_accel = (true_v - prev_true_v) / cfg.dt;
        const double lat_accel = true_v * true_omega;  // centripetal
        prev_true_v = true_v;

        const SensorSample meas = sensors.sample(
            cfg.dt, cmd.left, cmd.right, omega_des, true_omega, fwd_accel,
            lat_accel);
        const Pose est =
            estimator.update(cfg.dt, meas.erpm_left, meas.erpm_right, meas.gyro_z,
                             meas.accel_x, meas.accel_y, meas.accel_z);

        Record r;
        r.t = t;
        r.mode = mode;
        r.link_ok = link_ok ? 1 : 0;
        r.erpm_left = meas.erpm_left;
        r.erpm_right = meas.erpm_right;
        r.motor_current = meas.motor_current;
        r.gyro_z = meas.gyro_z;
        r.accel_x = meas.accel_x;
        r.accel_y = meas.accel_y;
        r.accel_z = meas.accel_z;
        r.gyro_bias_est = estimator.gyro_bias_estimate();
        r.stationary = estimator.last_stationary() ? 1 : 0;
        r.est = est;
        r.gt = gt;
        records.push_back(r);
        t += cfg.dt;
    };

    // --- Calibration: sit still so the gyro bias can be learned -----------
    LOG_INFO << "CALIBRATING: holding still " << cfg.calibration_seconds
             << " s for gyro-bias estimation";
    const int calib_steps =
        std::max(0, static_cast<int>(std::lround(cfg.calibration_seconds / cfg.dt)));
    for (int i = 0; i < calib_steps; ++i) {
        drive(0.0, 0.0, Mode::calibrating, /*link_ok=*/true);
    }
    LOG_INFO << "calibration done: gyro bias est "
             << estimator.gyro_bias_estimate() << " rad/s (true "
             << cfg.sensor_params.gyro_bias << ")";

    // --- Outbound leg: follow the recorded route, link healthy ------------
    const std::vector<Twist> commands =
        outbound_commands(cfg.dt, cfg.cruise_speed, route);
    LOG_INFO << "RECORDING: outbound " << total_length(route) << " m";
    for (const Twist& c : commands) {
        drive(c.v, c.omega, Mode::recording, /*link_ok=*/true);
    }

    const Pose gt_outbound_end = gt;
    const Pose est_outbound_end = estimator.pose();
    LOG_WARN << "LINK LOST -> FAILSAFE_BACKTRACK; trail "
             << estimator.breadcrumbs().size() << " crumbs, est end ("
             << est_outbound_end.x << ", " << est_outbound_end.y << ")";

    // --- Link loss -> backtrack along the reversed breadcrumb trail -------
    std::vector<Point> return_path = estimator.trail_xy();
    std::reverse(return_path.begin(), return_path.end());
    BacktrackController controller(return_path, cfg.pursuit);

    const int stop_steps = std::max(
        0, static_cast<int>(std::lround(cfg.backtrack_stop_seconds / cfg.dt)));
    const double bt_start_dist = estimator.total_distance();
    double next_stop_at = cfg.backtrack_stop_interval_m;

    int steps = 0;
    while (steps < cfg.max_backtrack_steps) {
        const Command cmd = controller.compute(estimator.pose());
        if (cmd.done) break;

        // Periodic halt to re-calibrate the gyro bias via ZUPT. Distance only
        // advances while moving (ZUPT zeroes it during the halt), so the
        // interval bookkeeping stays correct.
        if (cfg.backtrack_stop_interval_m > 0.0
            && estimator.total_distance() - bt_start_dist >= next_stop_at) {
            for (int k = 0; k < stop_steps; ++k) {
                drive(0.0, 0.0, Mode::failsafe_backtrack, /*link_ok=*/false);
            }
            LOG_DEBUG << "BACKTRACK_PARTIAL halt at " << next_stop_at
                      << " m; bias re-est " << estimator.gyro_bias_estimate();
            next_stop_at += cfg.backtrack_stop_interval_m;
        }

        drive(cmd.v, cmd.omega, Mode::failsafe_backtrack, /*link_ok=*/false);
        ++steps;
    }
    if (!records.empty()) records.back().mode = Mode::arrived;

    const double outbound_dist = total_length(route);
    const double final_error = distance(gt.x, gt.y, 0.0, 0.0);

    // True travelled distance (ground truth path length) and mean backtrack ERPM.
    double gt_travelled = 0.0;
    double bt_erpm_sum = 0.0;
    long bt_erpm_n = 0;
    for (std::size_t i = 1; i < records.size(); ++i) {
        gt_travelled += distance(records[i - 1].gt.x, records[i - 1].gt.y,
                                 records[i].gt.x, records[i].gt.y);
        if (records[i].mode == Mode::failsafe_backtrack) {
            bt_erpm_sum +=
                (std::abs(records[i].erpm_left) + std::abs(records[i].erpm_right))
                / 2.0;
            ++bt_erpm_n;
        }
    }

    SimSummary summary;
    summary.heading_source = cfg.estimator.heading_source;
    summary.calibrate_bias = cfg.estimator.calibrate_bias;
    summary.zupt = cfg.estimator.zupt;
    summary.true_gyro_bias = cfg.sensor_params.gyro_bias;
    summary.final_gyro_bias_est = estimator.gyro_bias_estimate();
    summary.outbound_distance_m = outbound_dist;
    summary.ticks = records.size();
    summary.breadcrumbs = estimator.breadcrumbs().size();
    summary.gt_outbound_end = gt_outbound_end;
    summary.est_outbound_end = est_outbound_end;
    summary.est_outbound_error_m =
        distance(gt_outbound_end.x, gt_outbound_end.y, est_outbound_end.x,
                 est_outbound_end.y);
    summary.est_heading_error_deg =
        std::abs(angle_diff(est_outbound_end.heading, gt_outbound_end.heading))
        * 180.0 / M_PI;
    summary.est_distance_error_m =
        std::abs(estimator.total_distance() - gt_travelled);
    summary.mean_backtrack_erpm = bt_erpm_n ? bt_erpm_sum / bt_erpm_n : 0.0;
    summary.gt_final = gt;
    summary.final_return_error_m = final_error;
    summary.final_error_pct = 100.0 * final_error / outbound_dist;
    summary.backtrack_completed = controller.done();

    if (controller.done()) {
        LOG_INFO << "ARRIVED: final return error " << summary.final_return_error_m
                 << " m (" << summary.final_error_pct << "%)";
    } else {
        LOG_ERROR << "backtrack DID NOT COMPLETE within " << cfg.max_backtrack_steps
                  << " steps; stopped " << summary.final_return_error_m
                  << " m from home";
    }
    if (summary.mean_backtrack_erpm > 0.0 && summary.mean_backtrack_erpm < 1000.0) {
        LOG_WARN << "mean backtrack ERPM " << summary.mean_backtrack_erpm
                 << " below sensorless floor (~1000-2000): sensored FOC required";
    }

    SimResult result;
    result.records = std::move(records);
    result.breadcrumbs = estimator.breadcrumbs();
    result.return_path = std::move(return_path);
    result.summary = summary;
    return result;
}

}  // namespace backtrack
