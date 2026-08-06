#include <cmath>

#include "backtrack/geometry.hpp"
#include "backtrack/simulation.hpp"
#include "test_harness.hpp"

using backtrack::angle_diff;
using backtrack::HeadingSource;
using backtrack::Mode;
using backtrack::Record;
using backtrack::run_simulation;
using backtrack::SimConfig;
using backtrack::SimResult;

namespace {
SimResult gyro_result() {
    SimConfig cfg;  // gyro heading, calibration + ZUPT on by default
    cfg.seed = 7;
    return run_simulation(cfg);
}
}  // namespace

TEST(records_full_session) {
    const SimResult r = gyro_result();
    bool has_calib = false, has_recording = false, has_backtrack = false;
    for (const Record& rec : r.records) {
        if (rec.mode == Mode::calibrating) has_calib = true;
        if (rec.mode == Mode::recording) has_recording = true;
        if (rec.mode == Mode::failsafe_backtrack) has_backtrack = true;
    }
    CHECK_TRUE(has_calib);
    CHECK_TRUE(has_recording);
    CHECK_TRUE(has_backtrack);
    CHECK_TRUE(r.records.back().mode == Mode::arrived);
}

TEST(link_ok_until_backtrack) {
    const SimResult r = gyro_result();
    for (const Record& rec : r.records) {
        const bool autonomous = rec.mode == Mode::failsafe_backtrack
                                || rec.mode == Mode::arrived;
        CHECK_TRUE(rec.link_ok == (autonomous ? 0 : 1));
    }
}

TEST(gyro_bias_is_calibrated) {
    // The learned bias should land near the true bias after the still window.
    const SimResult r = gyro_result();
    CHECK_NEAR(r.summary.final_gyro_bias_est, r.summary.true_gyro_bias, 2e-3);
}

TEST(breadcrumb_trail_recorded) {
    const SimResult r = gyro_result();
    CHECK_GT(r.breadcrumbs.size(), 100);
}

TEST(return_path_reverses_the_trail) {
    const SimResult r = gyro_result();
    const auto& start_of_return = r.return_path.front();
    const auto& end_of_return = r.return_path.back();
    CHECK_NEAR(end_of_return.x, 0.0, 2.0);
    CHECK_NEAR(end_of_return.y, 0.0, 2.0);
    CHECK_NEAR(start_of_return.x, r.summary.est_outbound_end.x, 1.0);
    CHECK_NEAR(start_of_return.y, r.summary.est_outbound_end.y, 1.0);
}

TEST(backtrack_completes) {
    const SimResult r = gyro_result();
    CHECK_TRUE(r.summary.backtrack_completed);
}

TEST(returns_home_within_acceptance) {
    // TZ acceptance: <= 15 m final position error over a ~500 m run (~3%).
    // Compass-free, so the margin is tighter than the magnetometer version.
    const SimResult r = gyro_result();
    CHECK_LE(r.summary.final_return_error_m, 15.0);
}

TEST(calibrated_gyro_beats_uncalibrated) {
    const SimResult cal = gyro_result();
    SimConfig raw_cfg;
    raw_cfg.seed = 7;
    raw_cfg.estimator.calibrate_bias = false;  // raw biased gyro
    const SimResult raw = run_simulation(raw_cfg);
    CHECK_LT(cal.summary.final_return_error_m, raw.summary.final_return_error_m);
}

TEST(gyro_beats_odometry_only) {
    const SimResult gyro = gyro_result();
    SimConfig odo_cfg;
    odo_cfg.seed = 7;
    odo_cfg.estimator.heading_source = HeadingSource::odometry;
    const SimResult odo = run_simulation(odo_cfg);
    CHECK_LT(gyro.summary.final_return_error_m, odo.summary.final_return_error_m);
}

namespace {
// Mean cos of the angle between ground-truth motion and hull heading during
// backtrack: +1 = driving nose-first, -1 = backing up (hull orientation kept).
double mean_motion_vs_heading(const SimResult& r) {
    double sum = 0.0;
    int n = 0;
    for (std::size_t i = 1; i < r.records.size(); ++i) {
        const Record& a = r.records[i - 1];
        const Record& b = r.records[i];
        if (b.mode != backtrack::Mode::failsafe_backtrack) continue;
        const double dx = b.gt.x - a.gt.x;
        const double dy = b.gt.y - a.gt.y;
        const double d = std::hypot(dx, dy);
        if (d < 1e-6) continue;
        sum += (dx * std::cos(b.gt.heading) + dy * std::sin(b.gt.heading)) / d;
        ++n;
    }
    return n ? sum / n : 0.0;
}
}  // namespace

TEST(reverse_mode_returns_home) {
    SimConfig cfg;
    cfg.seed = 7;
    cfg.pursuit.reverse = true;
    const SimResult r = run_simulation(cfg);
    CHECK_TRUE(r.summary.backtrack_completed);
    CHECK_LE(r.summary.final_return_error_m, 15.0);
}

TEST(reverse_mode_drives_backward_hull_kept) {
    // Reverse-only: the hull keeps its outbound orientation and backs up, so
    // motion is anti-parallel to heading. Forward mode drives nose-first.
    SimConfig rev_cfg;
    rev_cfg.seed = 7;
    rev_cfg.pursuit.reverse = true;
    SimConfig fwd_cfg;
    fwd_cfg.seed = 7;
    fwd_cfg.pursuit.reverse = false;
    CHECK_LT(mean_motion_vs_heading(run_simulation(rev_cfg)), -0.8);
    CHECK_GT(mean_motion_vs_heading(run_simulation(fwd_cfg)), 0.8);
}
