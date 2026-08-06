// On-vehicle real-time loop for the Link-Loss Backtrack system (CM4 build).
//
// Hardware edges (all pluggable behind hal/ interfaces):
//   * IMU   : GY-9250 (MPU-9250) on /dev/i2c-1
//   * drive : 2x FSESC 75450 (VESC) on SocketCAN can0, ids left=1 right=2
//   * link  : operator console on RS485 (/dev/serial0), 12-byte frames
//
// State machine (50 Hz):
//   CALIBRATING (hold still, learn gyro bias; refuses to finish until the
//     stationarity detector actually accepted samples)
//     -> IDLE (wait for the operator link)
//     -> MANUAL (relay operator commands to the VESCs, record breadcrumbs)
//     -> FAILSAFE_BACKTRACK on link loss (pure pursuit along the trail,
//        halting every N metres to re-calibrate -- BACKTRACK_PARTIAL)
//     -> ARRIVED (stop, wait for the link to come back)
//   A valid operator frame during backtrack returns control to MANUAL.
//
// Safety invariants:
//   * E-STOP latches: it survives link loss (no autonomous motion while
//     latched) and clears only after 10 consecutive valid drive frames.
//   * Any exception / shutdown path stops the motors (MotorStopGuard).
//   * A stale pose (IMU outage > 1 s) permanently disables backtrack until
//     restart; manual relay stays available.
//   * Non-finite or out-of-range commands never reach the CAN bus.
//
// Usage (all optional, defaults in brackets):
//   run_vehicle [--can can0] [--left-id 1] [--right-id 2]
//               [--uart /dev/serial0] [--baud 115200]
//               [--i2c /dev/i2c-1] [--imu-addr 0x68] [--imu-axes x,y,z]
//               [--track-width M] [--wheel-radius M] [--external-gear R]
//               [--rate 50] [--calib-seconds 15] [--link-timeout 0.5]
//               [--max-speed 2.0] [--max-yaw-rate 1.5] [--max-erpm 8000]
//               [--reverse] [--stop-interval 75] [--stop-seconds 2.5]
//               [--outdir data] [--log-level info] [--log-file PATH]

#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <ctime>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "backtrack/controller.hpp"
#include "backtrack/estimator.hpp"
#include "backtrack/geometry.hpp"
#include "backtrack/hal/link_supervisor.hpp"
#include "backtrack/hal/mpu9250_i2c.hpp"
#include "backtrack/hal/rs485_link.hpp"
#include "backtrack/hal/vesc_socketcan.hpp"
#include "backtrack/kinematics.hpp"
#include "backtrack/log.hpp"

namespace {

using namespace backtrack;

std::atomic<bool> g_stop{false};
void handle_signal(int) { g_stop = true; }

double monotonic_s() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + ts.tv_nsec * 1e-9;
}

void sleep_until_monotonic(double target_s) {
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(target_s);
    ts.tv_nsec = static_cast<long>((target_s - ts.tv_sec) * 1e9);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

// Whatever happens -- clean return, thrown exception, signal-driven shutdown
// -- the tracks get a zero command, retried long enough to survive a
// momentarily full CAN TX queue.
class MotorStopGuard {
public:
    explicit MotorStopGuard(hal::IDrive& drive) : drive_(drive) {}
    ~MotorStopGuard() {
        for (int i = 0; i < 10; ++i) {
            drive_.set_erpm(0.0, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    MotorStopGuard(const MotorStopGuard&) = delete;
    MotorStopGuard& operator=(const MotorStopGuard&) = delete;

private:
    hal::IDrive& drive_;
};

// "--imu-axes -y,x,z" -> body x = -chip_y, body y = +chip_x, body z = +chip_z
hal::MountConfig parse_mount(const std::string& spec) {
    hal::MountConfig m;
    bool used[3] = {false, false, false};
    std::size_t pos = 0;
    for (int i = 0; i < 3; ++i) {
        const std::size_t comma = spec.find(',', pos);
        std::string tok = spec.substr(
            pos, comma == std::string::npos ? std::string::npos : comma - pos);
        if (tok.empty()) throw std::runtime_error("imu-axes: bad spec " + spec);
        double sign = 1.0;
        if (tok[0] == '-') { sign = -1.0; tok = tok.substr(1); }
        else if (tok[0] == '+') { tok = tok.substr(1); }
        if (tok != "x" && tok != "y" && tok != "z") {
            throw std::runtime_error("imu-axes: bad axis " + tok);
        }
        const int axis = tok[0] - 'x';
        if (used[axis]) throw std::runtime_error("imu-axes: duplicate axis " + tok);
        used[axis] = true;
        m.axis[i] = axis;
        m.sign[i] = sign;
        if (comma == std::string::npos && i < 2) {
            throw std::runtime_error("imu-axes: need 3 axes");
        }
        pos = comma + 1;
    }
    return m;
}

struct Args {
    std::string can = "can0";
    int left_id = 1;
    int right_id = 2;
    std::string uart = "/dev/serial0";
    int baud = 115200;
    std::string i2c = "/dev/i2c-1";
    int imu_addr = 0x68;
    std::string imu_axes = "x,y,z";
    double track_width = 0.5;    // TBD: measure on the platform
    double wheel_radius = 0.12;  // TBD: sprocket effective radius
    double external_gear = 1.0;  // TBD: motor -> sprocket reduction
    double rate_hz = 50.0;
    double calib_seconds = 15.0;
    double link_timeout = 0.5;
    double max_speed = 2.0;      // clamp for BOTH manual and backtrack [m/s]
    double max_yaw_rate = 1.5;   // [rad/s]
    double max_erpm = 8000.0;    // hard actuation ceiling on the CAN bus
    bool reverse = false;        // reverse-only backtrack (FR-4)
    double stop_interval_m = 75.0;   // BACKTRACK_PARTIAL
    double stop_seconds = 2.5;
    std::string outdir = "data";
    std::string log_level = "info";
    std::string log_file;
};

void validate(const Args& a) {
    const auto need = [](bool ok, const char* what) {
        if (!ok) throw std::runtime_error(std::string("invalid argument: ") + what);
    };
    need(a.rate_hz >= 10.0 && a.rate_hz <= 500.0, "--rate must be 10..500 Hz");
    need(a.calib_seconds >= 1.0, "--calib-seconds must be >= 1");
    need(a.link_timeout >= 0.1 && a.link_timeout <= 10.0,
         "--link-timeout must be 0.1..10 s");
    need(a.max_speed > 0.0 && a.max_speed <= 10.0, "--max-speed must be 0..10");
    need(a.max_yaw_rate > 0.0 && a.max_yaw_rate <= 10.0,
         "--max-yaw-rate must be 0..10");
    need(a.max_erpm >= 100.0 && a.max_erpm <= 100000.0,
         "--max-erpm must be 100..100000");
    need(a.track_width >= 0.1 && a.track_width <= 3.0,
         "--track-width must be 0.1..3 m");
    need(a.wheel_radius >= 0.02 && a.wheel_radius <= 1.0,
         "--wheel-radius must be 0.02..1 m");
    need(a.external_gear >= 0.1 && a.external_gear <= 100.0,
         "--external-gear must be 0.1..100");
    need(a.left_id >= 0 && a.left_id <= 253, "--left-id must be 0..253");
    need(a.right_id >= 0 && a.right_id <= 253, "--right-id must be 0..253");
    need(a.left_id != a.right_id, "--left-id and --right-id must differ");
    need(a.stop_interval_m >= 0.0, "--stop-interval must be >= 0");
    need(a.stop_seconds >= 0.0, "--stop-seconds must be >= 0");
}

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
            return argv[++i];
        };
        if (arg == "--can") a.can = next();
        else if (arg == "--left-id") a.left_id = std::stoi(next());
        else if (arg == "--right-id") a.right_id = std::stoi(next());
        else if (arg == "--uart") a.uart = next();
        else if (arg == "--baud") a.baud = std::stoi(next());
        else if (arg == "--i2c") a.i2c = next();
        else if (arg == "--imu-addr") a.imu_addr = std::stoi(next(), nullptr, 0);
        else if (arg == "--imu-axes") a.imu_axes = next();
        else if (arg == "--track-width") a.track_width = std::stod(next());
        else if (arg == "--wheel-radius") a.wheel_radius = std::stod(next());
        else if (arg == "--external-gear") a.external_gear = std::stod(next());
        else if (arg == "--rate") a.rate_hz = std::stod(next());
        else if (arg == "--calib-seconds") a.calib_seconds = std::stod(next());
        else if (arg == "--link-timeout") a.link_timeout = std::stod(next());
        else if (arg == "--max-speed") a.max_speed = std::stod(next());
        else if (arg == "--max-yaw-rate") a.max_yaw_rate = std::stod(next());
        else if (arg == "--max-erpm") a.max_erpm = std::stod(next());
        else if (arg == "--reverse") a.reverse = true;
        else if (arg == "--stop-interval") a.stop_interval_m = std::stod(next());
        else if (arg == "--stop-seconds") a.stop_seconds = std::stod(next());
        else if (arg == "--outdir") a.outdir = next();
        else if (arg == "--log-level") a.log_level = next();
        else if (arg == "--log-file") a.log_file = next();
        else throw std::runtime_error("unknown argument: " + arg);
    }
    validate(a);
    return a;
}

enum class VehicleState { calibrating, idle, manual, backtrack, arrived };

const char* to_string(VehicleState s) {
    switch (s) {
        case VehicleState::calibrating: return "CALIBRATING";
        case VehicleState::idle: return "IDLE";
        case VehicleState::manual: return "MANUAL";
        case VehicleState::backtrack: return "FAILSAFE_BACKTRACK";
        case VehicleState::arrived: return "ARRIVED";
    }
    return "UNKNOWN";
}

// Incremental blackbox CSV (FR-7): same columns as the sandbox time-series so
// the analysis tooling works on both; gt_* is zero on real hardware.
class BlackboxCsv {
public:
    bool open(const std::string& outdir, double rate_hz) {
        flush_every_ = std::max(1L, std::lround(rate_hz));  // ~1 s of rows
        std::filesystem::create_directories(outdir);
        char name[64];
        const std::time_t t = std::time(nullptr);
        std::tm tm_buf{};
        localtime_r(&t, &tm_buf);
        std::strftime(name, sizeof(name), "vehicle_%Y%m%d_%H%M%S.csv", &tm_buf);
        path_ = outdir + "/" + name;
        out_.open(path_);
        if (!out_) return false;
        out_ << std::setprecision(9);
        out_ << "t,mode,link_ok,erpm_left,erpm_right,motor_current,gyro_z,"
                "accel_x,accel_y,accel_z,gyro_bias_est,stationary,"
                "est_x,est_y,est_heading,gt_x,gt_y,gt_heading\n";
        return true;
    }

    void row(double t, VehicleState state, bool link_ok,
             const hal::DriveStatus& ds, const hal::ImuSample& imu,
             const DeadReckoning& est) {
        const Pose p = est.pose();
        out_ << t << ',' << to_string(state) << ',' << (link_ok ? 1 : 0) << ','
             << ds.erpm_left << ',' << ds.erpm_right << ','
             << (ds.current_left + ds.current_right) << ',' << imu.gyro_z << ','
             << imu.accel_x << ',' << imu.accel_y << ',' << imu.accel_z << ','
             << est.gyro_bias_estimate() << ','
             << (est.last_stationary() ? 1 : 0) << ',' << p.x << ',' << p.y
             << ',' << p.heading << ",0,0,0\n";
        if (++rows_ % flush_every_ == 0) out_.flush();
    }

    const std::string& path() const { return path_; }

private:
    std::ofstream out_;
    std::string path_;
    std::size_t rows_{0};
    long flush_every_{50};
};

// Best-effort real-time setup: a page fault or scheduling delay stretches a
// tick, and every unaccounted millisecond becomes heading/position error in a
// dead-reckoning-only system. Degrades gracefully without privileges.
void setup_realtime() {
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        LOG_WARN << "mlockall failed (run with more privileges for RT safety)";
    }
    struct sched_param sp {};
    sp.sched_priority = 20;
    if (::sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        LOG_WARN << "SCHED_FIFO unavailable (add LimitRTPRIO / CAP_SYS_NICE); "
                    "running with default scheduler";
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse(argc, argv);
        log::init(log::level_from_string(args.log_level), args.log_file);
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        setup_realtime();

        const double dt_nominal = 1.0 / args.rate_hz;
        const DiffDriveModel model(args.track_width, args.wheel_radius,
                                   /*pole_pairs=*/5.0, /*internal_gear=*/2.35,
                                   args.external_gear);

        // The sandbox default gyro_still (0.03 rad/s) is tighter than a real
        // MPU-9250's zero-rate output (up to ~5 dps = 0.087 rad/s): with it,
        // stationarity would never trigger and the bias would never be
        // learned. 0.2 rad/s accepts any datasheet-compliant part; genuine
        // rotation that slow contributes negligible error during halts.
        EstimatorConfig est_cfg;
        est_cfg.gyro_still = 0.2;
        DeadReckoning estimator(model, est_cfg);

        hal::LinkSupervisor link(args.link_timeout);
        hal::Mpu9250I2c imu(args.i2c, args.imu_addr,
                            parse_mount(args.imu_axes));
        hal::VescSocketCan drive(args.can,
                                 static_cast<uint8_t>(args.left_id),
                                 static_cast<uint8_t>(args.right_id));
        hal::Rs485Link rs485(args.uart, args.baud);

        // Fail fast on any missing device: systemd restarts us (watchdog NFR),
        // and a vehicle without its sensors must not arm.
        if (!imu.init() || !drive.init() || !rs485.init()) return 1;

        // From here on, EVERY exit path (return, exception, signal) zeroes the
        // motors -- constructed after drive.init() so the bus exists.
        MotorStopGuard stop_guard(drive);

        BlackboxCsv blackbox;
        if (!blackbox.open(args.outdir, args.rate_hz)) {
            LOG_ERROR << "cannot open blackbox CSV in " << args.outdir;
            return 1;
        }
        LOG_INFO << "blackbox: " << blackbox.path();

        VehicleState state = VehicleState::calibrating;
        LOG_INFO << "CALIBRATING: hold still " << args.calib_seconds
                 << " s for gyro-bias estimation";

        std::unique_ptr<BacktrackController> controller;
        hal::OperatorFrame last_cmd{};
        hal::ImuSample imu_sample{};

        // IMU health with hysteresis: 5 consecutive failures = unhealthy,
        // 5 consecutive good reads to recover. An outage > 1 s means the pose
        // silently stopped tracking reality -> autonomy is disabled for good.
        int imu_good_streak = 0;
        int imu_bad_streak = 0;
        double imu_unhealthy_since = -1.0;
        bool estimate_valid = true;

        // E-stop latch: cleared only by 10 consecutive valid drive frames.
        bool estop_latched = false;
        int drive_frames_since_estop = 0;
        bool have_seq = false;
        uint8_t last_seq = 0;

        long calib_ticks = 0;
        long calib_still_ticks = 0;

        double last_imu_warn_s = -1e9;
        double last_drive_warn_s = -1e9;
        double last_state_warn_s = -1e9;
        long overruns = 0;
        double last_overrun_warn_s = -1e9;

        const double t0 = monotonic_s();
        double calib_end_s = t0 + args.calib_seconds;
        double prev_now = t0;
        double bt_start_dist = 0.0;
        double next_stop_at = 0.0;
        double halt_until_s = 0.0;
        double next_tick_s = t0;

        while (!g_stop) {
            const double now = monotonic_s();
            const double t = now - t0;
            // Integrate what actually elapsed, not what the schedule promised
            // (EINTR, page faults, CPU contention all stretch/shrink ticks).
            const double dt =
                clamp(now - prev_now, 0.5 * dt_nominal, 3.0 * dt_nominal);
            prev_now = now;

            // ---- inputs ---------------------------------------------------
            if (imu.read(imu_sample)) {
                imu_bad_streak = 0;
                ++imu_good_streak;
            } else {
                imu_good_streak = 0;
                ++imu_bad_streak;
                if (imu_bad_streak == 5) {
                    LOG_ERROR << "IMU unhealthy (5 consecutive read failures)";
                }
            }
            const bool imu_ok = imu_bad_streak < 5 && imu_good_streak >= 5;
            if (!imu_ok && imu_bad_streak >= 5) {
                if (imu_unhealthy_since < 0.0) imu_unhealthy_since = now;
                if (estimate_valid && now - imu_unhealthy_since > 1.0) {
                    estimate_valid = false;
                    LOG_ERROR << "pose estimate INVALID (IMU outage > 1 s): "
                                 "backtrack disabled until restart; manual "
                                 "relay remains available";
                }
                if (now - last_imu_warn_s > 1.0) {
                    LOG_ERROR << "IMU read failing (" << imu_bad_streak
                              << " consecutive)";
                    last_imu_warn_s = now;
                }
            } else if (imu_ok) {
                imu_unhealthy_since = -1.0;
            }

            const hal::DriveStatus ds = drive.poll(now);
            const bool drive_ok = ds.age_left_s < 0.2 && ds.age_right_s < 0.2;
            if (!drive_ok && state != VehicleState::calibrating
                && state != VehicleState::idle
                && now - last_drive_warn_s > 1.0) {
                LOG_WARN << "VESC status stale (left " << ds.age_left_s
                         << " s, right " << ds.age_right_s
                         << " s) -- check CAN bus / VESC Tool status rate";
                last_drive_warn_s = now;
            }

            hal::OperatorFrame f;
            if (rs485.poll(f)) {
                // A repeated seq is a duplicate (e.g. a radio modem flushing
                // its FIFO): it must not refresh the link-alive timer.
                if (!have_seq || f.seq != last_seq) {
                    have_seq = true;
                    last_seq = f.seq;
                    link.frame_received(now);
                    last_cmd = f;
                    if (f.mode == hal::OperatorMode::estop) {
                        if (!estop_latched) LOG_WARN << "operator E-STOP latched";
                        estop_latched = true;
                        drive_frames_since_estop = 0;
                    } else if (estop_latched) {
                        if (++drive_frames_since_estop >= 10) {
                            estop_latched = false;
                            LOG_INFO << "E-STOP cleared by operator (10 drive "
                                        "frames)";
                        }
                    }
                }
            }
            const bool link_ok = link.link_ok(now);

            // Dead reckoning runs in every state (breadcrumbs, ZUPT, bias).
            // ERPM from a stale VESC side reads as zero (driver fails safe).
            if (imu_bad_streak == 0) {
                estimator.update(dt, ds.erpm_left, ds.erpm_right,
                                 imu_sample.gyro_z, imu_sample.accel_x,
                                 imu_sample.accel_y, imu_sample.accel_z);
            }

            // ---- state machine -------------------------------------------
            double v = 0.0;
            double omega = 0.0;

            switch (state) {
                case VehicleState::calibrating:
                    ++calib_ticks;
                    if (estimator.last_stationary()) ++calib_still_ticks;
                    if (now >= calib_end_s) {
                        // The bias is only learned from ticks the stationarity
                        // detector accepted. If it never triggered (vibration,
                        // mis-tuned thresholds, someone moved the vehicle) the
                        // estimate is still zero -- extending is safer than
                        // silently driving with an uncorrected gyro.
                        if (calib_ticks > 0
                            && calib_still_ticks * 2 >= calib_ticks) {
                            LOG_INFO << "calibration done: gyro bias est "
                                     << estimator.gyro_bias_estimate()
                                     << " rad/s (" << calib_still_ticks << "/"
                                     << calib_ticks << " still ticks)";
                            state = VehicleState::idle;
                        } else {
                            LOG_ERROR << "calibration failed: only "
                                      << calib_still_ticks << "/" << calib_ticks
                                      << " ticks stationary -- keep the vehicle "
                                         "still; extending 5 s";
                            calib_end_s = now + 5.0;
                            calib_ticks = 0;
                            calib_still_ticks = 0;
                        }
                    }
                    break;

                case VehicleState::idle:
                    if (link_ok) {
                        LOG_INFO << "operator link up -> MANUAL";
                        state = VehicleState::manual;
                    }
                    break;

                case VehicleState::manual:
                    if (estop_latched) {
                        // Highest priority: latched E-STOP holds zero even
                        // through a link loss -- the last human intent was
                        // STOP, so no autonomous motion either.
                        if (!link_ok && now - last_state_warn_s > 5.0) {
                            LOG_WARN << "link lost while E-STOP latched: "
                                        "holding position (no backtrack)";
                            last_state_warn_s = now;
                        }
                    } else if (!link_ok) {
                        if (!estimate_valid) {
                            if (now - last_state_warn_s > 5.0) {
                                LOG_ERROR << "LINK LOST but pose estimate is "
                                             "invalid: holding position";
                                last_state_warn_s = now;
                            }
                            break;
                        }
                        std::vector<Point> return_path = estimator.trail_xy();
                        std::reverse(return_path.begin(), return_path.end());
                        if (return_path.size() < 2) {
                            LOG_WARN << "LINK LOST with a trail of "
                                     << return_path.size()
                                     << " point(s): nothing to backtrack, "
                                        "holding position";
                            state = VehicleState::arrived;
                            break;
                        }
                        LOG_WARN << "LINK LOST (silence " << link.silence_s(now)
                                 << " s) -> FAILSAFE_BACKTRACK; trail "
                                 << return_path.size() << " crumbs";
                        PursuitConfig pursuit;
                        pursuit.reverse = args.reverse;
                        pursuit.cruise_speed =
                            std::min(pursuit.cruise_speed, args.max_speed);
                        pursuit.max_yaw_rate =
                            std::min(pursuit.max_yaw_rate, args.max_yaw_rate);
                        controller = std::make_unique<BacktrackController>(
                            std::move(return_path), pursuit);
                        bt_start_dist = estimator.total_distance();
                        next_stop_at = args.stop_interval_m;
                        halt_until_s = 0.0;
                        state = VehicleState::backtrack;
                    } else {
                        v = clamp(last_cmd.v, -args.max_speed, args.max_speed);
                        omega = clamp(last_cmd.omega, -args.max_yaw_rate,
                                      args.max_yaw_rate);
                    }
                    break;

                case VehicleState::backtrack: {
                    if (link_ok) {
                        LOG_INFO << "operator link REGAINED -> MANUAL";
                        controller.reset();
                        state = VehicleState::manual;
                        break;
                    }
                    // No odometry feedback = no trustworthy pose updates while
                    // moving: hold until the VESC status stream returns.
                    if (!drive_ok) break;
                    if (now < halt_until_s) break;  // BACKTRACK_PARTIAL halt
                    if (args.stop_interval_m > 0.0
                        && estimator.total_distance() - bt_start_dist
                               >= next_stop_at) {
                        halt_until_s = now + args.stop_seconds;
                        LOG_INFO << "BACKTRACK_PARTIAL halt at " << next_stop_at
                                 << " m; bias re-est "
                                 << estimator.gyro_bias_estimate();
                        next_stop_at += args.stop_interval_m;
                        break;
                    }
                    const Command cmd = controller->compute(estimator.pose());
                    if (cmd.done) {
                        LOG_INFO << "ARRIVED at trail start; holding";
                        controller.reset();
                        state = VehicleState::arrived;
                        break;
                    }
                    v = cmd.v;
                    omega = cmd.omega;
                    break;
                }

                case VehicleState::arrived:
                    if (link_ok) {
                        LOG_INFO << "operator link up -> MANUAL";
                        state = VehicleState::manual;
                    }
                    break;
            }

            // ---- safety gates then actuation ------------------------------
            if (estop_latched) {
                v = 0.0;
                omega = 0.0;
            }
            if (imu_bad_streak >= 5 && state != VehicleState::manual) {
                // No usable heading: autonomous motion is not safe. Manual
                // relay stays available (the operator can still see the UGV).
                v = 0.0;
                omega = 0.0;
            }
            if (!std::isfinite(v) || !std::isfinite(omega)) {
                LOG_ERROR << "non-finite command (v=" << v << ", omega=" << omega
                          << ") -- forcing zero";
                v = 0.0;
                omega = 0.0;
            }
            v = clamp(v, -args.max_speed, args.max_speed);
            omega = clamp(omega, -args.max_yaw_rate, args.max_yaw_rate);

            const WheelSpeeds wheels = model.wheel_speeds_from_twist(v, omega);
            const double erpm_l =
                clamp(model.erpm_from_wheel_speed(wheels.left), -args.max_erpm,
                      args.max_erpm);
            const double erpm_r =
                clamp(model.erpm_from_wheel_speed(wheels.right), -args.max_erpm,
                      args.max_erpm);
            // Sent every tick: doubles as the keepalive for the VESC timeout.
            drive.set_erpm(erpm_l, erpm_r);

            blackbox.row(t, state, link_ok, ds, imu_sample, estimator);

            next_tick_s += dt_nominal;
            const double after = monotonic_s();
            if (next_tick_s < after) {  // loop body overran its slot
                ++overruns;
                if (after - last_overrun_warn_s > 5.0) {
                    LOG_WARN << "control loop overrun x" << overruns
                             << " (tick budget " << dt_nominal * 1e3 << " ms)";
                    last_overrun_warn_s = after;
                }
                next_tick_s = after + dt_nominal;
            }
            sleep_until_monotonic(next_tick_s);
        }

        LOG_INFO << "shutdown signal: stopping motors";
        return 0;  // MotorStopGuard zeroes the tracks on the way out
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
}
