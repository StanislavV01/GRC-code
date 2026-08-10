// Listen-only DRY RUN of the Link-Loss Backtrack brain against the live vehicle.
//
// Reads REAL odometry off the DroneCAN bus (esc.Status, node 50, esc_index
// 0 = left / 2 = right -- field-verified 2026-08-10), runs the dead-reckoning
// estimator with ODOMETRY-ONLY heading (a skid-steer derives yaw from the
// left/right track-speed difference, so NO IMU is required for a first pass) and
// records the breadcrumb trail. On a link-loss TRIGGER it runs the backtrack
// pure-pursuit controller and LOGS the (v, omega, ERPM) it WOULD command.
//
// IT NEVER WRITES TO THE CAN BUS. The drive HAL's set_erpm() is a no-op and this
// app does not even call it -- the "commands" exist only in the log. Safe to run
// on a live vehicle / on the stand. Bring can0 up LISTEN-ONLY first
// (deploy/can0-listen.service).
//
// Trigger link loss:   kill -USR1 <pid>      (start backtrack)
// Regain the link:     kill -USR2 <pid>      (back to recording)
//
// Usage (defaults in brackets):
//   run_observe [--can can0] [--node 50] [--left-index 0] [--right-index 2]
//               [--track-width M] [--wheel-radius M] [--external-gear R]
//               [--rate 50] [--reverse] [--cruise 1.0] [--max-yaw-rate 1.0]
//               [--outdir data] [--log-level info] [--log-file PATH]

#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "backtrack/controller.hpp"
#include "backtrack/estimator.hpp"
#include "backtrack/geometry.hpp"
#include "backtrack/hal/dronecan_esc.hpp"
#include "backtrack/kinematics.hpp"
#include "backtrack/log.hpp"

namespace {

using namespace backtrack;

std::atomic<bool> g_stop{false};
std::atomic<bool> g_trigger{false};  // SIGUSR1: link lost -> backtrack
std::atomic<bool> g_regain{false};   // SIGUSR2: link regained -> record

void on_stop(int) { g_stop = true; }
void on_usr1(int) { g_trigger = true; }
void on_usr2(int) { g_regain = true; }

double monotonic_s() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + ts.tv_nsec * 1e-9;
}

void sleep_until(double target_s) {
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(target_s);
    ts.tv_nsec = static_cast<long>((target_s - ts.tv_sec) * 1e9);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

struct Args {
    std::string can = "can0";
    int node = 50;
    int left_index = 0;
    int right_index = 2;
    double track_width = 0.5;
    double wheel_radius = 0.12;
    double external_gear = 1.0;
    double rate_hz = 50.0;
    bool reverse = false;
    double cruise = 1.0;
    double max_yaw_rate = 1.0;
    std::string outdir = "data";
    std::string log_level = "info";
    std::string log_file;
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
            return argv[++i];
        };
        if (arg == "--can") a.can = next();
        else if (arg == "--node") a.node = std::stoi(next());
        else if (arg == "--left-index") a.left_index = std::stoi(next());
        else if (arg == "--right-index") a.right_index = std::stoi(next());
        else if (arg == "--track-width") a.track_width = std::stod(next());
        else if (arg == "--wheel-radius") a.wheel_radius = std::stod(next());
        else if (arg == "--external-gear") a.external_gear = std::stod(next());
        else if (arg == "--rate") a.rate_hz = std::stod(next());
        else if (arg == "--reverse") a.reverse = true;
        else if (arg == "--cruise") a.cruise = std::stod(next());
        else if (arg == "--max-yaw-rate") a.max_yaw_rate = std::stod(next());
        else if (arg == "--outdir") a.outdir = next();
        else if (arg == "--log-level") a.log_level = next();
        else if (arg == "--log-file") a.log_file = next();
        else throw std::runtime_error("unknown argument: " + arg);
    }
    return a;
}

enum class State { record, backtrack, arrived };
const char* to_string(State s) {
    switch (s) {
        case State::record: return "RECORD";
        case State::backtrack: return "BACKTRACK(dry-run)";
        case State::arrived: return "ARRIVED";
    }
    return "?";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse(argc, argv);
        log::init(log::level_from_string(args.log_level), args.log_file);
        std::signal(SIGINT, on_stop);
        std::signal(SIGTERM, on_stop);
        std::signal(SIGUSR1, on_usr1);
        std::signal(SIGUSR2, on_usr2);

        const double dt_nominal = 1.0 / args.rate_hz;
        const DiffDriveModel model(args.track_width, args.wheel_radius,
                                   /*pole_pairs=*/5.0, /*internal_gear=*/2.35,
                                   args.external_gear);

        // Odometry-only heading: no IMU. Gyro-bias calibration is meaningless
        // without a gyro, so it is off; ZUPT still helps (it keys off odometry).
        EstimatorConfig cfg;
        cfg.heading_source = HeadingSource::odometry;
        cfg.calibrate_bias = false;
        cfg.zupt = true;
        DeadReckoning estimator(model, cfg);

        hal::DroneCanEsc drive(args.can, static_cast<uint8_t>(args.node),
                               args.left_index, args.right_index);
        if (!drive.init()) return 1;

        std::ofstream csv;
        {
            const std::string path = args.outdir + "/observe.csv";
            std::error_code ec;
            std::filesystem::create_directories(args.outdir, ec);
            csv.open(path);
            if (csv) {
                csv << std::setprecision(9);
                csv << "t,mode,erpm_left,erpm_right,age_left,age_right,est_x,"
                       "est_y,est_heading,crumbs,total_dist,cmd_v,cmd_omega,"
                       "would_erpm_left,would_erpm_right\n";
                LOG_INFO << "blackbox: " << path;
            }
        }

        LOG_WARN << "LISTEN-ONLY dry run: never transmits. "
                    "kill -USR1 " << ::getpid() << " to trigger backtrack, "
                    "-USR2 to regain link.";
        LOG_INFO << "RECORD: drive the vehicle; the trail is being recorded";

        State state = State::record;
        std::unique_ptr<BacktrackController> controller;

        const double t0 = monotonic_s();
        double prev = t0;
        double next_tick = t0;
        double last_print = t0;

        while (!g_stop) {
            const double now = monotonic_s();
            const double t = now - t0;
            const double dt = clamp(now - prev, 0.5 * dt_nominal, 3.0 * dt_nominal);
            prev = now;

            const hal::DriveStatus ds = drive.poll(now);
            // Level, gyro-free inertial inputs: stationarity keys off odometry.
            estimator.update(dt, ds.erpm_left, ds.erpm_right, 0.0, 0.0, 0.0,
                             cfg.gravity);

            double v = 0.0, omega = 0.0;

            if (g_regain.exchange(false)) {
                if (state != State::record) {
                    LOG_INFO << "link REGAINED -> RECORD";
                    controller.reset();
                    state = State::record;
                }
            }

            switch (state) {
                case State::record:
                    if (g_trigger.exchange(false)) {
                        std::vector<Point> path = estimator.trail_xy();
                        std::reverse(path.begin(), path.end());
                        if (path.size() < 2) {
                            LOG_WARN << "trigger with " << path.size()
                                     << " crumb(s): nothing to backtrack";
                            state = State::arrived;
                            break;
                        }
                        PursuitConfig pursuit;
                        pursuit.reverse = args.reverse;
                        pursuit.cruise_speed = std::min(pursuit.cruise_speed,
                                                        args.cruise);
                        pursuit.max_yaw_rate = std::min(pursuit.max_yaw_rate,
                                                        args.max_yaw_rate);
                        controller = std::make_unique<BacktrackController>(
                            std::move(path), pursuit);
                        LOG_WARN << "LINK LOST (dry run) -> BACKTRACK; trail "
                                 << estimator.breadcrumbs().size()
                                 << " crumbs; NOT commanding motors";
                        state = State::backtrack;
                    }
                    break;

                case State::backtrack: {
                    g_trigger.exchange(false);
                    const Command cmd = controller->compute(estimator.pose());
                    if (cmd.done) {
                        LOG_INFO << "ARRIVED at trail start (dry run)";
                        controller.reset();
                        state = State::arrived;
                        break;
                    }
                    v = cmd.v;
                    omega = cmd.omega;
                    break;
                }

                case State::arrived:
                    g_trigger.exchange(false);
                    break;
            }

            // What the controller WOULD have commanded -- logged, never sent.
            const WheelSpeeds w = model.wheel_speeds_from_twist(v, omega);
            const double would_l = model.erpm_from_wheel_speed(w.left);
            const double would_r = model.erpm_from_wheel_speed(w.right);

            if (csv) {
                const Pose p = estimator.pose();
                csv << t << ',' << to_string(state) << ',' << ds.erpm_left << ','
                    << ds.erpm_right << ',' << ds.age_left_s << ','
                    << ds.age_right_s << ',' << p.x << ',' << p.y << ','
                    << p.heading << ',' << estimator.breadcrumbs().size() << ','
                    << estimator.total_distance() << ',' << v << ',' << omega
                    << ',' << would_l << ',' << would_r << '\n';
            }

            if (now - last_print > 1.0) {
                last_print = now;
                const Pose p = estimator.pose();
                LOG_INFO << to_string(state) << "  erpm L/R "
                         << int(ds.erpm_left) << '/' << int(ds.erpm_right)
                         << "  pose (" << std::fixed << std::setprecision(2)
                         << p.x << ", " << p.y << ", " << p.heading << ")  dist "
                         << estimator.total_distance() << "  crumbs "
                         << estimator.breadcrumbs().size();
            }

            next_tick += dt_nominal;
            const double after = monotonic_s();
            if (next_tick < after) next_tick = after + dt_nominal;
            sleep_until(next_tick);
        }

        if (csv) csv.flush();
        LOG_INFO << "stopped (no motion was ever commanded)";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
}
