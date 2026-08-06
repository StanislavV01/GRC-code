// Run the compass-free Link-Loss Backtrack demo and write logs + plot.
//
// Usage:
//   run_demo [--source gyro|odometry] [--no-calib] [--no-zupt]
//            [--seed N] [--outdir DIR]

#include <filesystem>
#include <iostream>
#include <string>

#include "backtrack/estimator.hpp"
#include "backtrack/log.hpp"
#include "backtrack/logio.hpp"
#include "backtrack/sensors.hpp"
#include "backtrack/simulation.hpp"

namespace {

struct Args {
    backtrack::HeadingSource source = backtrack::HeadingSource::gyro;
    bool calibrate = true;
    bool zupt = true;
    bool reverse = false;
    double speed_counts = backtrack::kEncoderCountsPerRev;  // encoder by default
    std::string speed_sensor = "encoder";
    unsigned long seed = 7;
    std::string outdir = "data";
    std::string log_level = "info";
    std::string log_file;  // empty = stderr only
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
            return argv[++i];
        };
        if (arg == "--source") {
            const std::string v = next();
            a.source = (v == "odometry") ? backtrack::HeadingSource::odometry
                                         : backtrack::HeadingSource::gyro;
        } else if (arg == "--no-calib") {
            a.calibrate = false;
        } else if (arg == "--no-zupt") {
            a.zupt = false;
        } else if (arg == "--reverse") {
            a.reverse = true;
        } else if (arg == "--speed-sensor") {
            a.speed_sensor = next();
            if (a.speed_sensor == "hall") {
                a.speed_counts = backtrack::kHallCountsPerRev;
            } else if (a.speed_sensor == "encoder") {
                a.speed_counts = backtrack::kEncoderCountsPerRev;
            } else if (a.speed_sensor == "ideal") {
                a.speed_counts = 0.0;
            } else {
                throw std::runtime_error("speed-sensor: hall|encoder|ideal");
            }
        } else if (arg == "--seed") {
            a.seed = std::stoul(next());
        } else if (arg == "--outdir") {
            a.outdir = next();
        } else if (arg == "--log-level") {
            a.log_level = next();
        } else if (arg == "--log-file") {
            a.log_file = next();
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    return a;
}

const char* yesno(bool b) { return b ? "on" : "off"; }

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse(argc, argv);
        std::filesystem::create_directories(args.outdir);
        backtrack::log::init(backtrack::log::level_from_string(args.log_level),
                             args.log_file);

        backtrack::SimConfig cfg;
        cfg.seed = args.seed;
        cfg.estimator.heading_source = args.source;
        cfg.estimator.calibrate_bias = args.calibrate;
        cfg.estimator.zupt = args.zupt;
        cfg.pursuit.reverse = args.reverse;
        cfg.sensor_params.odo_counts_per_motor_rev = args.speed_counts;
        const backtrack::SimResult result = backtrack::run_simulation(cfg);

        const std::string ts = args.outdir + "/session_timeseries.csv";
        const std::string bc = args.outdir + "/breadcrumbs.csv";
        const std::string svg = args.outdir + "/route.svg";
        backtrack::write_timeseries_csv(ts, result.records);
        backtrack::write_breadcrumbs_csv(bc, result.breadcrumbs);
        backtrack::write_route_svg(svg, result);

        const backtrack::SimSummary& s = result.summary;
        const char* src = s.heading_source == backtrack::HeadingSource::gyro
                              ? "gyro"
                              : "odometry";
        std::cout << "=== Compass-free Backtrack demo (C++, IMU-only) ===\n";
        std::cout << "heading source      : " << src << '\n';
        std::cout << "backtrack mode      : "
                  << (args.reverse ? "reverse (no turn-around)" : "forward")
                  << '\n';
        std::cout << "speed sensor        : " << args.speed_sensor << '\n';
        std::cout << "bias calibration    : " << yesno(s.calibrate_bias) << '\n';
        std::cout << "ZUPT                : " << yesno(s.zupt) << '\n';
        std::cout << "gyro bias true/est  : " << s.true_gyro_bias << " / "
                  << s.final_gyro_bias_est << " rad/s\n";
        std::cout << "outbound distance   : " << s.outbound_distance_m << " m\n";
        std::cout << "ticks logged        : " << s.ticks << '\n';
        std::cout << "breadcrumbs         : " << s.breadcrumbs << '\n';
        std::cout << "heading drift @ end : " << s.est_heading_error_deg
                  << " deg\n";
        std::cout << "mean backtrack ERPM : " << s.mean_backtrack_erpm
                  << " (sensorless floor ~1000-2000)\n";
        std::cout << "odometry dist error : " << s.est_distance_error_m << " m\n";
        std::cout << "est outbound error  : " << s.est_outbound_error_m << " m\n";
        std::cout << "backtrack completed : "
                  << (s.backtrack_completed ? "true" : "false") << '\n';
        std::cout << "final return error  : " << s.final_return_error_m << " m ("
                  << s.final_error_pct << "% of route)\n\n";
        std::cout << "wrote " << ts << '\n';
        std::cout << "wrote " << bc << '\n';
        std::cout << "wrote " << svg << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
}
