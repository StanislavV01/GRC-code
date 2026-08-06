#include "backtrack/logio.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace backtrack {

namespace {

std::ofstream open_out(const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot open for writing: " + path);
    return out;
}

// Map a world point into SVG pixel space (north up).
struct Transform {
    double min_x, min_y, scale;
    int size, margin;
    std::pair<double, double> operator()(double x, double y) const {
        const double px = margin + (x - min_x) * scale;
        const double py = size - (margin + (y - min_y) * scale);
        return {px, py};
    }
};

void polyline(std::ostream& os, const std::vector<Point>& pts,
              const Transform& tf, const char* color, double width,
              const char* dash) {
    os << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\""
       << width << "\"";
    if (dash) os << " stroke-dasharray=\"" << dash << "\"";
    os << " points=\"";
    for (const Point& p : pts) {
        const auto [px, py] = tf(p.x, p.y);
        os << px << ',' << py << ' ';
    }
    os << "\" />\n";
}

}  // namespace

void write_timeseries_csv(const std::string& path,
                          const std::vector<Record>& records) {
    std::ofstream out = open_out(path);
    out << "t,mode,link_ok,erpm_left,erpm_right,motor_current,gyro_z,"
           "accel_x,accel_y,accel_z,gyro_bias_est,stationary,"
           "est_x,est_y,est_heading,gt_x,gt_y,gt_heading\n";
    out.precision(6);
    for (const Record& r : records) {
        out << r.t << ',' << to_string(r.mode) << ',' << r.link_ok << ','
            << r.erpm_left << ',' << r.erpm_right << ',' << r.motor_current << ','
            << r.gyro_z << ',' << r.accel_x << ',' << r.accel_y << ',' << r.accel_z
            << ',' << r.gyro_bias_est << ',' << r.stationary << ',' << r.est.x
            << ',' << r.est.y << ',' << r.est.heading << ',' << r.gt.x << ','
            << r.gt.y << ',' << r.gt.heading << '\n';
    }
}

void write_breadcrumbs_csv(const std::string& path,
                           const std::vector<Breadcrumb>& breadcrumbs) {
    std::ofstream out = open_out(path);
    out << "idx,x,y,heading,cumulative_dist\n";
    out.precision(6);
    for (std::size_t i = 0; i < breadcrumbs.size(); ++i) {
        const Breadcrumb& b = breadcrumbs[i];
        out << i << ',' << b.x << ',' << b.y << ',' << b.heading << ','
            << b.cumulative_dist << '\n';
    }
}

void write_route_svg(const std::string& path, const SimResult& result, int size,
                     int margin) {
    std::vector<Point> gt;
    std::vector<Point> est;
    gt.reserve(result.records.size());
    est.reserve(result.records.size());
    for (const Record& r : result.records) {
        gt.push_back(Point{r.gt.x, r.gt.y});
        est.push_back(Point{r.est.x, r.est.y});
    }

    double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
    bool first = true;
    const auto extend = [&](const std::vector<Point>& pts) {
        for (const Point& p : pts) {
            if (first) {
                min_x = max_x = p.x;
                min_y = max_y = p.y;
                first = false;
            } else {
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
            }
        }
    };
    extend(gt);
    extend(est);

    const double span = std::max({max_x - min_x, max_y - min_y, 1.0});
    const Transform tf{min_x, min_y, (size - 2.0 * margin) / span, size, margin};

    std::ofstream out = open_out(path);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << size
        << "\" height=\"" << size << "\" viewBox=\"0 0 " << size << ' ' << size
        << "\">\n";
    out << "<rect width=\"" << size << "\" height=\"" << size
        << "\" fill=\"#0f1117\" />\n";
    polyline(out, gt, tf, "#39d353", 3.0, nullptr);
    polyline(out, est, tf, "#f2a900", 1.6, "6,4");

    const auto [sx, sy] = tf(0.0, 0.0);
    out << "<circle cx=\"" << sx << "\" cy=\"" << sy
        << "\" r=\"7\" fill=\"#58a6ff\" />\n";
    const auto [fx, fy] = tf(result.summary.gt_final.x, result.summary.gt_final.y);
    out << "<circle cx=\"" << fx << "\" cy=\"" << fy
        << "\" r=\"7\" fill=\"#ff5555\" />\n";

    const char* labels[4] = {"true trajectory", "dead-reckoned estimate",
                             "true start (home)", "final position"};
    const char* colors[4] = {"#39d353", "#f2a900", "#58a6ff", "#ff5555"};
    for (int i = 0; i < 4; ++i) {
        const int y = 30 + i * 22;
        out << "<rect x=\"20\" y=\"" << (y - 11)
            << "\" width=\"16\" height=\"10\" fill=\"" << colors[i] << "\" />\n";
        out << "<text x=\"44\" y=\"" << y
            << "\" fill=\"#c9d1d9\" font-family=\"monospace\" font-size=\"14\">"
            << labels[i];
        if (i == 3) out << "  (err " << result.summary.final_return_error_m << " m)";
        out << "</text>\n";
    }
    out << "</svg>\n";
}

}  // namespace backtrack
