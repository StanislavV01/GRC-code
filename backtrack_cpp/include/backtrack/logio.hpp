#ifndef BACKTRACK_LOGIO_HPP
#define BACKTRACK_LOGIO_HPP

// Output helpers: write the blackbox CSVs and a dependency-free SVG plot.

#include <string>

#include "backtrack/simulation.hpp"

namespace backtrack {

void write_timeseries_csv(const std::string& path,
                          const std::vector<Record>& records);

void write_breadcrumbs_csv(const std::string& path,
                           const std::vector<Breadcrumb>& breadcrumbs);

void write_route_svg(const std::string& path, const SimResult& result,
                     int size = 900, int margin = 50);

}  // namespace backtrack

#endif  // BACKTRACK_LOGIO_HPP
