#ifndef BACKTRACK_LOG_HPP
#define BACKTRACK_LOG_HPP

// Minimal, dependency-free logger for the on-vehicle build.
//
// Opt-in: silent until log::init() is called (so unit tests stay quiet). Writes
// to stderr (captured by systemd/journald on the Pi) and, optionally, to a file.
// Each line carries a monotonic timestamp, level, and source location:
//
//   [   12.340] INFO  simulation.cpp:71  link lost -> FAILSAFE_BACKTRACK
//
// Usage:  LOG_INFO << "bias estimate " << bias << " rad/s";

#include <sstream>
#include <string>

namespace backtrack {
namespace log {

enum class Level { debug, info, warn, error };

void init(Level level, const std::string& file_path = "");
void set_level(Level level);
Level level_from_string(const std::string& s);
void write_line(Level lvl, const char* file, int line, const std::string& msg);

// RAII stream sink: accumulates one line, flushes on destruction.
class Line {
public:
    Line(Level lvl, const char* file, int line)
        : lvl_(lvl), file_(file), line_(line) {}
    ~Line() { write_line(lvl_, file_, line_, os_.str()); }
    Line(const Line&) = delete;
    Line& operator=(const Line&) = delete;
    std::ostringstream& stream() { return os_; }

private:
    Level lvl_;
    const char* file_;
    int line_;
    std::ostringstream os_;
};

}  // namespace log
}  // namespace backtrack

#define BT_LOG(lvl) \
    ::backtrack::log::Line(lvl, __FILE__, __LINE__).stream()
#define LOG_DEBUG BT_LOG(::backtrack::log::Level::debug)
#define LOG_INFO BT_LOG(::backtrack::log::Level::info)
#define LOG_WARN BT_LOG(::backtrack::log::Level::warn)
#define LOG_ERROR BT_LOG(::backtrack::log::Level::error)

#endif  // BACKTRACK_LOG_HPP
