#include "backtrack/log.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>

namespace backtrack {
namespace log {

namespace {
std::mutex g_mutex;
bool g_init = false;
Level g_level = Level::info;
std::ofstream g_file;
std::chrono::steady_clock::time_point g_start;

const char* level_name(Level l) {
    switch (l) {
        case Level::debug: return "DEBUG";
        case Level::info: return "INFO ";
        case Level::warn: return "WARN ";
        case Level::error: return "ERROR";
    }
    return "?????";
}

std::string basename(const char* path) {
    const std::string s(path);
    const std::size_t pos = s.find_last_of('/');
    return pos == std::string::npos ? s : s.substr(pos + 1);
}
}  // namespace

void init(Level level, const std::string& file_path) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_level = level;
    g_start = std::chrono::steady_clock::now();
    g_init = true;
    if (!file_path.empty()) {
        g_file.open(file_path, std::ios::app);
    }
}

void set_level(Level level) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_level = level;
}

Level level_from_string(const std::string& s) {
    if (s == "debug") return Level::debug;
    if (s == "warn") return Level::warn;
    if (s == "error") return Level::error;
    return Level::info;
}

void write_line(Level lvl, const char* file, int line, const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_init || static_cast<int>(lvl) < static_cast<int>(g_level)) return;

    const double t = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - g_start)
                         .count();
    char stamp[24];
    std::snprintf(stamp, sizeof(stamp), "[%9.3f]", t);

    std::ostringstream out;
    out << stamp << ' ' << level_name(lvl) << ' ' << basename(file) << ':'
        << line << "  " << msg << '\n';
    const std::string text = out.str();

    std::cerr << text;
    if (g_file.is_open()) {
        g_file << text;
        g_file.flush();
    }
}

}  // namespace log
}  // namespace backtrack
