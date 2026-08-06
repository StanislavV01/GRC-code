// Bench operator console: sends operator frames over a serial port (USB-RS485
// dongle on a laptop, or the Pi's own port in loopback) so the vehicle side
// can be tested without the real radio console.
//
//   op_console --uart /dev/ttyUSB0 [--baud 115200] [--v 0.3] [--omega 0.0]
//              [--estop] [--hz 20] [--seconds 0]
//
// Sends frames at --hz until --seconds elapse (0 = until Ctrl+C). Stopping it
// IS the link-loss test: the vehicle must enter FAILSAFE_BACKTRACK after its
// --link-timeout of silence.

#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "backtrack/hal/frame_codec.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void handle_signal(int) { g_stop = 1; }

speed_t baud_constant(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B0;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string uart = "/dev/ttyUSB0";
    int baud = 115200;
    double v = 0.0;
    double omega = 0.0;
    bool estop = false;
    double hz = 20.0;
    double seconds = 0.0;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            const auto next = [&]() -> std::string {
                if (i + 1 >= argc)
                    throw std::runtime_error("missing value for " + arg);
                return argv[++i];
            };
            if (arg == "--uart") uart = next();
            else if (arg == "--baud") baud = std::stoi(next());
            else if (arg == "--v") v = std::stod(next());
            else if (arg == "--omega") omega = std::stod(next());
            else if (arg == "--estop") estop = true;
            else if (arg == "--hz") hz = std::stod(next());
            else if (arg == "--seconds") seconds = std::stod(next());
            else throw std::runtime_error("unknown argument: " + arg);
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }

    if (!(hz > 0.0 && hz <= 1000.0)) {
        std::cerr << "error: --hz must be in (0, 1000]\n";
        return 1;
    }

    const speed_t speed = baud_constant(baud);
    if (speed == B0) {
        std::cerr << "error: unsupported baud " << baud << '\n';
        return 1;
    }
    const int fd = ::open(uart.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "error: open " << uart << ": " << std::strerror(errno)
                  << '\n';
        return 1;
    }
    struct termios tio {};
    if (::tcgetattr(fd, &tio) != 0) {
        std::cerr << "error: tcgetattr: " << std::strerror(errno) << '\n';
        ::close(fd);
        return 1;
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::cerr << "error: tcsetattr: " << std::strerror(errno) << '\n';
        ::close(fd);
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::cout << "sending " << (estop ? "E-STOP" : "drive") << " frames: v=" << v
              << " m/s, omega=" << omega << " rad/s @ " << hz << " Hz on "
              << uart << " (Ctrl+C = link loss)\n";

    backtrack::hal::OperatorFrame frame;
    frame.mode = estop ? backtrack::hal::OperatorMode::estop
                       : backtrack::hal::OperatorMode::drive;
    frame.v = v;
    frame.omega = omega;

    const double period_s = 1.0 / hz;
    struct timespec period;
    period.tv_sec = static_cast<time_t>(period_s);
    period.tv_nsec = static_cast<long>((period_s - period.tv_sec) * 1e9);
    const long max_frames =
        seconds > 0.0 ? static_cast<long>(seconds * hz) : -1;
    long sent = 0;
    while (!g_stop && (max_frames < 0 || sent < max_frames)) {
        uint8_t wire[backtrack::hal::kOpFrameSize];
        encode_operator_frame(frame, wire);
        if (::write(fd, wire, sizeof(wire)) != sizeof(wire)) {
            std::cerr << "write failed: " << std::strerror(errno) << '\n';
            break;
        }
        ++frame.seq;  // wraps at 255 by design
        ++sent;
        ::nanosleep(&period, nullptr);
    }
    ::close(fd);
    std::cout << "sent " << sent << " frames, stopping (link goes silent now)\n";
    return 0;
}
