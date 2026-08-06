#include "backtrack/hal/rs485_link.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "backtrack/log.hpp"

namespace backtrack {
namespace hal {

namespace {

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

Rs485Link::Rs485Link(std::string dev_path, int baud)
    : dev_path_(std::move(dev_path)), baud_(baud) {}

Rs485Link::~Rs485Link() {
    if (fd_ >= 0) ::close(fd_);
}

bool Rs485Link::init() {
    const speed_t speed = baud_constant(baud_);
    if (speed == B0) {
        LOG_ERROR << "RS485: unsupported baud " << baud_;
        return false;
    }

    if (fd_ >= 0) {  // re-init: do not leak the previous descriptor
        ::close(fd_);
        fd_ = -1;
    }
    fd_ = ::open(dev_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        LOG_ERROR << "RS485: cannot open " << dev_path_ << ": "
                  << std::strerror(errno);
        return false;
    }
    const auto fail = [this]() {
        ::close(fd_);
        fd_ = -1;
        return false;
    };

    struct termios tio {};
    if (::tcgetattr(fd_, &tio) != 0) {
        LOG_ERROR << "RS485: tcgetattr failed: " << std::strerror(errno);
        return fail();
    }
    ::cfmakeraw(&tio);              // raw 8N1, no echo, no signals
    tio.c_cflag |= CLOCAL | CREAD;  // ignore modem lines
    tio.c_cc[VMIN] = 0;             // non-blocking reads
    tio.c_cc[VTIME] = 0;
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
        LOG_ERROR << "RS485: tcsetattr failed: " << std::strerror(errno);
        return fail();
    }
    ::tcflush(fd_, TCIOFLUSH);

    LOG_INFO << "RS485: " << dev_path_ << " @ " << baud_ << " 8N1 ready";
    return true;
}

bool Rs485Link::poll(OperatorFrame& out) {
    // Drain cap: a babbling/shorted line must not trap the 50 Hz control loop
    // in here. 4 KiB per tick is ~28x the healthy 20 Hz frame stream.
    constexpr int kMaxReadsPerPoll = 16;
    uint8_t buf[256];
    bool got = false;
    OperatorFrame frame;
    for (int r = 0; r < kMaxReadsPerPoll; ++r) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n <= 0) break;
        for (ssize_t i = 0; i < n; ++i) {
            if (parser_.push(buf[i], frame)) {
                out = frame;  // keep the newest complete frame
                got = true;
            }
        }
    }
    return got;
}

}  // namespace hal
}  // namespace backtrack
