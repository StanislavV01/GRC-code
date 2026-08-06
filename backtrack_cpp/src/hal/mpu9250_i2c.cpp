#include "backtrack/hal/mpu9250_i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include "backtrack/hal/frame_codec.hpp"
#include "backtrack/log.hpp"

namespace backtrack {
namespace hal {

namespace {

// Register map (MPU-9250 datasheet / register map rev 1.6).
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelConfig2 = 0x1D;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

// GY-9250 boards ship several die revisions; all share this register map.
constexpr uint8_t kWhoAmIMpu9250 = 0x71;
constexpr uint8_t kWhoAmIMpu9255 = 0x73;
constexpr uint8_t kWhoAmIMpu6500 = 0x70;  // common on clone boards, no mag

}  // namespace

Mpu9250I2c::Mpu9250I2c(std::string dev_path, int address, MountConfig mount)
    : dev_path_(std::move(dev_path)), address_(address), mount_(mount) {}

Mpu9250I2c::~Mpu9250I2c() {
    if (fd_ >= 0) ::close(fd_);
}

namespace {
// One retry absorbs a single EINTR / NACK glitch without burning a slot in
// the control loop's consecutive-failure budget.
template <typename Op>
bool with_retry(Op op) {
    if (op()) return true;
    return op();
}
}  // anonymous namespace

bool Mpu9250I2c::write_reg(uint8_t reg, uint8_t value) {
    const uint8_t buf[2] = {reg, value};
    return with_retry([&]() { return ::write(fd_, buf, 2) == 2; });
}

bool Mpu9250I2c::read_regs(uint8_t reg, uint8_t* buf, std::size_t len) {
    return with_retry([&]() {
        if (::write(fd_, &reg, 1) != 1) return false;
        return ::read(fd_, buf, len) == static_cast<ssize_t>(len);
    });
}

bool Mpu9250I2c::init() {
    if (fd_ >= 0) {  // re-init: do not leak the previous descriptor
        ::close(fd_);
        fd_ = -1;
    }
    fd_ = ::open(dev_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        LOG_ERROR << "IMU: cannot open " << dev_path_ << ": "
                  << std::strerror(errno);
        return false;
    }
    const auto fail = [this]() {
        ::close(fd_);
        fd_ = -1;
        return false;
    };
    if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
        LOG_ERROR << "IMU: I2C_SLAVE 0x" << std::hex << address_ << " failed: "
                  << std::strerror(errno);
        return fail();
    }

    // Reset, wait for the PLL, then clock off the gyro PLL (stabler than the
    // internal oscillator).
    if (!write_reg(kRegPwrMgmt1, 0x80)) {
        LOG_ERROR << "IMU: device reset write failed";
        return fail();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!write_reg(kRegPwrMgmt1, 0x01)) {
        LOG_ERROR << "IMU: clock select write failed";
        return fail();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t who = 0;
    if (!read_regs(kRegWhoAmI, &who, 1)) {
        LOG_ERROR << "IMU: WHO_AM_I read failed";
        return fail();
    }
    if (who == kWhoAmIMpu9250 || who == kWhoAmIMpu9255) {
        LOG_INFO << "IMU: MPU-9250/9255 detected (WHO_AM_I 0x" << std::hex
                 << static_cast<int>(who) << ")";
    } else if (who == kWhoAmIMpu6500) {
        LOG_WARN << "IMU: WHO_AM_I 0x70 = MPU-6500 die (clone GY-9250 board?); "
                    "gyro+accel fine, continuing";
    } else {
        LOG_ERROR << "IMU: unexpected WHO_AM_I 0x" << std::hex
                  << static_cast<int>(who) << " -- wrong device/address?";
        return fail();
    }

    // 200 Hz internal rate, +-250 dps, +-4 g. DLPF is set to 20 Hz, NOT 41 Hz:
    // the 50 Hz control loop reads one snapshot per tick (4:1 decimation of
    // the 200 Hz stream), so per Nyquist anything above 25 Hz -- track and
    // sprocket vibration lives right there -- would alias into a DC yaw-rate
    // offset, the one error a compass-free estimator cannot forgive.
    const bool ok = write_reg(kRegSmplrtDiv, 0x04)      // 1 kHz / (1+4)
                    && write_reg(kRegConfig, 0x04)      // gyro DLPF 20 Hz
                    && write_reg(kRegGyroConfig, 0x00)  // +-250 dps
                    && write_reg(kRegAccelConfig, 0x08)   // +-4 g
                    && write_reg(kRegAccelConfig2, 0x04);  // accel DLPF 20 Hz
    if (!ok) {
        LOG_ERROR << "IMU: sensor configuration writes failed";
        return fail();
    }
    LOG_INFO << "IMU: configured (gyro +-250 dps, accel +-4 g, DLPF 20 Hz)";
    return true;
}

bool Mpu9250I2c::read(ImuSample& out) {
    // Burst ACCEL_XOUT_H..GYRO_ZOUT_L: accel xyz, temp, gyro xyz -- one
    // transaction keeps the axes coherent.
    uint8_t raw[14];
    if (!read_regs(kRegAccelXoutH, raw, sizeof(raw))) return false;

    const double accel_chip[3] = {mpu_accel_m_s2(be16(raw + 0)),
                                  mpu_accel_m_s2(be16(raw + 2)),
                                  mpu_accel_m_s2(be16(raw + 4))};
    const double gyro_chip[3] = {mpu_gyro_rad_s(be16(raw + 8)),
                                 mpu_gyro_rad_s(be16(raw + 10)),
                                 mpu_gyro_rad_s(be16(raw + 12))};

    // Chip frame -> body frame (x fwd, y left, z up) per mounting config.
    out.accel_x = mount_.sign[0] * accel_chip[mount_.axis[0]];
    out.accel_y = mount_.sign[1] * accel_chip[mount_.axis[1]];
    out.accel_z = mount_.sign[2] * accel_chip[mount_.axis[2]];
    out.gyro_z = mount_.sign[2] * gyro_chip[mount_.axis[2]];
    return true;
}

}  // namespace hal
}  // namespace backtrack
