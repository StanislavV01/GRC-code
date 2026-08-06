#ifndef BACKTRACK_HAL_MPU9250_I2C_HPP
#define BACKTRACK_HAL_MPU9250_I2C_HPP

// MPU-9250 (GY-9250 board) driver over Linux /dev/i2c-*. Linux-only TU.
//
// Configuration chosen for the backtrack estimator:
//   * gyro +-250 dps (max resolution -- the UGV yaws well under 250 dps)
//   * accel +-4 g
//   * DLPF 20 Hz on both (anti-alias for the 50 Hz loop; see init())
//   * internal sample rate 200 Hz; the 50 Hz control loop reads the latest
//
// The AK8963 magnetometer on the die is NOT used: the whole system design
// assumes the compass is unusable next to the 84 V FSESC power stage.

#include <cstdint>
#include <string>

#include "backtrack/hal/imu.hpp"

namespace backtrack {
namespace hal {

class Mpu9250I2c : public IImu {
public:
    Mpu9250I2c(std::string dev_path, int address = 0x68,
               MountConfig mount = MountConfig{});
    ~Mpu9250I2c() override;

    Mpu9250I2c(const Mpu9250I2c&) = delete;
    Mpu9250I2c& operator=(const Mpu9250I2c&) = delete;

    bool init() override;
    bool read(ImuSample& out) override;

private:
    bool write_reg(uint8_t reg, uint8_t value);
    bool read_regs(uint8_t reg, uint8_t* buf, std::size_t len);

    std::string dev_path_;
    int address_;
    MountConfig mount_;
    int fd_{-1};
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_MPU9250_I2C_HPP
