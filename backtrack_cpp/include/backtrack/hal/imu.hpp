#ifndef BACKTRACK_HAL_IMU_HPP
#define BACKTRACK_HAL_IMU_HPP

// IMU hardware abstraction. One implementation per chip:
//   * Mpu9250I2c  -- GY-9250 board (MPU-9250/9255/6500 die) over /dev/i2c-*
//   * Bno055I2c   -- future: Bosch BNO055 in raw (non-fusion) mode; the fusion
//                    modes need the magnetometer, which is unusable next to the
//                    84 V FSESC power stage, so we only take gyro+accel from it.
//
// The estimator consumes body-frame axes: x forward, y left, z up. Board
// mounting is corrected by MountConfig before samples leave the driver.

namespace backtrack {
namespace hal {

struct ImuSample {
    double gyro_z{0.0};   // body yaw rate [rad/s]
    double accel_x{0.0};  // body forward specific force [m/s^2]
    double accel_y{0.0};  // body lateral specific force [m/s^2]
    double accel_z{0.0};  // body vertical specific force [m/s^2] (~ +g at rest)
};

// body axis i (0=x fwd, 1=y left, 2=z up) = sign[i] * chip_axis[axis[i]]
struct MountConfig {
    int axis[3] = {0, 1, 2};
    double sign[3] = {1.0, 1.0, 1.0};
};

class IImu {
public:
    virtual ~IImu() = default;

    // Open the device and configure it. Returns false on failure (missing bus,
    // wrong WHO_AM_I, ...); the reason is logged by the implementation.
    virtual bool init() = 0;

    // Read one sample. Returns false on an I/O error (sample untouched).
    virtual bool read(ImuSample& out) = 0;
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_IMU_HPP
