#ifndef BACKTRACK_HAL_RS485_LINK_HPP
#define BACKTRACK_HAL_RS485_LINK_HPP

// Operator link over the RS485 half of the Waveshare RS485 CAN HAT. Linux-only
// TU. The HAT's SP3485 does automatic TX/RX direction switching, and we only
// receive on the vehicle side, so this is a plain raw-UART reader.
//
// Default device on Raspberry Pi OS is /dev/serial0 (symlink to the primary
// UART); enable it and free it from the login console first (see
// docs/HARDWARE_BRINGUP.md).

#include <string>

#include "backtrack/hal/operator_link.hpp"

namespace backtrack {
namespace hal {

class Rs485Link : public IOperatorLink {
public:
    explicit Rs485Link(std::string dev_path, int baud = 115200);
    ~Rs485Link() override;

    Rs485Link(const Rs485Link&) = delete;
    Rs485Link& operator=(const Rs485Link&) = delete;

    bool init() override;
    bool poll(OperatorFrame& out) override;

private:
    std::string dev_path_;
    int baud_;
    int fd_{-1};
    OperatorFrameParser parser_;
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_RS485_LINK_HPP
