#ifndef BACKTRACK_HAL_VESC_SOCKETCAN_HPP
#define BACKTRACK_HAL_VESC_SOCKETCAN_HPP

// Two VESC (FSESC 75450) track controllers on a Linux SocketCAN interface
// (Waveshare RS485 CAN HAT -> MCP2515 -> can0). Linux-only TU.
//
// Prerequisites on the vehicle (see docs/HARDWARE_BRINGUP.md):
//   * dtoverlay=mcp2515-can0 in config.txt, `ip link set can0 up type can
//     bitrate 500000` (deploy/can0.service does this at boot)
//   * VESC Tool: CAN IDs set (left/right), CAN status messages enabled at
//     >= 50 Hz, CAN baud 500k, app timeout enabled (motors auto-stop if the
//     CM4 goes silent).

#include <cstdint>
#include <string>

#include "backtrack/hal/drive.hpp"

namespace backtrack {
namespace hal {

class VescSocketCan : public IDrive {
public:
    VescSocketCan(std::string iface, uint8_t left_id, uint8_t right_id);
    ~VescSocketCan() override;

    VescSocketCan(const VescSocketCan&) = delete;
    VescSocketCan& operator=(const VescSocketCan&) = delete;

    bool init() override;
    void set_erpm(double left_erpm, double right_erpm) override;
    DriveStatus poll(double now_s) override;

private:
    void send_set_rpm(uint8_t controller_id, double erpm);

    std::string iface_;
    uint8_t left_id_;
    uint8_t right_id_;
    int fd_{-1};

    DriveStatus latest_{};
    double last_left_rx_s_{-1e9};
    double last_right_rx_s_{-1e9};
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_VESC_SOCKETCAN_HPP
