#ifndef BACKTRACK_HAL_DRONECAN_ESC_HPP
#define BACKTRACK_HAL_DRONECAN_ESC_HPP

// Odometry from DroneCAN esc.Status (uavcan.equipment.esc.Status, data type
// 1034) on a Linux SocketCAN interface -- the REAL odometry source on this
// vehicle. The drive ESCs' telemetry is published on the DroneCAN bus tagged by
// esc_index from one reporting node. Field-verified on the stand 2026-08-10
// (see docs/FIELD_NOTES.md): node 50, esc_index 0 = LEFT track,
// esc_index 2 = RIGHT track, +rpm = forward. The rpm field carries motor ERPM,
// exactly what DiffDriveModel expects.
//
// SAFETY: this HAL is READ-ONLY. set_erpm() is a deliberate no-op -- it never
// touches the CAN bus. Bring can0 up LISTEN-ONLY (deploy/can0-listen.service).
// Intended for observation / dry-run of the backtrack estimator against the live
// bus with zero risk of commanding the motors.

#include <cstddef>
#include <cstdint>
#include <string>

#include "backtrack/hal/drive.hpp"

namespace backtrack {
namespace hal {

class DroneCanEsc : public IDrive {
public:
    // node_id: the DroneCAN node publishing esc.Status (50 on this vehicle).
    // left_index / right_index: the esc_index values for the two tracks.
    DroneCanEsc(std::string iface, uint8_t node_id, int left_index,
                int right_index);
    ~DroneCanEsc() override;

    DroneCanEsc(const DroneCanEsc&) = delete;
    DroneCanEsc& operator=(const DroneCanEsc&) = delete;

    bool init() override;

    // No-op BY DESIGN: this HAL never transmits. Present only to satisfy IDrive
    // so observe/dry-run code can call it harmlessly.
    void set_erpm(double left_erpm, double right_erpm) override;

    DriveStatus poll(double now_s) override;

private:
    // DroneCAN v0 multi-frame transfer reassembly for one (node, type) stream.
    struct Reasm {
        uint8_t buf[64];
        std::size_t len{0};
        bool active{false};
        uint8_t transfer_id{0};
        uint8_t expect_toggle{0};
        int frames{0};
    };

    std::string iface_;
    uint8_t node_id_;
    int left_index_;
    int right_index_;
    int fd_{-1};

    Reasm reasm_{};
    DriveStatus latest_{};
    double last_left_rx_s_{-1e9};
    double last_right_rx_s_{-1e9};
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_DRONECAN_ESC_HPP
