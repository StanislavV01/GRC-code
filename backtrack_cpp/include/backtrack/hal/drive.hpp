#ifndef BACKTRACK_HAL_DRIVE_HPP
#define BACKTRACK_HAL_DRIVE_HPP

// Track-drive hardware abstraction: two FSESC 75450 (VESC firmware) on a CAN
// bus, one per track. The HAL is a dumb transport -- ERPM in, ERPM out; all
// kinematics (twist <-> wheels <-> ERPM) stay in DiffDriveModel.

namespace backtrack {
namespace hal {

struct DriveStatus {
    double erpm_left{0.0};
    double erpm_right{0.0};
    double current_left{0.0};   // motor current [A]
    double current_right{0.0};  // motor current [A]
    double age_left_s{1e9};     // seconds since last status frame per side
    double age_right_s{1e9};
};

class IDrive {
public:
    virtual ~IDrive() = default;

    virtual bool init() = 0;

    // Command both sides. Sent every control tick -- doubles as the keepalive
    // for the VESC app timeout (motors stop by themselves if we go silent).
    virtual void set_erpm(double left_erpm, double right_erpm) = 0;

    // Drain pending status frames and return the latest per side. `now_s` is
    // the caller's monotonic clock, used to compute the ages.
    virtual DriveStatus poll(double now_s) = 0;
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_DRIVE_HPP
