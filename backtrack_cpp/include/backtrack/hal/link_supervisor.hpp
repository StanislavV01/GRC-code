#ifndef BACKTRACK_HAL_LINK_SUPERVISOR_HPP
#define BACKTRACK_HAL_LINK_SUPERVISOR_HPP

// Link-loss detection (FR-1): the link is UP while valid operator frames keep
// arriving, DOWN after `timeout_s` of silence. Starts DOWN -- on boot, before
// the first valid frame, there is no link (and no trail to backtrack anyway).

namespace backtrack {
namespace hal {

class LinkSupervisor {
public:
    explicit LinkSupervisor(double timeout_s) : timeout_s_(timeout_s) {}

    void frame_received(double now_s) {
        last_frame_s_ = now_s;
        ever_ = true;
    }

    bool link_ok(double now_s) const {
        return ever_ && (now_s - last_frame_s_) <= timeout_s_;
    }

    // Seconds since the last valid frame (very large before the first one).
    double silence_s(double now_s) const {
        return ever_ ? now_s - last_frame_s_ : 1e9;
    }

private:
    double timeout_s_;
    double last_frame_s_{0.0};
    bool ever_{false};
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_LINK_SUPERVISOR_HPP
