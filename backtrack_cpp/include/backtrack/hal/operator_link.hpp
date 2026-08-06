#ifndef BACKTRACK_HAL_OPERATOR_LINK_HPP
#define BACKTRACK_HAL_OPERATOR_LINK_HPP

// Operator control-link abstraction (FR-1). The real transport is RS485 on the
// Waveshare RS485 CAN HAT; the frame format lives in frame_codec.hpp so it can
// be unit-tested without hardware.

#include "backtrack/hal/frame_codec.hpp"

namespace backtrack {
namespace hal {

class IOperatorLink {
public:
    virtual ~IOperatorLink() = default;

    virtual bool init() = 0;

    // Drain the receive buffer. Returns true if at least one complete valid
    // frame arrived since the last call; `out` then holds the newest one.
    virtual bool poll(OperatorFrame& out) = 0;
};

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_OPERATOR_LINK_HPP
