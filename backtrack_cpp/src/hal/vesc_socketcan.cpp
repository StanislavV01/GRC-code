#include "backtrack/hal/vesc_socketcan.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "backtrack/hal/frame_codec.hpp"
#include "backtrack/log.hpp"

namespace backtrack {
namespace hal {

VescSocketCan::VescSocketCan(std::string iface, uint8_t left_id,
                             uint8_t right_id)
    : iface_(std::move(iface)), left_id_(left_id), right_id_(right_id) {}

VescSocketCan::~VescSocketCan() {
    if (fd_ >= 0) ::close(fd_);
}

namespace {
// After this long without a status frame the last ERPM/current values are
// lies; report zeros so the dead-reckoning does not integrate phantom motion.
constexpr double kStatusStaleS = 0.25;
}  // namespace

bool VescSocketCan::init() {
    if (fd_ >= 0) {  // re-init: do not leak the previous descriptor
        ::close(fd_);
        fd_ = -1;
    }
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        LOG_ERROR << "CAN: socket() failed: " << std::strerror(errno);
        return false;
    }
    const auto fail = [this]() {
        ::close(fd_);
        fd_ = -1;
        return false;
    };

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        LOG_ERROR << "CAN: interface '" << iface_ << "' not found: "
                  << std::strerror(errno)
                  << " (is can0 up? see deploy/can0.service)";
        return fail();
    }

    // Only the two STATUS streams we consume reach userspace.
    struct can_filter filters[2];
    filters[0].can_id = vesc_can_id(VescPacket::status, left_id_) | CAN_EFF_FLAG;
    filters[0].can_mask = CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    filters[1].can_id = vesc_can_id(VescPacket::status, right_id_) | CAN_EFF_FLAG;
    filters[1].can_mask = CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters,
                     sizeof(filters)) < 0) {
        LOG_ERROR << "CAN: setsockopt(CAN_RAW_FILTER) failed: "
                  << std::strerror(errno);
        return fail();
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) <
        0) {
        LOG_ERROR << "CAN: bind(" << iface_ << ") failed: "
                  << std::strerror(errno);
        return fail();
    }

    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        LOG_ERROR << "CAN: O_NONBLOCK failed: " << std::strerror(errno);
        return fail();
    }

    LOG_INFO << "CAN: " << iface_ << " ready (left VESC id "
             << static_cast<int>(left_id_) << ", right id "
             << static_cast<int>(right_id_) << ")";
    return true;
}

void VescSocketCan::send_set_rpm(uint8_t controller_id, double erpm) {
    struct can_frame frame {};
    frame.can_id = vesc_can_id(VescPacket::set_rpm, controller_id) | CAN_EFF_FLAG;
    frame.can_dlc = 4;
    encode_vesc_set_rpm(erpm, frame.data);
    // Non-blocking best-effort: a full TX queue drops this tick's command; the
    // next tick (20 ms) resends. The VESC app timeout covers a dead bus.
    if (::write(fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        LOG_DEBUG << "CAN: SET_RPM tx to id " << static_cast<int>(controller_id)
                  << " dropped: " << std::strerror(errno);
    }
}

void VescSocketCan::set_erpm(double left_erpm, double right_erpm) {
    send_set_rpm(left_id_, left_erpm);
    send_set_rpm(right_id_, right_erpm);
}

DriveStatus VescSocketCan::poll(double now_s) {
    struct can_frame frame;
    while (true) {
        const ssize_t n = ::read(fd_, &frame, sizeof(frame));
        if (n != sizeof(frame)) break;  // EAGAIN or short read: queue drained
        if (!(frame.can_id & CAN_EFF_FLAG)) continue;
        if (frame.can_id & CAN_RTR_FLAG) continue;  // no payload
        const uint32_t ext_id = frame.can_id & CAN_EFF_MASK;

        VescStatus st;
        if (!decode_vesc_status(frame.data, frame.can_dlc, st)) continue;
        if (vesc_is_status(ext_id, left_id_)) {
            latest_.erpm_left = st.erpm;
            latest_.current_left = st.current;
            last_left_rx_s_ = now_s;
        } else if (vesc_is_status(ext_id, right_id_)) {
            latest_.erpm_right = st.erpm;
            latest_.current_right = st.current;
            last_right_rx_s_ = now_s;
        }
    }
    latest_.age_left_s = now_s - last_left_rx_s_;
    latest_.age_right_s = now_s - last_right_rx_s_;

    // Fail safe, not stale: a side that has gone quiet reports zero motion
    // instead of endlessly repeating its last reading.
    DriveStatus out = latest_;
    if (out.age_left_s > kStatusStaleS) {
        out.erpm_left = 0.0;
        out.current_left = 0.0;
    }
    if (out.age_right_s > kStatusStaleS) {
        out.erpm_right = 0.0;
        out.current_right = 0.0;
    }
    return out;
}

}  // namespace hal
}  // namespace backtrack
