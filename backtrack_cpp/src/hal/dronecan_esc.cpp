#include "backtrack/hal/dronecan_esc.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "backtrack/log.hpp"

namespace backtrack {
namespace hal {

namespace {

// ---- UAVCAN/DroneCAN v0 CAN ID (29-bit extended) ---------------------------
struct DecodedId {
    bool service;
    uint16_t type_id;
    uint8_t src;
};

DecodedId decode_id(uint32_t ext) {
    DecodedId d;
    d.service = (ext >> 7) & 1;
    d.src = ext & 0x7F;
    d.type_id = d.service ? static_cast<uint16_t>((ext >> 16) & 0xFF)
                          : static_cast<uint16_t>((ext >> 8) & 0xFFFF);
    return d;
}

float half_to_float(uint16_t h) {
    const uint32_t sign = (h & 0x8000u) << 16;
    uint32_t exp = (h >> 10) & 0x1F;
    uint32_t mant = h & 0x3FF;
    uint32_t out;
    if (exp == 0) {
        if (mant == 0) { out = sign; }
        else {
            exp = 127 - 15 + 1;
            while (!(mant & 0x400)) { mant <<= 1; --exp; }
            mant &= 0x3FF;
            out = sign | (exp << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        out = sign | 0x7F800000u | (mant << 13);
    } else {
        out = sign | ((exp - 15 + 127) << 23) | (mant << 13);
    }
    float f;
    std::memcpy(&f, &out, 4);
    return f;
}

constexpr uint16_t kEscStatusType = 1034;

}  // namespace

DroneCanEsc::DroneCanEsc(std::string iface, uint8_t node_id, int left_index,
                         int right_index)
    : iface_(std::move(iface)),
      node_id_(node_id),
      left_index_(left_index),
      right_index_(right_index) {}

DroneCanEsc::~DroneCanEsc() {
    if (fd_ >= 0) ::close(fd_);
}

bool DroneCanEsc::init() {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        LOG_ERROR << "DroneCanEsc: socket(PF_CAN) failed: " << std::strerror(errno);
        return false;
    }
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        LOG_ERROR << "DroneCanEsc: interface " << iface_
                  << " not found (ip link set " << iface_
                  << " up type can ... listen-only on?)";
        return false;
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOG_ERROR << "DroneCanEsc: bind failed: " << std::strerror(errno);
        return false;
    }
    const int fl = ::fcntl(fd_, F_GETFL, 0);
    if (fl < 0 || ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK) < 0) {
        LOG_ERROR << "DroneCanEsc: fcntl(O_NONBLOCK) failed";
        return false;
    }
    LOG_INFO << "DroneCanEsc: reading esc.Status from node " << int(node_id_)
             << " (left idx " << left_index_ << ", right idx " << right_index_
             << ") on " << iface_ << " -- LISTEN-ONLY, never transmits";
    return true;
}

void DroneCanEsc::set_erpm(double, double) {
    // Intentionally does nothing: read-only odometry HAL. Never writes to CAN.
}

DriveStatus DroneCanEsc::poll(double now_s) {
    if (fd_ < 0) return latest_;

    struct can_frame fr;
    while (::read(fd_, &fr, sizeof(fr)) == static_cast<ssize_t>(sizeof(fr))) {
        if (!(fr.can_id & CAN_EFF_FLAG)) continue;  // 11-bit = not DroneCAN
        const DecodedId d = decode_id(fr.can_id & CAN_EFF_MASK);
        if (d.service || d.type_id != kEscStatusType || d.src != node_id_) continue;
        if (fr.can_dlc < 1) continue;

        // --- DroneCAN v0 transfer reassembly (transfer_id + toggle) ---
        const uint8_t tail = fr.data[fr.can_dlc - 1];
        const bool sot = tail & 0x80;
        const bool eot = tail & 0x40;
        const uint8_t tog = (tail >> 5) & 1;
        const uint8_t tid = tail & 0x1F;
        const int payload = fr.can_dlc - 1;
        const uint8_t* body = nullptr;
        std::size_t body_len = 0;

        if (sot) {
            reasm_.active = true;
            reasm_.transfer_id = tid;
            reasm_.expect_toggle = 1;
            reasm_.frames = 1;
            reasm_.len = 0;
            if (payload > 0 &&
                static_cast<std::size_t>(payload) <= sizeof(reasm_.buf)) {
                std::memcpy(reasm_.buf, fr.data, payload);
                reasm_.len = payload;
            }
            if (eot) {  // single-frame -> no CRC prefix
                reasm_.active = false;
                body = reasm_.buf;
                body_len = reasm_.len;
            }
        } else if (reasm_.active && tid == reasm_.transfer_id &&
                   tog == reasm_.expect_toggle) {
            if (reasm_.len + payload <= sizeof(reasm_.buf)) {
                std::memcpy(reasm_.buf + reasm_.len, fr.data, payload);
                reasm_.len += payload;
            }
            reasm_.expect_toggle ^= 1;
            ++reasm_.frames;
            if (eot) {
                reasm_.active = false;
                if (reasm_.len >= 2) {  // multi-frame: strip 2-byte transfer CRC
                    body = reasm_.buf + 2;
                    body_len = reasm_.len - 2;
                } else {
                    body = reasm_.buf;
                    body_len = reasm_.len;
                }
            }
        } else {
            reasm_.active = false;  // interleaved / lost frame -> drop transfer
        }

        if (!body || body_len < 14) continue;  // need a full esc.Status

        // --- decode esc.Status (little-endian bit order) ---
        uint16_t v16, c16;
        std::memcpy(&v16, body + 4, 2);  // voltage f16
        std::memcpy(&c16, body + 6, 2);  // current f16
        int32_t rpm = static_cast<int32_t>(body[10])
                      | (static_cast<int32_t>(body[11]) << 8)
                      | (static_cast<int32_t>(body[12] & 0x03) << 16);
        if (rpm & 0x20000) rpm -= 0x40000;   // sign-extend int18
        const int esc_index = (body[13] >> 1) & 0x1F;
        const float current = half_to_float(c16);
        (void)v16;

        if (esc_index == left_index_) {
            latest_.erpm_left = rpm;
            latest_.current_left = current;
            last_left_rx_s_ = now_s;
        } else if (esc_index == right_index_) {
            latest_.erpm_right = rpm;
            latest_.current_right = current;
            last_right_rx_s_ = now_s;
        }
    }

    latest_.age_left_s = now_s - last_left_rx_s_;
    latest_.age_right_s = now_s - last_right_rx_s_;
    return latest_;
}

}  // namespace hal
}  // namespace backtrack
