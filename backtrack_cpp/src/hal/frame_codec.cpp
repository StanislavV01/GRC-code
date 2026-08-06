#include "backtrack/hal/frame_codec.hpp"

#include <cmath>
#include <cstring>

namespace backtrack {
namespace hal {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

int16_t clamp_i16(double v) {
    if (!std::isfinite(v)) return 0;  // NaN/inf must never reach the wire
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(std::lround(v));
}

}  // namespace

// ---------------------------------------------------------------- CRC / bytes

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

int16_t be16(const uint8_t* p) {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

int32_t be32(const uint8_t* p) {
    return static_cast<int32_t>((static_cast<uint32_t>(p[0]) << 24) |
                                (static_cast<uint32_t>(p[1]) << 16) |
                                (static_cast<uint32_t>(p[2]) << 8) |
                                static_cast<uint32_t>(p[3]));
}

void put_be16(uint8_t* p, int16_t v) {
    const uint16_t u = static_cast<uint16_t>(v);
    p[0] = static_cast<uint8_t>(u >> 8);
    p[1] = static_cast<uint8_t>(u & 0xFF);
}

void put_be32(uint8_t* p, int32_t v) {
    const uint32_t u = static_cast<uint32_t>(v);
    p[0] = static_cast<uint8_t>(u >> 24);
    p[1] = static_cast<uint8_t>((u >> 16) & 0xFF);
    p[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
    p[3] = static_cast<uint8_t>(u & 0xFF);
}

// ------------------------------------------------------------- operator frame

void encode_operator_frame(const OperatorFrame& f, uint8_t out[kOpFrameSize]) {
    out[0] = kOpSync0;
    out[1] = kOpSync1;
    out[2] = kOpVersion;
    out[3] = f.seq;
    out[4] = static_cast<uint8_t>(f.mode);
    put_be16(out + 5, clamp_i16(f.v * 1000.0));        // m/s -> mm/s
    put_be16(out + 7, clamp_i16(f.omega * 1000.0));    // rad/s -> mrad/s
    out[9] = 0;                                        // flags reserved
    const uint16_t crc = crc16_ccitt(out, 10);
    out[10] = static_cast<uint8_t>(crc >> 8);
    out[11] = static_cast<uint8_t>(crc & 0xFF);
}

bool decode_operator_frame(const uint8_t* buf, std::size_t len,
                           OperatorFrame& out) {
    if (len != kOpFrameSize) return false;
    if (buf[0] != kOpSync0 || buf[1] != kOpSync1) return false;
    if (buf[2] != kOpVersion) return false;
    const uint16_t want = crc16_ccitt(buf, 10);
    const uint16_t got =
        static_cast<uint16_t>((static_cast<uint16_t>(buf[10]) << 8) | buf[11]);
    if (want != got) return false;
    if (buf[4] > static_cast<uint8_t>(OperatorMode::estop)) return false;

    out.seq = buf[3];
    out.mode = static_cast<OperatorMode>(buf[4]);
    out.v = be16(buf + 5) / 1000.0;
    out.omega = be16(buf + 7) / 1000.0;
    return true;
}

bool OperatorFrameParser::push(uint8_t byte, OperatorFrame& out) {
    // Sync hunt: nothing buffered until the two-byte header matches.
    if (have_ == 0) {
        if (byte != kOpSync0) return false;
        buf_[have_++] = byte;
        return false;
    }
    if (have_ == 1) {
        if (byte != kOpSync1) {
            have_ = (byte == kOpSync0) ? 1 : 0;  // 0xA5 0xA5 0x5A... case
            buf_[0] = kOpSync0;
            return false;
        }
        buf_[have_++] = byte;
        return false;
    }

    buf_[have_++] = byte;
    if (have_ < kOpFrameSize) return false;

    const bool ok = decode_operator_frame(buf_, kOpFrameSize, out);
    if (ok) {
        have_ = 0;
        return true;
    }
    // CRC/format failure: resync on the next header candidate inside the
    // buffer (skip byte 0, look for the pair).
    std::size_t next = kOpFrameSize;
    for (std::size_t i = 1; i + 1 < kOpFrameSize; ++i) {
        if (buf_[i] == kOpSync0 && buf_[i + 1] == kOpSync1) {
            next = i;
            break;
        }
    }
    if (next == kOpFrameSize && buf_[kOpFrameSize - 1] == kOpSync0) {
        next = kOpFrameSize - 1;  // trailing lone 0xA5 may start a frame
    }
    if (next < kOpFrameSize) {
        std::memmove(buf_, buf_ + next, kOpFrameSize - next);
        have_ = kOpFrameSize - next;
    } else {
        have_ = 0;
    }
    return false;
}

// ------------------------------------------------------------------- VESC CAN

uint32_t vesc_can_id(VescPacket type, uint8_t controller_id) {
    return (static_cast<uint32_t>(type) << 8) | controller_id;
}

bool vesc_is_status(uint32_t ext_id, uint8_t controller_id) {
    return ext_id == vesc_can_id(VescPacket::status, controller_id);
}

void encode_vesc_set_rpm(double erpm, uint8_t out[4]) {
    // A non-finite command means upstream math broke; the only safe motor
    // command is zero (llround(NaN) is UB and would put garbage on the bus).
    double clamped = std::isfinite(erpm) ? erpm : 0.0;
    if (clamped > 2.0e9) clamped = 2.0e9;
    if (clamped < -2.0e9) clamped = -2.0e9;
    put_be32(out, static_cast<int32_t>(std::llround(clamped)));
}

bool decode_vesc_status(const uint8_t* data, std::size_t len, VescStatus& out) {
    if (len < 8) return false;
    out.erpm = static_cast<double>(be32(data));
    out.current = be16(data + 4) / 10.0;
    out.duty = be16(data + 6) / 1000.0;
    return true;
}

// ------------------------------------------------------------ MPU-9250 scales

double mpu_gyro_rad_s(int16_t raw) {
    return (raw / kMpuGyroLsbPerDps) * kPi / 180.0;
}

double mpu_accel_m_s2(int16_t raw) {
    return (raw / kMpuAccelLsbPerG) * kGravity;
}

}  // namespace hal
}  // namespace backtrack
