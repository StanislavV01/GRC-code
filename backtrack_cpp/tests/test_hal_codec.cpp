#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "backtrack/hal/frame_codec.hpp"
#include "backtrack/hal/link_supervisor.hpp"
#include "test_harness.hpp"

using namespace backtrack::hal;

// ------------------------------------------------------------------ CRC16

TEST(crc16_ccitt_known_vector) {
    // "123456789" -> 0x29B1 is the canonical CCITT-FALSE check value.
    const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK_TRUE(crc16_ccitt(data, sizeof(data)) == 0x29B1);
}

TEST(be16_be32_roundtrip) {
    uint8_t buf[4];
    put_be16(buf, -1234);
    CHECK_TRUE(be16(buf) == -1234);
    put_be32(buf, -123456789);
    CHECK_TRUE(be32(buf) == -123456789);
}

// -------------------------------------------------------- operator frame

TEST(operator_frame_roundtrip) {
    OperatorFrame in;
    in.seq = 42;
    in.mode = OperatorMode::drive;
    in.v = 1.5;        // m/s
    in.omega = -0.75;  // rad/s

    uint8_t wire[kOpFrameSize];
    encode_operator_frame(in, wire);

    OperatorFrame out;
    CHECK_TRUE(decode_operator_frame(wire, kOpFrameSize, out));
    CHECK_TRUE(out.seq == 42);
    CHECK_TRUE(out.mode == OperatorMode::drive);
    CHECK_NEAR(out.v, 1.5, 1e-3);
    CHECK_NEAR(out.omega, -0.75, 1e-3);
}

TEST(operator_frame_estop_roundtrip) {
    OperatorFrame in;
    in.mode = OperatorMode::estop;
    uint8_t wire[kOpFrameSize];
    encode_operator_frame(in, wire);
    OperatorFrame out;
    CHECK_TRUE(decode_operator_frame(wire, kOpFrameSize, out));
    CHECK_TRUE(out.mode == OperatorMode::estop);
}

TEST(operator_frame_rejects_corruption) {
    OperatorFrame in;
    in.v = 1.0;
    uint8_t wire[kOpFrameSize];
    encode_operator_frame(in, wire);

    OperatorFrame out;
    uint8_t bad[kOpFrameSize];

    std::memcpy(bad, wire, kOpFrameSize);
    bad[6] ^= 0x01;  // flip a payload bit -> CRC must fail
    CHECK_TRUE(!decode_operator_frame(bad, kOpFrameSize, out));

    std::memcpy(bad, wire, kOpFrameSize);
    bad[0] = 0x00;  // broken sync
    CHECK_TRUE(!decode_operator_frame(bad, kOpFrameSize, out));

    std::memcpy(bad, wire, kOpFrameSize);
    bad[2] = 9;  // unknown version
    CHECK_TRUE(!decode_operator_frame(bad, kOpFrameSize, out));

    CHECK_TRUE(!decode_operator_frame(wire, kOpFrameSize - 1, out));
}

TEST(operator_frame_clamps_out_of_range) {
    OperatorFrame in;
    in.v = 100.0;  // 100 m/s -> 100000 mm/s overflows int16 -> clamp
    uint8_t wire[kOpFrameSize];
    encode_operator_frame(in, wire);
    OperatorFrame out;
    CHECK_TRUE(decode_operator_frame(wire, kOpFrameSize, out));
    CHECK_NEAR(out.v, 32.767, 1e-3);
}

TEST(parser_finds_frame_in_garbage_stream) {
    OperatorFrame in;
    in.seq = 7;
    in.v = 0.5;
    uint8_t wire[kOpFrameSize];
    encode_operator_frame(in, wire);

    std::vector<uint8_t> stream = {0x00, 0xFF, 0xA5, 0x13};  // noise + fake sync
    stream.insert(stream.end(), wire, wire + kOpFrameSize);
    stream.push_back(0xEE);  // trailing noise

    OperatorFrameParser parser;
    OperatorFrame out;
    int got = 0;
    for (uint8_t b : stream) {
        if (parser.push(b, out)) ++got;
    }
    CHECK_TRUE(got == 1);
    CHECK_TRUE(out.seq == 7);
    CHECK_NEAR(out.v, 0.5, 1e-3);
}

TEST(parser_recovers_after_corrupt_frame) {
    OperatorFrame in;
    in.seq = 1;
    uint8_t good[kOpFrameSize];
    encode_operator_frame(in, good);

    uint8_t corrupt[kOpFrameSize];
    std::memcpy(corrupt, good, kOpFrameSize);
    corrupt[8] ^= 0xFF;  // payload corrupted, sync intact

    OperatorFrameParser parser;
    OperatorFrame out;
    int got = 0;
    for (std::size_t i = 0; i < kOpFrameSize; ++i) {
        if (parser.push(corrupt[i], out)) ++got;
    }
    CHECK_TRUE(got == 0);
    in.seq = 2;
    encode_operator_frame(in, good);
    for (std::size_t i = 0; i < kOpFrameSize; ++i) {
        if (parser.push(good[i], out)) ++got;
    }
    CHECK_TRUE(got == 1);
    CHECK_TRUE(out.seq == 2);
}

TEST(parser_handles_back_to_back_frames) {
    OperatorFrameParser parser;
    OperatorFrame out;
    int got = 0;
    for (uint8_t s = 0; s < 5; ++s) {
        OperatorFrame in;
        in.seq = s;
        uint8_t wire[kOpFrameSize];
        encode_operator_frame(in, wire);
        for (std::size_t i = 0; i < kOpFrameSize; ++i) {
            if (parser.push(wire[i], out)) ++got;
        }
    }
    CHECK_TRUE(got == 5);
    CHECK_TRUE(out.seq == 4);
}

// -------------------------------------------------------------- VESC CAN

TEST(vesc_can_id_layout) {
    // extended id = (packet_type << 8) | controller_id
    CHECK_TRUE(vesc_can_id(VescPacket::set_rpm, 1) == 0x0301u);
    CHECK_TRUE(vesc_can_id(VescPacket::status, 2) == 0x0902u);
    CHECK_TRUE(vesc_is_status(0x0901u, 1));
    CHECK_TRUE(!vesc_is_status(0x0901u, 2));
    CHECK_TRUE(!vesc_is_status(0x0301u, 1));
}

TEST(vesc_set_rpm_payload) {
    uint8_t buf[4];
    encode_vesc_set_rpm(-1500.0, buf);
    CHECK_TRUE(be32(buf) == -1500);
}

TEST(vesc_set_rpm_rejects_non_finite) {
    // NaN/inf from broken upstream math must command ZERO, not garbage.
    uint8_t buf[4];
    encode_vesc_set_rpm(std::nan(""), buf);
    CHECK_TRUE(be32(buf) == 0);
    encode_vesc_set_rpm(std::numeric_limits<double>::infinity(), buf);
    CHECK_TRUE(be32(buf) == 0);
    encode_vesc_set_rpm(1e12, buf);  // finite but absurd -> clamped, not UB
    CHECK_TRUE(be32(buf) == 2000000000);
}

TEST(operator_frame_encodes_non_finite_as_zero) {
    OperatorFrame in;
    in.v = std::nan("");
    in.omega = std::numeric_limits<double>::infinity();
    uint8_t wire[kOpFrameSize];
    encode_operator_frame(in, wire);
    OperatorFrame out;
    CHECK_TRUE(decode_operator_frame(wire, kOpFrameSize, out));
    CHECK_NEAR(out.v, 0.0, 1e-9);
    CHECK_NEAR(out.omega, 0.0, 1e-9);
}

TEST(vesc_status_decode) {
    // ERPM = 3000, current = 12.5 A (125), duty = 0.42 (420)
    uint8_t data[8];
    put_be32(data, 3000);
    put_be16(data + 4, 125);
    put_be16(data + 6, 420);
    VescStatus st;
    CHECK_TRUE(decode_vesc_status(data, 8, st));
    CHECK_NEAR(st.erpm, 3000.0, 1e-9);
    CHECK_NEAR(st.current, 12.5, 1e-9);
    CHECK_NEAR(st.duty, 0.42, 1e-9);
    CHECK_TRUE(!decode_vesc_status(data, 7, st));
}

// ------------------------------------------------------------- MPU scales

TEST(mpu_scale_conversions) {
    // full-scale raw 32767 @ 131 LSB/dps = 250.13 dps = 4.36559 rad/s
    CHECK_NEAR(mpu_gyro_rad_s(32767), 4.36559, 1e-4);
    CHECK_NEAR(mpu_gyro_rad_s(0), 0.0, 1e-12);
    // 1 g = 8192 LSB @ +-4 g
    CHECK_NEAR(mpu_accel_m_s2(8192), 9.80665, 1e-6);
    CHECK_NEAR(mpu_accel_m_s2(-8192), -9.80665, 1e-6);
}

// --------------------------------------------------------- link supervisor

TEST(link_supervisor_starts_down) {
    backtrack::hal::LinkSupervisor sup(0.5);
    CHECK_TRUE(!sup.link_ok(0.0));
    CHECK_TRUE(!sup.link_ok(100.0));
}

TEST(link_supervisor_up_then_times_out) {
    backtrack::hal::LinkSupervisor sup(0.5);
    sup.frame_received(10.0);
    CHECK_TRUE(sup.link_ok(10.0));
    CHECK_TRUE(sup.link_ok(10.49));
    CHECK_TRUE(!sup.link_ok(10.51));
    sup.frame_received(11.0);  // link restored
    CHECK_TRUE(sup.link_ok(11.3));
}
