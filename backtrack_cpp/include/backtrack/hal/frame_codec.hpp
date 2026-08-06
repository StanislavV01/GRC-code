#ifndef BACKTRACK_HAL_FRAME_CODEC_HPP
#define BACKTRACK_HAL_FRAME_CODEC_HPP

// Pure encode/decode logic for both wire protocols -- no I/O, no Linux headers,
// fully unit-testable on any host.
//
//   1. Operator link frame (RS485, our own protocol, 12 bytes):
//        [0] 0xA5  [1] 0x5A          sync
//        [2] version (=1)
//        [3] seq                     wraps 0..255, for drop diagnostics
//        [4] mode                    0 = drive, 1 = e-stop
//        [5..6] v      int16 BE      commanded speed [mm/s], + forward
//        [7..8] omega  int16 BE      commanded yaw rate [mrad/s], + CCW
//        [9] flags (reserved, 0)
//        [10..11] CRC16-CCITT BE     over bytes 0..9
//      The console sends ~20 Hz; silence > timeout = LINK LOST (FR-1).
//
//   2. VESC CAN frames (FSESC 75450 stock firmware):
//        extended ID = (packet_type << 8) | controller_id
//        CAN_PACKET_SET_RPM (3): payload int32 BE = ERPM
//        CAN_PACKET_STATUS  (9): int32 BE ERPM, int16 BE current*10 [A],
//                                int16 BE duty*1000
//
//   3. MPU-9250 raw register -> SI conversions (gyro +-250 dps, accel +-4 g).

#include <cstddef>
#include <cstdint>

namespace backtrack {
namespace hal {

// ---------------------------------------------------------------- CRC / bytes

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len);  // poly 0x1021/0xFFFF

int16_t be16(const uint8_t* p);
int32_t be32(const uint8_t* p);
void put_be16(uint8_t* p, int16_t v);
void put_be32(uint8_t* p, int32_t v);

// ------------------------------------------------------------- operator frame

constexpr uint8_t kOpSync0 = 0xA5;
constexpr uint8_t kOpSync1 = 0x5A;
constexpr uint8_t kOpVersion = 1;
constexpr std::size_t kOpFrameSize = 12;

enum class OperatorMode : uint8_t { drive = 0, estop = 1 };

struct OperatorFrame {
    uint8_t seq{0};
    OperatorMode mode{OperatorMode::drive};
    double v{0.0};      // [m/s]
    double omega{0.0};  // [rad/s]
};

void encode_operator_frame(const OperatorFrame& f, uint8_t out[kOpFrameSize]);

// Strict decode of exactly kOpFrameSize bytes: sync + version + CRC checked.
bool decode_operator_frame(const uint8_t* buf, std::size_t len,
                           OperatorFrame& out);

// Byte-stream reassembler for the UART: tolerates garbage between frames and
// resynchronizes on the 0xA5 0x5A header after a CRC failure.
class OperatorFrameParser {
public:
    // Feed one received byte; returns true when it completes a valid frame
    // (written to `out`).
    bool push(uint8_t byte, OperatorFrame& out);

private:
    uint8_t buf_[kOpFrameSize] = {};
    std::size_t have_{0};
};

// ------------------------------------------------------------------- VESC CAN

enum class VescPacket : uint8_t {
    set_duty = 0,
    set_current = 1,
    set_rpm = 3,
    status = 9,
};

// 29-bit extended CAN id (without the socketcan CAN_EFF_FLAG bit).
uint32_t vesc_can_id(VescPacket type, uint8_t controller_id);

// True if `ext_id` is CAN_PACKET_STATUS from `controller_id`.
bool vesc_is_status(uint32_t ext_id, uint8_t controller_id);

void encode_vesc_set_rpm(double erpm, uint8_t out[4]);

struct VescStatus {
    double erpm{0.0};
    double current{0.0};  // [A]
    double duty{0.0};     // -1..1
};

bool decode_vesc_status(const uint8_t* data, std::size_t len, VescStatus& out);

// ------------------------------------------------------------ MPU-9250 scales

// Full-scale settings this driver configures: gyro +-250 dps, accel +-4 g.
constexpr double kMpuGyroLsbPerDps = 131.0;
constexpr double kMpuAccelLsbPerG = 8192.0;

double mpu_gyro_rad_s(int16_t raw);
double mpu_accel_m_s2(int16_t raw);

}  // namespace hal
}  // namespace backtrack

#endif  // BACKTRACK_HAL_FRAME_CODEC_HPP
