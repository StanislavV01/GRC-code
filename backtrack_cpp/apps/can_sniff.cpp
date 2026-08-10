// Passive DroneCAN bus reconnaissance for the first hookup to the live
// vehicle. Run with can0 in LISTEN-ONLY mode (see docs/CAN_TEST_PLAN.md):
// the tool only reads -- it cannot ACK, inject, or disturb the bus even if
// it wanted to.
//
// What it answers:
//   * which nodes are alive (source node IDs + NodeStatus rate)
//   * which message types fly and how often (esc.RawCommand = who commands,
//     esc.Status = our odometry source, BatteryInfo = wattmeter, Fix2 = GPS)
//   * live RPM / voltage / current decoded from esc.Status
//   * the commanded values the autopilot sends via esc.RawCommand
//   * whether any VESC-native (non-DroneCAN) frames are present
//
// Transfer reassembly follows DroneCAN/UAVCAN v0 properly: frames are grouped
// by (source node, data type) and validated by transfer_id + toggle, so on a
// busy bus transfers from different messages are never spliced together.
// Payload bit order is little-endian (LSB first), matching the v0 spec.
//
// Usage:
//   can_sniff [--iface can0] [--seconds N] [--esc-csv PATH] [--cmd-csv PATH]

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

namespace {

volatile std::sig_atomic_t g_stop = 0;
void handle_signal(int) { g_stop = 1; }

// Optional CSV sinks (unthrottled, so low-rate wheel ESCs are never dropped).
std::FILE* g_esc_csv = nullptr;  // decoded esc.Status rows
std::FILE* g_cmd_csv = nullptr;  // decoded esc.RawCommand values
double g_t0 = 0.0;

double monotonic_s() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + ts.tv_nsec * 1e-9;
}

// ---- UAVCAN/DroneCAN v0 CAN ID layout (29-bit extended) --------------------
// message : [28:24] priority | [23:8] data type id | [7]=0 | [6:0] src node
// service : [28:24] priority | [23:16] svc type | [15] req | [14:8] dst |
//           [7]=1 | [6:0] src node

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

const char* type_name(bool service, uint16_t id) {
    if (service) {
        switch (id) {
            case 1: return "svc GetNodeInfo";
            case 11: return "svc GetParamInfo/ParamGetSet";
            default: return "svc ?";
        }
    }
    switch (id) {
        case 341: return "NodeStatus";
        case 1030: return "esc.RawCommand   <-- хто керує моторами";
        case 1031: return "esc.RPMCommand";
        case 1034: return "esc.Status       <-- НАША ОДОМЕТРІЯ";
        case 1090: return "actuator.Status";
        case 1091: return "power.CircuitStatus (ватметр)";
        case 1092: return "power.BatteryInfo (ватметр)";
        case 1060: return "gnss.Fix";
        case 1063: return "gnss.Fix2 (GPS)";
        case 1062: return "gnss.Auxiliary";
        case 1002: return "ahrs.MagneticFieldStrength";
        case 20007: return "actuator/att (vendor)";
        default: return "?";
    }
}

// float16 -> float (IEEE 754 half; payload stores it little-endian)
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

// Read `nbits` starting at `bit_off`, little-endian (LSB first) -- the v0 order.
uint32_t read_bits_lsb(const uint8_t* buf, std::size_t bit_off, int nbits) {
    uint32_t v = 0;
    for (int i = 0; i < nbits; ++i) {
        const std::size_t b = bit_off + i;
        v |= static_cast<uint32_t>((buf[b >> 3] >> (b & 7)) & 1) << i;
    }
    return v;
}

// ---- DroneCAN v0 multi-frame transfer reassembly, keyed per (node,type) ----
// Validates transfer_id + toggle so interleaved/lost frames on a busy bus are
// dropped rather than spliced into garbage.
struct Transfer {
    uint8_t buf[128];
    std::size_t len{0};
    bool active{false};
    uint8_t transfer_id{0};
    uint8_t expect_toggle{0};
    int frames{0};
};

// Returns true + (out,out_len) when a full transfer completes. For multi-frame
// transfers the 2-byte leading transfer CRC is stripped.
bool feed_frame(Transfer& t, const uint8_t* data, int dlc, const uint8_t** out,
                std::size_t* out_len) {
    if (dlc < 1) return false;
    const uint8_t tail = data[dlc - 1];
    const bool sot = tail & 0x80;
    const bool eot = tail & 0x40;
    const uint8_t tog = (tail >> 5) & 1;
    const uint8_t tid = tail & 0x1F;
    const int payload = dlc - 1;

    if (sot) {  // start of a new transfer
        t.active = true;
        t.transfer_id = tid;
        t.expect_toggle = 1;
        t.frames = 1;
        t.len = 0;
        if (payload > 0 && static_cast<std::size_t>(payload) <= sizeof(t.buf)) {
            std::memcpy(t.buf, data, payload);
            t.len = payload;
        }
        if (eot) {  // single-frame transfer -> no CRC prefix
            t.active = false;
            *out = t.buf;
            *out_len = t.len;
            return true;
        }
        return false;
    }
    // continuation frame: must match the in-progress transfer exactly
    if (!t.active || tid != t.transfer_id || tog != t.expect_toggle) {
        t.active = false;  // interleaved or dropped frame -> abandon
        return false;
    }
    if (t.len + payload <= sizeof(t.buf)) {
        std::memcpy(t.buf + t.len, data, payload);
        t.len += payload;
    }
    t.expect_toggle ^= 1;
    ++t.frames;
    if (eot) {
        t.active = false;
        if (t.len >= 2) { *out = t.buf + 2; *out_len = t.len - 2; }  // strip CRC
        else { *out = t.buf; *out_len = t.len; }
        return true;
    }
    return false;
}

// uavcan.equipment.esc.Status (1034): error_count u32 | voltage f16 |
// current f16 | temperature f16 | rpm int18 | power_rating_pct u7 |
// esc_index u5  == 110 bits == 14 bytes, little-endian bit order.
struct EscStatus {
    bool ok{false};
    int32_t rpm{0};
    float voltage{0.f};
    float current{0.f};
    uint32_t err{0};
    int esc_index{0};
};

EscStatus decode_esc_status(const uint8_t* p, std::size_t n) {
    EscStatus s;
    if (n < 14) return s;
    uint16_t v16, c16;
    std::memcpy(&s.err, p, 4);    // bytes 0..3
    std::memcpy(&v16, p + 4, 2);  // bytes 4..5 voltage
    std::memcpy(&c16, p + 6, 2);  // bytes 6..7 current
    // bytes 8..9 temperature (unused)
    // rpm int18 at bit 80 (byte 10), LSB first
    int32_t rpm = static_cast<int32_t>(p[10])
                  | (static_cast<int32_t>(p[11]) << 8)
                  | (static_cast<int32_t>(p[12] & 0x03) << 16);
    if (rpm & 0x20000) rpm -= 0x40000;  // sign-extend 18 bits
    s.rpm = rpm;
    s.voltage = half_to_float(v16);
    s.current = half_to_float(c16);
    s.esc_index = (p[13] >> 1) & 0x1F;  // bits 105..109
    s.ok = true;
    return s;
}

void print_esc_status_line(uint8_t src, const EscStatus& s) {
    std::printf("  esc.Status node %-3u  rpm %-7d  %6.1f V  %6.1f A  "
                "err %" PRIu32 "  (idx %d)\n",
                src, s.rpm, s.voltage, s.current, s.err, s.esc_index);
}

struct Key {
    bool service;
    uint16_t type_id;
    uint8_t src;
    bool operator<(const Key& o) const {
        if (service != o.service) return service < o.service;
        if (type_id != o.type_id) return type_id < o.type_id;
        return src < o.src;
    }
};

}  // namespace

int main(int argc, char** argv) {
    std::string iface = "can0";
    double seconds = 0.0;
    std::string esc_path, cmd_path;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--iface" && i + 1 < argc) iface = argv[++i];
        else if (arg == "--seconds" && i + 1 < argc) seconds = std::atof(argv[++i]);
        else if (arg == "--esc-csv" && i + 1 < argc) esc_path = argv[++i];
        else if (arg == "--cmd-csv" && i + 1 < argc) cmd_path = argv[++i];
        else {
            std::fprintf(stderr,
                         "usage: can_sniff [--iface can0] [--seconds N] "
                         "[--esc-csv PATH] [--cmd-csv PATH]\n");
            return 1;
        }
    }

    const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) { std::perror("socket(PF_CAN)"); return 1; }
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        std::fprintf(stderr, "interface %s not found (ip link set %s up ...?)\n",
                     iface.c_str(), iface.c_str());
        return 1;
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        return 1;
    }
    const int fl = ::fcntl(fd, F_GETFL, 0);
    if (fl < 0 || ::fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0) {
        std::perror("fcntl(O_NONBLOCK)");
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    g_t0 = monotonic_s();
    if (!esc_path.empty()) {
        g_esc_csv = std::fopen(esc_path.c_str(), "w");
        if (!g_esc_csv) { std::perror("fopen esc-csv"); return 1; }
        std::fprintf(g_esc_csv, "t,node,rpm,voltage,current,err,esc_index\n");
        std::printf("esc.Status -> %s\n", esc_path.c_str());
    }
    if (!cmd_path.empty()) {
        g_cmd_csv = std::fopen(cmd_path.c_str(), "w");
        if (!g_cmd_csv) { std::perror("fopen cmd-csv"); return 1; }
        std::fprintf(g_cmd_csv, "t,node,index,cmd\n");
        std::printf("esc.RawCommand -> %s\n", cmd_path.c_str());
    }
    std::printf("sniffing %s (passive)... Ctrl+C to stop\n\n", iface.c_str());

    std::map<Key, uint64_t> counts;
    std::map<uint32_t, Transfer> reasm;  // key = type_id<<8 | src
    uint64_t std_frames = 0;      // 11-bit IDs = НЕ DroneCAN (VESC-native?)
    uint64_t total = 0;
    double last_report = monotonic_s();
    const double t_end = seconds > 0.0 ? monotonic_s() + seconds : 1e18;
    double last_esc_print = 0.0;

    while (!g_stop && monotonic_s() < t_end) {
        struct pollfd pfd { fd, POLLIN, 0 };
        if (::poll(&pfd, 1, 200) <= 0) { /* timeout: fall through to report */ }
        struct can_frame fr;
        while (::read(fd, &fr, sizeof(fr)) == static_cast<ssize_t>(sizeof(fr))) {
            ++total;
            if (!(fr.can_id & CAN_EFF_FLAG)) { ++std_frames; continue; }
            const uint32_t ext = fr.can_id & CAN_EFF_MASK;
            const DecodedId d = decode_id(ext);
            ++counts[Key{d.service, d.type_id, d.src}];

            if (d.service || (d.type_id != 1034 && d.type_id != 1030)) continue;

            const uint32_t key = (static_cast<uint32_t>(d.type_id) << 8) | d.src;
            const uint8_t* out = nullptr;
            std::size_t olen = 0;
            if (!feed_frame(reasm[key], fr.data, fr.can_dlc, &out, &olen)) continue;

            if (d.type_id == 1034) {  // esc.Status -> odometry
                const EscStatus s = decode_esc_status(out, olen);
                if (!s.ok) continue;
                const double t = monotonic_s() - g_t0;
                if (g_esc_csv) {
                    std::fprintf(g_esc_csv,
                                 "%.4f,%u,%d,%.3f,%.3f,%" PRIu32 ",%d\n", t,
                                 d.src, s.rpm, s.voltage, s.current, s.err,
                                 s.esc_index);
                }
                const double now = monotonic_s();
                if (now - last_esc_print > 0.5) {  // console throttle
                    last_esc_print = now;
                    print_esc_status_line(d.src, s);
                }
            } else {  // 1030 esc.RawCommand -> int14[] tail array, LSB first
                if (!g_cmd_csv) continue;
                const double t = monotonic_s() - g_t0;
                const int nvals = static_cast<int>((olen * 8) / 14);
                for (int i = 0; i < nvals; ++i) {
                    int32_t v =
                        static_cast<int32_t>(read_bits_lsb(out, 14 * i, 14));
                    if (v & 0x2000) v -= 0x4000;  // sign-extend 14 bits
                    std::fprintf(g_cmd_csv, "%.4f,%u,%d,%d\n", t, d.src, i, v);
                }
            }
        }

        const double now = monotonic_s();
        if (now - last_report >= 3.0) {
            std::printf("---- %.0f s, %" PRIu64 " frames ----\n", now - last_report,
                        total);
            for (const auto& [k, n] : counts) {
                std::printf("  %s node %-3u  type %-5u  %6.1f Hz   %s\n",
                            k.service ? "svc" : "msg", k.src, k.type_id,
                            n / (now - last_report),
                            type_name(k.service, k.type_id));
            }
            if (std_frames) {
                std::printf("  !! %" PRIu64
                            " frames with 11-bit ID -- NOT DroneCAN "
                            "(VESC-native? інший протокол?)\n",
                            std_frames);
            }
            std::printf("\n");
            counts.clear();
            std_frames = 0;
            total = 0;
            last_report = now;
            if (g_esc_csv) std::fflush(g_esc_csv);
            if (g_cmd_csv) std::fflush(g_cmd_csv);
        }
    }
    if (g_esc_csv) { std::fflush(g_esc_csv); std::fclose(g_esc_csv); }
    if (g_cmd_csv) { std::fflush(g_cmd_csv); std::fclose(g_cmd_csv); }
    ::close(fd);
    return 0;
}
