// Passive DroneCAN bus reconnaissance for the first hookup to the live
// vehicle. Run with can0 in LISTEN-ONLY mode (see docs/CAN_TEST_PLAN.md):
// the tool only reads -- it cannot ACK, inject, or disturb the bus even if
// it wanted to.
//
// What it answers:
//   * which nodes are alive (source node IDs + NodeStatus rate)
//   * which message types fly and how often (esc.RawCommand = who commands,
//     esc.Status = our odometry source, BatteryInfo = wattmeter, Fix2 = GPS)
//   * live RPM / voltage / current decoded from esc.Status (best-effort
//     UAVCAN v0 decoding -- verify against the wheels actually turning)
//   * whether any VESC-native (non-DroneCAN) frames are present
//
// Usage:
//   can_sniff [--iface can0] [--seconds N]   (default: run until Ctrl+C)

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

// float16 -> float (IEEE 754 half, UAVCAN v0 uses LE byte order in payload)
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

// ---- multi-frame transfer reassembly (per source node, esc.Status only) ----
struct Reassembly {
    uint8_t buf[64];
    std::size_t len{0};
    bool active{false};
    uint8_t toggle{0};
};

// uavcan.equipment.esc.Status (1034), ~14 payload bytes = 2 CAN frames.
// Layout: error_count u32 | voltage f16 | current f16 | temperature f16 |
//         rpm int18 | power_rating_pct u7 | esc_index u5
void print_esc_status(uint8_t src, const uint8_t* p, std::size_t n) {
    if (n < 13) return;
    uint32_t err;
    uint16_t v16, c16;
    std::memcpy(&err, p, 4);
    std::memcpy(&v16, p + 4, 2);
    std::memcpy(&c16, p + 6, 2);
    // rpm: int18, packed MSB-first starting at byte 10
    int32_t rpm = (static_cast<int32_t>(p[10]) << 10)
                  | (static_cast<int32_t>(p[11]) << 2)
                  | (p[12] >> 6);
    if (rpm & 0x20000) rpm -= 0x40000;  // sign-extend 18 bits
    const int esc_index = (n > 13 ? p[13] : p[12]) & 0x1F;
    std::printf("  esc.Status node %-3u  rpm %-7d  %5.1f V  %5.1f A  "
                "err %" PRIu32 "  (idx~%d)\n",
                src, rpm, half_to_float(v16), half_to_float(c16), err,
                esc_index);
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
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--iface" && i + 1 < argc) iface = argv[++i];
        else if (arg == "--seconds" && i + 1 < argc) seconds = std::atof(argv[++i]);
        else {
            std::fprintf(stderr, "usage: can_sniff [--iface can0] [--seconds N]\n");
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
    std::printf("sniffing %s (passive)... Ctrl+C to stop\n\n", iface.c_str());

    std::map<Key, uint64_t> counts;
    std::map<uint8_t, Reassembly> esc_reasm;
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

            // esc.Status reassembly (2-frame transfer, tail byte last)
            if (!d.service && d.type_id == 1034 && fr.can_dlc >= 1) {
                const uint8_t tail = fr.data[fr.can_dlc - 1];
                const bool sof = tail & 0x80, eof = tail & 0x40;
                Reassembly& r = esc_reasm[d.src];
                if (sof) { r.len = 0; r.active = true; }
                if (r.active && r.len + fr.can_dlc < sizeof(r.buf)) {
                    std::memcpy(r.buf + r.len, fr.data, fr.can_dlc - 1);
                    r.len += fr.can_dlc - 1;
                }
                if (eof && r.active) {
                    r.active = false;
                    const double now = monotonic_s();
                    if (now - last_esc_print > 0.5) {  // не заливати консоль
                        last_esc_print = now;
                        if (sof) print_esc_status(d.src, r.buf, r.len);
                        else if (r.len > 2)  // multi-frame: strip transfer CRC
                            print_esc_status(d.src, r.buf + 2, r.len - 2);
                    }
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
        }
    }
    ::close(fd);
    return 0;
}
