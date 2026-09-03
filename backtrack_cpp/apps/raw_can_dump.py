#!/usr/bin/env python3
# Raw SocketCAN frame dumper — no external deps (stdlib socket + struct only),
# so it runs on the offline BlueOS host. LISTEN-ONLY: only reads the bus.
#
# Purpose: investigate what actually commands each track. can_sniff decodes
# DroneCAN message/service frames but not vendor payloads (type 20601 @ ~1kHz
# from node 50) or 11-bit VESC-native frames. This prints raw id + bytes so we
# can reverse-engineer the per-track command encoding.
#
# Usage:
#   raw_can_dump.py [--iface can0] [--seconds N] [--filter-type T] [--only-11bit]
#     --filter-type T : only 29-bit DroneCAN frames whose data-type-id == T
#     --only-11bit    : only standard (11-bit) frames (VESC-native candidates)
#     (default: summarize all, print a sample of each distinct id)

import socket
import struct
import sys
import time

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
FRAME_FMT = "=IB3x8s"  # can_id, can_dlc, pad3, data8


def decode_dtype(ext):
    service = (ext >> 7) & 1
    src = ext & 0x7F
    if service:
        tid = (ext >> 16) & 0xFF
    else:
        tid = (ext >> 8) & 0xFFFF
    return service, tid, src


def main():
    iface = "can0"
    seconds = 5.0
    filt = None
    only11 = False
    a = sys.argv[1:]
    i = 0
    while i < len(a):
        if a[i] == "--iface":
            iface = a[i + 1]; i += 2
        elif a[i] == "--seconds":
            seconds = float(a[i + 1]); i += 2
        elif a[i] == "--filter-type":
            filt = int(a[i + 1]); i += 2
        elif a[i] == "--only-11bit":
            only11 = True; i += 1
        else:
            i += 1

    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(1.0)

    t0 = time.monotonic()
    seen = {}          # key -> (count, last_bytes, first_t, last_t)
    samples = {}       # key -> list of (t, dlc, bytes) up to a few
    total = 0
    while time.monotonic() - t0 < seconds:
        try:
            frame = s.recv(16)
        except socket.timeout:
            continue
        can_id, dlc, data = struct.unpack(FRAME_FMT, frame)
        eff = bool(can_id & CAN_EFF_FLAG)
        payload = data[:dlc]
        total += 1
        if only11 and eff:
            continue
        if eff:
            ext = can_id & CAN_EFF_MASK
            service, tid, src = decode_dtype(ext)
            if filt is not None and tid != filt:
                continue
            key = ("svc" if service else "msg", tid, src)
        else:
            key = ("std11", can_id & 0x7FF, None)
        c, _, ft, _ = seen.get(key, (0, None, time.monotonic(), None))
        seen[key] = (c + 1, payload, ft, time.monotonic())
        if key not in samples:
            samples[key] = []
        if len(samples[key]) < 4:
            samples[key].append((round(time.monotonic() - t0, 3), dlc, payload.hex()))

    dur = time.monotonic() - t0
    print(f"== {dur:.1f}s, {total} frames total ==")
    for key in sorted(seen, key=lambda k: -seen[k][0]):
        kind, tid, src = key
        c = seen[key][0]
        hz = c / dur
        label = f"{kind} type={tid}" + (f" node={src}" if src is not None else "")
        print(f"{label:32s} {c:6d}  {hz:7.1f} Hz")
    print("\n== sample payloads (first few of each) ==")
    for key in sorted(seen, key=lambda k: -seen[k][0]):
        kind, tid, src = key
        label = f"{kind} type={tid}" + (f" node={src}" if src is not None else "")
        for (t, dlc, hx) in samples.get(key, []):
            print(f"  {label:28s} t={t:6.3f} dlc={dlc} data={hx}")


if __name__ == "__main__":
    main()
