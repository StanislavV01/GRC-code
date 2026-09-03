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


def half_to_float(h):
    import struct as _s
    return _s.unpack("<e", _s.pack("<H", h))[0]


def decode_actuator_array(buf):
    # uavcan.equipment.actuator.ArrayCommand: dynamic array of
    #   {actuator_id:u8, command_type:u8, command_value:float16}
    # After transfer reassembly the leading 2-byte CRC is already stripped.
    out = []
    n = len(buf) // 4
    for k in range(n):
        aid = buf[4 * k]
        ctype = buf[4 * k + 1]
        val = half_to_float(buf[4 * k + 2] | (buf[4 * k + 3] << 8))
        out.append((aid, ctype, val))
    return out


def decode_1010_mode(iface, seconds):
    # Reassemble DroneCAN transfers of type 1010 from node 10 and print decoded
    # actuator commands at ~3 Hz so a driven maneuver reveals the mapping.
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(1.0)
    reasm = {}  # src -> dict(buf,len,active,tid,tog)
    t0 = time.monotonic()
    last_print = 0.0
    latest = None
    while time.monotonic() - t0 < seconds:
        try:
            frame = s.recv(16)
        except socket.timeout:
            continue
        can_id, dlc, data = struct.unpack(FRAME_FMT, frame)
        if not (can_id & CAN_EFF_FLAG):
            continue
        ext = can_id & CAN_EFF_MASK
        service, tid_type, src = decode_dtype(ext)
        if service or tid_type != 1010 or src != 10:
            continue
        payload = data[:dlc]
        if dlc < 1:
            continue
        tail = payload[-1]
        sot = tail & 0x80
        eot = tail & 0x40
        tog = (tail >> 5) & 1
        transfer = tail & 0x1F
        body = payload[:-1]
        r = reasm.get(src)
        done = None
        if sot:
            reasm[src] = {"buf": bytearray(body), "tid": transfer, "tog": 1}
            if eot:
                done = bytes(reasm[src]["buf"]); reasm.pop(src, None)
        elif r is not None and transfer == r["tid"] and tog == r["tog"]:
            r["buf"] += body; r["tog"] ^= 1
            if eot:
                done = bytes(r["buf"]); reasm.pop(src, None)
        else:
            reasm.pop(src, None)
        if done is not None and len(done) >= 2:
            latest = decode_actuator_array(done[2:])  # strip transfer CRC
        now = time.monotonic() - t0
        if latest is not None and now - last_print > 0.33:
            last_print = now
            cells = "  ".join(f"id{a}:{v:+.3f}" for (a, ct, v) in latest)
            print(f"t={now:6.2f}  {cells}")
    return


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
        elif a[i] == "--decode-1010":
            decode_1010_mode(iface, seconds); return
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
