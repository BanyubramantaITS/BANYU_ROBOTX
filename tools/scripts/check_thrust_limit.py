#!/usr/bin/env python3
"""
Verify the Pixhawk thruster output cap before a real-vehicle run.

Reads MOT_PWM_MIN, MOT_PWM_MAX and MOT_PWM_TYPE from the autopilot through a MAVProxy
output (default udpin:127.0.0.1:14552, the nogps_odom_beacon port, free before launch) and
compares them with the range in banyu_bringup/config/banyu_defaults.parm. A wider range on
the Pixhawk, or a MOT_PWM_TYPE that ignores it, is refused.

Exit codes: 0 cap in place, 1 cap missing or wider than the parm, 2 no answer from the Pixhawk.
Stdlib only (no pymavlink). Self-check: `./check_thrust_limit.py --self-check`
"""

import argparse
import os
import socket
import struct
import sys
import time

MSG_HEARTBEAT = 0
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_VALUE = 22
CRC_EXTRA = {MSG_HEARTBEAT: 50, MSG_PARAM_REQUEST_READ: 214, MSG_PARAM_VALUE: 220}
MAV_AUTOPILOT_INVALID = 8
# AP_MotorsMulticopter::update_throttle_range forces PWM_MIN/MAX to 1000/2000 for DShot (4-7)
# and PWMRange/PWMAngle (8, 9), which silently removes the cap.
CAPPABLE_PWM_TYPES = (0, 1, 2, 3)
PARAMS = ("MOT_PWM_MIN", "MOT_PWM_MAX", "MOT_PWM_TYPE")

DEFAULT_PARM = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..",
    "banyu_bringup", "config", "banyu_defaults.parm"))


def x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def pack_v2(msgid: int, payload: bytes, seq: int, sysid: int = 255, compid: int = 191) -> bytes:
    header = struct.pack("<BBBBBBB", 0xFD, len(payload), 0, 0, seq & 0xFF, sysid, compid)
    header += struct.pack("<I", msgid)[:3]
    crc = x25_crc(header[1:] + payload + bytes([CRC_EXTRA[msgid]]))
    return header + payload + struct.pack("<H", crc)


def pack_param_request_read(name: str, target_sys: int, target_comp: int, seq: int) -> bytes:
    payload = struct.pack("<hBB16s", -1, target_sys, target_comp, name.encode())
    return pack_v2(MSG_PARAM_REQUEST_READ, payload, seq)


def parse_frames(data: bytes):
    """Yield (sysid, compid, msgid, payload) for CRC-valid frames of the messages we know."""
    i = 0
    while i < len(data):
        stx = data[i]
        if stx == 0xFD and i + 10 <= len(data):
            plen, incompat = data[i + 1], data[i + 2]
            sysid, compid = data[i + 5], data[i + 6]
            msgid = int.from_bytes(data[i + 7:i + 10], "little")
            body_end = i + 10 + plen
            frame_end = body_end + 2 + (13 if incompat & 0x01 else 0)
            crc_input = data[i + 1:body_end]
        elif stx == 0xFE and i + 6 <= len(data):
            plen = data[i + 1]
            sysid, compid, msgid = data[i + 3], data[i + 4], data[i + 5]
            body_end = i + 6 + plen
            frame_end = body_end + 2
            crc_input = data[i + 1:body_end]
        else:
            i += 1
            continue
        if frame_end > len(data):
            return
        if msgid in CRC_EXTRA:
            (crc,) = struct.unpack_from("<H", data, body_end)
            if crc == x25_crc(crc_input + bytes([CRC_EXTRA[msgid]])):
                yield sysid, compid, msgid, data[body_end - plen:body_end]
        i = frame_end


def decode_param_value(payload: bytes):
    value, _count, _index, raw_id, _ptype = struct.unpack("<fHH16sB", payload.ljust(25, b"\0"))
    return raw_id.split(b"\0", 1)[0].decode(errors="replace"), value


def is_autopilot_heartbeat(payload: bytes) -> bool:
    autopilot = payload.ljust(9, b"\0")[5]
    return autopilot != MAV_AUTOPILOT_INVALID


def read_parm(path: str) -> dict:
    values = {}
    with open(path) as f:
        for line in f:
            fields = line.split("#", 1)[0].split()
            if len(fields) >= 2 and fields[0] in PARAMS:
                values[fields[0]] = float(fields[1])
    return values


def read_pixhawk(host: str, port: int, timeout_s: float) -> dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(0.2)
    deadline = time.monotonic() + timeout_s
    peer, target, found = None, None, {}
    seq, next_request = 0, 0.0
    try:
        while time.monotonic() < deadline and len(found) < len(PARAMS):
            now = time.monotonic()
            if target is not None and now >= next_request:
                for name in PARAMS:
                    if name not in found:
                        sock.sendto(pack_param_request_read(name, *target, seq), peer)
                        seq += 1
                next_request = now + 1.0
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            for sysid, compid, msgid, payload in parse_frames(data):
                if msgid == MSG_HEARTBEAT and target is None and is_autopilot_heartbeat(payload):
                    peer, target = addr, (sysid, compid)
                elif msgid == MSG_PARAM_VALUE:
                    name, value = decode_param_value(payload)
                    if name in PARAMS:
                        found[name] = value
    finally:
        sock.close()
    return found


def evaluate(expected: dict, actual: dict):
    """Return a list of problems; empty means the cap is in place."""
    problems = []
    lo, hi = actual["MOT_PWM_MIN"], actual["MOT_PWM_MAX"]
    if lo < expected["MOT_PWM_MIN"]:
        problems.append(f"MOT_PWM_MIN is {lo:.0f}, cap needs >= {expected['MOT_PWM_MIN']:.0f}")
    if hi > expected["MOT_PWM_MAX"]:
        problems.append(f"MOT_PWM_MAX is {hi:.0f}, cap needs <= {expected['MOT_PWM_MAX']:.0f}")
    if not lo < 1500 < hi:
        problems.append(f"MOT_PWM range {lo:.0f}..{hi:.0f} does not straddle neutral 1500")
    pwm_type = int(actual["MOT_PWM_TYPE"])
    if pwm_type not in CAPPABLE_PWM_TYPES:
        problems.append(f"MOT_PWM_TYPE is {pwm_type}; ArduSub ignores MOT_PWM_MIN/MAX for it")
    return problems


def percent(lo: float, hi: float) -> float:
    return 100.0 * max(1500 - lo, hi - 1500) / 400.0


def parse_udpin(endpoint: str):
    addr = endpoint
    for prefix in ("udpin:", "udp:"):
        if addr.startswith(prefix):
            addr = addr[len(prefix):]
    host, port_s = addr.rsplit(":", 1)
    return host, int(port_s)


def self_check():
    pkt = pack_param_request_read("MOT_PWM_MAX", 1, 1, seq=7)
    frames = list(parse_frames(b"\x00junk" + pkt + pkt))
    assert len(frames) == 2 and frames[0][2] == MSG_PARAM_REQUEST_READ, frames
    value = struct.pack("<fHH16sB", 1700.0, 900, 12, b"MOT_PWM_MAX", 4).rstrip(b"\0")
    frame = pack_v2(MSG_PARAM_VALUE, value, 0, sysid=1, compid=1)
    ((_, _, msgid, payload),) = parse_frames(frame)
    assert msgid == MSG_PARAM_VALUE and decode_param_value(payload) == ("MOT_PWM_MAX", 1700.0)
    corrupt = bytearray(frame)
    corrupt[12] ^= 0xFF
    assert not list(parse_frames(bytes(corrupt)))
    cap = {"MOT_PWM_MIN": 1300.0, "MOT_PWM_MAX": 1700.0}
    assert not evaluate(cap, {"MOT_PWM_MIN": 1300.0, "MOT_PWM_MAX": 1700.0, "MOT_PWM_TYPE": 0})
    assert not evaluate(cap, {"MOT_PWM_MIN": 1350.0, "MOT_PWM_MAX": 1650.0, "MOT_PWM_TYPE": 0})
    assert len(evaluate(cap, {"MOT_PWM_MIN": 1100.0, "MOT_PWM_MAX": 1900.0, "MOT_PWM_TYPE": 0})) == 2
    assert evaluate(cap, {"MOT_PWM_MIN": 1300.0, "MOT_PWM_MAX": 1700.0, "MOT_PWM_TYPE": 6})
    assert read_parm(DEFAULT_PARM).keys() >= {"MOT_PWM_MIN", "MOT_PWM_MAX"}
    print("check_thrust_limit: self-check OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink", default="udpin:127.0.0.1:14552")
    ap.add_argument("--parm", default=DEFAULT_PARM, help="parm file holding the cap")
    ap.add_argument("--timeout", type=float, default=8.0, help="seconds to wait for the Pixhawk")
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    if args.self_check:
        self_check()
        return 0

    expected = read_parm(args.parm)
    if not {"MOT_PWM_MIN", "MOT_PWM_MAX"} <= expected.keys():
        print(f"check_thrust_limit: no MOT_PWM_MIN/MAX in {args.parm}", file=sys.stderr)
        return 1

    host, port = parse_udpin(args.mavlink)
    try:
        actual = read_pixhawk(host, port, args.timeout)
    except OSError as e:
        print(f"check_thrust_limit: cannot listen on {host}:{port}: {e}", file=sys.stderr)
        return 2
    missing = [p for p in PARAMS if p not in actual]
    if missing:
        print(f"check_thrust_limit: no answer for {', '.join(missing)} on {args.mavlink} "
              f"within {args.timeout:.0f} s (is run_mavproxy.sh running?)", file=sys.stderr)
        return 2

    problems = evaluate(expected, actual)
    lo, hi = actual["MOT_PWM_MIN"], actual["MOT_PWM_MAX"]
    if problems:
        print("check_thrust_limit: thruster cap NOT in place on the Pixhawk:", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        print(f"  Load {args.parm} in QGC (or set MOT_PWM_MIN/MAX; applies live).", file=sys.stderr)
        return 1
    print(f"check_thrust_limit: OK, MOT_PWM {lo:.0f}..{hi:.0f} "
          f"({percent(lo, hi):.0f}% of full command), MOT_PWM_TYPE {int(actual['MOT_PWM_TYPE'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
