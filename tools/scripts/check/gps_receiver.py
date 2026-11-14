#!/usr/bin/env python3
"""Live u-blox F9P receiver health over its USB port: fix, RTK, satellites, signal, antenna/RF.

Only polls (UBX NAV-PVT, NAV-SAT, MON-RF); never writes receiver config, so it can run while the
Pixhawk reads the F9P over UART (GPS1_TYPE 1) and rtk-rover writes RTCM into the USB port.
Do not run it together with run_gps_bridge.sh (both would read the same USB stream).

usage: gps_receiver.py [--port /dev/gps_rtk] [--seconds N]   (N=0: until Ctrl+C)
Exit code 0 when the receiver has a 3D fix (or better) at the end.
"""
import argparse
import struct
import sys
import time

FIX = {0: "none", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "time"}
CARRIER = ["none", "RTK FLOAT", "RTK FIXED", "?"]
GNSS = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 5: "QZSS", 6: "GLO"}
ANT_STATUS = ["init", "unknown", "OK", "SHORT", "OPEN"]
ANT_POWER = ["off", "on", "unknown"]
JAMMING = ["unknown", "ok", "warning", "CRITICAL"]
POLLS = [(0x01, 0x07), (0x01, 0x35), (0x0A, 0x38)]  # NAV-PVT, NAV-SAT, MON-RF


def ubx(cls, mid, payload=b""):
    body = struct.pack("<BBH", cls, mid, len(payload)) + payload
    a = b = 0
    for c in body:
        a = (a + c) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def read_frames(ser, buf, msgs):
    buf += ser.read(4096)
    while True:
        i = buf.find(b"\xb5\x62")
        if i < 0:
            return buf[-1:]
        if len(buf) < i + 6:
            return buf[i:]
        cls, mid, n = struct.unpack_from("<BBH", buf, i + 2)
        if len(buf) < i + 8 + n:
            return buf[i:]
        msgs[(cls, mid)] = buf[i + 6:i + 6 + n]
        buf = buf[i + 8 + n:]


def report(msgs):
    lines = []
    pvt = msgs.get((0x01, 0x07))
    if pvt:
        fix, flags, used = pvt[20], pvt[21], pvt[23]
        lon, lat = (v / 1e7 for v in struct.unpack_from("<ii", pvt, 24))
        h_acc, v_acc = (v / 1000.0 for v in struct.unpack_from("<II", pvt, 40))
        pdop = struct.unpack_from("<H", pvt, 76)[0] / 100.0
        lines.append(f"fix {FIX.get(fix, fix):5s} {CARRIER[(flags >> 6) & 3]:9s} "
                     f"corrections {'yes' if flags & 0x02 else 'no ':3s} used {used:2d} sats  pDOP {pdop:.1f}")
        if fix >= 2:
            lines.append(f"pos {lat:.7f}, {lon:.7f}  hAcc {h_acc:.2f} m  vAcc {v_acc:.2f} m")
    sat = msgs.get((0x01, 0x35))
    tracked = 0
    if sat:
        per = {}
        for k in range(sat[5]):
            gnss, _, cno = sat[8 + 12 * k:11 + 12 * k]
            if cno > 0:
                per.setdefault(GNSS.get(gnss, str(gnss)), []).append(cno)
                tracked += 1
        lines.append(f"tracked {tracked} sats with signal" + ("" if per else "  (none)"))
        for name, c in sorted(per.items()):
            lines.append(f"  {name:4s} {len(c):2d} sats  C/N0 max {max(c)} avg {sum(c) / len(c):.0f} dBHz"
                         f"  strong(>=35) {sum(x >= 35 for x in c)}")
    rf = msgs.get((0x0A, 0x38))
    if rf:
        for b in range(rf[1]):
            q = rf[4 + 24 * b:28 + 24 * b]
            agc = struct.unpack_from("<H", q, 14)[0] * 100 // 8191
            lines.append(f"RF{b}: antenna {ANT_STATUS[q[2]] if q[2] < 5 else q[2]}, "
                         f"power {ANT_POWER[q[3]] if q[3] < 3 else q[3]}, AGC {agc}%, "
                         f"jamming {JAMMING[q[1] & 3]} (ind {q[16]}/255)")
    if sat is not None and tracked == 0:
        lines.append("-> no satellite signal at all: antenna outdoors? connected? powered?")
    elif pvt and pvt[20] < 3:
        lines.append("-> signal but no fix yet: wait (cold start 1-2 min) or improve sky view")
    return lines, (pvt[20] if pvt else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/gps_rtk")
    ap.add_argument("--seconds", type=float, default=10.0)
    a = ap.parse_args()
    import serial

    ser = serial.Serial(a.port, 115200, timeout=0.2)
    tty = sys.stdout.isatty()
    end = time.time() + a.seconds if a.seconds > 0 else float("inf")
    buf, msgs, fix, shown = b"", {}, 0, 0
    try:
        while time.time() < end:
            for cls, mid in POLLS:
                ser.write(ubx(cls, mid))
            t = time.time() + 1.0
            while time.time() < t:
                buf = read_frames(ser, buf, msgs)
            lines, fix = report(msgs)
            if tty and shown:
                print(f"\033[{shown}F\033[J", end="")
            print(time.strftime("%H:%M:%S") + f"  {a.port}")
            print("\n".join(lines) if lines else "no UBX reply (wrong port? receiver busy?)", flush=True)
            shown = len(lines or [0]) + 1
            if not tty:
                print()
    except KeyboardInterrupt:
        pass
    return 0 if fix >= 3 else 1


if __name__ == "__main__":
    sys.exit(main())
