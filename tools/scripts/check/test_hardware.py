#!/usr/bin/env python3
"""Bench test of the wired AUV hardware, one PASS/FAIL line per check.

Tests (default: all, in this order):
  devices   udev names exist and are free
  kill      ESP32 indicator: lamps follow its0/its1/its2, then you press and release the
            physical kill and the board must report emergency and return (interactive)
  cameras   one MJPEG frame from each camera
  gps       u-blox fix type, satellites, accuracy (needs sky view; RTK needs rtk-rover + base)
  pixhawk   MAVLink through MAVProxy (127.0.0.1:14552): heartbeat, GPS seen by the Pixhawk
            (F9P on SERIAL3, GPS1_TYPE 1), live SB/SC switch positions from the TX16S

Stop the mission stack first (it owns the ports). Run through test_hardware.sh.
"""
import argparse
import os
import struct
import subprocess
import sys
import time

DEVICES = ["pixhawk", "esp32_kill", "gps_rtk", "front_camera", "bottom_camera"]
CAMERAS = {"front_camera": (640, 480), "bottom_camera": (1920, 1080)}
FIX_NAMES = {0: "no fix", 1: "dead reckoning", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "time only"}
results = []


def report(name, ok, detail=""):
    results.append((name, ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ""), flush=True)


def users_of(path):
    out = subprocess.run(["fuser", path], capture_output=True, text=True).stdout.split()
    return [f"{subprocess.run(['ps', '-o', 'comm=', '-p', p], capture_output=True, text=True).stdout.strip()}({p})"
            for p in out]


def ask(prompt, interactive):
    if not interactive:
        return
    input(f"  >> {prompt} then press Enter ")


def test_devices(args):
    print("devices")
    for dev in DEVICES:
        path = f"/dev/{dev}"
        if not os.path.exists(path):
            report(dev, False, "missing (plugged in? tools/udev/99-banyu.rules installed?)")
            continue
        users = users_of(path)
        # MAVProxy is supposed to hold the Pixhawk. usb_cam keeps idle handles on every
        # /dev/video*, which do not block capture, so cameras are checked by the cameras test.
        busy = [u for u in users if not (dev == "pixhawk" and u.startswith("mavproxy"))
                and dev not in CAMERAS]
        report(dev, not busy, f"-> {os.path.realpath(path)}" + (f", in use by {' '.join(busy)}" if busy else ""))


class Indicator:
    """ESP32 indicator board: send its0/its1/its2, read the 500 ms itsN reports."""

    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.buf = b""

    def send(self, state):
        self.ser.write(f"its{state}\n".encode())

    def latest(self, wait_s=1.3):
        # Reports before the command may still be queued; keep only the last one after the wait.
        self.ser.reset_input_buffer()
        self.buf = b""
        last = None
        end = time.time() + wait_s
        while time.time() < end:
            self.buf += self.ser.read(64)
            *lines, self.buf = self.buf.split(b"\n")
            for line in lines:
                line = line.strip().decode(errors="ignore")
                if line in ("its0", "its1", "its2"):
                    last = line
        return last


def test_kill(args):
    print("kill switch (ESP32 indicator, /dev/esp32_kill)")
    if users_of("/dev/esp32_kill"):
        report("kill port free", False, "stop banyu_teleop / esp32_trigger_node first")
        return
    board = Indicator("/dev/esp32_kill")
    first = board.latest()
    report("board reports", first is not None, first or "nothing in 1.3 s (firmware running?)")
    if first is None:
        return
    if first == "its2":
        board.send(0)
        if board.latest() == "its2":
            report("physical kill released", False,
                   "board stays at its2 after its0: kill switch pressed or not wired")
            return

    for state, lamp in ((0, "MANUAL"), (1, "AUTO"), (2, "EMERGENCY")):
        board.send(state)
        got = board.latest()
        report(f"lamp {lamp}", got == f"its{state}", f"sent its{state}, board reports {got}")
        if args.interactive:
            seen = input(f"  >> is only the {lamp} lamp on? [Y/n] ").strip().lower()
            report(f"lamp {lamp} visible", seen in ("", "y", "yes"))

    if not args.interactive:
        print("  --no-prompt: physical kill steps skipped")
        board.send(0)
        board.ser.close()
        return
    board.send(1)
    board.latest()
    ask("press (engage) the PHYSICAL KILL switch", args.interactive)
    got = board.latest()
    report("physical kill -> emergency", got == "its2", f"board reports {got} with AUTO commanded")
    board.send(1)
    ignored = board.latest()
    report("auto ignored while killed", ignored == "its2", f"board reports {ignored}")
    ask("release the PHYSICAL KILL switch", args.interactive)
    got = board.latest()
    report("release -> back to AUTO", got == "its1", f"board reports {got}")
    board.send(0)
    board.latest()
    board.ser.close()


def test_cameras(args):
    print("cameras")
    for cam, (w, h) in CAMERAS.items():
        path = f"/dev/{cam}"
        if not os.path.exists(path):
            report(cam, False, "missing")
            continue
        out = os.path.join(args.out_dir, f"{cam}.jpg")
        proc = subprocess.run(
            ["v4l2-ctl", "-d", path, f"--set-fmt-video=width={w},height={h},pixelformat=MJPG",
             "--stream-mmap", "--stream-skip=10", "--stream-count=1", f"--stream-to={out}"],
            capture_output=True, text=True, timeout=20)
        size = os.path.getsize(out) if proc.returncode == 0 and os.path.exists(out) else 0
        # A covered or dead sensor gives a tiny, nearly uniform JPEG.
        ok = proc.returncode == 0 and size > 20000
        if "busy" in proc.stdout + proc.stderr:
            report(cam, False, f"streaming in another process ({' '.join(users_of(path))}); stop it first")
            continue
        report(cam, ok, f"{w}x{h} frame {size} bytes -> {out}" + ("" if ok else " (lens covered? check the image)"))


def ubx(cls, mid, payload=b""):
    body = struct.pack("<BBH", cls, mid, len(payload)) + payload
    a = b = 0
    for c in body:
        a = (a + c) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def test_gps(args):
    print("gps (u-blox, /dev/gps_rtk)")
    if subprocess.run(["systemctl", "cat", "rtk-rover.service"], capture_output=True).returncode != 0:
        report("rtk-rover service", False, "not installed (setup/rtk_setup.sh)")
    else:
        rover = subprocess.run(["systemctl", "is-active", "rtk-rover.service"],
                               capture_output=True, text=True).stdout.strip()
        report("rtk-rover service", rover == "active", rover)
    if subprocess.run(["pgrep", "-f", "gps_to_pixhawk.py"], capture_output=True).returncode == 0:
        print("  run_gps_bridge.sh owns the receiver's USB stream; the pixhawk test shows the fix")
        return
    import serial
    ser = serial.Serial("/dev/gps_rtk", 115200, timeout=0.2)
    ser.write(ubx(0x06, 0x01, bytes([0x01, 0x07, 0x01])))   # NAV-PVT on this port, every epoch
    buf, pvt, end = b"", None, time.time() + args.gps_seconds
    while time.time() < end:
        try:
            buf += ser.read(512)
        except serial.SerialException:
            # USB CDC read hiccups happen on the shared hub; reopen instead of failing the test.
            ser.close()
            time.sleep(0.3)
            ser = serial.Serial("/dev/gps_rtk", 115200, timeout=0.2)
            continue
        i = buf.find(b"\xb5\x62\x01\x07")
        if i < 0 or len(buf) < i + 6 + 92 + 2:
            buf = buf[-200:] if i < 0 else buf
            continue
        p = buf[i + 6:i + 6 + 92]
        buf = buf[i + 100:]
        pvt = p
        fix, flags, sats = p[20], p[21], p[23]
        carr = (flags >> 6) & 3
        h_acc = struct.unpack_from("<I", p, 40)[0] / 1000.0
        print(f"\r  fix {FIX_NAMES.get(fix, fix)}  rtk {['none', 'FLOAT', 'FIXED', '?'][carr]}  "
              f"sats {sats}  hAcc {h_acc:.2f} m   ", end="", flush=True)
    print()
    ser.close()
    if pvt is None:
        report("receiver data", False, "no UBX NAV-PVT in time")
        return
    fix, carr, sats = pvt[20], (pvt[21] >> 6) & 3, pvt[23]
    lat, lon = (v / 1e7 for v in struct.unpack_from("<ii", pvt, 24)[::-1])
    report("3D fix", fix >= 3, f"{FIX_NAMES.get(fix, fix)}, {sats} sats, {lat:.7f}, {lon:.7f}")
    report("RTK fixed", carr == 2, ["no RTK (corrections not arriving)", "RTK float (wait)", "RTK fixed", "?"][carr])


def test_pixhawk(args):
    print("pixhawk (via MAVProxy udp 127.0.0.1:14552)")
    from pymavlink import mavutil
    try:
        mav = mavutil.mavlink_connection("udpin:127.0.0.1:14552")
    except OSError as e:
        report("udp 14552", False, f"{e} (nogps_odom_beacon / mission stack still running?)")
        return
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if hb is None:
        report("heartbeat", False, "nothing in 5 s (run_mavproxy.sh running?)")
        return
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    report("heartbeat", True, f"mode {mavutil.mode_string_v10(hb)}, {'ARMED' if armed else 'disarmed'}")

    gps = mav.recv_match(type="GPS_RAW_INT", blocking=True, timeout=3)
    if gps is None:
        report("pixhawk GPS", False, "no GPS_RAW_INT")
    else:
        names = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
        report("pixhawk GPS", gps.fix_type >= 3,
               f"{names.get(gps.fix_type, gps.fix_type)}, {gps.satellites_visible} sats"
               + ("" if gps.fix_type >= 3 else " (antenna sky view and cable?)"))

    print("  TX16S switches for 10 s (move SB and SC):")
    sb_seen, end = set(), time.time() + 10
    while time.time() < end:
        rc = mav.recv_match(type="RC_CHANNELS", blocking=True, timeout=1)
        if rc is None or rc.chancount == 0:
            continue
        sb, sc = rc.chan6_raw, rc.chan8_raw
        pos = "UP (auto)" if sb < 1200 else "DOWN (kill)" if sb > 1750 else "MID (manual)"
        sb_seen.add(pos)
        print(f"\r    SB(CH6) {sb:4d} {pos:13s} SC(CH8) {sc:4d}   ", end="", flush=True)
    print()
    report("TX16S SB positions", len(sb_seen) == 3,
           ", ".join(sorted(sb_seen)) if sb_seen else "no RC channels (receiver bound? TX16S on?)")


TESTS = {"devices": test_devices, "kill": test_kill, "cameras": test_cameras,
         "gps": test_gps, "pixhawk": test_pixhawk}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tests", nargs="*", help=f"tests to run: {', '.join(TESTS)} (default: all)")
    p.add_argument("--no-prompt", dest="interactive", action="store_false",
                   help="skip the questions (kill switch steps then only check the lamps)")
    p.add_argument("--gps-seconds", type=float, default=10.0)
    p.add_argument("--out-dir", default="/tmp/banyu_hw_test")
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    selected = args.tests or list(TESTS)
    unknown = [t for t in selected if t not in TESTS]
    if unknown:
        p.error(f"unknown test(s) {', '.join(unknown)}; choose from {', '.join(TESTS)}")
    for name in selected:
        try:
            TESTS[name](args)
        except Exception as e:   # one broken device must not stop the other tests
            report(name, False, f"error: {e}")
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed" + (f"; failed: {', '.join(failed)}" if failed else ""))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
