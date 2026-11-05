#!/usr/bin/env python3
"""Movement tests on the real AUV, one manoeuvre per run, without the ROS mission stack.

Tests:
  dive      arm, dive to --depth, hold it for --hold s, then disarm and float up
  forward   drive forward for --time s (at the surface, or at --depth)
  backward  drive backward for --time s
  sway      sway --dir left|right for --time s
  yaw       turn --angle deg --dir left|right on the compass (or --time s open loop)
  waypoint  surface GPS waypoints (RTK): --ne N,E (metres from the start), --wp LAT,LON, --file
  where     print the GPS fix and heading until Ctrl+C (no arming), to note waypoints

How it flies: MAVLink to the Pixhawk through MAVProxy (run/run_mavproxy.sh, udp 14552) and
RC_CHANNELS_OVERRIDE on the stick channels (RCMAP_*), as POR did. At --depth 0 it flies
STABILIZE (no depth hold, the vehicle floats); below the surface ALT_HOLD on the Bar30.
Depth is measured from the Bar30 pressure (SCALED_PRESSURE2) against the surface reading taken
before arming. Every run ends neutral and disarmed (the vehicle floats up), also on Ctrl+C,
SIGTERM, SIGHUP (SSH drop), a mode change or disarm from elsewhere, depth over the limit,
stale sensors or the time limit. Each run writes a 10 Hz CSV and a summary to
~/banyu_bags/move_tests/.

Stop banyu_teleop and the mission stack first: they drive the same sticks and arming.
Run it on the Pi inside tmux, so an SSH drop does not end it halfway.
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

os.environ.setdefault("MAVLINK20", "1")   # 18-channel RC_CHANNELS_OVERRIDE
from pymavlink import mavutil              # noqa: E402

RATE_HZ = 20.0
NEUTRAL = 1500
FULL_PWM = 400                 # PWM from neutral to full stick
IGNORE = 65535                 # RC override: leave this channel alone
RELEASE = 0                    # RC override: hand the channel back to the receiver
GPS_FIX = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
RHO = {"fresh": 997.0, "salt": 1025.0}
DEPTH_TAU_S = 0.5              # depth low-pass for control (the raw value is logged too)
MODES = {name: num for num, name in mavutil.mode_mapping_sub.items()}
STACK_PROCESSES = "banyu_teleop|banyu_mission|vehicle_execution_node|esp32_trigger_node"
LINEAR = {"forward": (1, 0), "backward": (-1, 0), "sway": (0, 1)}


class Abort(Exception):
    pass


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def ne_offset(lat0, lon0, lat1, lon1):
    """North/east metres from point 0 to point 1 (flat earth, fine over a few hundred metres)."""
    r = 6378137.0
    return (math.radians(lat1 - lat0) * r,
            math.radians(lon1 - lon0) * r * math.cos(math.radians(lat0)))


def offset_latlon(lat, lon, north, east):
    r = 6378137.0
    return (lat + math.degrees(north / r),
            lon + math.degrees(east / (r * math.cos(math.radians(lat)))))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Vehicle:
    def __init__(self, a):
        self.a = a
        self.sys = None
        self.mode = None
        self.armed = False
        self.hb_t = 0.0
        self.att = None            # (roll, pitch, heading deg 0..360), time
        self.att_t = 0.0
        self.press = None          # Bar30 hPa, raw
        self.press_f = None        # low-passed for control
        self.press_t = 0.0
        self.p0 = None             # surface datum hPa
        self.gps = None
        self.gps_t = 0.0
        self.volt = float("nan")
        self.sticks = (0.0, 0.0, 0.0, 0.0)
        self.channels = None       # (forward, lateral, throttle, yaw), 1-based
        self.reverse = {}
        self.statustext = []
        try:
            self.mav = mavutil.mavlink_connection(a.connect, source_system=255, source_component=191)
        except OSError as e:
            sys.exit(f"Error: cannot open {a.connect}: {e} (another tool on that port? "
                     "check/test_hardware.sh, nogps_odom_beacon)")

    # ---- link -------------------------------------------------------------------------------
    def wait_vehicle(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            m = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if m and m.type == mavutil.mavlink.MAV_TYPE_SUBMARINE:
                self.sys = m.get_srcSystem()
                self.handle(m)
                return True
        return False

    def handle(self, m):
        t = m.get_type()
        if m.get_srcSystem() != self.sys or t == "BAD_DATA":
            return
        now = time.time()
        if t == "HEARTBEAT" and m.get_srcComponent() == 1:
            self.mode = mavutil.mode_mapping_sub.get(m.custom_mode, str(m.custom_mode))
            self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.hb_t = now
        elif t == "ATTITUDE":
            self.att = (math.degrees(m.roll), math.degrees(m.pitch), math.degrees(m.yaw) % 360.0)
            self.att_t = now
        elif t == self.a.depth_msg:
            self.press = m.press_abs
            if self.press_f is None or now - self.press_t > 1.0:
                self.press_f = m.press_abs
            else:
                dt = now - self.press_t
                self.press_f += (m.press_abs - self.press_f) * dt / (DEPTH_TAU_S + dt)
            self.press_t = now
        elif t == "GPS_RAW_INT":
            self.gps = {"fix": m.fix_type, "lat": m.lat / 1e7, "lon": m.lon / 1e7,
                        "sats": m.satellites_visible, "hacc": getattr(m, "h_acc", 0) / 1000.0,
                        "cog": m.cog / 100.0, "vel": m.vel / 100.0}
            self.gps_t = now
        elif t == "SYS_STATUS":
            self.volt = m.voltage_battery / 1000.0
        elif t == "STATUSTEXT":
            text = m.text.rstrip("\x00").strip()
            if text not in self.statustext[-6:]:
                print(f"  [pixhawk] {text}", flush=True)
            self.statustext.append(text)

    def update(self):
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                return
            self.handle(m)

    def request_streams(self):
        self.mav.mav.request_data_stream_send(self.sys, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def param(self, name, timeout=2.0):
        for _ in range(3):
            self.mav.mav.param_request_read_send(self.sys, 1, name.encode(), -1)
            end = time.time() + timeout / 3
            while time.time() < end:
                m = self.mav.recv_match(blocking=True, timeout=0.2)
                if m is None:
                    continue
                if m.get_type() == "PARAM_VALUE" and m.param_id.rstrip("\x00") == name:
                    return m.param_value
                self.handle(m)
        return None

    # ---- derived state --------------------------------------------------------------------
    def depth(self, raw=False):
        if self.press is None or self.p0 is None:
            return float("nan")
        return ((self.press if raw else self.press_f) - self.p0) * 100.0 / (RHO[self.a.water] * 9.80665)

    def depth_fresh(self):
        return time.time() - self.press_t < 1.0

    def heading(self):
        return self.att[2] if self.att else float("nan")

    def gps_ok(self, min_fix):
        return self.gps is not None and time.time() - self.gps_t < 1.5 and self.gps["fix"] >= min_fix

    # ---- commands -------------------------------------------------------------------------
    def set_mode(self, mode):
        num = MODES[mode]
        for _ in range(5):
            self.mav.mav.set_mode_send(self.sys, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, num)
            end = time.time() + 1.0
            while time.time() < end:
                self.update()
                if self.mode == mode:
                    return
                time.sleep(0.05)
        raise Abort(f"Pixhawk did not switch to {mode} (still {self.mode})")

    def arm(self, on, force=False):
        self.mav.mav.command_long_send(self.sys, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       1 if on else 0, 21196 if force else 0, 0, 0, 0, 0, 0)
        end = time.time() + 3.0
        while time.time() < end:
            self.send_sticks(0, 0, 0, 0)
            self.update()
            if self.armed == on:
                return True
            time.sleep(0.05)
        return False

    def send_sticks(self, fwd, lat, thr, yaw):
        """Stick fractions -1..1: forward +, right +, up +, yaw right (clockwise) +."""
        self.sticks = (fwd, lat, thr, yaw)
        ch = [IGNORE] * 18
        for c, v in zip(self.channels, self.sticks):
            ch[c - 1] = int(round(NEUTRAL + self.reverse.get(c, 1) * clamp(v, -1.0, 1.0) * FULL_PWM))
        self.mav.mav.rc_channels_override_send(self.sys, 1, *ch)

    def release_sticks(self):
        ch = [IGNORE] * 18
        for c in self.channels:
            ch[c - 1] = RELEASE
        for _ in range(3):
            self.mav.mav.rc_channels_override_send(self.sys, 1, *ch)
            time.sleep(0.05)


class Run:
    def __init__(self, a):
        self.a = a
        self.v = Vehicle(a)
        self.phase = "preflight"
        self.t0 = time.time()
        self.deadline = None
        self.log = None
        self.summary = []
        self.hold_correcting = False
        self.dived = False

    def say(self, text):
        print(text, flush=True)
        self.summary.append(text)

    # ---- preflight ------------------------------------------------------------------------
    def preflight(self):
        a, v = self.a, self.v
        if not a.ignore_stack and a.test != "where":
            out = subprocess.run(["pgrep", "-af", STACK_PROCESSES], capture_output=True, text=True).stdout
            procs = [l[:110] for l in out.splitlines() if "pgrep" not in l and "auv_move" not in l]
            if procs:
                msg = "the mission stack / teleop is running (it drives the same sticks):\n    " + \
                      "\n    ".join(procs) + "\n  stop it first, or pass --ignore-stack"
                if a.dry_run:
                    print("WARNING " + msg)
                else:
                    sys.exit("Error: " + msg)

        print(f"Waiting for the Pixhawk on {a.connect} ...", flush=True)
        if not v.wait_vehicle(5.0):
            msg = f"no ArduSub heartbeat on {a.connect} in 5 s (run/run_mavproxy.sh running?)"
            if a.dry_run:
                print("WARNING " + msg + "; plan only")
                self.print_plan()
                sys.exit(0)
            sys.exit("Error: " + msg)
        v.request_streams()

        names = ("RCMAP_FORWARD", "RCMAP_LATERAL", "RCMAP_THROTTLE", "RCMAP_YAW")
        chans = [v.param(n) for n in names]
        if None in chans:
            print(f"WARNING could not read RCMAP_*; using --channels {a.channels}")
            v.channels = tuple(int(c) for c in a.channels.split(","))
        else:
            v.channels = tuple(int(c) for c in chans)
        for c in v.channels:
            rev = v.param(f"RC{c}_REVERSED")
            v.reverse[c] = -1 if rev else 1
        if set(v.channels) & {6, 8}:
            print("WARNING a stick channel is CH6 (SB) or CH8 (SC); check RCMAP_* on the Pixhawk")

        time.sleep(1.0)
        v.update()
        print(f"Pixhawk : sys {v.sys}, mode {v.mode}, {'ARMED' if v.armed else 'disarmed'}, "
              f"battery {v.volt:.1f} V")
        print(f"Sticks  : forward CH{v.channels[0]}, lateral CH{v.channels[1]}, "
              f"throttle CH{v.channels[2]}, yaw CH{v.channels[3]}"
              + "".join(f" (CH{c} reversed)" for c, r in v.reverse.items() if r < 0))
        print(f"Heading : {v.heading():.0f} deg" if v.att else "Heading : no ATTITUDE (!)")
        if v.gps:
            g = v.gps
            print(f"GPS     : {GPS_FIX.get(g['fix'], g['fix'])}, {g['sats']} sats, hAcc {g['hacc']:.2f} m, "
                  f"{g['lat']:.7f}, {g['lon']:.7f}")
        else:
            print("GPS     : no GPS_RAW_INT")

        problems = []
        if v.armed:
            problems.append("vehicle is already armed: disarm it first (QGC or SB Down)")
        if v.att is None:
            problems.append("no ATTITUDE from the Pixhawk")
        if a.test != "where" and not a.dry_run and v.mode is None:
            problems.append("no mode in the heartbeat")
        needs_depth = a.test == "dive" or a.depth > 0
        if v.press is None or not v.depth_fresh():
            if needs_depth:
                problems.append(f"no {a.depth_msg} (Bar30) data: ALT_HOLD and the depth checks need it "
                                "(BARO2_DEVID 0? try --depth-msg SCALED_PRESSURE3)")
            else:
                print(f"Depth   : no {a.depth_msg} (only logged at the surface, continuing)")
        else:
            samples, end = [], time.time() + 1.5
            while time.time() < end:
                v.update()
                samples.append(v.press)
                time.sleep(0.05)
            v.p0 = sum(samples) / len(samples)
            print(f"Depth   : surface datum {v.p0:.1f} hPa from {a.depth_msg} (vehicle must be at the "
                  f"surface now)")
            if abs(v.p0 - 1013.0) > 80:
                print(f"WARNING datum {v.p0:.0f} hPa is far from 1013 hPa: is the vehicle at the surface, "
                      "and is this the Bar30?")
        if a.test == "waypoint":
            need = 3 if a.allow_no_rtk else 5
            if not v.gps_ok(need):
                problems.append(f"GPS needs {GPS_FIX[need]} or better (have "
                                f"{GPS_FIX.get(v.gps['fix'], '?') if v.gps else 'nothing'}); "
                                "rtk-rover, antenna sky view"
                                + ("" if a.allow_no_rtk else ", or --allow-no-rtk"))
        if problems and a.test != "where":
            for p in problems:
                print("Error: " + p)
            if not a.dry_run:
                sys.exit(1)

    def print_plan(self):
        a = self.a
        lines = [f"Test    : {a.test}", f"Mode    : {a.mode}"]
        if a.test == "dive":
            lines.append(f"Plan    : dive to {a.depth} m, hold {a.hold} s, disarm and float up")
        elif a.test == "where":
            lines.append("Plan    : print GPS and heading, no arming")
        else:
            where = f"at {a.depth} m (dive first, settle {a.settle} s)" if a.depth > 0 else "at the surface"
            if a.test in LINEAR:
                what = {"forward": "forward", "backward": "backward", "sway": f"sway {a.dir}"}[a.test]
                lines.append(f"Plan    : {what} at {a.power:.0f}% stick for {a.time} s {where}, "
                             f"coast {a.coast} s, {'stay armed' if a.no_disarm else 'disarm'}")
            elif a.test == "yaw":
                amount = f"{a.time} s open loop" if a.time else f"{a.angle} deg on the compass"
                lines.append(f"Plan    : yaw {a.dir} {amount} at up to {a.power:.0f}% stick {where}, "
                             f"{'stay armed' if a.no_disarm else 'disarm'}")
            elif a.test == "waypoint":
                lines.append(f"Plan    : {len(a.targets_desc)} waypoint(s) at the surface, {a.power:.0f}% stick, "
                             f"arrive within {a.radius} m, hold {a.wp_hold} s each"
                             + (", then back to the start" if a.return_home else ""))
                for d in a.targets_desc:
                    lines.append(f"            {d}")
        lines.append(f"Limits  : abort below {a.depth_limit:.1f} m, run time {a.timeout:.0f} s")
        print("\n".join(lines))

    # ---- control loop ---------------------------------------------------------------------
    def open_log(self):
        os.makedirs(self.a.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.a.log_dir, f"{stamp}_{self.a.test}")
        self.log_path = base + ".csv"
        self.summary_path = base + ".txt"
        self.log_file = open(self.log_path, "w", newline="")
        self.log = csv.writer(self.log_file)
        self.log.writerow(["t", "phase", "mode", "armed", "depth_m", "depth_raw_m", "heading_deg", "roll_deg", "pitch_deg",
                           "lat", "lon", "fix", "sats", "hacc_m", "cog_deg", "gps_speed_mps", "battery_v",
                           "stick_fwd", "stick_lat", "stick_thr", "stick_yaw", "note"])
        self.last_flush = time.time()

    def write_log(self, note=""):
        v = self.v
        g = v.gps or {}
        att = v.att or (float("nan"),) * 3
        self.log.writerow([f"{time.time() - self.t0:.2f}", self.phase, v.mode, int(v.armed), f"{v.depth():.3f}",
                           f"{v.depth(raw=True):.3f}",
                           f"{att[2]:.1f}", f"{att[0]:.1f}", f"{att[1]:.1f}", f"{g.get('lat', '')}",
                           f"{g.get('lon', '')}", g.get("fix", ""), g.get("sats", ""), g.get("hacc", ""),
                           g.get("cog", ""), g.get("vel", ""), f"{v.volt:.2f}",
                           *(f"{s:.2f}" for s in v.sticks), note])
        if time.time() - self.last_flush > 1.0:
            self.log_file.flush()
            self.last_flush = time.time()

    def check_safety(self):
        a, v, now = self.a, self.v, time.time()
        if now - v.hb_t > 3.0:
            raise Abort("no heartbeat from the Pixhawk for 3 s")
        if not v.armed:
            raise Abort("vehicle was disarmed from elsewhere (kill switch, QGC, failsafe)")
        if v.mode != a.mode:
            raise Abort(f"mode changed to {v.mode} from elsewhere")
        if now > self.deadline:
            raise Abort(f"run time limit {a.timeout:.0f} s reached")
        if a.depth > 0 or a.test == "dive":
            if not v.depth_fresh():
                raise Abort(f"depth sensor ({a.depth_msg}) silent for 1 s")
            if v.depth() > a.depth_limit:
                raise Abort(f"depth {v.depth():.2f} m is below the limit {a.depth_limit:.2f} m")

    def step(self, fwd=0.0, lat=0.0, thr=0.0, yaw=0.0, note=""):
        self.v.update()
        self.check_safety()
        self.v.send_sticks(fwd, lat, thr, yaw)
        self.write_log(note)
        self.next_t = max(self.next_t + 1.0 / RATE_HZ, time.time() - 0.5)
        time.sleep(max(0.0, self.next_t - time.time()))

    def depth_stick(self, target):
        """Throttle stick toward the target depth; neutral inside the band, ALT_HOLD holds there."""
        if target <= 0:
            return 0.0
        err = target - self.v.depth()            # > 0: too shallow
        band = 0.10 if self.hold_correcting else self.a.depth_tol
        if abs(err) < band:
            self.hold_correcting = False
            return 0.0
        self.hold_correcting = True
        # min stick clears the ALT_HOLD throttle dead zone (THR_DZ)
        mag = min(self.a.dive_power / 100.0, 0.25 + 0.3 * abs(err))
        return -mag if err > 0 else mag

    def ramp(self, t_in):
        return clamp(t_in / self.a.ramp, 0.0, 1.0) if self.a.ramp > 0 else 1.0

    def stats(self, samples, name, unit, fmt=".2f"):
        vals = [s for s in samples if not math.isnan(s)]
        if vals:
            self.say(f"  {name}: min {min(vals):{fmt}} / mean {sum(vals) / len(vals):{fmt}} / "
                     f"max {max(vals):{fmt}} {unit}")

    # ---- phases ---------------------------------------------------------------------------
    def dive(self, target):
        self.phase = "dive"
        a, v = self.a, self.v
        start = time.time()
        timeout = a.dive_timeout or (15.0 + 15.0 * target)
        self.say(f"Diving to {target:.2f} m ...")
        while not v.depth() >= target - 0.10:   # NaN-safe
            if time.time() - start > timeout:
                raise Abort(f"did not reach {target:.2f} m in {timeout:.0f} s (depth {v.depth():.2f} m): "
                            "buoyancy, dive power or thrusters")
            self.hold_correcting = True
            self.step(thr=-min(a.dive_power / 100.0, 0.25 + 0.3 * (target - v.depth())))
        self.dived = True
        self.say(f"  reached {v.depth():.2f} m after {time.time() - start:.1f} s")

    def hold(self, target, seconds, phase="hold"):
        self.phase = phase
        depths, headings = [], []
        end = time.time() + seconds
        h0 = self.v.heading()
        while time.time() < end:
            self.step(thr=self.depth_stick(target))
            depths.append(self.v.depth())
            headings.append(wrap180(self.v.heading() - h0))
        return depths, headings

    def linear(self):
        a, v = self.a, self.v
        fwd, lat = LINEAR[a.test]
        if a.test == "sway" and a.dir == "left":
            lat = -lat
        p = a.power / 100.0
        self.phase = a.test
        g0 = dict(v.gps) if v.gps_ok(3) else None
        h0 = v.heading()
        depths, headings = [], []
        start = time.time()
        self.say(f"{a.test.capitalize()}{' ' + a.dir if a.test == 'sway' else ''} at {a.power:.0f}% "
                 f"for {a.time} s ...")
        while time.time() - start < a.time:
            r = self.ramp(time.time() - start)
            self.step(fwd=fwd * p * r, lat=lat * p * r, thr=self.depth_stick(a.depth))
            depths.append(v.depth())
            headings.append(wrap180(v.heading() - h0))
        g1 = dict(v.gps) if v.gps_ok(3) else None
        coast_d, coast_h = self.hold(a.depth, a.coast, "coast")
        g2 = dict(v.gps) if v.gps_ok(3) else None
        self.say(f"Result {a.test}:")
        self.stats(headings, "heading change", "deg", ".1f")
        if a.depth > 0:
            self.stats(depths, "depth", "m")
        for label, end_fix in (("thrust only", g1), ("incl. coast", g2)):
            if g0 and end_fix:
                n, e = ne_offset(g0["lat"], g0["lon"], end_fix["lat"], end_fix["lon"])
                h = math.radians(h0)
                body_f = n * math.cos(h) + e * math.sin(h)
                body_r = -n * math.sin(h) + e * math.cos(h)
                self.say(f"  GPS {label}: {math.hypot(n, e):.2f} m (body forward {body_f:+.2f} m, right "
                         f"{body_r:+.2f} m), {math.hypot(n, e) / a.time:.2f} m/s over the {a.time} s push")
        if g0 is None and a.depth == 0:
            self.say("  no GPS fix: distance not measured")

    def yaw(self):
        a, v = self.a, self.v
        sign = 1.0 if a.dir == "right" else -1.0
        p = a.power / 100.0
        self.phase = "yaw"
        turned, prev = 0.0, v.heading()
        start = time.time()
        depths = []
        if a.time:
            self.say(f"Yaw {a.dir} at {a.power:.0f}% for {a.time} s (open loop) ...")
            while time.time() - start < a.time:
                self.step(yaw=sign * p * self.ramp(time.time() - start), thr=self.depth_stick(a.depth))
                turned += wrap180(v.heading() - prev)
                prev = v.heading()
                depths.append(v.depth())
        else:
            self.say(f"Yaw {a.dir} {a.angle} deg at up to {a.power:.0f}% ...")
            limit = 15.0 + a.angle / 5.0
            rate, t_prev = 0.0, time.time()
            while True:
                # release early by the turn rate x --yaw-lead: the vehicle keeps turning after it
                remaining = a.angle - sign * turned
                if remaining < a.yaw_tol + abs(rate) * a.yaw_lead:
                    break
                if time.time() - start > limit:
                    raise Abort(f"turned only {sign * turned:.0f} of {a.angle} deg in {limit:.0f} s")
                # slow down over the last 45 deg; the minimum clears the yaw stick dead zone
                cmd = clamp(remaining / 45.0, 0.0, 1.0) * p
                cmd = max(cmd, min(p, 0.15))
                self.step(yaw=sign * cmd * self.ramp(time.time() - start), thr=self.depth_stick(a.depth))
                delta = wrap180(v.heading() - prev)
                turned += delta
                prev = v.heading()
                dt, t_prev = time.time() - t_prev, time.time()
                rate += (delta / max(dt, 1e-3) - rate) * 0.3
                depths.append(v.depth())
        elapsed = time.time() - start
        settle_end = time.time() + a.coast
        self.phase = "coast"
        while time.time() < settle_end:
            self.step(thr=self.depth_stick(a.depth))
            turned += wrap180(v.heading() - prev)
            prev = v.heading()
        self.say(f"Result yaw: turned {sign * turned:.1f} deg {a.dir} in {elapsed:.1f} s "
                 f"({abs(turned) / max(elapsed, 0.1):.0f} deg/s avg incl. coast)"
                 + ("" if a.time else f", error {sign * turned - a.angle:+.1f} deg after {a.coast} s settle"))
        if a.depth > 0:
            self.stats(depths, "depth", "m")

    def waypoint(self):
        a, v = self.a, self.v
        min_fix = 3 if a.allow_no_rtk else 5
        start = dict(v.gps)
        targets = []
        for kind, x, y, label in a.targets:
            if kind == "ne":
                lat, lon = offset_latlon(start["lat"], start["lon"], x, y)
            else:
                lat, lon = x, y
            n, e = ne_offset(start["lat"], start["lon"], lat, lon)
            if math.hypot(n, e) > a.max_distance:
                raise Abort(f"waypoint {label} is {math.hypot(n, e):.0f} m from the start "
                            f"(> --max-distance {a.max_distance:.0f} m)")
            targets.append((lat, lon, label))
        if a.return_home:
            targets.append((start["lat"], start["lon"], "start"))
        p = a.power / 100.0
        for i, (lat, lon, label) in enumerate(targets, 1):
            self.phase = f"wp{i}"
            n, e = ne_offset(v.gps["lat"], v.gps["lon"], lat, lon)
            dist0 = math.hypot(n, e)
            limit = a.wp_timeout or (30.0 + dist0 / 0.15)
            self.say(f"Waypoint {i}/{len(targets)} {label}: {lat:.7f}, {lon:.7f}, {dist0:.1f} m away")
            t_start = time.time()
            lost_since = None
            cog_err = []
            while True:
                v.update()
                if not v.gps_ok(min_fix):
                    lost_since = lost_since or time.time()
                    if time.time() - lost_since > a.gps_loss_timeout:
                        raise Abort(f"GPS below {GPS_FIX[min_fix]} or silent for {a.gps_loss_timeout:.0f} s")
                    self.step(note="gps_wait")
                    continue
                lost_since = None
                n, e = ne_offset(v.gps["lat"], v.gps["lon"], lat, lon)
                dist = math.hypot(n, e)
                n0, e0 = ne_offset(start["lat"], start["lon"], v.gps["lat"], v.gps["lon"])
                if math.hypot(n0, e0) > a.max_distance + 20.0:
                    raise Abort(f"{math.hypot(n0, e0):.0f} m from the start: geofence")
                if dist < a.radius:
                    break
                if time.time() - t_start > limit:
                    raise Abort(f"waypoint {label} not reached in {limit:.0f} s ({dist:.1f} m left)")
                err = wrap180(math.degrees(math.atan2(e, n)) - v.heading())
                yaw = clamp(err / 45.0, -1.0, 1.0) * a.yaw_power / 100.0
                if abs(err) > a.yaw_tol and abs(yaw) < 0.15:
                    yaw = math.copysign(min(0.15, a.yaw_power / 100.0), err)
                elif abs(err) <= a.yaw_tol:
                    yaw = 0.0
                fwd = 0.0
                if abs(err) < 60.0:
                    fwd = p * clamp(dist / a.slow_distance, 0.4, 1.0) * math.cos(math.radians(err))
                if v.gps["vel"] > 0.3:
                    cog_err.append(wrap180(v.gps["cog"] - v.heading()))
                self.step(fwd=fwd, yaw=yaw, note=f"dist={dist:.2f} brg_err={err:.0f}")
            self.say(f"  reached in {time.time() - t_start:.0f} s ({dist:.2f} m from the point)")
            if cog_err:
                cog_err.sort()
                self.say(f"  course over ground - compass heading while moving: median "
                         f"{cog_err[len(cog_err) // 2]:+.0f} deg (large = compass offset or current)")
            self.phase = f"wp{i}_hold"
            end = time.time() + a.wp_hold
            drift = []
            while time.time() < end:
                self.step()
                if v.gps_ok(3):
                    n, e = ne_offset(lat, lon, v.gps["lat"], v.gps["lon"])
                    drift.append(math.hypot(n, e))
            if drift:
                self.say(f"  after {a.wp_hold} s hold: {drift[-1]:.2f} m from the point (max {max(drift):.2f})")

    def where(self):
        v = self.v
        print("Ctrl+C to stop. Lines: fix, sats, hAcc, lat, lon, heading")
        try:
            while True:
                v.update()
                if v.gps:
                    g = v.gps
                    print(f"\r{GPS_FIX.get(g['fix'], g['fix']):9s} {g['sats']:2d} sats hAcc {g['hacc']:5.2f} m  "
                          f"{g['lat']:.7f},{g['lon']:.7f}  heading {v.heading():5.1f}   ", end="", flush=True)
                time.sleep(0.2)
        except KeyboardInterrupt:
            if v.gps:
                print(f"\nLast: --wp {v.gps['lat']:.7f},{v.gps['lon']:.7f}")

    # ---- run ------------------------------------------------------------------------------
    def finish(self, reason):
        v = self.v
        self.phase = "finish"
        if v.channels is None:
            return
        for _ in range(10):
            v.send_sticks(0, 0, 0, 0)
            time.sleep(0.05)
        disarm = not self.a.no_disarm or reason
        if disarm and v.armed:
            if not v.arm(False):
                print("  normal disarm refused, forcing")
                if not v.arm(False, force=True):
                    print("!!! DISARM FAILED: disarm from QGC or the kill switch NOW")
        v.release_sticks()
        signal.signal(signal.SIGINT, signal.default_int_handler)   # Ctrl+C may end the watch below
        self.say(f"Ended: {reason or 'done'}; vehicle {'ARMED (--no-disarm)' if v.armed else 'disarmed'}")
        if self.dived and not v.armed:
            self.phase = "float_up"
            print("Floating up (Ctrl+C to stop watching) ...", flush=True)
            end = time.time() + self.a.surface_watch
            t_start = time.time()
            try:
                while time.time() < end:
                    v.update()
                    self.write_log()
                    if v.depth_fresh() and v.depth() < 0.25:
                        self.say(f"  surfaced (depth {v.depth():.2f} m) {time.time() - t_start:.1f} s after disarm")
                        break
                    time.sleep(0.1)
                else:
                    self.say(f"  still at {v.depth():.2f} m after {self.a.surface_watch:.0f} s: "
                             "check buoyancy")
            except KeyboardInterrupt:
                pass

    def run(self):
        a, v = self.a, self.v
        self.preflight()
        if a.test == "where":
            self.where()
            return 0
        self.print_plan()
        if a.dry_run:
            print("Dry run: nothing armed or commanded.")
            return 0
        if not a.yes:
            try:
                answer = input("Type 'go' to ARM and start (area clear, kill switch ready): ").strip()
            except EOFError:
                answer = ""
            if answer != "go":
                print("Not started.")
                return 1
        for i in range(a.countdown, 0, -1):
            print(f"  arming in {i} ...", flush=True)
            time.sleep(1.0)

        self.open_log()
        self.say(f"{datetime.now():%Y-%m-%d %H:%M:%S} {' '.join(sys.argv[1:])}")
        reason = None
        try:
            v.set_mode(a.mode)
            if not v.arm(True):
                raise Abort("arming refused (see the [pixhawk] messages above)")
            self.say(f"Armed in {a.mode}, battery {v.volt:.1f} V")
            self.t0 = time.time()
            self.deadline = self.t0 + a.timeout
            self.next_t = time.time()
            if a.test == "dive" or a.depth > 0:
                self.dive(a.depth)
            if a.test == "dive":
                self.say(f"Holding {a.depth:.2f} m for {a.hold} s ...")
                depths, headings = self.hold(a.depth, a.hold)
                self.say("Result dive hold:")
                self.stats(depths, "depth", "m")
                self.stats(headings, "heading drift", "deg", ".1f")
            else:
                if a.depth > 0 and a.settle > 0:
                    self.hold(a.depth, a.settle, "settle")
                {"yaw": self.yaw, "waypoint": self.waypoint}.get(a.test, self.linear)()
        except Abort as e:
            reason = f"ABORT: {e}"
            print("!!! " + reason, flush=True)
        except KeyboardInterrupt:
            reason = "stopped by the operator (Ctrl+C / SIGTERM / SIGHUP)"
            print("\n!!! " + reason, flush=True)
        finally:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, signal.SIG_IGN)
            self.finish(reason)
            signal.signal(signal.SIGINT, signal.default_int_handler)
            self.log_file.close()
            with open(self.summary_path, "w") as f:
                f.write("\n".join(self.summary) + "\n")
            print(f"Log: {self.log_path}\n     {self.summary_path}")
        return 1 if reason else 0


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("test", choices=["dive", "forward", "backward", "sway", "yaw", "waypoint", "where"])
    g = p.add_argument_group("manoeuvre")
    g.add_argument("--depth", type=float, default=None,
                   help="depth in m (dive: required; other tests: 0 = at the surface, default)")
    g.add_argument("--hold", type=float, default=10.0, help="dive: seconds to hold the depth (default 10)")
    g.add_argument("--time", type=float, default=None,
                   help="forward/backward/sway: thrust seconds (default 5); yaw: open loop seconds instead of --angle")
    g.add_argument("--dir", choices=["left", "right"], default="right", help="sway / yaw direction (default right)")
    g.add_argument("--angle", type=float, default=90.0, help="yaw: degrees to turn on the compass (default 90)")
    g.add_argument("--power", type=float, default=None,
                   help="stick %% of full (default 30; yaw 25); above 50 needs --allow-high-power")
    g.add_argument("--dive-power", type=float, default=35.0,
                   help="max throttle stick %% while diving / correcting depth (default 35)")
    g.add_argument("--settle", type=float, default=3.0, help="seconds to hold the depth before the move (default 3)")
    g.add_argument("--coast", type=float, default=3.0,
                   help="neutral seconds after the move, logged before disarming (default 3)")
    g.add_argument("--ramp", type=float, default=1.0, help="stick ramp-up seconds (default 1)")
    g.add_argument("--mode", choices=["STABILIZE", "ALT_HOLD", "MANUAL"], default=None,
                   help="flight mode (default STABILIZE at the surface, ALT_HOLD below)")
    g.add_argument("--no-disarm", action="store_true",
                   help="surface tests: stay armed with neutral sticks at the end (aborts always disarm)")
    w = p.add_argument_group("waypoint")
    w.add_argument("--ne", action="append", default=[], metavar="N,E",
                   help="waypoint N metres north, E metres east of the start fix (repeatable)")
    w.add_argument("--wp", action="append", default=[], metavar="LAT,LON", help="absolute waypoint (repeatable)")
    w.add_argument("--file", help="waypoint file: one 'LAT,LON' or 'ne N,E' per line, # comments")
    w.add_argument("--return-home", action="store_true", help="go back to the start fix at the end")
    w.add_argument("--radius", type=float, default=1.5, help="arrival radius in m (default 1.5)")
    w.add_argument("--wp-hold", type=float, default=5.0, help="neutral seconds at each waypoint (default 5)")
    w.add_argument("--slow-distance", type=float, default=4.0,
                   help="slow down inside this distance, to 40%% of --power (default 4 m)")
    w.add_argument("--yaw-power", type=float, default=25.0, help="max yaw stick %% while steering (default 25)")
    w.add_argument("--max-distance", type=float, default=60.0,
                   help="refuse waypoints farther than this from the start; geofence at +20 m (default 60)")
    w.add_argument("--wp-timeout", type=float, default=None,
                   help="seconds per waypoint (default 30 + distance / 0.15 m/s)")
    w.add_argument("--gps-loss-timeout", type=float, default=5.0, help="abort after this long without a fix")
    w.add_argument("--allow-no-rtk", action="store_true", help="accept a plain 3D fix instead of RTK float/fixed")
    s = p.add_argument_group("safety and link")
    s.add_argument("--max-depth", type=float, default=None,
                   help="abort (disarm) below this depth (default target + 1.0 m, surface tests 1.0 m)")
    s.add_argument("--allow-deep", action="store_true", help="allow a target depth over 3 m")
    s.add_argument("--allow-high-power", action="store_true", help="allow --power / --dive-power over 50%%")
    s.add_argument("--timeout", type=float, default=None, help="hard run time limit in s (default per test)")
    s.add_argument("--dive-timeout", type=float, default=None, help="seconds to reach the depth (default 15 + 15/m)")
    s.add_argument("--yaw-tol", type=float, default=4.0, help="yaw / steering tolerance in deg (default 4)")
    s.add_argument("--yaw-lead", type=float, default=0.5,
                   help="yaw: release the stick this many seconds of turn rate early (default 0.5)")
    s.add_argument("--depth-tol", type=float, default=0.25,
                   help="depth error that starts a correction, m (default 0.25)")
    s.add_argument("--surface-watch", type=float, default=30.0, help="seconds to watch the float-up after a dive")
    s.add_argument("--water", choices=["fresh", "salt"], default="fresh", help="water density for the depth")
    s.add_argument("--depth-msg", default="SCALED_PRESSURE2", help="Bar30 pressure message (default SCALED_PRESSURE2)")
    s.add_argument("--connect", default="udpin:127.0.0.1:14552", help="MAVLink link (default MAVProxy out 14552)")
    s.add_argument("--channels", default="2,1,3,4",
                   help="forward,lateral,throttle,yaw channels if RCMAP_* cannot be read (default 2,1,3,4)")
    s.add_argument("--countdown", type=int, default=3, help="seconds between 'go' and arming")
    s.add_argument("--yes", action="store_true", help="do not ask for 'go'")
    s.add_argument("--ignore-stack", action="store_true", help="run even though teleop / the mission stack runs")
    s.add_argument("--dry-run", action="store_true", help="connect, check and print the plan; never arm")
    s.add_argument("--log-dir", default=os.path.expanduser(os.environ.get("BANYU_MOVE_LOG_DIR", "~/banyu_bags/move_tests")))
    a = p.parse_args()

    if a.test == "dive":
        if a.depth is None or a.depth <= 0:
            p.error("dive needs --depth > 0")
    a.depth = a.depth or 0.0
    if a.depth < 0:
        p.error("--depth must be >= 0")
    if a.test == "waypoint" and a.depth > 0:
        p.error("waypoint runs at the surface (GPS); leave --depth out")
    if a.depth > 3.0 and not a.allow_deep:
        p.error(f"--depth {a.depth} m is over 3 m: add --allow-deep")
    if a.power is None:
        a.power = 25.0 if a.test == "yaw" else 30.0
    if a.time is None and a.test in LINEAR:
        a.time = 5.0
    for name in ("power", "dive_power", "yaw_power"):
        val = getattr(a, name)
        if not 0 < val <= 100:
            p.error(f"--{name.replace('_', '-')} must be in (0, 100]")
        if val > 50 and not a.allow_high_power:
            p.error(f"--{name.replace('_', '-')} {val:.0f}% is over 50%: add --allow-high-power")
    a.mode = a.mode or ("ALT_HOLD" if a.depth > 0 or a.test == "dive" else "STABILIZE")
    if (a.depth > 0 or a.test == "dive") and a.mode != "ALT_HOLD":
        p.error("below the surface the depth hold needs --mode ALT_HOLD")
    if a.no_disarm and (a.depth > 0 or a.test == "dive"):
        p.error("--no-disarm is for surface tests only: under water the run must end disarmed")
    a.depth_limit = a.max_depth if a.max_depth is not None else (a.depth + 1.0 if a.depth > 0 else 1.0)
    if a.depth_limit <= a.depth:
        p.error("--max-depth must be deeper than --depth")

    a.targets, a.targets_desc = [], []
    lines = [f"ne {x}" for x in a.ne] + list(a.wp)
    if a.file:
        try:
            lines += [l.split("#", 1)[0].strip() for l in open(a.file)]
        except OSError as e:
            p.error(f"--file: {e}")
    for line in filter(None, lines):
        rel = line.startswith("ne ")
        try:
            x, y = (float(t) for t in line[3 if rel else 0:].replace(",", " ").split())
        except ValueError:
            p.error(f"bad waypoint '{line}': want 'LAT,LON' or 'ne N,E'")
        label = f"#{len(a.targets) + 1}"
        a.targets.append(("ne" if rel else "abs", x, y, label))
        a.targets_desc.append(f"{label}: {x:+.1f} m north, {y:+.1f} m east of the start" if rel
                              else f"{label}: {x:.7f}, {y:.7f}")
    if a.test == "waypoint" and not a.targets:
        p.error("waypoint needs --ne N,E, --wp LAT,LON or --file")

    if a.timeout is None:
        if a.test == "dive":
            a.timeout = 30.0 + 15.0 * a.depth + a.hold + 30.0
        elif a.test == "waypoint":
            a.timeout = 600.0
        else:
            a.timeout = (30.0 + 15.0 * a.depth + a.settle if a.depth > 0 else 0.0) + (a.time or 60.0) + a.coast + 30.0
    return a


def main():
    a = parse_args()
    # SSH drop (SIGHUP) or kill (SIGTERM) must end like Ctrl+C: neutral sticks and disarm.
    # After a hangup the terminal is gone and print() would raise before the disarm.
    def stop(signum, frame):
        if signum == signal.SIGHUP:
            sys.stdout = sys.stderr = open(os.devnull, "w")
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    sys.exit(Run(a).run())


if __name__ == "__main__":
    main()
