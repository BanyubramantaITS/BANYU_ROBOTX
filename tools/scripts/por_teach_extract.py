#!/usr/bin/env python3
"""Turn a manually driven run into a replay track for the POR replay profile.

Teach: launch the stack as usual (it records a bag), arm, drive the course with the
QGC joystick (or TX16S) in ALT_HOLD, disarm. The bag holds what the Pixhawk applied:
/mavros/rc/in (the joystick shows up there as RC overrides), heading and depth.

    python3 tools/scripts/por_teach_extract.py ~/banyu_bags/20261027_160000_por_x
    python3 tools/scripts/por_teach_extract.py <run> --list        # show armed windows
    python3 tools/scripts/por_teach_extract.py <run> --window 2 --name gate_loop

The track is cut from the first to the last stick input of the chosen armed window
(default: the last one) and written to $BANYU_REPLAY_DIR (default ~/banyu_replays).
Replay it with: launch_por.sh --real --profile replay [--replay-track <name>]

Without an RC receiver the Pixhawk reports no RC channels, so /mavros/rc/in stays empty.
Take the sticks from MAVProxy's telemetry log instead (QGC joystick MANUAL_CONTROL,
converted to stick PWM like ArduSub, following its "#Gain is NN%" messages):

    python3 tools/scripts/por_teach_extract.py <run> --tlog ~/ros_ws/mav.tlog --name rute1
"""
import argparse
import bisect
import math
import os
import re
import sys
from datetime import datetime

TOPIC_RC = "/mavros/rc/in"
TOPIC_STATE = "/mavros/state"
TOPIC_DEPTH = "/mavros/global_position/rel_alt"
TOPIC_IMU = "/mavros/imu/data"
TOPIC_POSE = "/mavros/local_position/pose"
MSG_MANUAL_CONTROL = 69
MSG_STATUSTEXT = 253
CRC_EXTRA = {MSG_MANUAL_CONTROL: 243, MSG_STATUSTEXT: 83}
COLUMNS = ["time_s", "forward_pwm", "lateral_pwm", "yaw_pwm", "heading_rad", "depth_m"]


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="run folder from ~/banyu_bags (or its bag/ directory)")
    parser.add_argument("--list", action="store_true", help="list armed windows and exit")
    parser.add_argument("--window", type=int, default=0,
                        help="armed window to extract, 1-based (default: the last)")
    parser.add_argument("--start", type=float,
                        help="cut start, seconds from bag start (default: first stick input)")
    parser.add_argument("--end", type=float,
                        help="cut end, seconds from bag start (default: last stick input + --tail)")
    parser.add_argument("--tail", type=float, default=1.0,
                        help="seconds kept after the last stick input (default 1.0)")
    parser.add_argument("--name", help="track name (default: run folder name + window)")
    parser.add_argument("--output", help="output .csv path (overrides --name)")
    parser.add_argument("--neutral", type=int, default=1500, help="neutral stick PWM")
    parser.add_argument("--deadband", type=int, default=25,
                        help="PWM deflection that counts as a stick input (default 25)")
    parser.add_argument("--forward-ch", type=int, default=2, help="RCMAP_FORWARD (1-based)")
    parser.add_argument("--lateral-ch", type=int, default=1, help="RCMAP_LATERAL (1-based)")
    parser.add_argument("--throttle-ch", type=int, default=3, help="RCMAP_THROTTLE (1-based)")
    parser.add_argument("--yaw-ch", type=int, default=4, help="RCMAP_YAW (1-based)")
    parser.add_argument("--tlog",
                        help="MAVProxy .tlog to take the sticks from (QGC joystick MANUAL_CONTROL) "
                             "instead of /mavros/rc/in, for a Pixhawk without an RC receiver")
    parser.add_argument("--js-gain", type=float, default=0.5,
                        help="--tlog: joystick gain before the first '#Gain is' message "
                             "(JS_GAIN_DEFAULT, default 0.5)")
    parser.add_argument("--force", action="store_true", help="overwrite an existing track")
    return parser.parse_args()


def bag_dir(run):
    run = os.path.abspath(os.path.expanduser(run))
    candidate = os.path.join(run, "bag")
    path = candidate if os.path.isdir(candidate) else run
    if not os.path.isfile(os.path.join(path, "metadata.yaml")):
        sys.exit(f"No metadata.yaml in {path}. After a power cut run "
                 f"banyu_bringup/scripts/recover_bag.sh {path} first.")
    return path


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (TOPIC_RC, TOPIC_STATE, TOPIC_DEPTH, TOPIC_IMU, TOPIC_POSE) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    data = {t: [] for t in (TOPIC_RC, TOPIC_STATE, TOPIC_DEPTH, TOPIC_IMU, TOPIC_POSE)}
    first = None
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        first = stamp_ns if first is None else min(first, stamp_ns)
        data[topic].append((stamp_ns, deserialize_message(raw, classes[topic])))
    if first is None:
        sys.exit(f"{path}: none of the needed topics were recorded")
    out = {}
    for topic, samples in data.items():
        samples.sort(key=lambda s: s[0])
        out[topic] = [((ns - first) * 1e-9, msg) for ns, msg in samples]
    return out, first


def x25_crc(data):
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def read_tlog(path):
    """Yield (unix_s, msgid, payload) for MANUAL_CONTROL and STATUSTEXT in a MAVProxy .tlog.

    A .tlog is a sequence of 8-byte big-endian microsecond timestamps, each followed by
    one raw MAVLink v1/v2 packet. Standard library only (no pymavlink on the vehicle).
    """
    with open(os.path.expanduser(path), "rb") as f:
        buf = f.read()
    i, n = 0, len(buf)
    while i + 18 <= n:
        ts = int.from_bytes(buf[i:i + 8], "big") * 1e-6
        magic, length = buf[i + 8], buf[i + 9]
        if magic == 0xFE:
            header, msgid, tail = 6, buf[i + 13], 0
        elif magic == 0xFD:
            header, msgid = 10, int.from_bytes(buf[i + 15:i + 18], "little")
            tail = 13 if buf[i + 10] & 0x01 else 0  # signed packet
        else:
            i += 1
            continue
        end = i + 8 + header + length + 2 + tail
        if not 1e9 < ts < 5e9 or end > n:
            i += 1
            continue
        if msgid in CRC_EXTRA:
            packet = buf[i + 8:i + 8 + header + length + 2]
            crc = x25_crc(packet[1:header + length] + bytes([CRC_EXTRA[msgid]]))
            if crc != int.from_bytes(packet[header + length:], "little"):
                i += 1
                continue
            # MAVLink 2 trims trailing zero bytes from the payload
            yield ts, msgid, packet[header:header + length] + bytes(64)
        i = end


def tlog_sticks(path, bag_first_ns, bag_end, start_gain):
    """Stick deflections (t, forward, lateral, yaw, throttle) as ArduSub applies MANUAL_CONTROL.

    ArduSub joystick.cpp: forward/lateral/yaw = 1500 + axis * 0.4 * gain, throttle =
    1500 + (z - 500) * 0.8 * gain (JS_THR_GAIN 1). The gain starts at start_gain and
    follows the vehicle's "#Gain is NN%" messages.
    """
    first = bag_first_ns * 1e-9
    gain, sticks, gains = start_gain, [], set()
    for ts, msgid, payload in read_tlog(path):
        if msgid == MSG_STATUSTEXT:
            text = payload[1:51].split(b"\0", 1)[0].decode("ascii", "replace")
            match = re.search(r"Gain is\s*(\d+)%", text)
            if match:
                gain = int(match.group(1)) / 100.0
            continue
        t = ts - first
        if t < 0.0 or t > bag_end:
            continue
        x, y, z, r = (int.from_bytes(payload[k:k + 2], "little", signed=True) for k in (0, 2, 4, 6))
        rpy = 0.4 * gain
        sticks.append((t, max(-400, min(400, round(x * rpy))), max(-400, min(400, round(y * rpy))),
                       max(-400, min(400, round(r * rpy))), round((z - 500) * 0.8 * gain)))
        gains.add(round(gain * 100))
    sticks.sort(key=lambda s: s[0])
    return sticks, sorted(gains)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(values):
    out = []
    for v in values:
        if out:
            v = out[-1] + math.remainder(v - out[-1], 2.0 * math.pi)
        out.append(v)
    return out


def interp(times, values, t):
    if not times:
        return float("nan")
    i = bisect.bisect_right(times, t)
    if i == 0:
        return values[0]
    if i == len(times):
        return values[-1]
    t0, t1 = times[i - 1], times[i]
    return values[i - 1] + (values[i] - values[i - 1]) * (t - t0) / (t1 - t0) if t1 > t0 \
        else values[i]


def armed_windows(states, bag_end):
    windows, start, modes = [], None, set()
    for t, msg in states:
        if msg.armed and start is None:
            start, modes = t, set()
        if msg.armed:
            modes.add(msg.mode)
        if not msg.armed and start is not None:
            windows.append((start, t, sorted(modes)))
            start = None
    if start is not None:
        windows.append((start, bag_end, sorted(modes)))
    return windows


def main():
    args = parse_args()
    path = bag_dir(args.run)
    print(f"Reading {path} ...")
    data, bag_first_ns = read_bag(path)
    bag_end = max(s[-1][0] for s in data.values() if s)
    windows = armed_windows(data[TOPIC_STATE], bag_end)

    def deflection(msg, channel):
        if channel < 1 or channel > len(msg.channels) or msg.channels[channel - 1] == 0:
            return 0
        return int(msg.channels[channel - 1]) - args.neutral

    # Sticks as (t, forward, lateral, yaw, throttle) PWM deflections
    if args.tlog:
        print(f"Reading sticks from {args.tlog} ...")
        rc, tlog_gains = tlog_sticks(args.tlog, bag_first_ns, bag_end, args.js_gain)
        if not rc:
            sys.exit(f"{args.tlog} has no MANUAL_CONTROL during this bag: was the QGC joystick on, "
                     "and is it the log of the MAVProxy that ran during the run?")
        stick_source = f"MANUAL_CONTROL in {os.path.abspath(os.path.expanduser(args.tlog))}"
    else:
        rc = [(t, deflection(m, args.forward_ch), deflection(m, args.lateral_ch),
               deflection(m, args.yaw_ch), deflection(m, args.throttle_ch)) for t, m in data[TOPIC_RC]]
        if not rc:
            sys.exit(f"{TOPIC_RC} is not in the bag: the sticks cannot be recovered.")
        if not any(m.channels for _, m in data[TOPIC_RC]):
            print(f"WARNING: {TOPIC_RC} has no channels (no RC receiver on the Pixhawk?). "
                  "Pass --tlog with MAVProxy's log to use the QGC joystick input.")
        stick_source = (f"{TOPIC_RC} forward CH{args.forward_ch}, lateral CH{args.lateral_ch}, "
                        f"yaw CH{args.yaw_ch}")

    def active(stick):
        return any(abs(v) > args.deadband for v in stick[1:])

    print(f"Armed windows (seconds from bag start, bag length {bag_end:.1f} s):")
    for i, (w0, w1, modes) in enumerate(windows, 1):
        n = sum(1 for s in rc if w0 <= s[0] <= w1 and active(s))
        print(f"  {i}: {w0:7.1f} - {w1:7.1f}  ({w1 - w0:5.1f} s)  modes {', '.join(modes)}"
              f"  stick samples {n}")
    if args.list:
        return
    if args.start is None or args.end is None:
        if not windows:
            sys.exit("The vehicle was never armed in this bag; pass --start and --end.")
        index = args.window if args.window else len(windows)
        if not 1 <= index <= len(windows):
            sys.exit(f"--window must be 1..{len(windows)}")
        w0, w1, _ = windows[index - 1]
        moving = [s[0] for s in rc if w0 <= s[0] <= w1 and active(s)]
        if not moving:
            sys.exit(f"Window {index} has no stick input above {args.deadband} PWM.")
        start = args.start if args.start is not None else moving[0]
        end = args.end if args.end is not None else min(w1, moving[-1] + args.tail)
    else:
        index, start, end = 0, args.start, args.end
    # Modes in force during the cut: the last state before it plus any change inside it
    before = [m.mode for t, m in data[TOPIC_STATE] if t <= start]
    modes = sorted(set(before[-1:]) | {m.mode for t, m in data[TOPIC_STATE] if start < t <= end})
    if end - start < 1.0:
        sys.exit(f"Cut {start:.1f}-{end:.1f} s is shorter than 1 s.")

    if data[TOPIC_IMU]:
        heading_src = data[TOPIC_IMU]
        heading_name = TOPIC_IMU
        heading_q = [m.orientation for _, m in heading_src]
    else:
        heading_src = data[TOPIC_POSE]
        heading_name = TOPIC_POSE
        heading_q = [m.pose.orientation for _, m in heading_src]
    if not heading_src:
        sys.exit("No heading recorded (/mavros/imu/data or /mavros/local_position/pose).")
    if not data[TOPIC_DEPTH]:
        sys.exit(f"{TOPIC_DEPTH} is not in the bag: the depth profile cannot be recovered.")
    head_t = [t for t, _ in heading_src]
    head_v = unwrap([yaw_of(q) for q in heading_q])
    depth_t = [t for t, _ in data[TOPIC_DEPTH]]
    depth_v = [-m.data for _, m in data[TOPIC_DEPTH]]
    heading0 = interp(head_t, head_v, start)

    rows, last_t = [], None
    for t, forward, lateral, yaw, _ in rc:
        if t < start or t > end or (last_t is not None and t <= last_t):
            continue
        last_t = t
        rows.append([
            t - start,
            # +right / +clockwise on the stick: the replay server expects RC deflections
            forward, lateral, yaw,
            interp(head_t, head_v, t) - heading0,
            interp(depth_t, depth_v, t)])
    if len(rows) < 2:
        sys.exit("Fewer than two RC samples in the cut.")
    shift = rows[0][0]
    for row in rows:
        row[0] -= shift
    for row in rows:
        for c in (1, 2, 3):
            row[c] = max(-500, min(500, row[c]))

    run_name = os.path.basename(os.path.dirname(path) if path.endswith("/bag") or
                                os.path.basename(path) == "bag" else path)
    if args.output:
        out = os.path.abspath(os.path.expanduser(args.output))
    else:
        directory = os.environ.get("BANYU_REPLAY_DIR") or os.path.expanduser("~/banyu_replays")
        name = args.name or (f"{run_name}_w{index}" if index else run_name)
        out = os.path.join(directory, name if name.endswith(".csv") else name + ".csv")
    if os.path.exists(out) and not args.force:
        sys.exit(f"{out} exists; pass --force to overwrite or choose another --name.")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    duration = rows[-1][0]
    rate = (len(rows) - 1) / duration if duration > 0 else 0.0
    with open(out, "w") as f:
        f.write("# BANYU POR replay track (tools/scripts/por_teach_extract.py)\n")
        f.write(f"# created: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# source: {path}\n")
        f.write(f"# cut: {start:.2f} - {end:.2f} s from bag start, window {index or 'manual'}, "
                f"modes {', '.join(modes) or 'unknown'}\n")
        # Bag timestamp of track time 0, for tools/scripts/por_path_compare.py
        f.write(f"# start_ns: {bag_first_ns + round((start + shift) * 1e9)}\n")
        f.write(f"# sticks: {stick_source}; heading from {heading_name}; depth from {TOPIC_DEPTH}\n")
        f.write(",".join(COLUMNS) + "\n")
        for row in rows:
            f.write(f"{row[0]:.3f},{row[1]:d},{row[2]:d},{row[3]:d},{row[4]:.4f},{row[5]:.3f}\n")

    print(f"\nWrote {out}")
    print(f"  {len(rows)} samples over {duration:.1f} s ({rate:.1f} Hz)")
    print(f"  max depth {max(r[5] for r in rows):.2f} m, end depth {rows[-1][5]:.2f} m")
    print(f"  heading change {math.degrees(rows[-1][4]):+.0f} deg "
          f"(range {math.degrees(min(r[4] for r in rows)):+.0f} .. "
          f"{math.degrees(max(r[4] for r in rows)):+.0f})")
    print(f"  stick max |fwd| {max(abs(r[1]) for r in rows)}, |lat| {max(abs(r[2]) for r in rows)}, "
          f"|yaw| {max(abs(r[3]) for r in rows)} PWM")
    if args.tlog:
        print(f"  joystick gain during the bag: {', '.join(f'{g}%' for g in tlog_gains)}"
              f" (start {args.js_gain * 100:.0f}% until the first '#Gain is' message)")
    if modes and modes != ["ALT_HOLD"]:
        print(f"  WARNING: driven in {', '.join(modes)}; the replay flies ALT_HOLD.")
    if rate < 5.0:
        print("  WARNING: RC sampled below 5 Hz; quick stick moves may be smoothed out.")


if __name__ == "__main__":
    main()
