#!/usr/bin/env python3
"""Compare a replay run's path with the manual (teach) run it replays, in simulation.

Both runs must be recorded with ground truth: launch_por.sh --sim --record
(/sim/ground_truth/odom). Teach with the safe profile, extract the track with
por_teach_extract.py, then fly it with --profile replay --record.

    python3 tools/scripts/por/por_path_compare.py ~/banyu_bags/<date>_por_replay_sim

The replay run folder holds a copy of the track it flew (params/*.csv); the track names its
teach bag and start time. Override with --track / --teach. The two runs are lined up on
track time (the replay's own clock, so depth pauses do not count as drift) and each path is
expressed relative to its own start pose. Writes <replay run>/path_compare.png.
"""
import argparse
import bisect
import glob
import math
import os
import sys

GROUND_TRUTH = "/sim/ground_truth/odom"
FEEDBACK = "/banyu/actions/replay_track/_action/feedback"
# POR world spawn poses (banyu_bringup/launch/gazebo_por.launch.py), for the plot only
GATE_X, GATE_POST_Y, BUOY_X = -4.0, 1.0, 6.0


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("replay", help="replay run folder (or its bag/ directory)")
    parser.add_argument("--teach", help="teach run folder (default: the track's source bag)")
    parser.add_argument("--track", help="track .csv (default: the copy in <replay>/params/)")
    parser.add_argument("--step", type=float, default=0.5, help="comparison step in seconds")
    parser.add_argument("--plot", help="output image (default: <replay run>/path_compare.png)")
    parser.add_argument("--no-plot", action="store_true", help="print the numbers only")
    return parser.parse_args()


def bag_dir(run):
    run = os.path.abspath(os.path.expanduser(run))
    path = os.path.join(run, "bag") if os.path.isdir(os.path.join(run, "bag")) else run
    if not os.path.isfile(os.path.join(path, "metadata.yaml")):
        sys.exit(f"No metadata.yaml in {path} (after a power cut run recover_bag.sh first).")
    return path


def run_dir(bag):
    return os.path.dirname(bag) if os.path.basename(bag) == "bag" else bag


def read_track_header(path):
    header = {}
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            key, _, value = line[1:].partition(":")
            header[key.strip()] = value.strip()
    times = []
    with open(path) as f:
        rows = [l for l in f if l.strip() and not l.startswith("#")]
    for line in rows[1:]:
        times.append(float(line.split(",")[0]))
    return header, times[-1]


def read_topics(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in types]
    out = {t: [] for t in topics}
    if not present:
        return out
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    classes = {t: get_message(types[t]) for t in present}
    while reader.has_next():
        topic, raw, ns = reader.read_next()
        out[topic].append((ns, deserialize_message(raw, classes[topic])))
    return out


def vehicle_poses(samples):
    """[(ns, x, y, z, yaw)] from the ground-truth odometry (vehicle world pose)."""
    poses = []
    for ns, msg in samples:
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        poses.append((ns, p.x, p.y, p.z, yaw))
    poses.sort()
    return poses


def interp(times, rows, t):
    i = bisect.bisect_right(times, t)
    if i == 0 or i == len(times):
        return None
    t0, t1 = times[i - 1], times[i]
    a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    r0, r1 = rows[i - 1], rows[i]
    yaw = r0[3] + a * math.remainder(r1[3] - r0[3], 2 * math.pi)
    return tuple(r0[k] + a * (r1[k] - r0[k]) for k in range(3)) + (yaw,)


def relative(pose, start):
    """Pose in the frame of the start pose (x forward, y left at the start)."""
    dx, dy = pose[0] - start[0], pose[1] - start[1]
    c, s = math.cos(start[3]), math.sin(start[3])
    return (c * dx + s * dy, -s * dx + c * dy, pose[2] - start[2],
            math.remainder(pose[3] - start[3], 2 * math.pi))


def main():
    args = parse_args()
    replay_bag = bag_dir(args.replay)
    track = args.track
    if not track:
        found = sorted(glob.glob(os.path.join(run_dir(replay_bag), "params", "*.csv")))
        if not found:
            sys.exit(f"No track copy in {run_dir(replay_bag)}/params/; pass --track.")
        track = found[0]
    header, duration = read_track_header(track)
    teach_bag = bag_dir(args.teach) if args.teach else header.get("source")
    if not teach_bag or not os.path.isdir(teach_bag):
        sys.exit(f"Teach bag '{teach_bag}' not found; pass --teach.")
    if "start_ns" not in header:
        sys.exit(f"{track} has no start_ns (made before por_path_compare existed): "
                 "re-run por_teach_extract.py on the teach bag.")
    teach_start_ns = int(header["start_ns"])

    print(f"Track  : {track} ({duration:.1f} s)")
    print(f"Teach  : {teach_bag}")
    print(f"Replay : {replay_bag}")
    teach = vehicle_poses(read_topics(teach_bag, [GROUND_TRUTH])[GROUND_TRUTH])
    replay_data = read_topics(replay_bag, [GROUND_TRUTH, FEEDBACK])
    replay = vehicle_poses(replay_data[GROUND_TRUTH])
    feedback = [(ns, m.feedback.track_time_s, m.feedback.paused) for ns, m in replay_data[FEEDBACK]]
    for name, poses in (("teach", teach), ("replay", replay)):
        if not poses:
            sys.exit(f"No {GROUND_TRUTH} in the {name} bag: record the sim "
                     "run with launch_por.sh --sim --record.")
    if not feedback:
        sys.exit(f"No {FEEDBACK} in the replay bag: was the replay mission started?")

    # Teach: track time = bag time since the track start. Replay: the server's track clock.
    teach_t = [(ns - teach_start_ns) * 1e-9 for ns, *_ in teach]
    teach_rows = [p[1:] for p in teach]
    fb_ns = [f[0] for f in feedback]
    fb_track = [f[1] for f in feedback]
    replay_t, replay_rows = [], []
    for ns, *pose in replay:
        if fb_ns[0] <= ns <= fb_ns[-1]:
            i = bisect.bisect_right(fb_ns, ns)
            j = min(i, len(fb_ns) - 1)
            t0, t1 = fb_ns[i - 1], fb_ns[j]
            a = (ns - t0) / (t1 - t0) if t1 > t0 else 0.0
            tt = fb_track[i - 1] + a * (fb_track[j] - fb_track[i - 1])
            if replay_t and tt <= replay_t[-1]:
                replay_rows[-1] = tuple(pose)  # clock held (paused): keep the latest pose
                continue
            replay_t.append(tt)
            replay_rows.append(tuple(pose))
    paused_s = sum((fb_ns[k] - fb_ns[k - 1]) * 1e-9 for k in range(1, len(feedback))
                   if feedback[k - 1][2])

    teach_start = interp(teach_t, teach_rows, 0.0) or teach_rows[0]
    replay_start = replay_rows[0]
    samples = []
    t = 0.0
    while t <= duration + 1e-9:
        a = interp(teach_t, teach_rows, t)
        b = interp(replay_t, replay_rows, t)
        if a and b:
            ra, rb = relative(a, teach_start), relative(b, replay_start)
            samples.append((t, a, b, ra, rb, math.hypot(ra[0] - rb[0], ra[1] - rb[1])))
        t += args.step
    if len(samples) < 2:
        sys.exit("The runs do not overlap in track time; check the bags and the track.")

    dx = replay_start[0] - teach_start[0]
    dy = replay_start[1] - teach_start[1]
    dyaw = math.degrees(math.remainder(replay_start[3] - teach_start[3], 2 * math.pi))
    print(f"\nStart offset replay - teach: {math.hypot(dx, dy):.2f} m, heading {dyaw:+.0f} deg "
          "(removed below: each path is relative to its own start)")
    print(f"Replay paused for depth: {paused_s:.1f} s\n")
    print(" track | teach fwd  left depth | replay fwd  left depth | error  heading diff")
    every = max(1, round(5.0 / args.step))
    for k, (t, a, b, ra, rb, err) in enumerate(samples):
        if k % every == 0 or k == len(samples) - 1:
            print(f" {t:5.1f} | {ra[0]:8.2f} {ra[1]:5.2f} {-a[2]:5.2f} | {rb[0]:9.2f} {rb[1]:5.2f}"
                  f" {-b[2]:5.2f} | {err:5.2f}  {math.degrees(math.remainder(rb[3] - ra[3], 2 * math.pi)):+6.0f}")
    errors = [s[5] for s in samples]
    worst = max(samples, key=lambda s: s[5])
    end = samples[-1]
    print(f"\nPosition error: mean {sum(errors) / len(errors):.2f} m, max {worst[5]:.2f} m "
          f"at {worst[0]:.1f} s, end {end[5]:.2f} m")
    print(f"Depth error at the end: {abs(end[1][2] - end[2][2]):.2f} m")

    if args.no_plot:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_path, ax_err, ax_depth) = plt.subplots(
        1, 3, figsize=(16, 5.5), gridspec_kw={"width_ratios": [2, 1.2, 1.2]})
    ax_path.plot([s[1][0] for s in samples], [s[1][1] for s in samples], label="teach", lw=2)
    # Replay drawn from the teach start so the shapes overlay even if the starts differ
    c, s_ = math.cos(teach_start[3]), math.sin(teach_start[3])
    rx = [teach_start[0] + c * r[0] - s_ * r[1] for r in (x[4] for x in samples)]
    ry = [teach_start[1] + s_ * r[0] + c * r[1] for r in (x[4] for x in samples)]
    ax_path.plot(rx, ry, label="replay (from teach start)", lw=2)
    ax_path.plot(teach_start[0], teach_start[1], "ko", label="start")
    ax_path.plot([GATE_X, GATE_X], [-GATE_POST_Y, GATE_POST_Y], "s", color="goldenrod",
                 label="gate posts")
    ax_path.plot(BUOY_X, 0.0, "o", color="orange", ms=10, label="marker")
    ax_path.set_aspect("equal")
    ax_path.set_xlabel("x (m, east)")
    ax_path.set_ylabel("y (m, north)")
    ax_path.set_title("Path (top view, ground truth)")
    ax_path.grid(True, alpha=0.3)
    ax_path.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=5, fontsize=8)
    ax_err.plot([s[0] for s in samples], errors)
    ax_err.set_xlabel("track time (s)")
    ax_err.set_ylabel("position error (m)")
    ax_err.set_title("Replay vs teach")
    ax_err.grid(True, alpha=0.3)
    ax_depth.plot([s[0] for s in samples], [-s[1][2] for s in samples], label="teach")
    ax_depth.plot([s[0] for s in samples], [-s[2][2] for s in samples], label="replay")
    ax_depth.invert_yaxis()
    ax_depth.set_xlabel("track time (s)")
    ax_depth.set_ylabel("depth (m)")
    ax_depth.set_title("Depth")
    ax_depth.grid(True, alpha=0.3)
    ax_depth.legend(fontsize=8)
    fig.tight_layout()
    out = args.plot or os.path.join(run_dir(replay_bag), "path_compare.png")
    fig.savefig(out, dpi=110)
    print(f"Plot   : {out}")


if __name__ == "__main__":
    main()
