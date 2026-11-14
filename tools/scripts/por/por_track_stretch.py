#!/usr/bin/env python3
"""Lengthen legs of a POR replay track, e.g. when a lower battery makes a leg come up short.

The replay repeats the recorded sticks by time, so a weaker battery covers less distance
in the same time. --hold T:S keeps the sticks (and the recorded heading and depth) of
track time T going for S more seconds and shifts everything after it. Pick T in the
steady part of the leg (see --list); holding a sample with yaw stick adds rotation.

    python3 tools/scripts/por/por_track_stretch.py full2 --list
    python3 tools/scripts/por/por_track_stretch.py full2 --hold 18:1.5 --name full2_long
    python3 tools/scripts/por/por_track_stretch.py full2 --hold 18:1.5 --hold 60:0.8

Tracks are looked up in $BANYU_REPLAY_DIR (default ~/banyu_replays); the original is
kept unless --name is the same track and --force is given.
"""
import argparse
import bisect
import os
import sys
from datetime import datetime

COLUMNS = ["time_s", "forward_pwm", "lateral_pwm", "yaw_pwm", "heading_rad", "depth_m"]
DEADBAND = 25


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("track", help="track name in $BANYU_REPLAY_DIR or a .csv path")
    parser.add_argument("--list", action="store_true", help="list the stick segments and exit")
    parser.add_argument("--hold", action="append", default=[], metavar="T:SECONDS",
                        help="keep the sticks of track time T for SECONDS longer (repeatable)")
    parser.add_argument("--name", help="output track name (default: <track>_edited)")
    parser.add_argument("--force", action="store_true", help="overwrite an existing track")
    return parser.parse_args()


def replay_dir():
    return os.environ.get("BANYU_REPLAY_DIR") or os.path.expanduser("~/banyu_replays")


def track_path(name):
    if os.path.sep in name or name.endswith(".csv") and os.path.isfile(name):
        return os.path.abspath(os.path.expanduser(name))
    return os.path.join(replay_dir(), name if name.endswith(".csv") else name + ".csv")


def read_track(path):
    comments, rows = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                comments.append(line)
            elif line.split(",")[0] == COLUMNS[0]:
                if line.split(",") != COLUMNS:
                    sys.exit(f"{path}: unexpected columns {line}")
            else:
                v = line.split(",")
                rows.append([float(v[0]), int(v[1]), int(v[2]), int(v[3]), float(v[4]), float(v[5])])
    if len(rows) < 2:
        sys.exit(f"{path}: fewer than two samples")
    return comments, rows


def stick_pattern(row):
    return tuple(0 if abs(v) <= DEADBAND else (1 if v > 0 else -1) for v in row[1:4])


def describe(pattern):
    names = (("forward", "backward"), ("right", "left"), ("yaw right", "yaw left"))
    moves = [names[i][0 if s > 0 else 1] for i, s in enumerate(pattern) if s]
    return " + ".join(moves) or "sticks neutral"


def list_segments(rows):
    segments = []
    for row in rows:
        pattern = stick_pattern(row)
        if segments and segments[-1][0] == pattern:
            segments[-1][2] = row
        else:
            segments.append([pattern, row, row])
    print(f"{'start':>6} {'end':>6} {'dur':>5}  {'motion':<26} {'fwd':>9} {'lat':>9} {'yaw':>9}")
    for pattern, first, last in segments:
        if last[0] - first[0] < 0.3:
            continue
        inside = [r for r in rows if first[0] <= r[0] <= last[0]]
        span = [f"{min(r[c] for r in inside):4d}..{max(r[c] for r in inside):<4d}" for c in (1, 2, 3)]
        print(f"{first[0]:6.1f} {last[0]:6.1f} {last[0] - first[0]:5.1f}  {describe(pattern):<26} "
              f"{span[0]} {span[1]} {span[2]}")
    print(f"track length {rows[-1][0]:.1f} s")


def hold(rows, t, seconds):
    times = [r[0] for r in rows]
    if not 0.0 <= t < times[-1]:
        sys.exit(f"--hold time {t} is outside the track (0 .. {times[-1]:.1f} s)")
    i = bisect.bisect_right(times, t) - 1
    period = (times[-1] - times[0]) / (len(times) - 1)
    held = rows[i]
    inserted, k = [], 1
    while k * period < seconds:
        inserted.append([held[0] + k * period] + held[1:])
        k += 1
    shifted = [[r[0] + seconds] + r[1:] for r in rows[i + 1:]]
    return rows[:i + 1] + inserted + shifted, held


def main():
    args = parse_args()
    source = track_path(args.track)
    if not os.path.isfile(source):
        sys.exit(f"No track {source}")
    comments, rows = read_track(source)
    if args.list or not args.hold:
        list_segments(rows)
        if not args.hold:
            return

    holds = []
    for spec in args.hold:
        try:
            t, seconds = (float(v) for v in spec.split(":"))
        except ValueError:
            sys.exit(f"--hold expects T:SECONDS, got '{spec}'")
        if seconds <= 0.0:
            sys.exit(f"--hold {spec}: SECONDS must be positive")
        holds.append((t, seconds))
    # Apply later holds first so every T refers to the original track time
    edits = []
    for t, seconds in sorted(holds, reverse=True):
        rows, held = hold(rows, t, seconds)
        edits.append((t, seconds, held))
        if stick_pattern(held)[2]:
            print(f"WARNING: the sample at {t:.1f} s has yaw stick {held[3]}; holding it adds rotation.")

    name = args.name or os.path.splitext(os.path.basename(source))[0] + "_edited"
    out = track_path(name)
    if os.path.abspath(out) == os.path.abspath(source) and not args.force:
        sys.exit("Refusing to overwrite the source track; pass --force or choose another --name.")
    if os.path.exists(out) and not args.force:
        sys.exit(f"{out} exists; pass --force to overwrite or choose another --name.")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        for line in comments:
            f.write(line + "\n")
        f.write(f"# edited: {datetime.now().isoformat(timespec='seconds')} from {source}; "
                + ", ".join(f"hold {t:.2f} s +{s:.2f} s" for t, s, _ in sorted(edits))
                + " (track times after a hold no longer match the teach bag)\n")
        f.write(",".join(COLUMNS) + "\n")
        for r in rows:
            f.write(f"{r[0]:.3f},{r[1]:d},{r[2]:d},{r[3]:d},{r[4]:.4f},{r[5]:.3f}\n")

    print(f"Wrote {out}")
    for t, seconds, held in sorted(edits):
        print(f"  {t:6.1f} s: {describe(stick_pattern(held))} (fwd {held[1]}, lat {held[2]}, "
              f"yaw {held[3]}) held {seconds:.2f} s longer")
    print(f"  track length {rows[-1][0]:.1f} s")


if __name__ == "__main__":
    main()
