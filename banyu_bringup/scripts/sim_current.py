#!/usr/bin/env python3
"""SIMULATION ONLY: a water current for the Gazebo hydrodynamics plugin (gz topic /ocean_current, gz.msgs.Vector3d).

  sim_current.py --east <m/s> --north <m/s> [--gust <m/s>] [--topic /ocean_current]

Holds the steady current and adds a gust of --gust m/s along it for 5 s of every 30 s (a boat wake). Republishes every
second so a late-starting world still gets it. Self-check: --self-check.
"""
import argparse
import math
import subprocess
import sys
import time


def current_at(t, east, north, gust, period=30.0, gust_s=5.0):
    """Current (east, north) at t seconds; the gust is along the steady current (east if there is none)."""
    if gust and (t % period) >= period - gust_s:
        norm = math.hypot(east, north)
        ux, uy = (east / norm, north / norm) if norm > 1e-9 else (1.0, 0.0)
        return east + gust * ux, north + gust * uy
    return east, north


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--east', type=float, default=0.0)
    ap.add_argument('--north', type=float, default=0.0)
    ap.add_argument('--gust', type=float, default=0.0)
    ap.add_argument('--topic', default='/ocean_current')
    ap.add_argument('--self-check', action='store_true')
    args = ap.parse_args(argv)
    if args.self_check:
        assert current_at(0.0, 0.2, 0.0, 0.1) == (0.2, 0.0)
        assert current_at(27.0, 0.2, 0.0, 0.1) == (0.30000000000000004, 0.0)
        assert current_at(27.0, 0.0, 0.0, 0.1) == (0.1, 0.0)
        print('sim_current self-check OK')
        return 0
    start = time.time()
    print(f'sim_current: steady ({args.east}, {args.north}) m/s, gust {args.gust} m/s on {args.topic}', flush=True)
    while True:
        # Gazebo hydrodynamics takes the current in the world frame (ENU here: x east, y north).
        x, y = current_at(time.time() - start, args.east, args.north, args.gust)
        subprocess.run(['gz', 'topic', '-t', args.topic, '-m', 'gz.msgs.Vector3d', '-p', f'x: {x} y: {y} z: 0'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        time.sleep(1.0)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
