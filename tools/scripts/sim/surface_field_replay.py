#!/usr/bin/env python3
"""Replay a recorded UAV buoy field (SafePassage -> /banyu/surface/obstacles) through a port of NavigateSurfaceGps' planner.

The log has one line per change of the obstacle set (docs/task2_results/20261112_uav_safe_passage.log, from the
2026-11-12 trial). Results and the parameter choice: docs/task2_surface_planning_trial.md.
Variants: obstacle memory (keep buoys that drop out of a message), planning margin added to each disc, and committing
to a detour waypoint (hysteresis). True buoys are the largest message (the first one, 8 of the 9 real buoys; the
UAV never reported the ninth); --truth clusters takes the clusters seen in most messages instead (6, the earlier
assumption). GPS error is an AR(1) process matched to the Nov 10 stationary statistics (20 s spread p90 0.33 m) plus an
optional UAV/UUV receiver bias. --skip N starts the UUV's memory after the first N messages: the UAV publishes only on
a change of its map, with volatile QoS, so a UUV stack started after the first message never hears it.
"""
import argparse
import math
import os
import random
import re

R = 6378137.0
LINE = re.compile(r'^(\d\d):(\d\d):(\d\d) OBSTACLES\[uav_safe_passage\] n=\d+ (.*)$')
PT = re.compile(r'\(([-\d.]+),([-\d.]+),r([\d.]+)\)')


def enu(lat, lon, lat0, lon0):
    return ((lon - lon0) * math.radians(1) * math.cos(math.radians(lat0)) * R, (lat - lat0) * math.radians(1) * R)


# ---- geometry (port of task4_zones.hpp, static discs only) ----
def seg_point_dist(p, a, b):
    abx, aby = b[0] - a[0], b[1] - a[1]
    l2 = abx * abx + aby * aby
    t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / l2))
    return math.hypot(p[0] - a[0] - abx * t, p[1] - a[1] - aby * t)


def seg_clear(p, q, discs):
    return all(seg_point_dist(c, p, q) >= r for c, r in discs)


def inside_any(p, discs):
    return any(math.hypot(p[0] - c[0], p[1] - c[1]) < r for c, r in discs)


def detour_aim(goal, discs):
    o = (0.0, 0.0)
    if seg_clear(o, goal, discs):
        return goal, math.hypot(*goal)
    plen = math.hypot(*goal)
    pn = (-goal[1] / plen, goal[0] / plen) if plen > 1e-6 else (0.0, 1.0)
    nodes = [o]
    for c, r in discs:
        ad = (-pn[0], -pn[1])  # static disc: axis along -path normal, as in the C++
        nrm = (-ad[1], ad[0])
        rr = r * 1.25
        mid = (goal[0] * 0.5, goal[1] * 0.5)
        for side in (-1.0, 1.0):
            for cand in ((c[0] + nrm[0] * side * rr, c[1] + nrm[1] * side * rr),
                         (c[0] + nrm[0] * side * rr, c[1] + nrm[1] * side * rr),
                         (c[0] - ad[0] * rr + nrm[0] * side * rr, c[1] - ad[1] * rr + nrm[1] * side * rr),
                         (c[0] + ad[0] * rr + nrm[0] * side * rr, c[1] + ad[1] * rr + nrm[1] * side * rr),
                         (c[0] + pn[0] * side * rr, c[1] + pn[1] * side * rr)):
                if not inside_any(cand, discs):
                    nodes.append(cand)
    nodes.append(goal)
    n = len(nodes)
    dist = [math.inf] * n
    prev = [None] * n
    done = [False] * n
    dist[0] = 0.0
    for _ in range(n):
        u = min((i for i in range(n) if not done[i]), key=lambda i: dist[i], default=None)
        if u is None or math.isinf(dist[u]):
            break
        done[u] = True
        for v in range(n):
            if done[v] or not seg_clear(nodes[u], nodes[v], discs):
                continue
            d = dist[u] + math.dist(nodes[u], nodes[v])
            if d < dist[v]:
                dist[v], prev[v] = d, u
    if math.isinf(dist[-1]):
        return None
    s = n - 1
    while prev[s] not in (0, None):
        s = prev[s]
    return nodes[s], dist[-1]


def exit_vector(discs, clearance=0.5):
    deepest = None
    for c, r in discs:
        d = math.hypot(*c) - r
        if d < 0 and (deepest is None or d < deepest[0]):
            deepest = (d, c)
    if deepest is None:
        return None
    d, c = deepest
    ln = math.hypot(*c)
    u = (-c[0] / ln, -c[1] / ln) if ln > 1e-6 else (1.0, 0.0)
    return u, -d + clearance


# ---- recorded stream ----
def load(path):
    msgs = []
    for line in open(path):
        m = LINE.match(line.strip())
        if m:
            t = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))
            msgs.append((t, [(float(a), float(b), float(r)) for a, b, r in PT.findall(m.group(4))]))
    return msgs


REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
DEFAULT_LOG = os.path.join(REPO, 'docs', 'task2_results', '20261112_uav_safe_passage.log')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', nargs='?', default=DEFAULT_LOG, help='obstacle log (default: the 2026-11-12 trial)')
    parser.add_argument('--margins', default='0,0.3,0.6,0.75,0.9', help='surface_obstacles.margin_m values to compare')
    parser.add_argument('--bias', default='0,1.0,1.5', help='UAV/UUV GPS receiver bias magnitudes, m')
    parser.add_argument('--truth', choices=('largest', 'clusters'), default='largest', help='where the true buoys come from')
    parser.add_argument('--skip', type=int, default=0, help='obstacle messages the UUV missed at the start')
    args = parser.parse_args()
    margins = [float(m) for m in args.margins.split(',')]
    biases = [(b / math.sqrt(2), b / math.sqrt(2)) for b in (float(x) for x in args.bias.split(','))]
    msgs = load(args.log)
    lat0, lon0 = msgs[0][1][0][0], msgs[0][1][0][1]
    stream = [(t, [enu(a, b, lat0, lon0) for a, b, _ in pts], pts[0][2]) for t, pts in msgs]
    # True buoys: clusters present in at least half of the messages.
    clusters = []
    for _, pts, _ in stream:
        for p in pts:
            for c in clusters:
                if math.dist(c['m'], p) < 1.5:
                    c['n'] += 1
                    c['pts'].append(p)
                    c['m'] = (sum(q[0] for q in c['pts']) / len(c['pts']), sum(q[1] for q in c['pts']) / len(c['pts']))
                    break
            else:
                clusters.append({'m': p, 'n': 1, 'pts': [p]})
    if args.truth == 'clusters':
        truth = [c['m'] for c in clusters if c['n'] >= len(stream) / 2]
    else:
        truth = max((pts for _, pts, _ in stream), key=len)
    print(f'{len(stream)} obstacle messages, true buoys {len(truth)}: ' + ' '.join(f'({x:.1f},{y:.1f})' for x, y in truth))

    t_first, t_last = stream[0][0], stream[-1][0]

    plain_sets = []
    memory_sets = []
    merged = []
    for i, (ts, pts, r) in enumerate(stream):
        if i < args.skip:
            pts = []
        plain_sets.append([(p, r) for p in pts])
        for p in pts:  # newest position wins for a merged buoy
            for i, q in enumerate(merged):
                if math.dist(p, q) < 1.0:
                    merged[i] = p
                    break
            else:
                merged.append(p)
        memory_sets.append([(p, r) for p in merged])
    stamps = [s[0] for s in stream]

    def obstacles_at(t, memory):
        # Messages are logged on change only: the set holds until the next logged one.
        import bisect
        i = bisect.bisect_right(stamps, t) - 1
        if i < 0:
            return []
        return (memory_sets if memory else plain_sets)[i]

    def run(start, target, t_start, variant, rng, bias):
        memory, margin, commit = variant
        x, y = start
        vx = vy = 0.0
        cvx = cvy = 0.0
        ex = ey = 0.0
        dt = 0.1
        min_clear = math.inf
        committed = None
        flips = 0
        last_side = None
        tau_v, sigma, corr = 1.0, 0.15, 10.0
        for k in range(int(240 / dt)):
            t = t_start + k * dt
            a = math.exp(-dt / corr)
            ex = a * ex + math.sqrt(1 - a * a) * rng.gauss(0, sigma)
            ey = a * ey + math.sqrt(1 - a * a) * rng.gauss(0, sigma)
            mx, my = x + ex + bias[0], y + ey + bias[1]
            for b in truth:
                min_clear = min(min_clear, math.dist((x, y), b))
            goal = (target[0] - mx, target[1] - my)
            dist_goal = math.hypot(*goal)
            if dist_goal <= 0.4:
                return True, min_clear, k * dt, flips
            if k % 5:
                vx += (cvx - vx) * dt / tau_v
                vy += (cvy - vy) * dt / tau_v
                x += vx * dt
                y += vy * dt
                continue
            discs = [((p[0] - mx, p[1] - my), r + margin) for p, r in obstacles_at(t, memory)]
            discs = [d for d in discs if math.hypot(goal[0] - d[0][0], goal[1] - d[0][1]) >= d[1]]
            ev = exit_vector(discs)
            if ev:
                u, d = ev
                sp = max(0.2, min(0.6, 0.5 + d))
                cvx, cvy = u[0] * sp, u[1] * sp
            else:
                aim = None
                if commit and committed is not None:
                    wp = (committed[0] - mx, committed[1] - my)
                    if math.hypot(*wp) > 0.5 and not inside_any(wp, discs) and seg_clear((0, 0), wp, discs):
                        res = detour_aim(goal, discs)
                        # Keep the committed waypoint unless the fresh plan is clearly shorter (2 m).
                        via = math.hypot(*wp) + math.dist(wp, goal)
                        if res is None or res[1] > via - 2.0 or seg_clear(wp, goal, discs) is False and res[1] > via - 2.0:
                            aim = wp
                    if aim is None:
                        committed = None
                if aim is None:
                    res = detour_aim(goal, discs)
                    if res is None:
                        cvx = cvy = 0.0
                        aim = (0.0, 0.0)
                    else:
                        aim = res[0]
                        if aim != goal:
                            committed = (aim[0] + mx, aim[1] + my)
                if aim != (0.0, 0.0):
                    side = math.copysign(1, goal[0] * aim[1] - goal[1] * aim[0]) if aim != goal else 0
                    if side and last_side and side != last_side:
                        flips += 1
                    if side:
                        last_side = side
                    al = math.hypot(*aim)
                    sp = min(0.5 * max(dist_goal, al), 0.4)
                    cvx, cvy = (aim[0] / al * sp, aim[1] / al * sp) if al > 1e-3 else (0.0, 0.0)
            vx += (cvx - vx) * dt / tau_v
            vy += (cvy - vy) * dt / tau_v
            x += vx * dt
            y += vy * dt
        return False, min_clear, 240.0, flips

    # Crossings of the field in four directions, offsets every 1 m, at several moments of the stream
    # (including the minutes when real buoys were missing from the messages).
    xs = [p[0] for p in truth]
    ys = [p[1] for p in truth]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    scenarios = []
    for off in range(-6, 7, 2):
        scenarios += [((cx + off, cy - 14), (cx + off, cy + 14)), ((cx + off, cy + 14), (cx + off, cy - 14)),
                      ((cx - 14, cy + off), (cx + 14, cy + off)), ((cx + 14, cy + off), (cx - 14, cy + off))]
    # The first message (8 buoys) / the 5-buoy minutes / the 6-buoy minutes.
    starts = [t_first + 5, t_first + 520, t_last - 10]
    variants = {'baseline (no memory, margin, commit)': (False, 0.0, False)}
    for m in margins:
        variants[f'memory+margin{m:g}'] = (True, m, False)
        variants[f'memory+margin{m:g}+commit'] = (True, m, True)
    for bias in biases:
        print(f'\nreceiver bias {math.hypot(*bias):.1f} m')
        print(f'{"variant":36s} {"reached":>8s} {"min clear":>9s} {"p5 clear":>9s} {"<0.8 m":>7s} {"<1.0 m":>7s} {"flips":>6s} {"time":>6s}')
        for name, variant in variants.items():
            rng = random.Random(7)
            cases = [(s, g, t0) for s, g in scenarios for t0 in starts]
            res = [run(s, g, t0, variant, rng, bias) for s, g, t0 in cases]
            for (s, g, t0), r in zip(cases, res):
                if not r[0]:
                    print(f'    not reached: start ({s[0]:.1f},{s[1]:.1f}) goal ({g[0]:.1f},{g[1]:.1f}) t+{t0 - t_first:.0f}s')
            clears = sorted(r[1] for r in res)
            ok = [r for r in res if r[0]]
            print(f'{name:36s} {len(ok):4d}/{len(res):<3d} {clears[0]:9.2f} {clears[len(clears) // 20]:9.2f} '
                  f'{sum(c < 0.8 for c in clears):7d} {sum(c < 1.0 for c in clears):7d} {sum(r[3] for r in res):6d} '
                  f'{sum(r[2] for r in ok) / max(1, len(ok)):6.1f}')


if __name__ == '__main__':
    main()
