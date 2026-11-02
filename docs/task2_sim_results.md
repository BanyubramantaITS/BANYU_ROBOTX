# Task 2 simulation results

Verdict: `task2_feasibility_verdict.md`. Every hypothesis and run: `task2_run_log.md`. Updated 2026-11-02.

## Per tier (Bar30, random spawn, Gazebo ground truth)

| Tier | Node 1 | Node 2 | Node 3 | Node 4 | Points per pass | Failing step seen |
|---|---|---|---|---|---|---|
| Core | PASS | PASS | PASS | PASS | 450 | none in the last batch |
| Advanced | PASS (+ GREEN seed 6) | PASS | PASS (after H24, H25) | PASS (+ GREEN seed 12) | 475 | order (H24), Surface after a false Bar30 fault (H25) |
| Disruptive | PASS (seeds 2, 43, 6) | PASS (seeds 1, 3; seed 18 450) | PASS (seeds 5, 35, 34) | PASS (seed 9, 12); seed 24 FAIL 50 | 500 | order unverified (seed 18), duplicate records (seed 24) |

## Minimal sensors (no depth sensor, STABILIZE), Core

| Run | Result | Failing step |
|---|---|---|
| First run (H23 code) | order right, repair GREEN, traverse never ended | floated up off the pipe at the far end |
| H26 | 200 (order right) | repair: floated off the switch |
| H27 / H27b / H28 | 25-200 | repair: contact 0.15-0.3 m off the switch, sink too fast |
| H29-H36 | dive failures | heave thrust too coarse; the pipe not below the start buoy; time limit |
| H37 / H37b | dive locks; 50, 25 | repair missed; the vehicle floated to the surface or sank to the seabed |

## Scenarios

Only `--pipe-depth 8` (Bar30) started: the fixed 12.5 m dive is below the pipe (no score, stopped). The other scenarios were not run.

## Tools

- `tools/scripts/task2_sim_batch.sh --out <dir> --tier <t> --seeds "..." [--random-spawn] [--opts "<launch_task2.sh options>"]`
  scores each run with `task2_score_run.py` (PASS/FAIL and points from Gazebo).
- `tools/scripts/launch_task2.sh` options: `--sensors full|minimal`, `--pipe-depth`, `--pipe-move dx dy dyaw`, `--visibility`,
  `--current e n [gust]`, `--buoy-error x y`, `--fcu-faults compass=..,gps_offset=..,gps_drop=..`, `--light-fault n:kind`, `--fisheye`,
  `--ir-led`, `--camera-width`, `--site marina_bay`.
- Raw logs (`sim.log`, `rings.jsonl`) stay out of the repo.
