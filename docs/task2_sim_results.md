# Task 2 simulation results

Verdict and success rates: `task2_feasibility_verdict.md`. Every hypothesis and result (H1-H23, the bugs found by the random spawn,
the decisions): `task2_run_log.md`. Earlier work: `task2_interim_results.md`, `task2_mode_experiments.md`, `task2_core_repair_by_camera.md`.

## Summary (Gazebo ground truth, simulation only, stopped 2026-10-29)

| Tier | Spawn at the middle (4 nodes) | Random spawn 12-20 m away | UUV points in the sim |
|---|---|---|---|
| Core | 5 of 5 pass | 4 of 4 pass (last batch) | 450 |
| Advanced | 4 of 4 pass (one run each) | node 1 pass, node 3 order wrong before H23 (not re-run) | 450 + 25 |
| Disruptive | 4 of 4 pass (one run each; tin/circle RED/BLUE, RED/GREEN, GREEN/BLUE, GREEN/GREEN) | not run | 450 + 50 |

Not done: 5 of 5 repeats, the other 4 colour pairs in the sim, fisheye and turbid-water detector tests, the Marina Bay datum, vertical pipe tracking,
hydrophone and search prior. The plan items are listed in the verdict.

## Files

- `task2_results/core_repair/`: trimmed mock GCS logs (survey and resource delivery as received by the GCS) and the ground-truth pipeline status of the batch runs
  (`batch_*`, `a1`-`a4`, `d1`-`d2`, `dis_*`, `adv_*`, `rs_*`). `batch_summary.txt` is the Core batch.
- `task2_results/` (older): mode experiments and the first Core matrices.

## How to reproduce

- One run, Gazebo GUI: `tools/scripts/task2_sim_e2e.sh --gui --out <dir> --tier core|advanced|disruptive [--launch-arg --light-seed --launch-arg N] [--launch-arg --spawn-seed --launch-arg N]`
- A batch over the light seeds 2, 1, 5, 9 (damaged node 1, 2, 3, 4) with the ground-truth state printed: `tools/scripts/task2_sim_batch.sh --out <dir> --tier <t> [--seeds "..."] [--random-spawn]`
- Light seeds for the 9 tin/circle pairs: BLUE/BLUE 12, BLUE/GREEN 18, BLUE/RED 3, GREEN/RED 6, RED/RED 24 (node and Advanced circle in `task2_run_log.md`).
- Clean up: `tools/scripts/task2_sim_cleanup.sh`. Recorder: `tools/sos_integration/ring_recorder.py`. Wrap long batches in `systemd-inhibit --what=sleep:idle` (a suspend froze runs once).
- Gazebo sometimes does not finish loading (1 in 3); the e2e script retries. Do not rebuild while a sim runs, and check that `install/` is built from this branch.
