# Task 2 Core-Disruptive simulation: interim results (run stopped by the operator)

Status: the autonomous run was **stopped before the Disruptive stage finished**. Nothing was pushed. Branch:
`feat/task2-core-disruptive-sim`. Plan: `task2_core_disruptive_plan.md`. Step log: `task2_run_log.md`.
Raw result files (trimmed JSONL) are in `task2_results/`.

## What works (verified in the sim)

| Item | Evidence |
|---|---|
| GPS loss under water in the sim (`sim_gps_gate.py`, worst case 0.1 m) | GPS off on diving, on 10 s after surfacing; self-check passes; fixed two bugs (ParamSetV2 type, datum jump at arming) |
| Pixhawk mode behaviour with no GPS | `task2_mode_experiments.md`, `task2_results/mode_experiments/` |
| Decision: GUIDED on the surface, ALT_HOLD + RC override under water | Same document; GUIDED entered after GPS loss is accepted but stops tracking (-0.08 m for 6 m commanded, depth not held) |
| Mode switch in the stack (`autonomy_mode: "GUIDED,ALT_HOLD"`, `output_mode: auto`, BT `SetFcuMode`) | Core run: mission SUCCEEDED, dive and pipeline follow in ALT_HOLD with the GPS cut |
| Flash-code decoder (`ResourceCodeDecoder`) | 10 unit tests: 9 tin x circle pairs, 10% dropped frames, 10 fps, green tin, mid-code start, Advanced, repair verification; a swapped tin/circle makes 5 fail |
| Evidence path (`survey_evidence_node` consumes `ResourceCode`) | 6 new unit tests (9 total); `repair_verified` was never set before this work, so tier 3 and 4 reports were always rejected (`REPAIR_UNVERIFIED`) |
| `mock_gcs.py` (scripted commands, course, relay to UAV, mock UAV, checks) | Loopback test: pass exits 0, wrong expectation exits 1, `command_seq` 1,2,3; first accepted `run-start` in the sim |
| `launch_task2.sh` | Written by a Sonnet subagent, verified by me (5 dry-runs), accepted unchanged |
| Regression | vehicle_execution 60 tests pass |

## What did not work or is not proven

| Item | State |
|---|---|
| Core ring order | **FAILED in the sim.** Report `GREEN GREEN RED GREEN`, expected `GREEN RED GREEN GREEN` (damaged node 2). It is not a plain reversal: another run with the same truth reported `RED GREEN GREEN GREEN`. The recorded poses are about 1.2 m east of the world nodes (the camera sees a ring before the vehicle is over it), which fits: ring 0 = node 1, an extra green ring 0.07 m and 0.2 s before the red one (likely a flicker or reflection), the red ring = node 2, ring 3 = node 3, and node 4 never recorded. So the red is pushed from slot 2 to slot 3 and the list still has four entries. This is a reading of vehicle poses, not proven with Gazebo ground truth per ring. (An earlier version of this file blamed a false ring near the buoy; that was probably wrong.) Raw: `task2_results/e2e/e2e_core9_ring_events.jsonl` |
| Disruptive end to end | **Not proven.** First run hung 10 minutes in `RepairRedRing` (no ring was ever recorded) and the decoder started with `code_length=1` although the tier was disruptive. Cause not found yet |
| Advanced tier | Not run |
| Red-node matrix (R1-R16), S-cases, 9 pairs in the sim | Not run: `light_seed` is not forwarded by `main_launch.launch.py`, so the damaged node is always 2 |
| Real-robot GUIDED drift from IMU error | Not verifiable: `SIM_ACC1_BIAS_*` and `INS_ACCOFFS_*` have no effect with the Gazebo (JSON) backend |
| Hydrophone, front-camera buoy colour | Not worked on |

## Fixes that were not in the plan (sim and tooling)

1. QoS of `PipelineBuoys`: the subscriber was transient_local against the new volatile contract.
2. `probe_rect: []` crashed the Task 2 pipeline detector at start.
3. The sim fixture coordinates in `task2_params.yaml` did not match `main_world.world`; the end of the pipe was never
   reached and the mission looped until the traverse timeout (it also happened with the unmodified GUIDED setup).
4. `gps_tolerance_m` 3 m with a spawn 3.8 m from the buoy: the vehicle dived west of the first light node.
5. `mock_gcs` sent `run-start` before the mission reader was matched, and a missing Course made the UUV reject evidence.
6. Gazebo sometimes does not finish loading (one launch in three); stale `banyu_mission`, `gz` and `mavros_node`
   processes survive `ros2 launch` and break the next run.
7. Do not rebuild packages while a sim is running: it breaks the running nodes.

## Feasibility, as far as measured

- Under water with only IMU, Bar30, cameras and the GPS lost at 0.1 m, a Core run **completed** in the sim with the
  chosen mode strategy. The survey order was wrong, so the Core light-order points (150) are **not** shown to be reachable yet.
- Repair of the red ring completed in Core and the flash-code reader is unit-tested, but reading the code in the sim,
  the report to the GCS and the relay to the UAV for Advanced/Disruptive are **not shown**.
- Open question for the robot, not the sim: the speed scale of the RC override (sim 0.51 m/s for +90 PWM) and IMU drift.

## Next steps (not done)

1. Find why `RepairRedRing` hangs in the Disruptive run and why the decoder gets `code_length=1`.
2. Reset the ring tracker at the start of the traverse; investigate the missed node.
3. Forward `light_seed` through `main_launch.launch.py`; run the red-node matrix per tier.
4. Run Advanced and Disruptive end to end and write `task2_feasibility_verdict.md`.
