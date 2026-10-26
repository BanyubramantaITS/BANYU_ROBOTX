# Task 2 run log

Running log for the autonomous run of `docs/task2_core_disruptive_plan.md` (branch
`feat/task2-core-disruptive-sim`). One entry per step: decision, result, next.

## Step 0: rx_msgs sync
- `git pull --ff-only` in `RobotX-SoS-communication-ITSN` took 1 commit (`cc7d9fa`): `PipelineBuoys`
  now uses the plain default QoS (reliable, volatile, keep_last 10), repeated about 1 Hz.
- Finding: `pipeline_buoys_node` subscribed with reliable + transient_local (depth 1). A transient_local
  subscription is incompatible with a volatile publisher, so it would have received nothing from the real USV.
- Decision: subscribe with `QoS(10).reliable()` (volatile; compatible with both kinds of publisher). Update
  `dds_relay.py` (route is VOLATILE) and `mock_usv.py` (publishes default QoS). `rx_msgs` rebuilt,
  `banyu_communication` rebuilt.
- Open item: `/system/mission/course` is still subscribed latched on the UUV side while the spec says plain
  default QoS. Not changed here; the mock GCS will republish `Course` periodically.

## Step 1: sim_gps_gate and mode experiments (in progress)
- Added `banyu_bringup/scripts/sim_gps_gate.py` (sim only, `gps_gate:=True`, `gps_loss_depth_m`,
  `gps_reacquire_delay_s` launch args; self-check passes). It writes `SIM_GPS1_ENABLE` through
  `/mavros/param/set` from `/banyu/vehicle/water_depth`.
- Added `tools/scripts/gps_loss_mode_experiment.py` (sim only): dives, holds a mode for N s, logs JSONL,
  surfaces, and times how long GUIDED takes to be accepted again.
- Sim bring-up fixes found on the way:
  - `TMUX` is inherited from the Claude Code wrapper and ArduPilot's `run_in_terminal_window.sh` then calls
    `tmux new-window`, which is unsupported, so ArduSub never starts. Launch with `unset TMUX` and
    `SITL_RITW_TERMINAL=bash`.
  - `task2_pipeline_detector` failed on `probe_rect: []` (rclcpp cannot type an empty YAML list). Removed the
    line from `task2_params.yaml` (default is an empty vector).
  - `underwater_odometry_node` was not built in `install/`; rebuilt the Task 2 packages.
- Decision: M2 (GUIDED + zero-displacement ExternalNav beacon) is documented as unsafe in
  `task2_underwater_position.md`; it is not run, only recorded as "not run, known unsafe".

## Delegations
- D1: `tools/scripts/launch_task2.sh` -> Claude subagent (sonnet) in a separate git worktree (scratchpad).
  Files allowed: that one file. Proof: `bash -n`, `--help`, `--sim`/`--real` `--dry-run`, `--real --gps-gate`
  must fail. Outcome: ACCEPTED after my verification: read the whole file, `git status` in the worktree showed only that
  file, `bash -n` ok, re-ran the five dry-runs (sim, sim disruptive + gate, real, real + gate refused, bad tier
  refused), checked `sub_bringup.launch.py` really has `mode` and `use_gps`, and that the cleanup lines match
  `launch_por.sh`. No changes needed. Note: its pre-flight cleanup kills all ardusub/mavproxy, so autonomous
  runs use `--no-clean`.

## Step 1 results so far
- sim_gps_gate fixed twice: `ParamSetV2.value` is `rcl_interfaces/ParameterValue` (not `mavros_msgs/ParamValue`),
  and SITL calibrated depth reads ~0.43 m at the surface (datum drift), so thresholds are relative to a baseline
  taken at start (`surface_depth_m` can override it). GPS went OFF at 0.12 m relative depth and back ON 10 s
  after surfacing.
- M3 ALT_HOLD + RC override (neutral sticks, GPS cut): held depth within 0.04 m and yaw within 0.0 deg over
  30 s at 1.0 m; GUIDED was accepted again 2.0 s after surfacing (GPS back).

## Step 1 findings (mode experiments, partial) and process notes
- **Do not rebuild packages while a sim run is in progress.** A `colcon build` of `banyu_msgs` during a trial
  replaced the installed Python message module mid-run: the experiment script and `sim_gps_gate` died
  (`No module named banyu_msgs.msg._resource_code`) and `banyu_mission` aborted (zmq). The batch was rerun.
- Sim cleanup: stopping `ros2 launch` with SIGINT leaves `banyu_mission`, `gz`, `mavros_node` and `ardusub`
  children alive (a stale `banyu_mission` holds the Groot2 port and kills the next run's mission node).
  `cleanup_sim.sh` (scratch) kills everything that carries `ROS_DOMAIN_ID=40` in its environment.
- GUIDED_ENTER (dive in ALT_HOLD, GPS cut, request GUIDED 20 s later): GUIDED was **accepted**, but the vehicle
  then ignored the setpoints: a 0.3 m/s forward command for 20 s moved it -0.08 m (Gazebo ground truth), and
  depth was no longer held (it rose 1.2 m in 40 s). So GUIDED entered after the GPS is gone is a trap: the mode
  switch succeeds and the vehicle silently stops tracking.
- GUIDED already active when the GPS is cut (earlier run, no forward command): depth drift 0.03 m, no
  cancellation, EKF3 "stopped aiding" messages. The sim IMU has no bias, so this is the optimistic case.
- ALT_HOLD + neutral sticks with the GPS cut: depth drift 0.04 m, yaw drift 0 deg over 30 s.
- MANUAL (no depth hold): rose 1.4 m in 60 s (positive buoyancy) and did not surface by itself.

## Step 1 decision and Step 2 implementation (W2)
- Decision (evidence in `task2_mode_experiments.md`): surface transit stays GUIDED with the GPS; the dive and
  everything under water use ALT_HOLD + RC override. IMU error could not be injected into the JSON-backend sim
  (`SIM_ACC1_BIAS_*` and `INS_ACCOFFS_*` had no effect), so GUIDED's robot-side drift is unverified; the decision
  rests on M1b (GUIDED entered after GPS loss silently stops tracking) and on ALT_HOLD not needing a position.
- Implemented: `control.autonomy_mode: "GUIDED,ALT_HOLD"` + `control.output_mode: "auto"` in
  `vehicle_execution_node` (setpoint output in the first mode, RC override in the second, RC released once on the
  way back); the same two-mode list accepted by `vehicle_status_tracker` and `MissionState`; new BT action
  `SetFcuMode` placed before `DiveToDepth`; `mission.underwater_mode` blackboard entry.
- Implemented (W5/W6, tiers 3 and 4): `ResourceCodeDecoder` + `resource_code_decoder_node`, `banyu_msgs/ResourceCode`,
  `survey_evidence_node` consumes it (fills `repair_verified`, `resource_code_complete`, tin and circle; Advanced
  reports COLOR_ANY as the tin), BT `RepairRedRing` holds station for `hold_total` and runs `DecodeResourceCode` in
  parallel for tiers 3 and 4. Core keeps the plain `hold_duration` hold.
- Finding while wiring: nothing ever set `repair_verified`, and the report ledger rejects tier 3 and 4 reports with
  `REPAIR_UNVERIFIED`. The decoder now supplies it (steady green longer than 3 s after the probe hold).
- Unit tests: decoder 10 (9 colour pairs, dropouts, 10 fps, green tin, mid-code start, Advanced, verified repair,
  mutation check: swapping tin and circle fails 5 of them), survey evidence 9 (6 new), vehicle_execution 60 (no
  regression).
- Delegation outcome summary so far: D1 (launch_task2.sh, sonnet) accepted.
- e2e core run 2: Gazebo/SITL did not finish loading (log stalled after the FCU frame lines, no water depth), relaunch 1 of 3.

## Step 3 (Core) bring-up findings
Getting one clean end-to-end Core run in the sim took four fixes, none of them in the planned work items:
1. `sim_gps_gate` cut the GPS at the surface when the vehicle armed: arming zeroes the baro and the calibrated depth
   datum jumps by 0.15-0.33 m, which the gate read as a dive. Fixed by following fast depth steps (rate > 0.8 m/s)
   as datum jumps.
2. `mock_gcs.py` sent `run-start` before the mission manager's reader was matched (volatile topic), so the start was
   silently lost: it now waits for both readers plus a settle time. It also had to publish `/system/mission/course`:
   without it the UUV rejects survey evidence (`COURSE_UNAVAILABLE`) and `WaitSurveyComplete` fails.
3. The Task 2 sim fixture coordinates in `task2_params.yaml` (buoys at 44 m east, pipe at x 47-52) did not match
   `main_world.world` (pipeline at x 2.3, green buoy +3.8, red buoy -3.8). The end-of-pipe marker was never
   reached, so the mission looped Follow/Search/Acquire until the traverse timeout. The fixture now follows the world.
   The same loop also happened with the unmodified GUIDED setup, so it was not caused by the ALT_HOLD change.
4. `main_launch.launch.py` does not forward `light_seed` to the world launch, so `light_seed:=N` is ignored and the
   damaged node is always 2 (default). Seeds cannot select the red node until this is wired.
- Sim start-up: Gazebo sometimes never finishes loading (one launch in three, as AGENTS.md says); the scratch e2e driver
  retries up to three times.
- First complete Core run (tier core, GPS cut at 0.1 m, GUIDED on the surface, ALT_HOLD under water, mock GCS):
  mission SUCCEEDED, `PipelineSurvey` received by the mock GCS about 80 s after the start. The survey lists the red
  ring FIRST (`RED GREEN GREEN GREEN`), but the world's damaged node is 2, which is second from the active (green)
  buoy end: the ring order is wrong and is the next thing to investigate.
