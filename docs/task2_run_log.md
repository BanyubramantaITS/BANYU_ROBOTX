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
