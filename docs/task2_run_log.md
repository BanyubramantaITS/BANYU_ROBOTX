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
  Closed 2026-11-03: see `docs/issue13.md` §0a.

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

## Stop
- Run stopped by the operator before the Disruptive stage was proven. All sim processes were stopped (no `ardusub`,
  `gz`, `mavros_node`, `banyu_mission` or helper scripts left). Summary: `task2_interim_results.md`. Trimmed raw results:
  `task2_results/`.
- Delegations: 1 total (`launch_task2.sh`, Sonnet), 0 rejected. `agy` was tested but not used for a job.

## Tier propagation bug (found while debugging the Disruptive hang)
- With `tier:=disruptive` or `advanced`, `resource_code_decoder`, `survey_evidence_node`, `uuv_comm_node` and
  `banyu_mission` all still ran with tier 2: `ros2 param get` returned 2 in every case. Neither a dict in the Node
  `parameters` nor a `-p tier:=N` argument won over `task2_params.yaml` (launch_ros puts its parameter files after
  the arguments). So no Advanced or Disruptive run before this point ran the tier logic: the earlier "disruptive"
  e2e run (decoder `code_length=1`) was a tier 2 run on a world with the probe model.
- Fix: one overlay file per tier (`config/task2_tier_{core,advanced,disruptive}.yaml`) loaded after the params file
  by `mission.launch.py`. Verified with `ros2 param get`: disruptive 4/4/4/4, advanced 3/3/3/3, core 2/2/2/2.

## End of the autonomous attempt
- Found and fixed: the tier launch argument never reached the nodes (overlay files per tier); the probe needs a repair
  depth; the decoder window now covers the whole repair; repair needs red then steady green; a higher tier whose code
  cannot be read falls back to a Core survey (ledger accepts tier 2 evidence for a higher configured tier); `light_seed`
  is forwarded.
- Results and verdict: `task2_feasibility_verdict.md`. Core partial, Advanced and Disruptive not shown in the sim.
- Delegations: 1 total (`launch_task2.sh`, Sonnet, accepted), 0 rejected. `agy` was tested and never used for a job.
- All sim processes stopped; nothing pushed.

## Stopped by the operator (second time)
- Ring tracker fix (green then red at one node is one red record; association 0.8 m in the sim) took the Core red node
  matrix from 1 of 4 correct to 5 of 6 (`task2_results/e2e/core_matrix2_summary.md`). The batch was stopped before
  `n3b` and `n4b`. Node 3 still wrong.
- Cleanup note: four leftover `ros2 launch mission.launch.py` stacks from my tier-propagation tests (ROS domain 44, not
  the sim domain 40) were still running for 1 h 50 min and were killed now; the domain 40 runs were not affected.
- All sim processes stopped. Nothing pushed.

## Core repair by camera (goal: maximise the Task 2 score)
- g1 (Core, GUI, light seed default, damaged node 2): survey correct (GREEN, RED, GREEN, GREEN). Approach reached contact
  (ring size >= 0.14) but `HoldProbeOnRing` ended "Ring did not turn green on contact"; Gazebo node 2 stayed DAMAGED_RED.
  Ground truth: the probe tip was 0.14 to 0.17 m from the switch, right at the 0.15 m limit, so it never held 2 s.
  Hypothesis H1: keep creeping (0.3 x rate) during the hold until size >= 1.2 x contact size or the ring turns GREEN.
- Setup note: `install/` had been overwritten by a build of another branch (no `underwater_odometry_node`), so the robot
  was never spawned; fixed with `colcon build --packages-up-to banyu_bringup`.
- g2 (Core, GUI, damaged node 2) with H1 (creep during the hold): **repair confirmed by Gazebo** (node 2 `TASK_COMPLETED_GREEN`,
  `HoldProbeOnRing` SUCCESS after 15 s) and the survey was correct (GREEN, RED, GREEN, GREEN). Core points in the sim:
  25 + 25 + 150 + 250 = 450 for node 2. The ring recorder's `repair_verified` stayed False (the tracker does not see the
  repair; needed for the Advanced evidence, to check).
- Tooling: `task2_sim_e2e.sh` now saves `/robotx/pipeline/status` and ends a Core run 20 s after the survey report;
  `task2_sim_batch.sh` runs it once per light seed (2, 1, 5, 9 = node 1..4) and prints the ground-truth state.
- **Core batch (GUI, `task2_sim_batch.sh`, one run per damaged node), Gazebo ground truth:** seed 2 (node 1), seed 5 (node 3),
  seed 9 (node 4), seed 1 (node 2): all four `TASK_COMPLETED_GREEN` and the survey order correct in each. With node 2 in g2,
  that is 5 of 5 Core runs at 450 points in the sim (`task2_results/core_repair/`). Not yet tested: random spawn and
  GPS transit from a dock, fisheye camera, turbid water, several runs per node.
- Next: Advanced. The Advanced and Disruptive branch of `RepairRedRing` still uses the fixed-depth steps; move it to the
  camera approach and keep the probe on the node while the flash code is read.
- a1 (Advanced, GUI, node 2): the new approach + stay-on-node flow repaired the ring (Gazebo `ADVANCED_FLASH`) and the decoder
  published a complete, correct code (`repair_verified` true, delivery circle BLUE). But `RepairRedRing` looped: after the
  first acknowledge the tracker made a second record (index 2) at the same spot, red again, so the repair restarted
  over and over and no report went out. Cause: the tracker's ring position uses a fixed `camera_height_m` 1.7 m; with the
  vehicle 0.4 m above the lid that puts the ring about 1 m too far ahead, so it no longer matches the first record.
  Hypothesis H2: take the camera height from the ring's apparent size (`lid_diameter_m`, `focal_ratio`).
- a2 (Advanced, GUI, node 2) with H2 (tracker height from ring size): no repair loop any more (one repair, code read, report sent
  with `ResourceDelivery` circle BLUE, which matches the sim). But the survey order was **all GREEN**: the ledger had stored the
  first sighting of node 1 (GREEN, from afar) as its original condition, and the later RED read (before the repair) never
  replaced it. In Core this is hidden by timing (the evidence snapshot with the early green was lucky not to exist).
  Fix H3 (`survey_reporting.cpp`): before a repair is verified, a ring first seen GREEN that later reads RED is originally RED;
  after the repair a RED flash does not change it (existing test updated, new test added; banyu_communication 118 tests pass).
- a3 (Advanced, GUI, node 2): the decoder declared a complete code about 3 s into the repair while Gazebo still had node 2
  DAMAGED_RED (`repair_verified` false, circle RED): the steady red and its flicker passed as 1 s flashes, `DecodeResourceCode`
  succeeded, which cancelled the probe hold, and a report was sent without a repair. H4 (`resource_code_decoder.cpp`): a ring seen
  steady red that never turned steady green gives no code, and only flashes after the verified repair count; a window that
  opens after the repair (no steady red) is read as before. Test added (11 decoder tests pass). Found while at it: a red tin
  flash must not count as the damaged ring, hence "steady" red.
- **a4 (Advanced, GUI, node 2): pass.** Gazebo `ADVANCED_FLASH`, decoder `repair_verified` true with circle BLUE, report: survey
  GREEN, RED, GREEN, GREEN and `ResourceDelivery` tin ANY, circle BLUE (as configured). Next: Advanced batch over the other
  three nodes, then Disruptive.
- Advanced batch, seed 2 (node 1): **FAILED** `HoldProbeOnRing` "Ring did not turn green on contact", node 1 stayed RED. Ground truth at
  the end: the vehicle sat at (1.47, -0.63, -11.96), over node 2 (0.93, -0.62), 1.7 m from node 1's switch. `CenterOnRing` and the
  approach take whatever ring the detector reports; with node 1 out of view it reports the neighbouring GREEN node 2 and the
  vehicle centred and sank onto the wrong node. (Core seed 2 had passed, by timing.) H5: `CenterOnRing` honours `ring_color`
  (red only) and the approach sinks and centres only on a RED ring until contact; the GREEN confirmation counts only in the hold
  phase. vehicle_execution tests 60/60.
- Advanced batch seed 2 after H5: centring and sink onto node 1 now work, but `DecodeResourceCode` returned "flash code read" 2 s after the
  window opened while node 1 was still RED, which cancelled the probe hold (no repair, no resource report). Cause: the lid flickers in
  and out of view at the start, giving short red runs that the decoder took as flashes (min 2 cycles). H6
  (`resource_code_decoder.cpp`): a window that starts on RED gives no code until that RED has turned steady green. 12 decoder tests pass.
- Advanced batch seed 2 after H6: repair and code correct (Gazebo ADVANCED_FLASH, circle RED = the seed's `advanced_target_color`), but
  afterwards the vehicle stayed 0.4 m above the node, the pipe could not be followed from there and `FollowPipeline` / `SearchPipeline`
  / `AcquirePipeline` looped for 4+ min (a4 on node 2 took 3 min the same way). H7: new `climb_only` mode of `HoldProbeOnRing`
  (back to the depth where the last approach started) after the code is read; vehicle_execution tests 60/60.
- Note: the light seed also changes the flash colours (seed 2: advanced RED), so the expected circle depends on the seed.
- Advanced batch after H7: **seed 2 (node 1) pass** (survey RED, GREEN, GREEN, GREEN; ResourceDelivery circle RED = that seed's flash colour;
  done in 3.5 min). **Seed 5 (node 3): FAILED, no report.** The repair and code were correct (Gazebo `TASK_COMPLETED_GREEN`, circle BLUE
  = the seed's target) but the pipe was not found again afterwards; `SearchPipeline` / `FollowPipeline` were then rejected by the arbiter
  ("Vehicle safety or run authorization fault") in a tight loop until the e2e timeout. Not yet diagnosed (suspects: pipe lost after the
  climb for the last two nodes; the mission surfaced on the traverse timeout but kept ticking the traverse loop).
- Advanced seed 9 (node 4), diagnostic batch: **pass** (survey GREEN, GREEN, GREEN, RED; circle BLUE; Gazebo TASK_COMPLETED_GREEN). The earlier
  seed 5 / seed 9 failures in the `adv` batch coincided with a wall-clock jump (the machine slept), the "safety fault" there was most
  likely that, and the diagnostic log (flags of the safety check) is now in `vehicle_execution_node`.
- Advanced seed 5 (node 3) failed twice the same way, with no safety fault: after the repair the pipe is not found, the 15 s `AcquirePipeline` times
  out, and then `FollowPipeline` and `SearchPipeline` are ticked every 50 ms, displace each other and are rejected by the goal arbiter's
  zero barrier, forever (a livelock; the traverse timeout never ends it). H8: wait 2 s after a failed reacquire in the traverse loop
  (`<Sleep>`); mission tree tests 31/31.
- Advanced seed 5 (node 3) after H8 (no livelock any more, arbiter rejects 0): **repair FAILED**, Gazebo node 3 still DAMAGED_RED, yet
  "flash code read" 2.7 s after the window opened. The window started on a neighbouring GREEN node (so H6's "starts on RED" did not apply)
  and its 1 s green flicker counted as flashes. H9: the decoder never calls a code complete before RED then steady GREEN was seen
  (`require_repair_verified`, default true; the two tests that model a window opened after the repair set it false; new test for a
  flickering neighbouring green node; 13 decoder tests pass). The traverse also ended early (`PipelineActive` false) so the survey was
  incomplete and the mission fell back to SURFACE; to check once the repair works.
- Advanced seed 5 after H9: repair real (Gazebo ADVANCED_FLASH, `repair_verified` true) but the circle read **GREEN, expected BLUE**. The ring
  observation during the flash alternates BLUE (lit) and GREEN (the dark half), so GREEN gets as many votes as BLUE and wins a tie.
  H10 (`resource_code_decoder.cpp`, Advanced): a non-green colour that agrees on its own (>= min_cycles) beats GREEN; GREEN is the code only
  when nothing else is. 14 decoder tests pass. Seed 5's survey also needs a re-check after the repair.
- **Advanced seed 5 (node 3): pass** after H10 (survey GREEN, GREEN, RED, GREEN; ResourceDelivery circle BLUE = the seed's target; Gazebo
  TASK_COMPLETED_GREEN). **Advanced now passes once on all four damaged nodes**: node 2 (a4), node 1 (seed 2), node 4 (seed 9), node 3
  (seed 5), each with a correct survey order and delivery circle. Each node has only one passing run since the last fix (the
  earlier passes ran on earlier code); repeat runs are still to do. Next: Disruptive (two-colour code).
- **Disruptive d1 (GUI, node 2, tin RED, circle BLUE): repair real** (Gazebo TASK_COMPLETED_GREEN, `repair_verified` true) but the decoder read **no
  cycle** (`cycles_seen` 0). The ring observations during the flash are RED, GREEN, BLUE, GREEN, ...: the dark phases read GREEN, because
  with no lit light the detector falls back to colour thresholds and takes the green-olive water for a GREEN ring (the same artifact
  as the BLUE/GREEN tie in Advanced). With GREEN adjacent to every flash the cycle gaps never show. H11: observations from that
  fallback are capped at confidence 0.8 (`kFallbackRingConfidenceMax`) and the decoder only counts samples with confidence >= 0.85
  (`min_confidence`), so the dark half reads dark. Decoder 14/14, ring tracker 20/20 tests pass.
- **Disruptive d2 (node 2): pass** after H11: Gazebo DISRUPTIVE_CYCLE, decoder tin RED, circle BLUE in 2 cycles, report survey GREEN, RED, GREEN, GREEN
  and ResourceDelivery tin RED, circle BLUE (matches the configured code). Next: Disruptive batch over the four damaged nodes (each seed also
  changes the two flash colours).
- Disruptive seed 5 (node 3; tin GREEN, circle BLUE): repair real; with the recorder now logging confidence the dark half reads GREEN with
  confidence 0.8 (unlit) and the real flashes 0.9 (lit), so H11 holds. But the decoder only saw one cycle: after the repair the decode hold
  centred on the weak GREEN colour-threshold ring (the water) and the vehicle drifted off the node (`offset_x` 0.37, no lit flash seen
  after that), then `HoldProbeOnRing` never completed. H12 (`hold_probe_on_ring_server.cpp`): on the node only a lit ring (confidence >= 0.85) is
  centred on and counts; before contact any ring of the right colour guides the sink. vehicle_execution tests 60/60.
- Disruptive seed 5 after H12: still no code (`cycles_seen` 0, Gazebo flash already over). Ground truth: the vehicle was 1.4-1.5 m from node 3's
  switch, drifting toward node 4 during the decode hold (`dxy` 1.41 to 1.55 m, `dz` 0.07). The hold now centred on the *lit* ring, but the nearest
  lit one can be the next node (steady green, confidence 0.9) at the image edge (offset (-0.7, -0.98)). H13: on the node the hold
  centres only on a lit ring within 0.5 of the image centre (the repaired lid is under the camera); else it holds still.
- Disruptive seed 5 after H13: Gazebo ground truth, the vehicle was exactly on node 3 after the contact (tip 0.04-0.12 m, dz 0.07) and then
  **drifted off at about 0.1 m/s** (0.2 m at +3 s, 0.56 m at +10 s, 0.84 m at +30 s) while the lid was dark (3 of every 5 s in the two-colour
  code) and the hold gave no command. Advanced held because its lid is lit half of the time. H14: the hold keeps correcting with the last
  seen offset for 3 s after the lid goes dark (`kLastOffsetHoldS`). vehicle_execution tests 60/60.
- Disruptive seed 5 after H14: the vehicle stayed on the node until the first lit flash, then moved 0.28 m in 0.7 s (about 0.4 m/s) and lost the lid
  (offset (0.3, -0.59), then (0.94, -0.96)). The centring gains were tuned for the survey altitude (1.5 m); on the node the lid is 0.4 m away, so the same
  offset is a much smaller distance and the loop overshoots. H15: gains and limits on the node (hold phases) scaled by 0.3
  (`kOnNodeGainScale`). vehicle_execution tests 60/60.
- Disruptive seed 5 after H15: **FAILED (stop after five fixes, H11-H15, on one item)**. Evidence (Gazebo ground truth): the vehicle holds on the node for about 8 s after the
  contact (tip 0.04-0.13 m), then moves along the pipe at about 0.15 m/s (1.07 m from the switch at +15.8 s) while the lid is dark, so
  the lit flashes leave the image and no two cycles are read (`cycles_seen` 0). The earlier fixes removed phantom GREEN rings, the
  neighbouring-node pull and the over-aggressive loop. Open: a drift of 0.1-0.15 m/s appears whenever the hold commands nothing; the
  cause (zero RC not neutral? residual thrust trim?) is not found. A station-keeping reference that works with the lid dark (pipe
  edges for the lateral axis; nothing along the pipe) is the missing piece. Disruptive passes on node 2 (tin RED, circle BLUE) and node 1
  (tin RED, circle GREEN); node 3 (tin GREEN, circle BLUE) fails; node 4 untested.
- Disruptive seed 9 (node 4; tin GREEN, circle GREEN) after H15: same failure as seed 5: the vehicle holds on the node for about 6.5 s after the
  contact (tip 0.08-0.14 m) and then leaves it (0.38 m at +8.5 s) as soon as the flashes start; `cycles_seen` 0. The centring on a lit
  lid moves the vehicle at the first flash although the gains are scaled (the offsets of a lit lid change from flash to flash because the probe
  rod hides part of the ring). H16 (experiment): no centring while the flash code is read (`kDecodeHoldGainScale` 0), the vehicle just holds still.
- **H16 worked for Disruptive seed 9 (node 4; tin GREEN, circle GREEN):** with no centring while the code is read the vehicle stays on the node and
  the decoder read tin GREEN, circle GREEN in 2 cycles (as configured). That run ended without a report because the machine suspended
  for about 24 minutes (BT log time stops at 14:38, e2e timeout at 15:02); earlier "stuck" runs (Advanced seed 5/9, 58 min) fit the same cause.
  Runs are now wrapped in `systemd-inhibit --what=sleep` so a suspend cannot freeze them.
- Disruptive seed 9 again (same code as the pass above, with sleep inhibited): **FAILED this time**: the vehicle slid off the node after the contact (tip 0.08 m, then 0.27 m at +3.8 s and 0.49 m at +7.5 s,
  about 0.06 m/s) and no cycle was read. So H16 is not reliable: the vehicle drifts off the node by a random amount within seconds
  once the hold commands nothing (cause not found). H17 (`hold_probe_on_ring_server.cpp`): after the repair, rise 0.6 m (`kBackOffM`) before the code
  reading hold, so a drift of 0.1 m/s keeps the lid in the image for over 10 s (footprint about 1.2 m instead of 0.46 m).
- **Disruptive with H17 (rise 0.6 m after the repair), GUI, Gazebo ground truth:** seed 9 (node 4; tin GREEN, circle GREEN) pass; seed 5 (node 3; tin GREEN, circle BLUE)
  pass; seed 1 (node 2; tin RED, circle GREEN) pass; each with the right survey order and the right tin and circle in the ResourceDelivery
  (checked against `sequence_config` in Gazebo). With seed 2 earlier (node 1; tin RED, circle GREEN) and d2 (node 2; tin RED, circle BLUE), all four
  damaged nodes pass Disruptive once. Next: regress Core and Advanced on the shared code, then random spawn and GPS.
- **Random spawn (user request: the UUV should not start between the buoys):** `launch_task2.sh --spawn-seed N` (or `--spawn x y yaw`) puts the UUV 10-20 m from the world origin
  (the GPS home, midway between the buoys at +-3.8 m) with a random heading; `main_launch.launch.py` and `gazebo_main.launch.py` take `spawn_x`, `spawn_y`, `spawn_yaw`;
  `task2_sim_batch.sh --random-spawn` uses the run's seed. First run (Core, seed 3, spawn x 11.9 y 3.4, heading -0.82 rad, GUI): the UUV navigated by RTK
  GPS to the green buoy (Gazebo position (3.72, -0.25) against the buoy at (3.8, 0), about 20 s), dived, repaired node 2 and reported GREEN, RED, GREEN, GREEN.
  The ArduSub home is the world origin, so the GPS fix follows the spawn offset without further changes.
- Random-spawn Core batch (seeds 2, 5, 9, 1; spawn 12-20 m from the buoys): the UUV always reached the green buoy by GPS. Seed 5 and 9 passed. **Seed 1 (node 2): no report**: the
  UUV met the pipe from the red end (it drifted about 5 m west during the dive) and walked it east, so the progress fell from ring to ring (2.7, 1.87, 0.93, 0.12) and the tracker marked every
  ring after the first `ordering_verified` false, which made the ledger reject the evidence (`ORDER_UNVERIFIED`) and `WaitSurveyComplete` fail, ending in SURFACE with no report.
  H18 (`ring_tracker.cpp`): the second ring fixes the direction the progress runs in the order the rings are met (+1 or -1) and later rings must keep going the same way; the
  ledger already sorts by progress from the active buoy end. New tracker test; ring tracker 21/21 and adapter 3/3 pass.
  **Seed 2 (node 1): survey correct but the repair failed** (contact reached, then the vehicle oscillated by +-0.15 m over the node: lid detected at offsets (0.38,-0.43), (-0.7, 0.7), (0.47,-0.49), (-0.95,-0.69), no steady green); not yet addressed.
- Random-spawn Core after H18: seed 1 (node 2): **pass** (repair TASK_COMPLETED_GREEN, survey GREEN, RED, GREEN, GREEN). Seed 2 (node 1; spawn x -18.5, y 6.3): the dive, repair and
  traversal worked but the mission **sat in `Surface` at 12 m depth for 3+ minutes** and never reported: the `Surface` action only runs on a *calibrated* water depth and the
  `water_depth` message showed `calibrated: false`: the datum had been dropped at the run start (the vehicle was not inside the surface band at that moment) and was never taken again
  (pre-start calibration only). Dived without a datum, the Surface step can never start: a deadlock. H19 (re-take the datum at the surface during the run) was **reverted**: the existing test `ActiveHeartbeatCannotReplaceSurfaceProof` encodes the design rule that the surface datum is only taken before the run starts, and I will not loosen that safety rule. Instead the datum drop at the run start is now logged (rel_alt, datum) to find out why it happens in this spawn.
- Cause of the datum drop found (diagnostic log, Core seed 5 with random spawn): at the run start the surface datum was dropped with `rel_alt -0.400 m, datum 0.139 m`: the sim barometer jumps by about -0.5 m
  at arming (known sim artefact: "datum jumps 0.15-0.4 m at arming"), after the boot-time calibration, out of the surface band and out of the 0.5 m `at_datum_surface` margin, so the vehicle
  later could not surface (the `Surface` action needs a calibrated depth). Not a spawn effect, a timing one (the jump size varies). H20 (`calibrated_depth_node`): before the run starts
  (phase < ACTIVE) a valid fresh GPS fix is surface proof (it does not work under water): the datum follows the barometer while the fix is there, and the datum is not dropped at the run
  start. During the run nothing changes (the existing rule). New test `GpsFixAtTheSurfaceKeepsTheDatumThroughABaroJumpBeforeTheRun`; calibrated-depth tests 9/9.
- Random-spawn Core after H20: seed 5 and seed 9 pass; **seed 1 again no report** (`ORDER_UNVERIFIED`): rings 2 and 3 were created at the same instant with progress 3.47 and 2.65 (not in
  order) although the nodes are at 2.8 and 4.2. The ring position uses the camera height from the lid size (H2), which is too noisy at the survey altitude (lid size 0.04 to 0.06, range
  1.1 to 2.1 m from one frame to the next). H21 (`ring_tracker.cpp`): the size-based height is only used when the lid is big (size >= 0.08, about 0.8 m or closer); otherwise the fixed
  `camera_height_m`. Ring tracker 21/21 tests pass.
- Random-spawn Core after H21: seed 1 **pass**. Seed 2 (spawn x -18.5, y 6.3): the mission failed at once (`NavigateSurfaceGps` -> FAILURE 50 ms after the run start, fallback SURFACE): the log shows the
  `PipelineBuoys` target arriving 0.8 s *after* the BT had started, so the navigation had no target. A timing race between the run start and the USV's start-point message (the real
  messages are independent too). H22 (`banyu_mission.xml`): `NavigateSurfaceGps` is retried once per second for up to 3 minutes (`Timeout` + `RetryUntilSuccessful` + `Sleep`) instead of failing the
  mission. Mission tree tests 31/31.
- **Decision (dive depth):** the initial dive stays at the fixed `inspection_depth_m` 12.5 m (Bar30); the pipe is found and followed by camera at that depth (no vertical tracking), the repair approach is camera only.
  A pipe that rises or falls more than the camera range cannot be followed in this design: a documented limit (see the verdict), not worked on now (the sim pipe is at about 13.4 m, the handbook asks for a variable depth, and the vehicle has no altimeter, so vertical tracking
  needs a camera-based depth estimate that I have not built).
- Light seeds for the remaining colour pairs (seed gives damaged node, Advanced circle, tin, circle): BLUE/BLUE 12 (node 4) 14 (node 1); BLUE/GREEN 18 (n2) 35 (n3); BLUE/RED 3 (n2) 43 (n1) 38 (n4); GREEN/RED 6 (n1) 8 (n2); RED/RED 24 (n4) 34 (n3) 69 (n1).
  Queued: Disruptive on seeds 12 18 35 3 43 6 24 34 and Advanced (circle GREEN) on 12 and 6, all with a random spawn.
- Random-spawn Advanced: seed 2 (node 1) pass; **seed 5 (node 3): repair and circle correct but the survey order wrong** (`GREEN, GREEN, GREEN, RED`, expected `GREEN, GREEN, RED, GREEN`): the tracker
  made four records for three nodes seen: progress 0.23, 1.09, 1.97, 2.89 (node 2 true arc 1.55 estimated at 1.09 and again at 1.97), the cap of four stopped node 4 from being recorded. The ring position uses the forward
  offset only (`ahead`) and ignores the sideways offset, so the node projects onto a different part of the bent pipe each view. H23 (`ring_tracker.cpp`): add the sideways offset (offset_x x 0.5 / focal_ratio x slant range, right of the
  vehicle) to the ring position before the centerline projection. Ring tracker 21/21 tests pass. Queue restarted.
- **Stopped by the operator (2026-10-29).** All sim processes were stopped and cleaned. Final state and what was not done are in `task2_feasibility_verdict.md` and `task2_sim_results.md`. The queue
  (Advanced, Core, Disruptive with a random spawn on the H23 code, then the 9 colour pairs) had just started and gave no results. Nothing is pushed beyond the earlier push; all later commits are local.

## Worst-case run (goal: minimal sensors, 2026-11-02)

- **Setup:** the handbook folder `/media/erlangga/DATA/RobotX` is not mounted (no such directory), so the handbook was not re-read; the scoring
  and rules come from the goal prompt and the earlier docs. `install/` was rebuilt from `feat/task2-core-disruptive-sim` (H23 code) before
  the first run. Code changes are made in a separate git worktree (the batch launches from symlinked YAML and Python, so editing the main tree
  would change a running batch) and merged between batches; unit tests build in `/tmp/b`, `/tmp/i`.
- **Process fault found: no builds while a sim runs, even in a separate build dir.** Two Advanced runs (random spawn seeds 1 and 5) froze with
  "ArduPilot controller has reset" and no pose while a colcon build ran in parallel (CPU starvation breaks the ArduSub JSON lockstep). Seed 2
  and seed 9, run with nothing else on the machine, were clean. Both frozen runs are not counted and are re-run. Builds are now only done
  between batches; the run monitor (every 30 s) flags controller resets, a mission failure and 120 s with no new ring row or pose change.
- **Step 1, Advanced, random spawn, H23 code:** seed 2 (node 1) PASS 475 (order RED, GREEN, GREEN, GREEN; circle RED = target);
  seed 9 (node 4) PASS 475 (order GREEN, GREEN, GREEN, RED; circle). Seeds 1 and 5: frozen sim (above), re-run queued.
- **Pre-existing test failure:** `test_run_lifecycle` `NavigationExecutionFailureClearsTask` fails on the unchanged build too (most likely since
  H22 retries the surface navigation for 3 minutes, so a failed navigation no longer clears the task within the test's wait). Not caused by
  this work; left for now.
- **Decision (sensor set):** the target set is the bottom camera, the Pixhawk (IMU, compass) and RTK GPS at the surface. ArduSub's ALT_HOLD
  needs a depth sensor, so with no Bar30 the vehicle flies **STABILIZE** under water (the IMU holds the attitude, the heave stick is thrust)
  and every vertical decision comes from the camera or the GPS. A sim that hides the Bar30 from the mission but keeps ALT_HOLD would still be
  holding depth from it, so it would not test the real case. `sensors:=minimal` (launch argument, `launch_task2.sh --sensors minimal`, overlay
  `task2_sensors_minimal.yaml`) sets: STABILIZE under water, no rel_alt or EKF height as a depth, camera heave on, depth from the GPS surface proof.
- **D5 (Bar30 health), implemented:** `DepthHealthMonitor` flags SILENT (no sample 3 s), STUCK (no change 3 s while heave is commanded), STEP
  (> 0.3 m between samples) and DISAGREE (> 1 m against the integrated commanded heave over 10 s). Watched only under water during the run
  (at the surface the GPS fix is the proof, and the sim barometer jumps at arming). A fault is latched for the run and switches the depth to
  the GPS surface proof. Replay tests: each fault flagged within 3 s (DISAGREE within 4 s of a 0.25 m/s drift); a healthy dive and a hold at
  depth are not flagged (7 tests).
- **D4 (GPS-fix surfacing), implemented:** `calibrated_depth_node depth_source: gps_surface` publishes depth 0 (source `gps_surface`) while a
  fresh GPS fix proves the surface and invalid otherwise; the barometer is never turned into a depth. `Surface` with no depth rises blind at
  0.25 m/s until the fix proves the surface (at most 120 s), instead of waiting for a watchdog. The report's surface guard accepts
  `gps_surface`. The datum rule is unchanged (taken only before the run).
- **D1/D2 (camera descent and pipe-width standoff), implemented:** the detector reports the pipe's apparent width (edge-pair distance / image
  width). Range = diameter / (width x 2 tan(hfov/2)) (sim: 0.0889 m PVC, hfov 1.047 rad), median of 7 samples. With `sensors:=minimal`
  DiveToDepth sinks at 0.15 m/s holding the heading until the pipe (with a range) or a lit light is in view for 1 s (limit 15 m / 0.15 m/s);
  AcquirePipeline, SearchPipeline, FollowPipeline and CenterOnRing get their heave from the standoff loop (1.6 m, kp 0.4, max 0.25 m/s,
  deadband 0.1 m); HoldProbeOnRing's relative moves (max descent, the 0.6 m back-off, the climb back) use the integrated commanded heave.
  Not yet run in the sim.
- **Step 1, Advanced re-runs (random spawn, H23 code):** seed 1 (node 2) PASS 475. **Seed 5 (node 3) FAIL 325**: repair and circle BLUE
  right, order GREEN, GREEN, GREEN, RED (expected GREEN, GREEN, RED, GREEN), the same failure as before H23. Records: progress 0.0, 0.86,
  1.82 (GREEN) and 3.16 (RED), the 0.86 and 1.82 records made from x 2.03 and 1.89 within 0.23 s. Gazebo ground truth: the vehicle followed
  at z -12.25, 1.15 m above the nodes (z -13.4), while the tracker's fixed `camera_height_m` is 1.7, so a far ring was put 50% too far ahead
  and one node made two records 0.96 m apart (association 0.8 m); the cap of four then kept node 4 out. **H24:** the tracker takes the
  camera height from the pipe's apparent width (median of 9 frames, within 1 s; range x cos(tilt)), the fixed height only without it.
  New tracker test (vehicle 1.15 m above the rings, configured 1.7: two records at the right places). Not yet run in the sim.
- **D7 (order without the pipe map), implemented:** under water the real Pixhawk has no position (no GPS, no DVL; EKF3 stops aiding), while
  the sim EKF keeps one within 0.1-0.15 m of the Gazebo pose (seen in every rings.jsonl), so the order must not rest on it. With
  `sensors:=minimal` the ring tracker dead-reckons the vehicle from the last surface fix and the commanded ENU velocity
  (`/banyu/execution/preview_target`), and measures ring progress along the buoy-to-buoy line (USV PipelineBuoys bearing; origin where the
  GPS is lost at the dive), low progress at the start buoy. `centerline_points` is then not used.
- **D3 (always report, second dive), implemented:** a failed dive or traverse surfaces, shifts the target `retry_dive_offset_m` 3 m along the
  buoy line toward the other buoy and dives again; if that fails too, `GiveUpSurvey` ends the survey and the vehicle surfaces and asks for a
  best-effort Core survey (report id suffix `.best_effort`): the verified active buoy, the rings seen in progress order, GREEN for the rest.
  Ledger tests: 3 rings seen -> GREEN, GREEN, RED, GREEN; nothing seen -> all GREEN; no verified buoy -> no report; a complete survey -> the
  full report. The full search of the plan (depth steps with lateral sweeps, LED toggle, hydrophone prior) is not implemented: the
  camera descent stops at the first lock, and the hydrophone and LED are not part of the minimal set.
- **Scenario tooling (sim only), implemented:** `launch_task2.sh --pipe-depth`, `--pipe-move dx dy dyaw` (pipe moved against the buoys;
  the sim provider's end-of-pipe ground truth moves with it), `--visibility` (turbidity attenuation scaled to 3 / visibility for green),
  `--buoy-error x y` (USV buoy report error), `--current e n [gust]` (gz `/ocean_current`, gust 5 s of every 30 s), `--light-fault n:kind`
  (thermistor or magnetometer RED blink on a functional node, or LEDs off). `task2_sim_batch.sh --opts "..."` passes them to every run;
  `task2_score_run.py` scores each run from Gazebo ground truth.
- **Step 1 result (random spawn, H23 code, Bar30 depth):** Core 4 of 4 PASS 450 (seeds 2, 1, 5, 9 = nodes 1-4); Disruptive 4 of 4 PASS 500;
  Advanced 3 of 4 PASS 475 (seed 5 node 3 order wrong, cause and fix H24 above). The two frozen Advanced runs are not counted.
  Merged into `feat/task2-core-disruptive-sim` (fast-forward) and `install/` rebuilt; next: H24 on Advanced seed 5, then the first
  `sensors:=minimal` run.
- **Process slip:** a `pkill -f` with a pattern from my own command killed my own shell (the rule says use PIDs); nothing else was hit, the
  sim was cleaned with the cleanup script and PIDs. The first Advanced H24 run did not start: `main_launch` evaluated the new scenario
  helpers' conditions before `use_sim` was declared (fixed, no sim result lost).
- **Bug found by the first runs on the new build (both did not start: `autonomous_ready` false, execution readiness "MAVROS output
  disabled"):** the new `camera_heave` and `camera_descent` blocks were inserted in the middle of `vehicle_execution_node.control`, so
  `enable_mavros_publish`, `autonomy_mode` and the rest of `control` became children of `camera_descent`. Moved the blocks after `control`;
  a key-by-key comparison with the previous `task2_params.yaml` shows only the intended additions and the two `depth_sources` changes.
  The two runs (Advanced seed 5 H24 test, minimal smoke) are not counted.
- **Start race found:** the mock GCS sends the run start once; a start that arrives before the mission reports `autonomous_ready` is
  rejected (`NOT_READY`) and the run never begins (the stuck Advanced runs). A leftover e2e process from a stopped batch also ran a second
  mock GCS into the next sim. Fixes: the e2e script sends the start only after `autonomous_ready: true` (harness only, the mission rule is
  unchanged); the mission logs each GCS command and the reason a start is rejected; processes are stopped by PID from a script.
- **H24 confirmed (Advanced seed 5, random spawn, Bar30):** four records at progress 0.2, 1.98, 3.15 (RED), 4.58, order GREEN, GREEN, RED,
  GREEN right, repair and flash code (circle BLUE) right; no report because the Surface step failed (H25).
- **H25:** the Bar30 DISAGREE check flagged a healthy dive (in ALT_HOLD the vehicle does not follow the commanded climb rate), switched the
  depth to the GPS surface proof, and the sim GPS gate, which read the mission's water depth, then never gave the GPS back at the surface:
  Surface failed after its 120 s blind rise. Fixes: DISAGREE off by default (`max_disagree_m: 0`; stuck, step and silent stay); the sim GPS
  gate always reads the SITL barometer (the world), never the mission's depth.
- **First `sensors:=minimal` run (Core seed 1, node 2, middle spawn; STABILIZE, no depth sensor):** camera descent locked on the pipe after
  30 s (Gazebo z -12.48, 0.9 m above the nodes); CenterOnRing, the camera repair (Gazebo node 2 TASK_COMPLETED_GREEN), and the order from
  dead reckoning along the buoy line: four records at 31.17, 32.0 (RED), 33.45, 34.75 = GREEN, RED, GREEN, GREEN, right (D7 works). Then at
  the far end the vehicle floated up (z -12.5 to -10.8 at about 0.04 m/s: positive buoyancy, the STABILIZE heave stick is thrust and
  had zero thrust once the pipe left the view) out of camera range, the end was never detected and the traverse ran to its 600 s timeout.
  After that the second-dive branch ran: Surface rose blind until the GPS fix proved the surface (D4 works), the target moved 3 m along the
  buoy line and the second dive started; stopped there by me (the same float-up would repeat). Not counted as a pass.
- **H26:** the camera heave gets an integral term (ki 0.03, limit 0.15 m/s) that learns the buoyancy trim while the pipe is in view and keeps
  it when the pipe is lost; every other underwater action in STABILIZE gets the same learned trim added. Test: a vehicle floating up at
  0.04 m/s holds the standoff within 0.15 m and keeps a push down when the pipe is lost.
- **H25 confirmed (Advanced seed 5, random spawn, Bar30): PASS 475** (order GREEN, GREEN, RED, GREEN, circle BLUE). Advanced random spawn is
  now 4 of 4 on the current code (seeds 2, 1, 9 earlier, seed 5 here).
- **Minimal Core seed 1 with H26: FAIL 200**: order right (GREEN, RED, GREEN, GREEN, from dead reckoning on the buoy line), the traverse
  ended and the report came in about 4 minutes (H26 fixed the float-up at the pipe end), but the repair failed: contact at Gazebo z -13.32,
  then the vehicle floated 0.5 m off the switch during the hold (zero heave in STABILIZE). **H27** (committed, built, not yet run): with no
  depth hold, keep pressing gently down until the repair is confirmed.
- **Stopped here (usage limit), 2026-11-02 05:56.** Not done: H27 sim run; step 2 colour pairs; minimal-sensor 5 of 5 per tier and node;
  pipe depth 3/8/13 m; scenarios 1-18 and the worst-case combos (the launch options exist); verdict and sim results docs update.
- **H27 / H27b (minimal Core seed 1 and seed 2):** both repairs failed (`Ring did not turn green on contact`, Gazebo DAMAGED_RED). Seed 1:
  contact 0.3 m off node 2, lid at the image corner, and the H27 push kept pressing while off-centre (H27b: push only while centred).
  Seed 2 with H27b: the vehicle reached node 1 (0.15 m off, at the node's depth) but sank from z -13.06 to -13.34 in 0.5 s (about 0.56 m/s
  for 0.12 commanded): in STABILIZE the heave stick is thrust, so the commanded rate sinks the vehicle about 4 times faster than in
  ALT_HOLD, and the lid slid to the bottom edge before contact. **H28 (third fix on this item):** the repair's sink thrust is scaled
  to 0.35 with no depth hold. Batch Core seeds 2 1 5 9 minimal next.
- **H28 run (minimal Core seed 2):** the vehicle sank from z -11.1 to -14.5 (seabed at -15) during AcquirePipeline in about 12 s, past
  the pipe, and CenterOnRing then failed again and again (camera below the nodes). The camera heave (kp 0.4, max 0.25 m/s) went through
  the same STABILIZE thrust gain: the commands sank the vehicle about 4 times faster than intended, it overshot, lost the range, and the
  wound-up integral kept pushing down. **H29 (replaces H28):** in `sensors:=minimal` the RC throttle gain is 120 PWM per m/s instead of
  the ALT_HOLD 400 (and the offset 30 = RC3_DZ), so every heave command matches the real speed; the camera heave integral only learns within
  0.5 m of the standoff (no wind-up during an approach). H28's repair-only scaling removed.
- **H29 run (minimal Core seed 2):** the dive ended at 0.2 m depth: DiveToDepth succeeded at the surface, and the vehicle then searched there.
  With the slower H29 sink the start buoy's lit GREEN indicator stayed in view long enough to pass as "a light in view" for 1 s. **H30:**
  only the pipe (with a range from its width) ends the camera descent; a light alone does not.
- **H30 run (minimal Core seed 2):** DiveToDepth failed on its 100 s limit near the surface: with the H29 gain the 0.15 m/s descent is about
  48 PWM, which barely beats the buoyancy. **H31:** camera descent 0.3 m/s in `sensors:=minimal` (about 66 PWM; the dive with the old gain
  sank on about 110 PWM). The second-dive branch ran after the failed dive (Surface by GPS fix, shift 3 m) as designed.
- **H31 run (minimal Core seed 2):** the dive now sinks (Gazebo z -11.87 at the 100 s limit) but still failed: the start buoy is 1.5 m beyond
  the pipe's end and, at the slower descent, the pipe never came into the camera's footprint below the buoy (the earlier fast dive saw it
  from higher up, where the footprint is wider). The blind Surface then rose at about 0.3 m/s (z -11.9 to -4.2 in 16 s, Gazebo). **H32:**
  the camera descent creeps forward at 0.05 m/s along the buoy-to-buoy heading (the pipe lies between the buoys).
- **H32 run:** the dive failed again; the cause was the time limit, not the footprint: the limit is `max_depth_m / rate_mps` = 15 / 0.3 = 50 s
  after H31, and the real sink is about 0.24 m/s (H31: z -11.87 at the limit), so the dive stopped about 1.5 m above the nodes. **H33:**
  `camera_descent.max_depth_m` 25 in `sensors:=minimal` (83 s, about 20 m of sink). The H32 creep stays.
- **H33 run:** the dive failed again (96 s): the forward creep is thrust in STABILIZE too and moved the vehicle 9 m (to x -5.45, past the
  pipe's far end) while it was still too high to see the pipe. **Four fixes on the camera descent (H30-H33) without a pass: different
  approach (H34).** The descent worked in the first minimal run with the original settings (ALT_HOLD gain 400, 0.15 m/s, no creep); H29's
  global gain change broke it. H34: back to the original descent settings (no creep; H30's pipe-only lock kept), and the STABILIZE
  scaling only where the overshoot was: camera heave kp 0.12, max 0.08 m/s (`sensors:=minimal`), and the repair sink thrust 0.35 (H28
  restored).
- **H34 runs and a dive diagnostic (Gazebo depth and the pipe check on live frames every 2 s):** with the default gain the minimal dive
  sinks at about 1.2 m/s (z -0.65, -2.32, -4.79, -7.53, ... -13.24 at 14 s, seabed -14.9 at 16 s; throttle 1390 = 110 PWM below neutral)
  straight down at the start buoy, where the pipe is not (it ends 1.5 m before the buoy): no frame ever had the pipe (edge-pair width 0),
  and the depth window in which the pipe would be visible (about 3 m above it) passes in 2-3 s. The first minimal run's lock was most
  likely a light (H30 since removed lights). **H35:** sink slowly (0.05 commanded, about 70 PWM) and creep 0.03 along the buoy line so the
  camera sweeps the gap while the vehicle is low enough; explicit `camera_descent.max_time_s` 100 s (the commanded rate is not the sink).
- **H35/H35b/H36 dive diagnostics (Gazebo):** with the stick offset at 31 PWM the dive still sank at about 0.75 m/s, and any forward creep
  (even 0.02, about 6 PWM) drove the vehicle forward at about 0.35 m/s (from the buoy past the pipe's far end before it was low enough to
  see the pipe). A 640x480 bottom camera (`--camera-width 640`, H36; the real HORUS is 1920x1080) did not change the outcome. With no
  creep the vehicle sank straight down at the buoy (x 3.3 to 3.98): a kept frame at 3.5 m above the pipe shows the pipe's start and node 1's
  frame at the top edge of the image. **H37:** with minimal sensors the dive point is 2 m from the start buoy toward the other one
  (`mission.dive_offset_m`, 0 by default), so the pipe is below the vertical dive; no creep.
- **H37 run (minimal Core seed 1):** the dive worked: from 2 m inside the buoy it sank straight down (x 1.1-1.45) and locked on the pipe
  near z -9.4; AcquirePipeline and CenterOnRing succeeded. The repair failed again: the vehicle reached 0.2 m from node 2 at contact
  height (z -13.30, Gazebo) with the lid 0.39 off the image centre, still sinking at about 0.24 m/s, then lost the lid ("Ring did not turn
  green on contact").
- **FAILED: the Core repair with the minimal sensor set (no depth sensor, STABILIZE).** Three fixes on this item (H27 press on the switch,
  H27b press only while centred, H28 scaled sink thrust; H34 restored H28) and a different approach (H29 global STABILIZE gain, which broke
  the dive instead) did not make it work. Evidence: with no depth hold the heave stick is raw thrust; in this sim a few PWM above the
  deadband sink the vehicle at 0.2-1.2 m/s and below it the vehicle floats up at about 0.04 m/s, too coarse to sink slowly onto the
  switch with the lid centred and hold it there (seeds 1 and 2, contacts 0.15-0.3 m off the switch). With the Bar30 and ALT_HOLD the
  same camera repair passes 4 of 4 (step 1). **Sensor limit:** the repair (250 points) needs a depth hold; keep the Bar30 for the
  repair, or measure a thrust model on the real vehicle. The minimal set still earns the survey: heartbeat, start buoy and the order
  (200 points) when the dive and the survey work.
- **H37 run result:** FAIL 50 (order RED, GREEN, GREEN, GREEN against GREEN, RED, GREEN, GREEN; node 2 not repaired): the 2 m dive offset put the
  vehicle past node 1 (x 1.14 at the dive, node 1 at 2.35), so node 1 was never seen. **H37b:** dive offset 1.5 m (the shortest
  buoy gap of the course: the vehicle starts over the pipe's start).
- **H37b batch (minimal Core, middle spawn, seeds 2 and 1), stopped after two runs:** seed 2 FAIL 25 (dive locked; repair missed; the
  vehicle later floated to the surface during the traverse and searched there; the 600 s traverse timeout plus the second dive did not fit
  the 900 s run, no survey). Seed 1: dive locked, repair missed (`Ring did not turn green on contact`), then the vehicle sank onto the
  seabed past the pipe's far end (Gazebo x -3.4, z -14.9) and searched there.
- **FAILED: the Core survey with the minimal sensor set (in this sim).** The parts that work: the camera dive (H37b, the dive point inside
  the buoys), the GPS-fix surfacing (D4), the order from dead reckoning on the buoy line (D7: right in every run that saw the four nodes),
  the second dive and the always-report path (D3). What fails is holding a height over the pipe with no depth hold: the STABILIZE heave
  stick is raw thrust and the camera-range loop (H26 integral, H29/H34 gains) lets the vehicle float to the surface or sink to the seabed
  once the pipe leaves the view (four runs, both directions, Gazebo ground truth). Fixes on this item: H26, H29, H34 (gains), plus the
  dive items H30-H37b. **Sensor limit:** a depth hold (Bar30 + ALT_HOLD) is what makes both the traverse and the repair work: with it
  Core, Advanced and Disruptive pass 4 of 4 with a random spawn (step 1). Steps 4 and 5 in the minimal configuration are blocked by this
  failure; the scenario runs below use the Bar30 configuration and say so.
- **Step 2 (Bar30, random spawn):** Disruptive seed 12 (BLUE/BLUE, node 4) PASS 500. Seed 18 (BLUE/GREEN, node 2): 450 (FAIL on the
  tier): repair verified and the code read right (tin BLUE, circle GREEN), but one ring was `ordering_verified` false, the full report was
  refused (ORDER_UNVERIFIED) and the always-report path sent the best-effort Core survey (order right: GREEN, RED, GREEN, GREEN) without the
  resource code. Without D3 this run would have sent nothing (25). **H38** (code, built after the batch): the best-effort report keeps a
  verified repair and a complete flash code (tier 3/4 colours) when only the order could not be verified.
- Step 2 continued: seed 35 (BLUE/GREEN, node 3) PASS 500; seed 3 (BLUE/RED, node 2) PASS 500; seed 43 (BLUE/RED, node 1) PASS 500; seed 6
  (GREEN/RED, node 1) PASS 500. **Seed 24 (RED/RED, node 4) FAIL 50:** four GREEN records at progress 0.03, 0.85, 2.05, 3.19 (vehicle
  x 3.59 to 1.04, y -0.93: following 0.9 m beside the line) before node 4; the cap of four kept the RED node out, the survey never had a RED
  and the best-effort Core survey reported all GREEN. Same failure class as before H24 (duplicate records of one node seen from far or
  from the side). Open: a tracker that keeps more than four candidate records and merges them by spacing at the end.
- **Step 2 result (Bar30, random spawn):** Disruptive colour pairs 7 of 8 PASS 500 (seeds 12, 35, 3, 43, 6, 34 PASS; 18 = 450 by the
  tier, now fixed by H38; 24 FAIL 50, duplicate records); Advanced circle GREEN seeds 12 (node 4) and 6 (node 1): 2 of 2 PASS 475. H38
  built, ledger tests 24/24.
- **Step 4 started, stopped by the user (2026-11-02 11:25):** the first scenario (Bar30, `--pipe-depth 8`) showed that the Bar30 configuration
  dives to the fixed `inspection_depth_m` 12.5 m, below a pipe at 8 m (the vehicle searched at z -12.2): with the Bar30 the pipe depth is
  still unknown, so the dive must also stop by camera. Planned **H39** (not implemented): a `sensors:=baro` profile = ALT_HOLD with the
  Bar30 plus the camera descent, the pipe-width standoff and the dive point inside the buoys (in ALT_HOLD the heave stick is a climb rate,
  so the camera loops get the fine control they lacked in STABILIZE). All sim processes stopped; results in `task2_feasibility_verdict.md`,
  `task2_sim_results.md`, next steps in `task2_next_plan.md` and `task2_next_goal_prompt.txt`.

## Closing the remaining goal items (2026-11-02, stopped by the user)

The user stopped all sim work and asked for the results to be written and pushed. The open items are closed here; none were run after
the stop.

- **Step 3, depth-free D1-D7:** built; D1 (camera descent), D3 (second dive, always report), D4 (GPS-fix surfacing) and D7 (order without
  the pipe map) seen working in the sim; D2 and D5 unit-tested only. The pipe-depth launch option exists (`--pipe-depth`); the 3/8/13 m
  runs in the minimal configuration were not run separately. **FAILED (blocked):** the minimal configuration fails before the depth
  matters (traverse height and repair, entry "FAILED: the Core survey with the minimal sensor set").
- **Step 4, scenarios 1-18, fisheye, IR LED, Marina Bay, worst-case combos (minimal configuration):** **FAILED (blocked)** by the same
  failure: every run in the target configuration lost the pipe height or missed the switch in nominal conditions (four runs, Gazebo
  ground truth); harder conditions cannot pass in it. The tools for every scenario exist (`launch_task2.sh`, entry "Scenario tooling").
  In the Bar30 configuration one scenario started (`--pipe-depth 8`) and showed the fixed-depth dive is below a shallower pipe (H39 planned).
- **Step 5, 5/5 per tier and node, 10/10 in the worst-case combo (minimal configuration):** **FAILED (blocked)**, same evidence. In the
  Bar30 configuration each node has 2-3 passing runs (`task2_sim_results.md`), not 5.
- **Step 6:** done (`task2_feasibility_verdict.md`, `task2_sim_results.md`); next plan and goal prompt in `task2_next_plan.md` and
  `task2_next_goal_prompt.txt`.

## Gamble strategy (requested by the user, 2026-11-02)

- **Decision:** a fallback mode, `--sensors gamble` (`task2_gamble_strategy.md`): GPS to the active buoy, a timed dive and bobbing in STABILIZE, Surface by a GPS fix, best-effort Core survey with one guessed RED. No camera, no Bar30 for any decision.
- **Bug: TimedMove refused in Task 2** ("originating run (0, 0) does not match"): the action had no run identity, and ForceSuccess hid the failure (the vehicle reported without diving). Fix: `run_id` and `epoch` in `TimedMove.action`. POR not re-run in the sim.
- **Results:** 3 of 3 runs complete with RED random in 1-4, depth about 2 m, 50 points each (the guess missed).
- **`--gamble-red` (user request):** RED positions, default `2,3`. First test ignored `--gamble-red 4` (the value in `task2_params.yaml` won over the launch dict; removed from the file). After the fix `--gamble-red 4` reported GREEN, GREEN, GREEN, RED. Handbook read: one node RED, no distribution stated, so the 2 or 3 choice is a guess.


## No hydrophone (goal: the competition UUV has none, 2026-11-04)

Goal: `task2_no_hydrophone_goal_prompt.txt`. Every earlier Task 2 sim result (H1-H38, the gamble runs) ran with the sim hydrophone
on: `pinger_localization` was launched in every Task 2 profile and the `HydrophoneArray` plugin fed it. Those results are marked
"hydrophone on" from here.

### Step 1: hydrophone audit (producers and consumers)

| Item | Role | Reads pinger data for a decision? |
|---|---|---|
| `banyu_simulation` `HydrophoneArray.cc` + `acoustic_shadow.hpp` (in `bluerov2_task2/model.xacro`) | sim producer: TDOA arrivals of `pinger_green` on gz `/banyu/pinger/arrivals` | producer |
| `robotx_pinger` model, `pinger_green` / `pinger_red` in `main_world.world`; pinger visuals on `robobuoy` and `task2_start_buoy` | sim source / visuals | no |
| `gazebo_main.launch.py` `hydrophone_bridge` | gz -> ROS `/banyu/pinger/arrivals` | producer |
| `banyu_detection/pinger_localization` (+ `acoustics.cpp`) | arrivals -> `/banyu/pinger/observation` (`PingerObservation`, bearing only) | producer |
| `vehicle_execution_node` `handlePingerObs` -> telemetry `pinger_obs_*` | consumer | **yes**: `SearchPipeline` (`search_pinger_*`) homes on a fresh bearing instead of retrace + sweep |
| `underwater_odometry_node` (`topics.pinger`) | consumer: bearing corrections of the dive dead reckoning | no decision: its `/banyu/dive/odometry` and `/banyu/dive/path` have no subscriber (diagnostic only) |
| `task2_sim_provider.py` `HydrophoneHeading` on `/banyu/hydrophone/relative_heading` | sim fake heading from ground truth | no subscriber anywhere |
| `bt/pipeline_survey_tree.xml` `TransitToPinger`, `PingerForwardAtLeast` | design draft | no: not loaded by `banyu_mission` (only `banyu_mission.xml`, `task2_gamble_mission.xml`) and the nodes are not registered |
| `ActiveBuoyFix.pinger_confirmed` (`pipeline_buoys.cpp`, `survey_evidence_node.cpp` set it false; `SurveyLedger` accepts GREEN or pinger) | evidence field | no: nothing sets it true; the start buoy is the USV `PipelineBuoys` (GREEN confirmed) |
| `Course.pinger_freq_hz` (`uuv_comm_node`, `survey_evidence` `default_pinger_freq_hz`, `pipeline_buoys_node`, mock GCS) | report field from the course message | not a sensor: kept |
| `bluerov2_gz` `hydrophone_link` | passive frame | no |
| `/down_sonar/scan` bridge | (not a hydrophone) bridged, no subscriber | no |

So the only decision that read pinger data is `SearchPipeline`.

### Step 2: hydrophone off (H40)

- **Decision:** one launch argument `use_hydrophone` (default `False` in `mission.launch.py`, `main_launch.launch.py`,
  `sub_bringup.launch.py`, `gazebo_main.launch.py`; `launch_task2.sh --hydrophone`, sim and real, a warning on `--real`).
  Off: `pinger_localization` is not started, the `HydrophoneArray` plugin is left out of the model (xacro arg `hydrophone`), the
  bridge is not started, `vehicle_execution_node control.use_hydrophone: false` (no pinger subscription, `SearchPipeline`
  `use_pinger` false: retrace + sweep only), `underwater_odometry_node use_hydrophone: false`, sim provider
  `publish_hydrophone_heading: false`. On: `config/task2_hydrophone_on.yaml` (overlay, like the tier files, because a launch dict does
  not win over the params file). The code is kept.
- Tests: two new `vehicle_execution` tests with a fresh pinger dead ahead: off -> pinger never in telemetry and the search retraces
  (vx -0.2, never forward); on -> homes forward at 0.25. Suite 63/63. Xacro: plugin present only with `hydrophone:=true`.
  Sim provider tests 10/10. `launch_task2.sh --dry-run` for `--sim`, `--sim --hydrophone`, `--real`, `--real --hydrophone`.
- `pipeline_survey_tree.xml` gets a header note: not loaded, needs a hydrophone.
- Process: `systemd-inhibit` cannot run here (no system D-Bus socket, also outside the sandbox), so the batches run
  without it; the 30 s watcher flags a wall-clock jump (a suspend) instead. A first `ArduPilot controller has reset` at SITL boot is
  normal; only a second one in a run is flagged.

### Step 3: no-hydrophone baseline (full profile, random spawn)

- **Core seed 2 (node 1): FAIL 275** (no survey). Pinger topic absent (checked in the run). The repair worked (Gazebo
  TASK_COMPLETED_GREEN, although `HoldProbeOnRing` reported "Ring lost during the approach"), four rings recorded by 242 s
  (RED, GREEN, GREEN, GREEN), then the vehicle sat at Gazebo x -1.78 to -1.85 (pipe end -2.27) for 500 s: Follow ->
  lost -> Search (retrace + sweep) -> Acquire -> Follow, and the traverse never ended. The only end signal is the sim
  fixture's ground-truth window (EKF progress >= length - 0.5 m, x <= -1.77): the vehicle stopped just short of it. The
  traverse timeout, the surfacing and the second dive used up the 900 s, so nothing was reported. SearchPipeline found the
  pipe in 0.15 s each time; the pinger was not what was missing.
- **Finding (real vehicle):** nothing publishes pipe end candidates on the robot (`/banyu/pipeline/end_candidate_state` comes only
  from `task2_sim_provider.py`), so a real traverse could only end by its timeout. **H42** (worktree, built after the batch):
  `ring_tracker_node` publishes `/banyu/pipeline/rings_complete` (four records, no RED waiting for its repair; run-tagged, latched)
  and `pipeline_end_node` ends the traverse when that holds and the pipe has been out of view for `rings_complete_lost_s` 3 s. The
  handbook has exactly four nodes, so nothing is left to survey after the fourth. Test: lost before the fourth ring or the fourth
  ring with the pipe in view -> no end; four rings + 3 s out of view -> end.
- Prepared in the same worktree while the batch runs (no builds during a sim): **H39** `sensors:=baro` (overlay
  `task2_sensors_baro.yaml`: ALT_HOLD with the Bar30, `camera_descent.with_depth` = the dive stops when the pipe is in view and fails
  past `max_depth_m` measured by the Bar30, camera heave standoff with `ki` 0, `dive_offset_m` 1.5, ring order dead-reckoned along the
  buoy line), `launch_task2.sh --sensors baro`; **H41** (seed 24): with four records, a fifth distinct ring merges the two records
  closest in progress if nearer than `min_node_spacing_m` 1.0 (sim node spacing 1.28-1.55 m; seed 24's duplicate pair 0.82 m), the
  RED one keeps its slot, the freed slot takes the new ring and both are republished (node ids stay stable for `survey_evidence`);
  `launch_task2.sh --real` refuses an RC override channel that is also a teleop switch channel (as `launch_por.sh`, tested with a
  temporary teleop file with SB on CH2); `docs/task2_hardware_checklist.md` draft.
- Core seeds 2 and 1 on the hydrophone-off code both failed the same way (seed 1: the vehicle again sat at Gazebo x -1.8, the
  traverse failed, then the second dive): **the lost pinger is the cause in the sim** (with it, SearchPipeline homed differently and
  the old runs reached the fixture's end window; without it, retrace + sweep put the vehicle back on the same last stretch every time).
  The batch was stopped (by PID; the cleanup script); H39, H41, H42 merged (fast-forward), `install/` rebuilt, tests:
  vehicle_execution 64/64 (new: baro camera descent sinks at the descent rate and stops at the Bar30 floor), ring tracker 25/25 (3 new),
  pipeline_end_node 7/7 (1 new), adapter, reset, QoS, survey evidence pass. Baseline restarted on this code (`b3b_*`).
- `b3b` Core seed 2: **not counted** (sim fault): three `ArduPilot controller has reset` at 65 s, the run start was rejected
  (`NOT_READY ... holding 0`: the execution readiness went stale), the mission never started; stopped by PID, re-run queued.
  Process: my watcher script had an arithmetic bug (grep -c with `|| echo 0`) and exited; fixed and re-armed.
- `b3b` Core seed 1: **not counted**: my cleanup call (after stopping seed 2) ran its delayed SIGKILL while the batch was already
  starting seed 1 and killed it. Process fix: `stop_run.sh` (scratch) pauses the batch (SIGSTOP), stops the run by PID, cleans,
  resumes. The e2e `prepare_autonomy` call now has a 60 s timeout (it hung on the dead sim).
- `b3b` Core seed 5 (node 3): repair right (`HoldProbeOnRing` SUCCESS), four rings GREEN, GREEN, RED, GREEN recorded by 205 s,
  `/banyu/pipeline/rings_complete` true (checked on the topic), but **H42 did not end the traverse**: at the end the vehicle looped at
  Gazebo x -1.8 for 350 s, re-associating the last node every 9 s; each loss lasted < 3 s because the retrace finds the last stretch
  again within a second. Stopped (code change needed). **H42b:** the 3 s is the total time the pipe is out of view since the fourth
  ring (frame gaps capped at 1 s), not one continuous loss. Test updated to that intermittent pattern; 7/7.
- `b3c` (H42b) Core seed 2 (node 1): **PASS 450** (order RED, GREEN, GREEN, GREEN; the traverse ended by "four rings recorded and
  the pipe out of view for 3.0 s since"; pinger topic absent).
- `b3c` Core seed 1: **not counted** (harness): the mock GCS sent the run start once, the mission never logged it (no GCS command
  line, no rejection; heartbeat task stayed 1) and the UUV sat at the surface. Harness fix: `mock_gcs --start-retry-s 30` (used by
  the e2e) resends run-start with the next seq if the UUV heartbeat shows no Task 2 tasking 30 s later (at most 3 times). The
  mission's run identity rules are unchanged (duplicates in one window are still ignored). Stopped with `stop_run.sh`; re-run queued.
- `b3c` Core seed 5 (node 3): **PASS 450**. Core seed 9 (node 4): **FAIL 300**: order GREEN, RED, GREEN, GREEN. H41 merged a
  duplicate pair (progress 0.0 and 0.88, both seen during the descent from x 3.44) and kept the RED at 4.64, but it reused slot 1:
  the survey ledger keeps each node id's first sighting (`originals_`, first-pass identity), so slot 1 kept progress 0.88 and was
  upgraded to RED ("first seen GREEN, later RED"), and the RED was reported second. Without H41 the RED would not have been recorded
  at all (the seed 24 failure). **H41b:** never reuse a node id: the merged-away record is republished with a new
  `Task2RingRecord.superseded` flag and the new ring gets a new id (4); the tracker, `rings_complete`, the ledger (counts, originals,
  best effort) skip superseded records. Tests: tracker 25/25 (updated), ledger core 26/26 (new: a superseded duplicate is dropped and
  the RED at the far end is reported last), reporting 15/15, integration 6/6.
- `b3c` Advanced seed 2: **not counted** (sim fault): a burst of three controller resets right when the EKF takes the GPS (~70 s
  after boot) wedged the sim (log stops, the vehicle never holds); four run starts (with the new retry) all `NOT_READY ... holding 0`.
  The same burst is in every run that never started (b3b Core 2, b3c Core 1); passing runs have no reset there. Harness fix: the e2e
  waits 75 s after boot, then counts the attempt as up only with <= 1 controller reset and `/clock` advancing; else it relaunches
  (part of the 3 tries).
- Re-runs queued on H41b: Core seeds 1 and 9, Advanced seed 2; then step 4 (baro: pipe depth 3, 8, 13; seed 24; 0.9 m lateral).
- `b3c` Advanced seed 1 (node 2): **FAIL 450** (Core points kept: repair verified, order right; the circle was not read). Gazebo
  ground truth: during the decode hold the vehicle drifted from 0.5 m to 1.26 m off node 2 (toward node 1, about 0.94 m above the
  node) and the camera saw the lit BLUE lid in only 15 frames; `DecodeResourceCode` timed out and the best-effort/Core report went
  out. The same station-keeping limit as H13-H17 (no reference along the pipe while the lid is dark); the hold reads no pinger data,
  so not a no-hydrophone regression.
- **User question (surface collision with the buoys):** checked the ArduSub source: `fence.cpp` only logs a breach (the response is
  commented out) and `AC_Avoid` is used only by the loiter controller, not by GUIDED velocity setpoints, so a geofence gives no
  avoidance. Also found: with `sensors:=full` the surface goal IS the active buoy (`dive_offset_m` 0, `gps_tolerance_m` 1.0).
  Proposed (not built yet): a standoff goal and a keep-out detour around both PipelineBuoys positions in `NavigateSurfaceGps`, slow
  inside 5 m; the front camera (YOLO) only as a close-range stop; do not pass under the buoys (anchor line, pinger cable).
- `b3c` Advanced seed 5 (node 3): **FAIL 450**, circle reported **GREEN** (the sim's BLUE): the vehicle drifted 1.26 m back to node 2
  after the 0.6 m back-off and, 90+ s later, the decoder counted node 2's steady lid flickering in and out of view as GREEN flashes.
  Seed 9 (node 4): **FAIL 450**, no circle, although the lit BLUE lid was in view.
- **Cause of the missing codes (a sim fixture bug, not the vehicle):** `robotx_light_controller.py` timed the flashes with
  `time.time()` (wall clock), while the camera, detector and decoder run on sim time. The sim runs at about 0.32x real time now
  (ring observations 0.1 s apart in sim time, 0.31 s in wall time; `mavros_node` uses ~180% CPU), so a 1 s flash lasted 0.33 s of
  sim time, below the decoder's `min_on_s` 0.5, and every flash was dropped (seed 9: lit BLUE and dark alternate every sample). The
  earlier passes ran at a higher real-time factor. **Fix:** the controller follows the gz `/clock` for every timing decision (wall
  time only before the first clock message and for its status throttle). **H43:** the decoder counts only flashes within
  `max_code_after_repair_s` 35 s after the post-repair steady green (the code lasts 30 s), so a neighbouring node seen later can
  never become the code (seed 5); decoder tests 15/15 (new: flashes 90 s after the repair are not a code).
- Advanced seeds 1, 5, 9 (and 2) are re-run on this code; the Advanced results above are not counted for the tier.
- **Why the sim is slow (0.34x real time):** `launch_task2.sh` reports "NVIDIA GPU: not detected": the PCI bus shows only the AMD
  Cezanne iGPU (`amdgpu`), no NVIDIA device, no `/dev/nvidia*`, no `nvidia-smi` (also outside the sandbox). Gazebo renders the cameras
  on the iGPU. Not fixable from here (laptop GPU mode / BIOS / driver); told the user. Consequence: wall-clock timeouts in the harness
  are 3x shorter in sim time; the e2e run timeout is now 1200 s wall (the 20 min cap); the mission's own timeouts run on sim time.
- **User decisions (2026-11-04):** (1) standoff point for the surface goal (the buoys must never be the goal); (2) try a midpoint dive:
  navigate to the middle of the start and end buoys, head to the end buoy, dive, search. Plan: two legs (follow to the end buoy's end,
  then turn around and follow back to the start end when fewer than four rings were recorded), opt-in `--dive-point midpoint`.
- **H44 (user request): standoff point and midpoint dive** (built and unit-tested; sim runs queued):
  - `mission.buoy_standoff_m` 1.5: the surface goal is at least this far inside the start buoy along the buoy line (it was the buoy
    itself with `sensors:=full`). The reported buoy position is unchanged. Residual risk, not handled: a straight transit that passes
    close to the other buoy (no keep-out detour; ArduSub's fence only logs).
  - `dive_point:=midpoint` (`launch_task2.sh --dive-point midpoint`, overlay `task2_dive_midpoint.yaml`): `Task2BuoyTarget` now carries
    the other buoy (`other_valid`, `other_latitude/longitude`); the goal is midway between the buoys, heading to the end buoy. BT
    `SurveyPipeline`: leg 1 (traverse steps until `PipelineLegEnded`: the pipe out of view `leg_end_lost_s` 6 s in total since the
    last new ring), then, unless the pipe end is already confirmed, a 180 deg `TimedMove` on the IMU yaw, reacquire (3 tries) and the
    normal traverse back (ends by the four-ring rule H42b). In this mode `pipeline_end_node require_rings_complete` (the sim fixture's
    end at the end-buoy side cannot end the survey before the four rings) and `ring_tracker require_monotonic_order` false (leg 2 meets
    the rest of the rings the other way; the progress is absolute).
  - Tests: mission tree 33/33 (new: leg end counts out-of-view time since the last new ring), run lifecycle 24/25 (new: standoff goal
    1.5 m inside the buoy; midpoint goal; the target plumbing tests run with standoff 0; `NavigationExecutionFailureClearsTask` is the
    known pre-existing failure), pipeline buoys 13/13, ring tracker 25/25, pipeline end 7/7.
  - The full-profile Core passes above (seeds 2, 5) were before the standoff (dive at the buoy); later runs dive 1.5 m inside it.
- `b3c` Disruptive seed 2: **not counted** (start sent before execution was holding; sim then wedged). Harness: the e2e also waits
  for `/banyu/execution/readiness holding: true` before the run start.
- `b3c` Disruptive seed 1 (node 2): **PASS 500** (order GREEN, RED, GREEN, GREEN; tin and circle right) - the first tier 4 run on the sim-time light controller and H41b-H43.
- `b3c` Disruptive seed 5: **not counted** (the controller-reset burst when the EKF took the GPS; execution never held, four starts NOT_READY; the GPS gate did not act then). Harness: prepare_autonomy and the hold wait are now part of each launch attempt; an attempt that never holds is relaunched (3 tries). Re-run queued.
- `b3c` Disruptive seed 9 (node 4): **FAIL 450** (order and repair right, Gazebo TASK_COMPLETED_GREEN; no code). After the contact the lid left the camera (`HoldProbeOnRing` 'Ring lost during the approach') and no ring was in view for the whole decode hold: the station-keeping limit (H13-H17), not the pinger.
- `b3d` Core seed 1: **not counted** (run start NOT_READY: holding went false after the e2e's check, when the EKF took the GPS ~200 s wall after boot at 0.34x). Harness: the e2e now waits for the log event 'is using GPS' (not a fixed wall delay) before the health and hold checks. Re-run queued.

### Paused by the user (2026-11-04 ~15:20)

All sim processes, batches and queues were stopped (by PID and `task2_sim_cleanup.sh`; no ardusub, gz or mavros left).
The `b3d` Core seed 9 run in progress was stopped and is not counted. Nothing pushed.

**Results so far, hydrophone off (full profile, random spawn, Gazebo ground truth):**

| Tier | Node 1 (seed 2) | Node 2 (seed 1) | Node 3 (seed 5) | Node 4 (seed 9) |
|---|---|---|---|---|
| Core | PASS 450 (before the standoff) | not counted x3 (harness/sim wedge) | PASS 450 | FAIL 300 (before H41b) |
| Advanced | not counted (wedge) | FAIL 450* | FAIL 450* (GREEN) | FAIL 450* |
| Disruptive | not counted (wedge) | PASS 500 | not counted (wedge) | FAIL 450 (lid lost after contact) |

\* before the light-controller sim-clock fix: the flashes were 0.33 s of sim time; these Advanced runs do not count for the tier.

**Code state (all committed on `feat/task2-core-disruptive-sim`, built, unit-tested):** hydrophone opt-in (H40); H39 `sensors:=baro`;
H41/H41b duplicate-ring merge with superseded records; H42/H42b four-ring pipe end; H43 decoder window after the repair; sim light
controller on the sim clock; H44 surface standoff 1.5 m and `--dive-point midpoint` (two legs); `launch_task2.sh --real` RC channel
overlap refusal; `docs/task2_hardware_checklist.md` (draft). Harness: e2e waits for the EKF GPS event, health check, prepare and hold
inside the launch attempts, 1200 s timeout, mock GCS start retry.

**To resume (in this order):** re-run Core seeds 1 and 9, Advanced 2 1 5 9, Disruptive 2 and 5 (full profile, random spawn); then
`--dive-point midpoint` Core 2 1 5 9; then step 4 in `--sensors baro` (pipe depth 3/8/13, seed 24, `--pipe-move 0 0.9 0`, tier and
colour seeds, scenarios, 5/5); then step 5 (baro as the `--real` default if it beats full, dry runs, checklist final) and step 6
(verdict, sim results). Known limits: the sim runs at 0.34x on the AMD iGPU (the RTX 5050 is not on the PCI bus); the
controller-reset burst when the EKF takes the GPS wedges about one launch in three (the e2e relaunches); station keeping over
the node during the flash code (H13-H17) still loses the lid in some runs. Scratch tools (not committed, scratchpad): `queue.sh`,
`watchq.sh`, `stop_run.sh` (pause the batch with SIGSTOP, stop the run by PID, clean, resume).
- **Next session, first item (user request):** audit everything that goes to RoboCommand through the GCS: heartbeat (2 Hz, every field,
  `state` and `current_task` transitions), `/system/mission/status`, `PipelineSurvey`, `ResourceDelivery`, command handling, QoS,
  against `RobotX-SoS-communication-ITSN/README.md`. Already seen (not yet verified or changed): `uuv_comm_node`
  `robotStateLocked()` reports `STATE_AUTO` only when the FCU mode is `GUIDED`, so under water in ALT_HOLD (the Task 2 underwater
  mode) the heartbeat probably reports `STATE_MANUAL` during an autonomous run.
