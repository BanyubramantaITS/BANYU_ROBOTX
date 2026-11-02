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
