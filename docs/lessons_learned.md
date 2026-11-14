# Problems and Lessons Learned

Collected from the repo history, docs and the graphify knowledge graph (2026-11-15).
Each entry: the problem we hit, the lesson, and where it is recorded. Newest field work first.

---

## 1. Latest field session (2026-11-14, PR #25)

| Problem | Lesson | Source |
|---|---|---|
| The SH "prepare autonomy" button was mapped to index 4 (CH5), the same index as `yaw_channel: 4`. On the TX16S, SH is CH7. | Verify every RC channel on the real transmitter. The overlap check in `launch_task2.sh` only compares against `task2_params.yaml`, not the stick channels inside `teleop_rcin.yaml`. | `8422868`, `banyu_teleop/config/teleop_rcin.yaml` |
| The transit stopped 0.8 m off the buoy line after a detour, and the camera missed the first pipe nodes. | The dive point must be on the buoy line: `gps_tolerance_m` went from 1.0 to 0.4 to 0.3. RTK can do better; the real limit is the USV buoy position error. | `4305097`, `task2_params.yaml` |
| The sim depths (12.5 / 13.9 m) were used on the real robot. | Keep real-site depths in `hardware_params.yaml` (`use_sim == False`): inspection 2.5 m, repair 3.0 m. | `63569f6` |
| `degraded_thrusters_2_3.parm` had never been verified (SITL ignores `SERVOx_FUNCTION 0`). | Delete workarounds that have not been checked on the robot once the hardware fix (new ESC board) lands. | `1113484` |
| On the Pi, `ros2 topic pub --once` exits before discovery and the command is lost. | Send one-shot commands through a service (`/banyu/trigger/command`): its reply confirms the command arrived. | `3373978` |
| A switch held down at startup could be read as a press. | Detect momentary switches on the low→high edge; reject prepare while a run is active. | `de404f9` |

## 2. Lessons from the 2026-11-12 trial (fixed 11-12 / 11-13)

| Problem | Lesson | Source |
|---|---|---|
| The best UAV SafePassage was the first message (8 of 9 buoys); later ones had 5–6 and B2 never came back. The node cleared its buoy memory when the first course arrived. | Reset state only on a real change (a switch between two known courses), not on the first arrival. | `14ebdc2` |
| The UAV map drops real buoys for minutes at a time. | Merge new buoys into the remembered field and expire them slowly (`merge_m`, `forget_s`) instead of replacing the list. | `docs/task2_surface_planning_trial.md` |
| Replanning every tick switched detour side about 450 times per sweep. | Keep the chosen detour point (`keepDetourPoint` hysteresis). | same |
| A surface obstacle margin of 0.9 m or more boxed the planner in. | `surface_obstacles.margin_m = 0.75` is the largest margin that still reaches every crossing. | same |
| At the 50% PWM cap (1300–1700) the vertical thrusters could barely dive. | Use the full 1100–1900 range now that the ESC power board supplies full current. | `0923d14` |
| The gamble dive in STABILIZE bobbed up and down. | Gamble also needs the Bar30: hold depth in ALT_HOLD, then fly the legs. | `9300efe` |
| YAML buoy coordinates were still required at run start and their heading was reused. | Buoys come from one source only: the USV `PipelineBuoys`. | `c421db2` |
| A sim on a laptop could reach the real robot's MAVROS (same ROS domain, subnet discovery). | Sim scripts set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. | `833bb7d` |
| The MAVROS param plugin's endless download starved the stack. | Disable the param, waypoint, geofence and rallypoint plugins on the robot (`mavros_config_real.yaml`). | `75113ef` |
| Replay tests used an invented buoy field. | Replay the trial's real message sequence in the mock, the replay and the unit tests. | `14ebdc2`, `4deda27` |

## 3. ArduSub / MAVLink

| Problem | Lesson | Source |
|---|---|---|
| Position targets (`0xDF8`) through AC_WPNav crawl and let depth drift. | Stream velocity setpoints (`type_mask 0x5C7`) at 10–50 Hz. | `AGENTS.md`, `docs/auv_mission_architecture.md` |
| Without a yaw rate, ArduSub keeps the last yaw slew and turns the vehicle to heading 0. | Every setpoint carries an explicit zero yaw rate. | `AGENTS.md` |
| GUIDED after GPS loss accepts setpoints, ignores them and loses depth, with no error (M1b). | GUIDED only on the surface; ALT_HOLD + RC override underwater (M3). | `docs/task2_mode_experiments.md` |
| SITL GPS keeps working underwater, which hid the GUIDED failure. | `sim_gps_gate` cuts GPS below 0.1 m depth in sim. | `docs/task2_core_disruptive_plan.md` |
| ArduSub took RC override only from sysid 1; MAVROS uses 255, so all POR motion was ignored. | Set `MAV_GCS_SYSID 255`. | `docs/por/README.md` |
| `FS_THR_ENABLE 0` keeps the thrusters on the last stick value after RC loss; 2 surfaces underwater. | Use `FS_THR_ENABLE 1`. | same |
| ArduSub's own fence only logs a breach. | Enforce the course fence and buoy keep-out in `vehicle_execution_node`. | `AGENTS.md`, `docs/task2_waypoint_mission.md` |
| STABILIZE and ALT_HOLD on the surface pushed the hull under and cut the link. | Stand by in MANUAL: neutral sticks give zero thrust. | `AGENTS.md` |
| Launching MAVROS from a launch file injects a `__node` remap that crashes it (mavros#1977). | Start MAVROS with `launch_mavros.sh`, never from a launch file. | `AGENTS.md` |
| `nogps_odom_beacon` reports zero displacement, so velocity control adds thrust forever. | Never use it for GUIDED underwater. | `docs/task2_underwater_position.md` |
| With no DVL and no GPS, EKF3 has no position. | Use EKF3 source set 2 with zero-delta `VISION_POSITION_DELTA` (depth and heading only). | `banyu_bringup/README.md` |
| The sim barometer drifts 0.1–0.4 m. | Calibrated depth = datum − rel_alt, with the datum captured at the surface before every run. | `AGENTS.md` |

## 4. ROS 2 / BehaviorTree

| Problem | Lesson | Source |
|---|---|---|
| `ReactiveSequence` re-ticks from its first child, so a `StatefulActionNode` never reaches `onRunning`. | Use a memory Sequence for the phase chain under a root ReactiveFallback for abort. | `docs/auv_mission_architecture.md` |
| `Course` is published volatile, so latched subscriptions never received it. | Subscribe with `QoS(10).reliable()` and have the GCS republish at about 1 Hz. | `docs/issue13.md` |
| launch_ros puts params files after arguments, so `tier:=` never reached the nodes. | Use per-tier overlay files (`task2_tier_*.yaml`). | `docs/task2_run_log.md` |
| The light flashes used wall time while the sim ran at about 0.32×. | Drive sim timing from the sim clock. | same |
| The RoboCommand heartbeat flapped 592 times (AUTO required GUIDED; freshness used wall clock). | Audit heartbeat conditions and their clocks. | same |
| Live sim traffic leaked into unit tests. | Run tests on `ROS_DOMAIN_ID=42`. | `.agents/skills/banyu-sim-test-and-monitor` |
| Orphaned ardusub/mavproxy processes cause port 5501 bind errors. | Clean up stale processes before each sim. | same |
| A stale client could move the vehicle. | Authorize every run with `run_id` + `epoch`. A single `RunCoordinator` owns the lifecycle. | `docs/actions.md`, `docs/issue13.md` |

## 5. Perception / hardware

| Problem | Lesson | Source |
|---|---|---|
| A solid green image: 1080p YUYV needs about 124 MB/s and USB 2.0 carries about 35–40 MB/s. | Use MJPG with `usb_cam` (`v4l2_camera` crashes on MJPG). | `docs/horus_camera.md` |
| A node seen green and then red was recorded twice, pushing RED one slot late. | Upgrade the existing record to RED; never downgrade. | `docs/task2_results/e2e/core_matrix2_summary.md` |
| One green frame on a flickering lid was taken as the repair. | Require 1.5 s of steady GREEN. | `docs/task2_core_repair_by_camera.md` |
| Dark phases read as GREEN against olive water. | Cap fallback ring confidence at 0.8 (H11). | `docs/task2_run_log.md` |
| Red flicker and green from a neighbouring node passed as flashes. | The decoder requires RED, then steady GREEN (H4–H9). | same |
| The pipe depth varies, so a fixed `repair_depth_m` fails. | Approach by camera (`HoldProbeOnRing`), using the lit-lid size for range. | `docs/task2_core_repair_by_camera.md` |
| The vehicle drifts about 0.1 m/s with no command after repair. | Rise 0.6 m before reading the code so the lid stays in view (H17). | `docs/task2_run_log.md` |
| The hard-coded sim pipe map does not exist at Marina Bay. | Order rings by first sighting. | `docs/task2_future_plan_depth_free.md` |
| Without a depth sensor there is no ALT_HOLD, and STABILIZE heave is too coarse. | Keep the Bar30. | `docs/task2_feasibility_verdict.md` |
| `gps_to_pixhawk.py` capped the satellite count and froze the position. | Use `gps_forwarder` (RTK chain to `GPS1_TYPE 14`). | `docs/hardware_setup.md` |
| Sim results used a hydrophone the competition UUV does not have. | `use_hydrophone` defaults false; earlier results were invalid (H40). | `AGENTS.md` |
| The link drops when the AUV dives. | Record every run (MCAP, 60 s splits, stop below 2 GB free). | `banyu_bringup/README.md` |
| 90° turns overshoot about 14° and the error builds up. | Fly a strafe box with a fixed yaw. | `docs/por/sim_results.md` |
| `AcquireGate` moving sideways could put the sub above the gate. | Dive in place. | `docs/actions.md` |

## 6. Process

- **The sim is right about logic and wrong about the robot.** Channel numbers, depths, thrust limits and discovery behaviour on the Pi all had to be checked on the real robot.
- Log every field failure as a numbered hypothesis (H1–H51 in `docs/task2_run_log.md`): symptom, cause, fix, result.
- Write a reason into every commit body. The 11-14 field fixes have no body, so their reasons had to be rebuilt from the diffs.
- Agent workflow: re-verify every worker's report; review on two axes (spec and standards); workers never start subagents; keep a progress ledger that survives context compaction.

## 7. Carrying these lessons to a new robot (mini PC + Pixhawk/ArduSub, different mission)

Most of the lessons above come from the platform (ROS 2 on a mini PC, ArduSub on a Pixhawk, MAVROS between them), not from Task 2. They carry straight over to a new robot.

### Talking to ArduSub
- Stream velocity setpoints (`type_mask 0x5C7`, 10–50 Hz), not position targets.
- Put an explicit zero yaw rate in every setpoint.
- Use GUIDED only while GPS is valid (on the surface). Underwater, use ALT_HOLD + RC override.
- A depth sensor (Bar30) is required: ALT_HOLD needs it.
- No DVL means no underwater x/y. Design the mission around heading, depth and time, plus camera-based approach. If a task needs underwater position, budget for a DVL or a vision position source.
- Stand by in MANUAL on the surface.
- Day-one parameters: `MAV_GCS_SYSID 255`, `FS_THR_ENABLE 1`, PWM limits that match what the ESC power board can supply.
- ArduSub's fence only logs a breach. Enforce any geofence or keep-out zone in your own execution node.

### MAVROS and ROS 2 on the mini PC
- Start MAVROS from a script, never from a launch file (mavros#1977).
- Disable unused MAVROS plugins (param, waypoint, geofence, rallypoint).
- Send one-shot commands through a service, not `ros2 topic pub --once`.
- Isolate the sim: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` for the sim, a separate `ROS_DOMAIN_ID` for tests.
- Choose QoS on purpose: a volatile publisher with a latched subscriber means messages never arrive.
- Use per-mode overlay YAML files, not launch arguments that params files override.
- Use the sim clock in sim.

### Mission architecture
- Keep three layers separate:
  - mission (BehaviorTree)
  - execution and safety (fence, mode switching, setpoint streaming)
  - vehicle interface (MAVROS)

  For a new mission, rewrite only the BT and its actions.
- BT: a memory Sequence for the phase chain, under a ReactiveFallback for abort. Never use `ReactiveSequence` for stateful phases.
- One run coordinator owns the run. Authorize every run with `run_id` + `epoch`.
- Trigger momentary switches on the rising edge. Reject arm or prepare commands while a run is active.

### Hardware and perception
- USB 2.0 cannot carry 1080p YUYV: use MJPG with `usb_cam`.
- Debounce every detection over time. Upgrade a record when better evidence arrives, but never downgrade it.
- Don't hard-code sim maps or coordinates. Order targets by first sighting and use live sources.
- Record every run (MCAP, split files, stop below a free-disk limit). The link drops underwater.

### Sim vs. real
- The sim validates logic, not the robot. Channels, depths, PWM limits, discovery timing and underwater GPS were all wrong on the real robot even though they worked in sim.
- Make the sim honest: cut GPS underwater, and remove sensors the real robot doesn't have.
- Keep real-robot values in a separate hardware params file (`use_sim == False`).
- Check every RC channel on the physical transmitter. Add a startup check that no two functions share a channel.
- Delete workarounds that were never verified on the robot once the hardware fix lands.

### Process
- Log each field failure as a numbered hypothesis: symptom, cause, fix, result.
- Write a reason into every commit body.
- Replay real field data in tests, not invented scenarios.

### What to reuse vs. rewrite
- **Reuse:** `banyu_vehicle_interface`, `banyu_vehicle_execution` (fence and mode logic), `banyu_teleop`, the MAVROS launch scripts and config, the recording setup, and the sim isolation.
- **Rewrite:** the `banyu_mission` BT and actions, and the task-specific detection.

## 8. Open items found while writing this

- [ ] `teleop_rcin.yaml:25` still says "SH = CH5 = index 4", but the value is 6 (CH7).
- [ ] The default `prepare_channel` is still 4 in `banyu_teleop.cpp:57` and `launch_task2.sh:357`, the same index as `yaw_channel`.
- [ ] `hardware_params.yaml` calls 2.5 / 3.0 m "pool depths", but the commit says Marina Bay. Confirm the real site depth.
- [ ] `docs/por/README.md:61` and `:468` still describe the deleted `degraded_thrusters_2_3.parm`.
- [ ] The overlap check in `launch_task2.sh` does not compare `prepare_channel` against the teleop stick channels.
