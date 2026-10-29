# Vehicle actions: what the UUV can do

A catalogue of every action the mission can run: what exists today, and what is planned. Use
it to look up a building block before writing or changing a behavior tree.

- **ROS actions** are `banyu_msgs/action/*.action` goals that `banyu_vehicle_execution/vehicle_execution_node`
  serves under `/banyu/actions/<name>`. Each one has a matching BT node in `banyu_mission`.
- **BT-only nodes** run inside `banyu_mission` itself (service calls, mode changes, blackboard work)
  and have no action server.
- **Planned** nodes are used in a design tree or a design doc but have no implementation yet.

Source of truth: action definitions in `banyu_msgs/action/`, BT registration in
`banyu_mission/src/action/register_actions.cpp` and `banyu_mission/src/banyu_mission.cpp`, servers in
`banyu_vehicle_execution/src/action_servers/`. If this page and the code disagree, the code wins:
fix this page in the same change.

## How actions run

- **One owner at a time.** Every server asks the arbiter (`GoalArbiter`, via
  `VehicleExecutionNode::requestGoal`) before accepting a goal. The arbiter activates it, queues it
  as pending, or hands control over from the running goal, which is then aborted as displaced.
  Goals are rejected while the hardware status isn't ready (when MAVROS publishing is enabled).
  Cancelling a goal (or halting its BT node) stops it and zeroes its motion intent.
- **Each server returns a motion intent every control tick.** `vehicle_execution_node` turns it into
  the output: velocity setpoints on `/mavros/setpoint_raw/local` (GUIDED, Task 2) or RC override
  sticks on `/mavros/rc/override` (ALT_HOLD, POR). Which one depends on `control.output_mode` in the
  params file, not on the action.
- **`timeout_s` on POR actions is a watchdog, not a duration.** It is how long a missing input
  (stale detection, no depth) is tolerated before the goal fails. The action ends when its own
  success condition is met.
- **Task 2 goals carry `run_id` and `epoch`.** The BT node fills them from the current run. With
  `require_run_authorization` on (Task 2), goals are rejected unless the run is authorized and the
  goal's `run_id`/`epoch` match the active run, so a stale client can't move the vehicle.
- **Frames.** Headings are ENU radians (counter-clockwise from east). Depth is metres, positive
  down, 0 = surface. `TimedMove` speeds are body FLU (+x forward, +y left, +z up). Image offsets
  are normalized ±1, +y = image-down.
- **Results.** Every result has `success` and a human-readable `reason`. Every feedback has a
  `stamp`. Both are recorded in the mission bag (hidden action topics are included).

## Summary

| Action | Used by | Server | Status |
|---|---|---|---|
| [`TimedMove`](#timedmove) | POR timed, hybrid, vision-assisted, replay | `TimedMoveServer` | Implemented, default POR |
| [`ReplayTrack`](#replaytrack) | POR replay | `ReplayTrackServer` | Implemented |
| [`AcquireGate`](#acquiregate) | POR vision | `AcquireGateServer` | Implemented |
| [`SearchGate`](#searchgate) | POR vision, hybrid, vision-assisted | `SearchGateServer` | Implemented |
| [`AlignGate`](#aligngate) | POR vision, hybrid, vision-assisted | `AlignGateServer` | Implemented |
| [`PassGate`](#passgate) | POR vision, hybrid, vision-assisted | `PassGateServer` | Implemented |
| [`SearchMarker`](#searchmarker) | POR vision | `SearchMarkerServer` | Implemented |
| [`AlignMarker`](#alignmarker) | POR vision, vision-assisted | `AlignMarkerServer` | Implemented |
| [`OrbitMarker`](#orbitmarker) | POR vision | `OrbitMarkerServer` | Implemented |
| [`TurnAround`](#turnaround) | POR vision, hybrid, vision-assisted | `TurnAroundServer` | Implemented |
| [`NavigateSurfaceGps`](#navigatesurfacegps) | Task 2 | `NavigateSurfaceGpsServer` | Implemented (surface, GPS) |
| [`AlignHeading`](#alignheading) | Task 2 | `AlignHeadingServer` | Implemented (surface) |
| [`DiveToDepth`](#divetodepth) | Task 2 | `DiveToDepthServer` | Implemented, sim only ¹ |
| [`AcquirePipeline`](#acquirepipeline) | Task 2 | `AcquirePipelineServer` | Implemented, sim only ¹ |
| [`SearchPipeline`](#searchpipeline) | Task 2 | `SearchPipelineServer` | Implemented, sim only ¹ |
| [`FollowPipeline`](#followpipeline) | Task 2 | `FollowPipelineServer` | Implemented, sim only ¹ |
| [`CenterOnRing`](#centeronring) | Task 2 | `CenterOnRingServer` | Implemented, sim only ¹ |
| [`HoldProbeOnRing`](#holdprobeonring) | Task 2 | `HoldProbeOnRingServer` | Implemented, sim only ¹ |
| [`Surface`](#surface) | Task 2 | `SurfaceServer` | Implemented |

¹ These run underwater in GUIDED. On the robot, GUIDED loses its position estimate once GPS is
gone, so they have only been proven in sim, where SITL GPS leaks underwater. See
[Planned: Task 2 underwater without GPS](#task-2-underwater-without-gps) and
`docs/task2_underwater_position.md`.

POR trees: timed = `por_timed_mission.xml` (default), vision = `por_mission.xml`, hybrid =
`por_hybrid_mission.xml`, vision-assisted = `por_vision_assisted_mission.xml`, replay =
`por_replay_mission.xml`. Task 2 = `banyu_mission.xml`.

---

## POR actions

POR flies ALT_HOLD + RC override: the server's velocity intent becomes stick deflections, so
speeds are open loop (calibrated in the pool, `docs/por/tuning.md`), and depth and heading come
from the Bar30 and IMU.

### TimedMove

Drive one body-frame segment for a fixed time: the dead-reckoning building block of the timed
POR. It can hold depth on the barometer, stop the yaw at a target angle, wait for a depth, and
stop forward motion near an obstacle.

| Goal field (BT port) | Meaning |
|---|---|
| `segment_name` | Label for logs and feedback |
| `duration_s` | Segment length |
| `speed_x_mps`, `speed_y_mps`, `speed_z_mps` | Body FLU speeds |
| `yaw_rate_rad_s` | Yaw rate during the segment |
| `use_baro_depth`, `target_depth_m`, `depth_kp`, `max_depth_speed_mps` | Closed-loop depth hold instead of `speed_z_mps` |
| `use_imu_yaw_cap`, `target_yaw_delta_rad` | Stop yawing once the IMU has turned this much |
| `check_depth_reached`, `depth_tolerance_m` | Also wait until the depth is within tolerance (default off) |
| `enable_collision_guard`, `collision_distance_m` | Halt forward motion when an obstacle is nearer (default off) |

Result: `elapsed_s`, `final_depth_m`, `accumulated_yaw_rad`. Feedback: `elapsed_s`, `depth_m`,
`accumulated_yaw_rad`.

```xml
<TimedMove segment_name="dive_to_gate" duration_s="{dive_duration_s}"
  speed_x_mps="0.0" speed_y_mps="0.0" speed_z_mps="-0.2" yaw_rate_rad_s="0.0"
  use_baro_depth="true" target_depth_m="{gate_depth_m}" depth_kp="{depth_kp}"
  max_depth_speed_mps="{max_depth_speed_mps}" use_imu_yaw_cap="false" target_yaw_delta_rad="0.0"/>
```

### ReplayTrack

Replay a manual "teach" drive: recorded forward, lateral and yaw sticks are replayed open loop.
The recorded depth and heading are tracked closed loop with the Bar30 and IMU. The track clock
pauses (sticks neutral) while the vehicle is far off the recorded depth.

| Goal field | BT port | Meaning |
|---|---|---|
| `track_name`, `time_s[]`, `forward_pwm[]`, `lateral_pwm[]`, `yaw_pwm[]`, `heading_rad[]`, `depth_m[]` | `track` (a loaded track, picked by the `replay_track` launch argument) | The track. Arrays share one index; `time_s` starts at 0 and increases |
| `depth_kp`, `max_depth_speed_mps` | same | Depth loop |

Result: `elapsed_s`, `paused_s`, `final_depth_m`, `final_heading_error_rad`. Feedback includes
`track_time_s`, `duration_s`, `paused`, `target_depth_m`.

Tracks are cut from a bag with `tools/scripts/por_teach_extract.py`.

### AcquireGate

Dive in place to the gate depth and hold until a fresh gate detection arrives. It doesn't move
horizontally: moving here could put the sub above the gate and hit it on the way down. Forward
motion belongs to `PassGate`.

Goal: `target_depth_m`, `depth_tolerance_m`, `timeout_s`. Result: `final_depth_m`, `gate_visible`.

### SearchGate

Move forward at `search_speed_mps` until a fresh gate detection (object 0) appears. Goal:
`search_speed_mps`, `timeout_s`. Result: `gate_found`.

### AlignGate

Center the gate on the image crosshair by driving the normalized offsets to zero (sway for x,
heave for y). It succeeds after the offsets stay inside the deadband for `hold_time_s`, and it
keeps correcting during the hold.

Goal: `deadband_x/y`, `gain_x/y`, `max_speed_y`, `max_speed_z`, `hold_time_s`, `timeout_s`.
Result: `final_offset_x/y`.

### PassGate

Drive through the gate while keeping it centered, until `target_distance_m` is covered.
`minimum_distance_m` is the closest allowed range to the gate. On the return leg
(`return_leg=true`), losing the gate enters a **blind push** that keeps going forward for up to
`blind_duration_s` (log: `Executing: PassGate Detection Lost, Continue pushing through`). Seeing
the gate again ends the blind push.

Goal: `return_leg`, `speed_mps`, `target_distance_m`, `minimum_distance_m`, `blind_duration_s`,
`timeout_s`. Result: `travelled_m`, `minimum_observed_distance_m`.

### SearchMarker

Move forward until a fresh yellow-buoy detection (object 1) appears. Goal: `search_speed_mps`,
`timeout_s`. Result: `final_distance_m`.

### AlignMarker

Center the marker and hold a standoff range between `standoff_min_m` and `standoff_max_m`
(never closer than `minimum_distance_m`). With `lateral_only=true` it only centers sideways, with
no range hold and no depth change. The vision-assisted strategy uses this because monocular pole
range is unreliable.

Goal: `deadband_x/y`, `gain_x/y`, `max_speed_y/z`, `minimum_distance_m`, `standoff_min_m`,
`standoff_max_m`, `hold_time_s`, `timeout_s`, `lateral_only` (default false). Result:
`final_offset_x/y`, `final_distance_m`.

### OrbitMarker

Sway around the marker at `radius_m` while yawing to keep it centered, until the integrated yaw
reaches `angle_rad` (a full turn = 2π). Yaw uses feedback plus the feedforward
ω = −sway / radius, so the marker stays on the crosshair rather than lagging behind it.

Goal: `orbit_speed_mps`, `radius_m`, `range_gain`, `max_range_speed_mps`, `yaw_gain`,
`max_yaw_rate_rad_s`, `vertical_gain`, `max_vertical_speed_mps`, `angle_rad`, `timeout_s`.
Result: `accumulated_yaw_rad`, `final_distance_m`.

### TurnAround

Yaw at `turn_yaw_rate_rad_s` until the gate is captured in the image (offset within
`capture_deadband`, steered by `capture_gain`). The turn is capped at `max_angle_rad`.

Goal: `turn_yaw_rate_rad_s`, `capture_gain`, `capture_deadband`, `max_angle_rad`, `timeout_s`.
Result: `accumulated_yaw_rad`, `gate_captured`.

---

## Task 2 actions

Task 2 flies GUIDED with velocity-only setpoints (`type_mask 0x5C7`, explicit zero yaw rate).
Every goal carries `run_id` and `epoch`, which the BT node fills in. BT port names differ from the
goal field names (no unit suffix), as listed below.

### NavigateSurfaceGps

Transit on the surface to a GPS point (WGS84 degrees) and succeed within `tolerance_m`.

| BT port | Goal field |
|---|---|
| `target_latitude`, `target_longitude` | `target_latitude_deg`, `target_longitude_deg` |
| `start_latitude`, `start_longitude` | `start_latitude_deg`, `start_longitude_deg` |
| `tolerance` | `tolerance_m` |
| `retarget_threshold_m` (BT only, default 1.0) | Re-sends the goal when the USV buoy target moves more than this |

Result: `final_latitude/longitude_deg`, `final_error_m`. Feedback: `distance_remaining_m`.

### AlignHeading

Yaw in place to `target_heading` within `tolerance` (ENU rad). Result: `final_heading_rad`,
`final_error_rad`.

### DiveToDepth

Descend to `target_depth` and hold `target_heading`, each within its tolerance.
Ports: `target_depth`, `target_heading`, `depth_tolerance`, `heading_tolerance`. Result:
`final_depth_m`, `final_heading_rad`.

### AcquirePipeline

Wait for the detector to report the pipeline tracked on `camera_name`. With `use_heading_prior`,
the pipe heading must match `heading_prior` within `heading_tolerance`. It fails with
`Acquisition timed out without acquiring pipeline` (wrap it in a BT `Timeout`).
Result: `acquired`, `pipeline_id`, `pipeline_heading_rad`.

### SearchPipeline

Sweep the heading over `search_span` at `sweep_rate`, starting from `heading_prior` when
`use_heading_prior` is set, until the pipeline is seen. It fails with `Search span exhausted
without acquiring pipeline`. Result: `found`, `pipeline_id`, `found_heading_rad`.

### FollowPipeline

Follow the pipe using `/banyu/guidance/velocity_intent` (surge, sway and yaw rate from
`pipeline_guidance_node`). With `enable=true` it **never finishes on its own**: it runs until the
BT halts it (red ring pending, pipeline lost, pipeline ended, timeout). With `enable=false` it
succeeds at once (`Pipeline following stopped`). While guidance isn't tracking it holds still and
reports `status: searching`. Feedback: `following`, `status`, `lateral_error_m`.

### CenterOnRing

Center the ring named `ring_color` (e.g. `red`) under the camera within `tolerance` (normalized
offset). It fails with `Ring not reacquired within search timeout` when the ring is lost too long.
Result: `ring_color`, `final_offset_x/y`.

### HoldProbeOnRing

Keep the probe aligned on the ring continuously for `hold_duration` seconds within
`alignment_tolerance`. Result: `held_duration_s`, `maximum_alignment_error`. Followed in the tree
by `AcknowledgeRedRing`.

### Surface

Rise to `target_depth` (must be 0 = the surface on the calibrated depth) within
`depth_tolerance`, and succeed only after the surface depth is verified continuously.
`watchdog_timeout_s` (default 0 = off) fails the goal when the depth input is missing for that
long (`Surface water-depth input unavailable or timed out`). Result: `final_depth_m`,
`final_error_m`.

---

## BT-only nodes (no action server)

### Actions

| Node | Tree | What it does |
|---|---|---|
| `ArmAndSetGuided` | POR, Task 2 | Sets the mode (`autonomy_mode`: ALT_HOLD for POR, GUIDED for Task 2) and arms through MAVROS (`services.set_mode`, `services.arming`) |
| `HandleAbortOrTeleop` | all | Runs the abort path when the interrupt branch fires |
| `AcknowledgeRedRing` | Task 2 | Calls `/banyu/pipeline/acknowledge_ring_repair` after a hold |
| `LockBuoyTarget` | Task 2 | Freezes the USV buoy target on the blackboard after `NavigateSurfaceGps` |
| `WaitSurveyComplete` | Task 2 | Waits for the survey evidence to be complete |
| `ConfirmSurfaceAction` | Task 2 | Confirms the vehicle is at the surface before reporting |
| `PublishPipelineSurvey` | Task 2 | Calls `/banyu/communication/trigger_pipeline_survey` so `uuv_comm_node` sends the report |

### Conditions

| Node | Meaning |
|---|---|
| `RequireMissionStarted` | The run was started (`/banyu/mission/start`, or `CMD_RUN_START` for Task 2) |
| `RequireVehicleReady` | Vehicle status is ready for autonomy |
| `RequireGpsAvailable` | A usable GPS fix is present |
| `AutonomyInterrupted` | Abort, teleop or kill switch is active (drives the root interrupt branch) |
| `PipelineActive` / `PipelineLost` / `PipelineEnded` | Pipeline tracking state |
| `RedRingPending` | A red ring is waiting for repair |

---

## Planned

### Advanced and Disruptive pipeline survey (`pipeline_survey_tree.xml`)

`banyu_mission/bt/pipeline_survey_tree.xml` describes a Task 2 tree for an ALT_HOLD vehicle without
a position estimate (bottom camera, echosounder, hydrophones, Bar30, compass, GPS only at the
surface). It is **not loaded** and none of the nodes below are registered. Building it means
adding these nodes (as ROS actions in `vehicle_execution_node` where they move the vehicle).

| Node | Ports | Purpose |
|---|---|---|
| `MissionActive` | | Run is live (like `RequireMissionStarted`) |
| `AbortedOrTeleop` | | Interrupt condition (like `AutonomyInterrupted`) |
| `ArmVehicle` | `arm` | Arm or disarm, without the mode change |
| `SetFlightMode` | `mode` (`ALT_HOLD`, `STABILIZE`, …) | Mode change mid-mission |
| `SurfaceVehicle` | | Rise to the surface (ALT_HOLD version of `Surface`) |
| `TransitToPinger` | | Surface transit to the active (GREEN, pinging) buoy |
| `RecordBuoyFix` | | Store the active buoy GPS fix for the report |
| `FindPipe` | | Find the pipeline under the camera (ALT_HOLD `SearchPipeline` + `AcquirePipeline`) |
| `SetSurveyDepth` | | Settle at the survey depth over the pipe |
| `TurnOnPipe` | `reverse` | Square up along the pipe, or turn 180° along it |
| `PingerForwardAtLeast` | `min_forward` | Condition: the pinger bearing is ahead by at least this (from TDOA) |
| `FollowPipe` | `direction` (±1), `record`, `stop_at_red`, `resume` | Follow the pipe, record segment colors, optionally stop at the RED node |
| `RepairLight` | | Press the magnetic probe on the RED node until it turns GREEN (in STABILIZE, so the vertical stick is thrust) |
| `ReadFlash` | | Read the two-color flash after repair (delivery circle, plus the tin color for Disruptive) |
| `SendResourceRequest` | | Report the delivery request, which the GCS forwards to the UAV as `CMD_DELIVERY` |
| `SendReport` | | Send the pipeline survey once floating |

The tree selects the tier with `_skipIf` on a `tier` blackboard string (`core`, `advanced`,
`disruptive`). `banyu_mission` today declares the tier as the integer `mission.task2_tier`
(2–4, default 2), so loading this tree also needs that mapping.

### Task 2 underwater without GPS

Open decision, see `docs/task2_underwater_position.md`. The preferred option (A) keeps GUIDED for
the surface transit and switches to ALT_HOLD + RC override after the dive, as POR does. That needs:

- a mid-mission mode switch (`SetFlightMode` above, plus `vehicle_execution_node` switching
  `control.output_mode` from setpoint to `rc_override`);
- `FollowPipeline`, `CenterOnRing`, `HoldProbeOnRing`, `AcquirePipeline`, `SearchPipeline` and
  `Surface` running on RC override sticks, with pool-calibrated stick gains;
- a sim test with SITL GPS disabled underwater (`SIM_GPS1_ENABLE 0` after the dive).

### Task 4 (dynamic incident)

`uuv_comm_node` only acknowledges `CMD_TASK4_ASSISTANCE` and reports not-ready
(`docs/issue13.md`). Transit, hold, confirmation and avoidance are not implemented, and no Task 4
actions exist yet.

## Adding an action

1. Define `banyu_msgs/action/<Name>.action` (goal / result with `success` and `reason` / feedback
   with `stamp`) and add it to `banyu_msgs/CMakeLists.txt`.
2. Implement a `TypedActionServer<…>` in `banyu_vehicle_execution/src/action_servers/` that returns
   a motion intent from `tick()`. Register it in `vehicle_execution_node.cpp` with an
   `actions.<name>` parameter (default `/banyu/actions/<name>`).
3. Add an `ActionClientNode<…>` BT node in `banyu_mission` with its ports and register it in
   `register_actions.cpp`.
4. Add the `actions.<name>` parameter (and any new ones) to `por_params*.yaml` / `task2_params.yaml`
   and check `mission.launch.py` passes them (see the implementation rule in `AGENTS.md`).
5. Add gtest coverage in `banyu_vehicle_execution/test/` and update this page.
