# AUV mission architecture

This document describes the current ROS 2 Humble implementation in
`src/BANYU_ROBOTX`. The transport-neutral `SendMissionReport` action server is
implemented by `banyu_communication/uav_report_node`; the physical vehicle
action servers and the vehicle-command bridge are not yet implemented. They
are called out explicitly below.

## 1. Architecture overview

The stack is split into transport-neutral mission contracts, perception and
state machines, guidance, vehicle status, and bringup:

- **`banyu_bringup`** starts Gazebo/SITL or hardware-facing processes and
  supplies configuration. `launch/main_launch.launch.py` composes the
  Gazebo, SITL/MAVROS, and mission launches. `task2_sim_provider.py` is a
  deterministic Task 2 fixture, not a vehicle controller.
- **`banyu_msgs`** owns the mission messages, services, and action contracts.
- **`banyu_mission`** owns mission lifecycle, BehaviorTree.CPP registration,
  blackboard configuration, action clients, and safety conditions.
- **`banyu_detection`** contains the Task 2 camera detector, the older
  `por_detection` target detector, the ring tracker, and the pipeline-end
  confirmation library.
- **`banyu_guidance`** converts a valid pipeline observation into a bounded
  body-frame `VelocityIntent`. It intentionally does not depend on MAVROS or
  a mission tree.
- **`banyu_vehicle_interface`** converts MAVROS FCU, RC, and GPS inputs into
  the MAVROS-neutral `/banyu/vehicle_status` message.
- **`banyu_communication`** implements the transport-neutral
  `SendMissionReport` action server. `uav_report_node` publishes a
  `MissionReport`, accepts only a matching acknowledged `MissionReportAck`
  with the same CRC, and bounds publication attempts while honoring
  cancellation.
- **`banyu_teleop`** publishes MAVROS `ManualControl` messages. In GUIDED,
  ArduSub does not use those RC overrides for motion; the operator takeover
  signal is still used as a mission safety input.

The current mission flow is surface transit, heading alignment, dive, pipeline
acquisition, pipeline traversal with ring repair, surfacing, and report
submission. It is represented by `banyu_mission/bt/banyu_mission.xml`.

## 2. ASCII node graph

```text
                         +-----------------------------+
                         | banyu_bringup launch         |
                         | Gazebo / SITL / MAVROS       |
                         +--------------+--------------+
                                        |
          /front_camera/image_raw      | MAVROS topics
                     +-----------------+------------------+
                     |                                    |
                     v                                    v
       +---------------------------+          +--------------------------+
       | task2_pipeline_detector   |          | vehicle_status_node       |
       | HSV/line + ring candidate  |          | FCU + RC + GPS freshness  |
       +-------------+-------------+          +------------+-------------+
                     |                                     |
       +-------------+-------------+                       |
       |                           |                       v
       v                           v            /banyu/vehicle_status
/banyu/pipeline/observation  /banyu/pipeline/ring_observation
       |                           |                       |
       |                           v                       |
       |                 +----------------------+          |
       |                 | ring_tracker_node    |          |
       |                 | pose association +   |          |
       |                 | ring records         |          |
       |                 +--+---------------+---+          |
       |                    |               |              |
       |                    v               v              |
       |          /banyu/pipeline/ring_state  acknowledge   |
       |                                    service        |
       |                                                   |
       v                                                   v
+--------------------------+                 +-----------------------------+
| pipeline_guidance_node   |                 | banyu_mission               |
| PipelineFollower         |                 | MissionState + BT tree      |
+------------+-------------+                 | action clients + conditions |
             |                               +-----+-----------------------+
             v                                     |
 /banyu/guidance/velocity_intent                   |
             |                                     |
             |       physical action servers      |
             +------ (UNIMPLEMENTED) -------------+
                                                   |
                                  vehicle command bridge
                                  (UNIMPLEMENTED)
                                                   |
                                                   v
                                        MAVROS setpoint_raw/local
                                                   |
                                                   v
                                                ArduSub
```

The report branch is implemented separately from vehicle control:

```text
SendMissionReport client
          |
          v
+----------------------------+
| banyu_communication         |
| uav_report_node             |
| implemented action server  |
+-------------+--------------+
              | MissionReport
              v
     external report transport
       (outside this repository)
              |
              | matching MissionReportAck + CRC
              v
       uav_report_node
              |
              v
       action result/cancel
```

The `por_detection` executable is an alternative camera path publishing
`/banyu/detection`; it is not the detector used by the Task 2 pipeline tree.
`task2_sim_provider.py` publishes Task 2 fixture route, heading, ring state,
and a surface-only start-buoy fix. It does not implement physical vehicle
action servers. `banyu_communication/uav_report_node` is the exception: it is
the implemented transport-neutral `SendMissionReport` action server.

## 3. Package structure

```text
src/BANYU_ROBOTX/
├── banyu_bringup/
│   ├── launch/{main_launch,mission,sitl_mavros,gazebo_main,gazebo_por,teleop}.launch.py
│   └── scripts/{task2_sim_provider,ekf_source_switch,dvl_mavlink_bridge,dvl_health}.py
├── banyu_msgs/
│   ├── msg/              # mission, perception, status, and report messages
│   ├── action/           # ten action contracts
│   └── srv/AcknowledgeRingRepair.srv
├── banyu_mission/
│   ├── bt/banyu_mission.xml
│   ├── include/banyu_mission/{banyu_mission,action_nodes}.hpp
│   └── src/{banyu_mission,main}.cpp
├── banyu_detection/
│   ├── src/{task2_pipeline_detector,por_detection,ring_tracker_node,
│   │        ring_tracker,ring_tracker_adapter,pipeline_end_confirmer}.cpp
│   └── include/banyu_detection/*.hpp
├── banyu_guidance/
│   ├── msg/VelocityIntent.msg
│   ├── include/banyu_guidance/pipeline_follower.hpp
│   └── src/{pipeline_follower,pipeline_guidance_node}.cpp
├── banyu_vehicle_interface/
│   ├── include/banyu_vehicle_interface/vehicle_status_tracker.hpp
│   └── src/{vehicle_status_tracker,vehicle_status_node}.cpp
├── banyu_communication/
│   ├── include/banyu_communication/report_state_machine.hpp
│   ├── src/{report_state_machine,uav_report_node}.cpp
│   └── test/test_report_state_machine.cpp
└── banyu_teleop/
    ├── include/banyu_teleop/banyu_teleop.hpp
    └── src/banyu_teleop.cpp
```

## 4. Modular BT XML

The source of truth is `banyu_mission/bt/banyu_mission.xml`. `Main` uses a
reactive interrupt branch and a plain memory `Sequence` for the mission
phases. The named subtrees are:

```xml
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <ReactiveFallback name="mission_root">
      <Sequence name="interrupt_sequence">
        <AutonomyInterrupted/>
        <HandleAbortOrTeleop/>
      </Sequence>
      <Sequence name="mission_phases">
        <SubTree ID="MissionPreflight"/>
        <SubTree ID="SurfaceTransitToStart"/>
        <SubTree ID="DiveAndAcquirePipeline"/>
        <SubTree ID="TraversePipeline"/>
        <SubTree ID="SurfaceAndReport"/>
      </Sequence>
    </ReactiveFallback>
  </BehaviorTree>

  <BehaviorTree ID="MissionPreflight">
    <Sequence>
      <RequireVehicleReady/>
      <RequireGpsAvailable/>
      <RequireMissionStarted/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="SurfaceTransitToStart">
    <Sequence>
      <NavigateSurfaceGps target_latitude="{target_latitude}" target_longitude="{target_longitude}"
        start_latitude="{start_latitude}" start_longitude="{start_longitude}"
        tolerance="{gps_tolerance}"/>
      <AlignHeading target_heading="{heading}" tolerance="{heading_tolerance}"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="DiveAndAcquirePipeline">
    <Sequence>
      <DiveToDepth target_depth="{inspection_depth}" target_heading="{heading}"
        depth_tolerance="{depth_tolerance}" heading_tolerance="{heading_tolerance}"/>
      <AcquirePipeline camera_name="{camera_name}" use_heading_prior="true"
        heading_prior="{heading_prior}" heading_tolerance="{heading_tolerance}"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="TraversePipeline">
    <WhileDoElse>
      <PipelineActive/>
      <ReactiveFallback name="traverse_reactive">
        <Sequence name="repair_red_ring">
          <RedRingPending/>
          <SubTree ID="RepairRedRing"/>
          <AcknowledgeRedRing/>
        </Sequence>
        <Sequence name="reacquire_lost_pipeline">
          <PipelineLost/>
          <SubTree ID="ReacquirePipeline"/>
        </Sequence>
        <FollowPipeline enable="true"/>
      </ReactiveFallback>
      <PipelineEnded/>
    </WhileDoElse>
  </BehaviorTree>

  <BehaviorTree ID="RepairRedRing">
    <Sequence>
      <CenterOnRing ring_color="red" tolerance="{ring_tolerance}"/>
      <HoldProbeOnRing hold_duration="{hold_duration}" alignment_tolerance="{ring_tolerance}"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="ReacquirePipeline">
    <Sequence>
      <SearchPipeline camera_name="{camera_name}" use_heading_prior="true"
        heading_prior="{heading_prior}" search_span="{search_span}" sweep_rate="{sweep_rate}"/>
      <AcquirePipeline camera_name="{camera_name}" use_heading_prior="false"
        heading_prior="{heading_prior}" heading_tolerance="{heading_tolerance}"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="SurfaceAndReport">
    <Sequence>
      <Surface target_depth="{surface_depth}" depth_tolerance="{surface_tolerance}"/>
      <SendMissionReport mission_id="{mission_id}" ring_order="{ring_order}"
        crc32="{crc32}" max_retries="{max_retries}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

`ReactiveSequence` must not replace the phase `Sequence`: BT 4.9 stateful
actions need the running child to retain state between ticks. The root
`ReactiveFallback` supplies interrupt reactivity without restarting a running
phase action.

## 5. Blackboard ports

`BanyuMission::initialize()` populates these blackboard values from parameters
before creating `Main`:

| Value | Type | XML consumer |
|---|---|---|
| `target_latitude`, `target_longitude` | `double` | `NavigateSurfaceGps` |
| `start_latitude`, `start_longitude` | `double` | `NavigateSurfaceGps` |
| `gps_tolerance` | `double` | `NavigateSurfaceGps` |
| `heading`, `heading_prior` | `double` | heading, dive, acquire, search actions |
| `heading_tolerance` | `double` | heading, dive, acquire |
| `inspection_depth`, `depth_tolerance` | `double` | `DiveToDepth` |
| `camera_name` | `string` | acquire/search |
| `search_span`, `sweep_rate` | `double` | `SearchPipeline` |
| `ring_tolerance`, `hold_duration` | `double` | ring center/hold |
| `surface_depth`, `surface_tolerance` | `double` | `Surface` |
| `mission_id` | `string` | `SendMissionReport` |
| `ring_order` | `vector<uint8_t>` | `SendMissionReport` |
| `crc32` | `uint32_t` | `SendMissionReport` |
| `max_retries` | `uint8_t` | `SendMissionReport` |

The corresponding node input ports are declared in
`banyu_mission/include/banyu_mission/action_nodes.hpp`. Conditions and
`AcknowledgeRedRing` expose no ports. Runtime sensor data is **not** passed via
the blackboard: `MissionState` stores it behind a mutex and conditions read
its snapshot.

## 6. Message, service, and action interfaces

### Topics and messages

| Interface | Default topic / role |
|---|---|
| `banyu_msgs/msg/VehicleStatus` | `/banyu/vehicle_status`; FCU, RC, authority, and GPS freshness |
| `banyu_msgs/msg/PipelineObservation` | `/banyu/pipeline/observation`; `INVALID=0`, `NO_PIPE=1`, `TRACKED=2`, normalized offsets, heading error, confidence |
| `banyu_msgs/msg/PipelineRingObservation` | `/banyu/pipeline/ring_observation`; visible/color/offset/confidence |
| `banyu_msgs/msg/PipelineRingState` | `/banyu/pipeline/ring_state`; accepted ring `index`, color, confidence |
| `banyu_msgs/msg/Task2RingRecord` | `/banyu/pipeline/accepted_ring`; frame, color, confidence, map pose |
| `banyu_msgs/msg/Detection` | `/banyu/detection`; `object_id` 0 gate, 1 yellow buoy, offsets, apparent size, distance, confidence |
| `banyu_msgs/msg/HydrophoneHeading` | fixture heading contract: relative heading, confidence, valid |
| `banyu_msgs/msg/MissionReport` / `MissionReportAck` | transport-neutral report boundary; `uav_report_node` publishes the report and consumes matching acknowledgements |
| `banyu_msgs/msg/Task2Report` | complete Task 2 result record, including rings and surface/map evidence |
| `banyu_guidance/msg/VelocityIntent` | `/banyu/guidance/velocity_intent`; bounded body FLU surge/sway/yaw intent |

`PipelineObservation` and `PipelineRingObservation` from
`task2_pipeline_detector.cpp` share the source image header and strictly
increasing `frame_sequence`. For `NO_PIPE` or `INVALID`, pipeline control
fields are NaN and must not be consumed. Image +x is right and +y is down.

`banyu_mission` provides `/banyu/mission/start` and
`/banyu/mission/abort`, both `std_srvs/srv/Trigger`. The ring tracker provides
`/banyu/pipeline/acknowledge_ring_repair` using
`banyu_msgs/srv/AcknowledgeRingRepair`:

```text
Request:  uint32 ring_index
Response: bool success, string reason, uint32 accepted_ring_index
```

### Actions

All actions use ROS order `Goal --- Result --- Feedback`, and all terminal
results begin with `bool success` and `string reason`.

| Action | Goal | Result | Feedback |
|---|---|---|---|
| `NavigateSurfaceGps` | target/start lat/lon degrees, `tolerance_m` | final lat/lon, `final_error_m` | stamp, current lat/lon, remaining distance |
| `AlignHeading` | `target_heading_rad`, `tolerance_rad` | final heading/error | stamp, current heading/error |
| `DiveToDepth` | depth/heading targets and tolerances | final depth/heading | stamp, current depth/heading and errors |
| `AcquirePipeline` | camera, heading-prior flag/value/tolerance | `acquired`, pipeline ID/heading | stamp, visibility/confidence/distance/heading error |
| `SearchPipeline` | camera, optional heading prior, span, sweep rate | `found`, pipeline ID/heading | stamp, heading, sweep progress, visibility |
| `FollowPipeline` | `enable` | `enabled`, final status | stamp, following/status/lateral and distance errors |
| `CenterOnRing` | ring color, tolerance | color and final x/y offsets | stamp, visibility, offsets, alignment error |
| `HoldProbeOnRing` | hold duration and alignment tolerance | held duration and maximum error | stamp, elapsed/alignment error/aligned |
| `Surface` | target depth and tolerance | final depth/error | stamp, current depth/error |
| `SendMissionReport` | mission ID, `uint8[] ring_order`, CRC, `max_retries` after the initial publication, report time | acknowledgement, total publication `attempts`, acknowledged CRC | stamp, attempt, acknowledgement, status |

The mission launch configures action clients under `/banyu/actions/<suffix>`;
the report client is `/banyu/actions/send_mission_report`. The
`mission.launch.py` instance of `banyu_communication` serves that report action and publishes
`MissionReport` on `/banyu/uav/mission_report`, consuming
`MissionReportAck` from `/banyu/uav/mission_report_ack`. When run standalone,
`uav_report_node` defaults to relative names `send_mission_report`,
`mission_report`, and `mission_report_ack`, `max_attempts=4`, and a
`retry_period_ms=1000` timer.

`max_retries` in the action goal means retries after the initial publication:
`N` requests at most `min(255, N + 1)` total publications. The communication
server's `max_attempts` parameter is a hard cap on total publications and is
the bringup-side total derived from that formula. The `255` goal value is the
saturation case and requests 255 total publications rather than an
unrepresentable 256. In `mission.launch.py`, the Task 2 default is
`task2_report_max_retries=3`; bringup derives
`task2_report_max_attempts=min(255, 3 + 1)=4` for `uav_report_node`, with a
`1000 ms` retry period. The action and mission `max_retries` contract always
means retries after the initial publication.

`SendMissionReport` is therefore an implemented action-server boundary. The
other nine action names remain client contracts only: no physical action
server for navigation, heading, depth, pipeline motion, ring alignment,
probing, or surfacing exists in this repository.

## 7. BT-node mapping

| BT ID | Implementation | Boundary / state used |
|---|---|---|
| `AutonomyInterrupted` | registered simple condition in `banyu_mission.cpp` | `MissionState::autonomyInterrupted()` |
| `RequireVehicleReady` | simple condition | FCU/RC freshness, mode, authority, takeover |
| `RequireGpsAvailable` | simple condition | valid and fresh GPS in `MissionState` |
| `RequireMissionStarted` | simple condition | mission lifecycle/start flag |
| `PipelineLost` / `PipelineActive` / `PipelineEnded` | simple conditions | pipeline timeout/tracking and end topic |
| `RedRingPending` | simple condition | ring state with red pending record |
| `HandleAbortOrTeleop` | simple action | sets `MissionState::abort_requested_` |
| `AcknowledgeRedRing` | `AcknowledgeRedRingNode` | ROS service request for the pending record index |
| `NavigateSurfaceGps`, `AlignHeading`, `DiveToDepth`, `AcquirePipeline`, `SearchPipeline`, `FollowPipeline`, `CenterOnRing`, `HoldProbeOnRing`, `Surface` | concrete `ActionClientNode<ActionT>` subclasses | ROS 2 action clients; physical servers are not implemented |
| `SendMissionReport` | `SendMissionReportNode` action client | Served by implemented `banyu_communication/uav_report_node`; report transport remains external |

The source test `banyu_mission/test/test_mission_tree.cpp` verifies XML
loading, registration, lifecycle/safety behavior, ring acknowledgement, and
semantic result checks.

## 8. Representative C++ skeleton and code paths

### Manager and two-phase initialization

`banyu_mission/src/main.cpp` creates the node, then calls `initialize()`:

```cpp
auto node = std::make_shared<banyu_mission::BanyuMission>();
node->initialize();
executor.add_node(node);
executor.spin();
```

The constructor creates `MissionState`, declares timing and safety
parameters, and creates the start/abort Trigger services. `initialize()` is
separate because `shared_from_this()` is needed when registering action
clients and is invalid inside the constructor. It validates mission
coordinates/parameters, fills the blackboard, registers nodes, loads the XML,
creates `Main`, starts the Groot2 publisher, and starts the tick thread.

The tick loop only ticks while lifecycle is `Active`. A terminal tree status
halts the tree and marks the state `Succeeded`, `Failed`, or `Interrupted`.
The destructor sets the stop flag, halts the tree, and joins the tick thread.

### Registration and a condition

`registerMissionNodes()` in `banyu_mission/src/banyu_mission.cpp` registers
conditions and constructs each action node with the configured action name:

```cpp
factory.registerSimpleCondition("PipelineLost", [state](BT::TreeNode &) {
  return state->pipelineLost() ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
});

factory.registerNodeType<FollowPipelineNode>(
  "FollowPipeline", node, action_name("follow_pipeline"));
```

The condition callback reads the mutex-protected `MissionState`; it does not
read a blackboard sensor port.

### Action client, result validation, and halt

`ActionClientNode<ActionT>` in `action_nodes.hpp` is a
`BT::StatefulActionNode`. `onStart()` maps BT inputs into a goal, requires an
already available action server, increments a generation, and sends the goal
asynchronously. `onRunning()` returns `RUNNING` until the result callback
records completion. Success requires both an action transport result of
`SUCCEEDED` and semantic result validation.

`onHalted()` marks cancellation for the current generation and calls
`async_cancel_goal` if a handle exists. Generation checks prevent late goal or
result callbacks from changing a newer invocation. A missing server or goal
mapping/submission failure returns `FAILURE`; this is why the unimplemented
physical action-server layer is a hard integration dependency.

### `FollowPipeline`

`FollowPipelineNode::makeGoal()` reads the one input port `enable` and sets
`FollowPipeline::Goal.enable`. Its semantic validation requires
`result.success` and `result.enabled == goal.enable`, not merely a successful
transport. The present node is an action client; it does not call
`PipelineFollower` directly.

### `HoldProbeOnRing`

`HoldProbeOnRingNode` declares `hold_duration` and `alignment_tolerance`, maps
them to `hold_duration_s` and `alignment_tolerance`, and otherwise uses the
generic action-client lifecycle. The future physical server must keep the
probe aligned for the complete duration and return the measured maximum error;
the current repository contains no probe actuator or vehicle command path.

### `uav_report_node`

`banyu_communication/src/uav_report_node.cpp` creates the implemented
`rclcpp_action::Server<banyu_msgs::action::SendMissionReport>`. It rejects a
second active report, validates that `mission_id` and `ring_order` are
non-empty and that the supplied CRC matches
`mission_report_crc32(mission_id, ring_order)`, then publishes the corresponding
`banyu_msgs/msg/MissionReport`. `ReportRetryStateMachine` interprets the goal's
`max_retries` as retries after that initial publication, while the node's
`max_attempts` is the hard cap on total publications. For a goal value `N`,
the requested server total is `min(255, N + 1)`; the node uses a timer to
publish retries until that total cap is reached.

The acknowledgement callback accepts an ack only when it is acknowledged and
both `mission_id` and `crc32` match the active report. It then succeeds the
action with the acknowledged CRC and total publication count. The cancel
callback transitions the retry state to `Canceled`, stores the cancellation
result, and clears the active goal before returning; cancellation therefore
terminalizes before ACK or retry-timer handling can win. A short cancel timer
then marks the action goal canceled after the action framework reports it as
canceling. Exhausted attempts abort with an acknowledgement-timeout result.
This is a transport-neutral report server, not a radio implementation. The
external endpoint that produces `MissionReportAck` is outside this repository.

## 9. Perception, ring, and end architecture

### Pipeline and target perception

`banyu_detection/src/task2_pipeline_detector.cpp` subscribes to
`/front_camera/image_raw`, optionally reads `/camera/camera_info`, builds a
bright/low-saturation pipeline mask, selects a line using connected components
and `cv::fitLine`, and emits normalized look-ahead offset, heading error, and
confidence. A color component associated with that line produces the paired
ring observation. Invalid images still produce an `INVALID` pipeline result
and an invisible ring result with the same frame sequence.

`por_detection.cpp` is a separate HSV contour detector publishing one best
`banyu_msgs/msg/Detection` on `/banyu/detection`; it estimates distance from
known object height and focal length. It is marked WIP by the repository
architecture and has no completed target-classifier/mission integration.

### Ring tracking

`RingTrackerNode` keeps a map-frame pose history from
`/mavros/local_position/pose` and selects a pose close to the observation
timestamp. The `RingTracker` state machine uses `ENTRY`, `TRACKING`, `EXIT`,
and `DEBOUNCE` confirmation windows, rejects sequence regressions/gaps and
malformed observations, requires spatial/path separation before rearming, and
rejects duplicate red records. An accepted ring is published as
`Task2RingRecord` and its `PipelineRingState` is published. A red record sets
`red_pending`; only the acknowledgement service can clear it. Seeing red
again is not proof of repair.

The tracker requires a valid `map` pose with bounded source timestamp skew for
an accepted record. Camera-only updates can advance state but cannot append a
record.

### Pipeline end

`PipelineEndConfirmer` is a reusable library state machine, not a ROS node. It
requires an explicit end candidate, a tracked/finite observation, sufficient
confidence, contiguous frame sequence, and stable offsets/heading for the
configured number of frames. `NO_PIPE`, `INVALID`, sequence gaps, and loss do
not confirm an end. `MissionState` listens to `/banyu/pipeline/end` as a
`std_msgs/msg/Bool`, but this repository has no publisher wiring that
connects `PipelineEndConfirmer` to that topic. That detector-to-end adapter is
therefore still an integration item.

## 10. Guidance, controller, and ArduSub boundary

`PipelineFollower` is a pure C++ component in
`banyu_guidance/src/pipeline_follower.cpp`. For a fresh, valid, tracked
observation above the configured confidence threshold it computes:

- forward surge proportional to confidence and capped by `max_surge_mps`;
- body-left sway as `-sway_gain_mps * offset_x`, capped;
- counter-clockwise body yaw as `-yaw_gain_rad_s * heading_error_rad`, capped.

No-pipeline, invalid, stale, future-skewed, or low-confidence inputs produce
zero motion with an explicit status. `PipelineGuidanceNode` publishes the
result at `/banyu/guidance/velocity_intent` in `base_link` by default. The
message is an intent only; there is currently no authority/arbitration node
and no MAVROS setpoint publisher consuming it.

The intended lower boundary is:

```text
body FLU intent -> authority/arbitration -> ROS ENU MAVROS setpoint
                 -> MAVLink NED conversion in MAVROS -> ArduSub GUIDED
```

The authority/vehicle-command bridge is **unimplemented**. In the current
stack, all physical action servers that would own depth, heading, motion,
ring alignment, probing, and surfacing are also **unimplemented**. The
transport-neutral report action server is implemented separately by
`uav_report_node` and does not issue vehicle commands.
For Phase A, this execution layer remains non-actuating and unlaunched.

When implemented, local MAVROS pose and setpoint fields are ROS ENU:
`x=east`, `y=north`, `z=up`; MAVROS converts the local setpoint to MAVLink
NED internally. Mission NED offsets must therefore be converted explicitly:
east to ROS x, north to ROS y, and down to negative ROS z. ArduSub accepts
`setpoint_raw/local` in GUIDED only, and velocity setpoints must stream
continuously. The output convention with an explicit yaw rate uses mask
`0x5C7`: position, acceleration/force, and yaw angle are ignored, but yaw rate
is not. Every setpoint carries a finite explicit `yaw_rate`, including zero
when no turn or orbit is commanded.

Depth is not altitude. The action contracts use metres positive downward and
define the surface as depth zero. `/mavros/local_position/pose.pose.position.z`
is ENU z relative to the EKF origin and must not be treated as water depth.
The bringup EKF source switch uses `/mavros/global_position/rel_alt`, whose
value is negative below the surface, for depth-band switching. Underwater,
SRC2 uses DVL velocity with no global horizontal position; relative local
pose remains usable but drifts. GPS-dependent navigation and global homing
are surface-only. The `NavigateSurfaceGps` phase must complete before the
dive, and a future `Surface`/GPS action must return to the configured surface
band before relying on GPS.

## 11. Failure handling

- **Preflight:** `MissionState` fails closed until FCU state, RC, and their
  freshness windows are valid, takeover is false, the configured mode is
  `GUIDED`, and optional authority matches. GPS must be valid and fresh for
  the preflight condition.
- **Interruption:** while active, `autonomyInterrupted()` succeeds on an
  abort request or any loss of readiness. The root interrupt branch calls
  `HandleAbortOrTeleop`, which requests abort; the tick loop marks the final
  lifecycle `Interrupted`. The abort service is `/banyu/mission/abort`.
  There is no automatic physical stop command in this repository.
- **Action failures:** unavailable physical servers, invalid goal inputs, goal
  submission failures, non-`SUCCEEDED` transport results, and semantic result
  failures return BT `FAILURE`. The generic client cancels goals on halt and
  ignores stale callbacks by generation. `SendMissionReport` is served by
  `uav_report_node`: malformed reports abort immediately, mismatched acks are
  ignored, matching acks succeed, `max_retries` counts only publications after
  the initial send, the requested total is `min(255, max_retries + 1)`, and
  total publications stop at the server's hard `max_attempts` cap.
  Cancellation terminalizes the retry state before ACK or timer handling, so
  it wins over both. The external report transport is not part of this
  repository.
- **Pipeline loss/end:** `PipelineLost` selects reacquisition; the
  `ReacquirePipeline` subtree searches and acquires again. `PipelineEnded`
  exits traversal. Loss alone cannot satisfy the end confirmer.
- **Ring integrity:** malformed observations, sequence regressions/gaps,
  insufficient entry/exit confirmation, duplicate red, missing timestamped
  map pose, and insufficient rearm distance are ignored. Acknowledgement must
  name the currently accepted red record and succeeds only once.
- **Guidance:** stale, invalid, lost, or low-confidence observations result
  in zero surge, sway, and yaw. This is a computation-level stop only until a
  future authority consumes the intent.
- **GPS/underwater:** no global GPS-dependent behavior should be attempted
  while the vehicle is submerged and SRC2 is active. Underwater local
  position is relative and subject to drift; it is not a replacement for a
  surface fix.

## 12. Incremental test plan

The repository already contains focused tests; the remaining steps should be
added without coupling pure components to a vehicle:

1. **Contracts and build:** build `banyu_msgs` first, then the mission,
   detection, guidance, vehicle-interface, and communication packages. Confirm
   generated action/message field names match this document.
2. **Vehicle status:** run
   `banyu_vehicle_interface/test/test_vehicle_status_tracker.cpp` and add
   tests for stale FCU/RC/GPS, takeover threshold, mode, and authority.
3. **Guidance unit tests:** run
   `banyu_guidance/test/test_pipeline_follower.cpp`; retain bounds, sign,
   freshness, confidence, and explicit-zero tests.
4. **Perception state machines:** run the ring tracker, adapter, QoS, and
   end-confirmer tests under `banyu_detection/test/`. Add an integration test
   proving that no end topic is emitted from loss and that an accepted ring
   has a timestamp-matched map pose.
5. **BT parsing/registration:** run `test_mission_tree.cpp` and the launch
   argument sanity check. Verify all named subtrees create successfully and
   verify the phase chain remains a plain `Sequence`.
6. **Report and action-server tests:** run
   `banyu_communication/test/test_report_state_machine.cpp` for CRC
   validation, matching acknowledgements, `max_retries` as retries after the
   initial publication, `min(255, max_retries + 1)` total-publication
   derivation, the `max_attempts` hard total-publication cap, and cancellation
   winning before ACK/timer handling. Add an action-level test
   for `uav_report_node` covering goal rejection, report publication,
   matching/mismatched ack, exhaustion, and cancellation terminalization.
   Before hardware, implement test-only mock servers for the nine physical
   action names and exercise goal mapping, semantic result failures, late
   callbacks, halt/cancel, and a server disappearing during a running node.
7. **Perception-to-guidance:** replay camera frames and check paired frame
   sequences, normalized signs, stale behavior, and bounded
   `/banyu/guidance/velocity_intent` output.
8. **Vehicle bridge in SITL:** implement the single authority/setpoint bridge,
   then verify one continuous `setpoint_raw/local` publisher, GUIDED gating,
   ENU/NED conversion, `0x5C7` velocity mask, and finite explicit yaw rate.
9. **Surface/depth limits:** test GPS transit while surfaced, source-set
   switching at depth hysteresis thresholds, DVL/local-pose behavior
   underwater, and rejection of GPS navigation outside the surface band.
10. **End-to-end simulation:** start from a clean process set, run the Task 2
    fixture and camera path separately, inject ring repair acknowledgement,
    force pipeline loss and end evidence, surface, and verify the final
    lifecycle/report result. Physical action servers and the vehicle bridge
    must be present before this is called a mission test rather than a tree
    wiring test.
