# AUV Mission Architecture — BANYU_ROBOTX

This document describes the current ROS 2 Jazzy implementation in `src/BANYU_ROBOTX`. The stack provides autonomous underwater ROV capabilities for both the **Proof of Readiness (POR)** competition task and **Task 2 (Autonomous Underwater Pipeline Inspection and Ring Repair)**.

---

## 1. Architecture Overview

The autonomy stack is cleanly decoupled into transport-neutral mission coordination, perception, ring tracking, guidance, vehicle execution, vehicle telemetry, and external GCS / RobotX SoS communication:

- **`banyu_bringup`**: Launch entrypoint and configuration provider.
  - `launch/main_launch.launch.py`: Unified simulation bringup composing Gazebo Harmonic, ArduSub SITL, MAVROS, and mission nodes.
  - `launch/mission.launch.py`: Mode-selected autonomy stack (`mode:=por` or `mode:=task2`).
  - `task2_sim_provider.py`: Deterministic Task 2 fixture source launched only when `mode:=task2 use_sim:=True`.
  - Config files: `por_params.yaml`, `task2_params.yaml`, `mavros_config.yaml`, and `banyu_defaults.parm`.
- **`banyu_msgs`**: ROS 2 message, service, and action definitions for mission coordination, perception, tracking, and vehicle status.
- **`banyu_mission`**: BehaviorTree.CPP v4 mission manager.
  - Evaluates `bt/por_timed_mission.xml` (POR default; `por_mission.xml`, `por_hybrid_mission.xml` or `por_vision_assisted_mission.xml` depending on `mission.use_vision*`) or `bt/banyu_mission.xml` (Task 2).
  - Handles mission lifecycle, blackboard management, action clients, and reactive safety fallbacks.
- **`banyu_detection`**:
  - `por_detection`: Classical vision HSV detector for gate and marker buoy detection.
  - `task2_pipeline_detector`: Line-fitting and color-component detector for underwater pipeline and repair rings from the downward camera (`/bottom_camera/image_raw`).
  - `ring_tracker_node`: Temporal ring tracker associating detections with vehicle map poses.
  - `pipeline_end_node`: End-of-pipe confirmation logic.
  - `survey_evidence_node`: Aggregates active buoy fixes and ring detections to assemble complete `Task2SurveyEvidence`.
- **`banyu_guidance`**:
  - `pipeline_guidance_node`: Converts valid `PipelineObservation`s into bounded FLU body-frame `VelocityIntent`s (`banyu_guidance/msg/VelocityIntent.msg`).
- **`banyu_vehicle_interface`**:
  - `vehicle_status_node`: Aggregates MAVROS state, RC, battery, and GPS data into `/banyu/vehicle_status`.
- **`banyu_vehicle_execution`**:
  - `vehicle_execution_node`: Hosts all physical vehicle action servers (`por_action_servers`, `NavigateSurfaceGps`, `AlignHeading`, `DiveToDepth`, `AcquirePipeline`, `SearchPipeline`, `FollowPipeline`, `CenterOnRing`, `HoldProbeOnRing`, `Surface`).
  - Arbitrates motion commands (`GoalArbiter`), monitors watchdog timing (`RequirementWatchdog`), and continuously streams velocity-only setpoints (`0x5C7` ENU) to `/mavros/setpoint_raw/local` and `/banyu/execution/preview_target`.
  - `calibrated_depth_node`: Computes relative depth from surface datum (`datum - rel_alt`).
- **`banyu_communication`**:
  - `uuv_comm_node`: Public RobotX SoS interface bridge speaking `rx_msgs` over domain 40/10 (`/system/...`). Manages run preparation, run lifecycle, 2 Hz heartbeats, and publishes the final pipeline survey.
  - `pipeline_buoys_node`: Consumes USV buoy reports (`rx_msgs/PipelineBuoys`), validates positions, and provides locked target coordinates (`/banyu/task2/buoy_target`) and active buoy fixes (`/banyu/task2/active_buoy_fix`).
- **`banyu_teleop`**:
  - RadioMaster TX16S RC transmitter interface and safety switch handler: processes `/mavros/rc/in` for unified Pixhawk thruster arming and ESP32 kill relay control (`SB`, `kill_channel: 5`), and flight mode selection (`SC`, `mode_channel: 7`). Unmaps `SF`. Also provides land-test mock auto mode with manual stick lockout for safe bench testing. Single source of truth: `banyu_teleop/config/teleop_rcin.yaml`.

---

## 2. System Node and Data Flow

### Task 2 Data Flow

```text
       +-----------------------+              +-------------------------+
       | USV / GCS (Domain 10) |              | Gazebo Sim / SITL       |
       +-----------+-----------+              +------------+------------+
                   |                                       |
                   | rx_msgs (DDS Relay)                   | camera & sensor topics
                   v                                       v
       +-----------------------+              +-------------------------+
       | pipeline_buoys_node   |              | task2_pipeline_detector |
       | validates USV buoys   |              | line fit & ring detect  |
       +-----------+-----------+              +------------+------------+
                   |                                       |
                   | /banyu/task2/buoy_target              | /banyu/pipeline/observation
                   |                                       v
                   |                          +-------------------------+
                   |                          | pipeline_guidance_node  |
                   |                          | computes velocity intent|
                   |                          +------------+------------+
                   v                                       |
       +-----------------------+                           | /banyu/guidance/velocity_intent
       | banyu_mission         |                           v
       | BehaviorTree.CPP v4   |              +-------------------------+
       | action clients & BT   | <----------> | vehicle_execution_node  |
       +-----------+-----------+  actions     | action servers, arbiter |
                   |                          | setpoint stream (0x5C7) |
                   |                          +------------+------------+
                   | trigger survey                        |
                   v                                       v
       +-----------------------+              +-------------------------+
       | survey_evidence_node  |              | MAVROS                  |
       | assembles survey data |              | /mavros/setpoint_raw/   |
       +-----------+-----------+              | local                   |
                   |                                       |
                   v                                       v
       +-----------------------+              +-------------------------+
       | uuv_comm_node         |              | ArduSub Autopilot       |
       | publishes to GCS      |              | GUIDED velocity control |
       +-----------------------+              +-------------------------+
```

---

## 3. Package Structure

```text
src/BANYU_ROBOTX/
├── banyu_bringup/
│   ├── launch/{main_launch,mission,sub_bringup,sitl_mavros,gazebo_main,gazebo_por,teleop}.launch.py
│   ├── config/{por_params,task2_params,hardware_params,mavros_config}.yaml
│   └── scripts/{task2_sim_provider,nogps_odom_beacon}.py
├── banyu_msgs/
│   ├── msg/              # VehicleStatus, ActiveBuoyFix, PipelineObservation, Detection, etc.
│   ├── action/           # 18 POR and Task 2 action contracts
│   └── srv/              # AcknowledgeRingRepair, TriggerPipelineSurvey, ResetTask2, TriggerCommand
├── banyu_mission/
│   ├── bt/{banyu_mission,por_mission,por_timed_mission,por_hybrid_mission,por_vision_assisted_mission}.xml  (+ pipeline_survey_tree.xml: legacy Task 2 tree, not loaded)
│   ├── include/banyu_mission/{banyu_mission,mission_state,run_coordinator,action_nodes}.hpp
│   └── src/{banyu_mission,main}.cpp
├── banyu_detection/
│   ├── include/banyu_detection/*.hpp
│   └── src/{task2_pipeline_detector,por_detection,ring_tracker_node,
│            pipeline_end_node,survey_evidence_node}.cpp
├── banyu_guidance/
│   ├── msg/VelocityIntent.msg
│   ├── include/banyu_guidance/pipeline_follower.hpp
│   └── src/{pipeline_follower,pipeline_guidance_node}.cpp
├── banyu_vehicle_interface/
│   ├── include/banyu_vehicle_interface/*.hpp
│   └── src/{vehicle_status_node,vehicle_status_tracker,esp32_bridge_node,esp32_trigger_node}.cpp
├── banyu_vehicle_execution/
│   ├── include/banyu_vehicle_execution/*.hpp
│   ├── src/action_servers/*.cpp
│   └── src/{vehicle_execution_node,command_encoder,goal_arbiter,calibrated_depth_node}.cpp
├── banyu_communication/
│   ├── include/banyu_communication/*.hpp
│   └── src/{uuv_comm_node,pipeline_buoys_node,survey_reporting}.cpp
└── banyu_teleop/
    ├── include/banyu_teleop/banyu_teleop.hpp
    └── src/banyu_teleop.cpp
```

---

## 4. Behavior Tree Flow

The source of truth for mission execution is:
- **`banyu_mission/bt/por_timed_mission.xml`** for POR by default (`por_mission.xml` for the vision variant).
- **`banyu_mission/bt/banyu_mission.xml`** for Task 2.

### Task 2 Behavior Tree (`banyu_mission.xml`)

```xml
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <ReactiveFallback name="mission_root">
      <Sequence name="interrupt_sequence">
        <AutonomyInterrupted />
        <HandleAbortOrTeleop />
      </Sequence>
      <Sequence name="mission_phases">
        <SubTree ID="MissionPreflight" _autoremap="true" />
        <SubTree ID="SurfaceTransitToStart" _autoremap="true" />
        <SubTree ID="DiveAndAcquirePipeline" _autoremap="true" />
        <SubTree ID="TraversePipeline" _autoremap="true" />
        <SubTree ID="SurfaceAndReport" _autoremap="true" />
      </Sequence>
    </ReactiveFallback>
  </BehaviorTree>

  <BehaviorTree ID="MissionPreflight">
    <Sequence>
      <RequireMissionStarted />
      <RequireVehicleReady />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="SurfaceTransitToStart">
    <Sequence>
      <NavigateSurfaceGps target_latitude="{target_latitude}" target_longitude="{target_longitude}"
        start_latitude="{start_latitude}" start_longitude="{start_longitude}"
        tolerance="{gps_tolerance}" />
      <LockBuoyTarget />
      <AlignHeading target_heading="{heading}" tolerance="{heading_tolerance}" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="DiveAndAcquirePipeline">
    <Sequence>
      <DiveToDepth target_depth="{inspection_depth}" target_heading="{heading}"
        depth_tolerance="{depth_tolerance}" heading_tolerance="{heading_tolerance}" />
      <RetryUntilSuccessful num_attempts="3">
        <Fallback>
          <Timeout msec="{acquire_timeout_ms}">
            <AcquirePipeline camera_name="{camera_name}" use_heading_prior="true"
              heading_prior="{heading_prior}" heading_tolerance="{heading_tolerance}" />
          </Timeout>
          <SubTree ID="ReacquirePipeline" _autoremap="true" />
        </Fallback>
      </RetryUntilSuccessful>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="TraversePipeline">
    <Sequence>
      <ForceSuccess>
        <Timeout msec="{traverse_timeout_ms}">
          <KeepRunningUntilFailure>
            <WhileDoElse>
              <PipelineActive />
              <ReactiveFallback name="traverse_reactive">
                <Sequence name="repair_red_ring">
                  <RedRingPending />
                  <SubTree ID="RepairRedRing" _autoremap="true" />
                  <AcknowledgeRedRing />
                </Sequence>
                <Sequence name="reacquire_lost_pipeline">
                  <PipelineLost />
                  <SubTree ID="ReacquirePipeline" _autoremap="true" />
                </Sequence>
                <FollowPipeline enable="true" />
              </ReactiveFallback>
              <AlwaysFailure />
            </WhileDoElse>
          </KeepRunningUntilFailure>
        </Timeout>
      </ForceSuccess>
      <PipelineEnded />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="RepairRedRing">
    <Sequence>
      <CenterOnRing ring_color="red" tolerance="{ring_tolerance}" />
      <HoldProbeOnRing hold_duration="{hold_duration}" alignment_tolerance="{ring_tolerance}" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="ReacquirePipeline">
    <Sequence>
      <SearchPipeline camera_name="{camera_name}" use_heading_prior="true"
        heading_prior="{heading_prior}" search_span="{search_span}" sweep_rate="{sweep_rate}" />
      <Timeout msec="{acquire_timeout_ms}">
        <AcquirePipeline camera_name="{camera_name}" use_heading_prior="false"
          heading_prior="{heading_prior}" heading_tolerance="{heading_tolerance}" />
      </Timeout>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="SurfaceAndReport">
    <Sequence>
      <Timeout msec="5000">
        <WaitSurveyComplete />
      </Timeout>
      <Surface target_depth="{surface_depth}" depth_tolerance="{surface_tolerance}" />
      <ConfirmSurfaceAction />
      <Timeout msec="5000">
        <PublishPipelineSurvey />
      </Timeout>
    </Sequence>
  </BehaviorTree>
</root>
```

> [!IMPORTANT]
> **Sequential Execution Rule**: Do **not** use `ReactiveSequence` for the phase chain. BT 4.9 `StatefulActionNode` ticks `onStart()` on the first tick and `onRunning()` on subsequent ticks. A `ReactiveSequence` re-ticks from the first child every tick and halts non-running siblings, causing actions to loop prematurely. Plain memory `Sequence` preserves child state, while abort reactivity is provided by the root `ReactiveFallback`.

---

## 5. Vehicle Execution & Velocity Control

> POR uses `ALT_HOLD` with `control.output_mode: rc_override` (velocity intents become RC stick overrides on `/mavros/rc/override`). The `0x5C7` setpoint contract below applies to Task 2 (`GUIDED`).

Navigation is implemented as a **velocity-loop** controller, not a position-setpoint controller. ArduSub's position-target mode (`0xDF8`) routes through AC_WPNav, which drifts in depth and crawls in simulation.

### Velocity Control Contract (`0x5C7`)
`banyu_vehicle_execution/vehicle_execution_node` streams continuous velocity setpoints to `/mavros/setpoint_raw/local`:
- **Type Mask**: `0x5C7` (ignoring position, acceleration, and yaw angle; yaw rate is actively commanded).
- **Coordinate Frame**: `FRAME_LOCAL_NED` (MAVLink frame enum; MAVROS converts ROS ENU velocity components internally).
- **Explicit Zero Yaw Rate**: Every setpoint must provide an explicit finite `yaw_rate = 0.0` when no turn is commanded, preventing ArduSub from persisting previous slew rates.
- **Continuous Streaming**: Setpoints must stream at 10–50 Hz; dropping setpoints drops ArduSub into failsafe.

---

## 6. Public Communication Contract (RobotX SoS)

Communication with the external Ground Control Station (GCS) and RoboCommand uses the official `rx_msgs` package:
- **Domains**: UUV operates on ROS Domain 40; GCS on Domain 10.
- **Relay**: Only topics under `/system/...` cross the DDS router boundary.
- **Run Lifecycle**:
  1. Operator prepares autonomy via `/banyu/mission/prepare_autonomy`. This opens a fresh epoch window and resets sequence expectations.
  2. RoboCommand sends `CMD_RUN_START` via `/system/mission/command`.
  3. `uuv_comm_node` authorizes the run, transitions `RunLifecycle` to `ACTIVE`, and unlocks mission execution.
  4. On mission completion and surface ascent, `survey_evidence_node` triggers survey assembly and `uuv_comm_node` publishes the sealed `PipelineSurvey` report.
