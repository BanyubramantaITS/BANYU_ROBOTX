# banyu_msgs contracts

This package defines the core ROS 2 message, service, and action contracts for the BANYU ROV autonomy stack.

## Action Interfaces

Actions follow the ROS order `Goal --- Result --- Feedback`. Every terminal result begins with `bool success` and `string reason`, followed by action-specific fields. Feedback contains progress updates, including timestamps and distance/offset errors.

- **Proof of Readiness (POR)**:
  - `AcquireGate.action` — Acquire gate in field of view (defaults to dive in place).
  - `AlignGate.action` — Visually center and align vehicle heading and depth to the gate.
  - `PassGate.action` — Traverse through the gate; supports blind push on return leg if visual lock is temporarily lost.
  - `SearchMarker.action` — Search pattern to locate the marker (orange in the per-object HSV profiles).
  - `AlignMarker.action` — Visually center on the marker.
  - `OrbitMarker.action` — Clockwise/counter-clockwise orbit around marker applying feedforward yaw rate ($\omega_{\text{ff}} = -y / r$).
  - `TurnAround.action` — 180° in-place yaw maneuver.
  - `SearchGate.action` — Search pattern to re-acquire gate on return leg.
  - `TimedMove.action` — Open-loop / sensor-assisted timed leg execution with depth and heading hold for timed and dead-thruster POR strategies.
  - `Surface.action` — Ascent to surface with surfaced-depth verification.

- **Task 2 (Pipeline & Rings)**:
  - `NavigateSurfaceGps.action` — GPS waypoint navigation at surface with retargeting support.
  - `AlignHeading.action` — In-place heading alignment to target pipeline orientation.
  - `DiveToDepth.action` — Vertical descent to target inspection depth.
  - `AcquirePipeline.action` — Visual detection and initial lock on pipeline structure.
  - `SearchPipeline.action` — Local search sweep when pipeline tracking is lost.
  - `FollowPipeline.action` — Autonomous pipeline tracking guided by velocity intent.
  - `CenterOnRing.action` — Visual centering on detected repair ring.
  - `HoldProbeOnRing.action` — Station keeping while magnetic probe interacts with repair ring.
  - `Surface.action` — Ascent to surface upon mission completion.

## Key Messages

- `VehicleStatus.msg` — MAVROS-neutral health summary (`fcu_connected`, `armed`, `fcu_status_fresh`, `rc_fresh`, `operator_takeover`, `mode`, `authority`, `gps_valid`, `gps_fresh`, `operational_mode` 0=DISARMED/1=MANUAL/2=AUTONOMOUS, `hardware_kill_active`). Calibrated depth is published separately by `calibrated_depth_node`.
- `ActiveBuoyFix.msg` & `Task2BuoyTarget.msg` — USV pipeline buoy coordinates and locked target navigation state.
- `Detection.msg` — Single best visual detection per frame for POR (`object_id`: 0=gate, 1=marker buoy, 2/3=green/red blob for Task 2; normalized `[-1, 1]` offsets).
- `PipelineObservation.msg` & `PipelineRingObservation.msg` — Line fit, lookahead angles, and detected rings from downward inspection camera.
- `PipelineRingState.msg` — Associated ring tracker state across temporal observations.
- `Task2SurveyEvidence.msg` & `Task2Report.msg` — Survey evidence assembled during pipeline inspection for RoboCommand / GCS reporting.
- `ExecutionReadiness.msg` — `vehicle_execution_node` readiness for the current run (`/banyu/execution/readiness`), consumed by the mission before it starts motion.
- `WaterDepth.msg` — Run-tagged calibrated depth from `calibrated_depth_node` (`/banyu/vehicle/water_depth`; `valid`, `calibrated`).
- `PipelineEndCandidate.msg` & `PipelineEndState.msg` — End-of-pipe candidate (sim provider / detector) and the confirmed end state from `pipeline_end_node`.
- `PingerObservation.msg` — Horizontal bearing to the active buoy's pinger from `pinger_localization` (`/banyu/pinger/observation`), used by `SearchPipeline`.
- `HydrophoneHeading.msg` — Relative heading to the pinger. Published only by `task2_sim_provider.py`; no node subscribes to it (superseded by `PingerObservation`).
- `Task2RingRecord.msg` & `Task2SurveyProgress.msg` — Per-ring entry in the survey evidence and the run-tagged survey progress (`/banyu/communication/survey_progress`).
- `RunLifecycle.msg` — Epoch-tagged autonomy state transitions (phases `IDLE`, `PREPARING`, `READY`, `INITIALIZING`, `ACTIVE`, `SURVEY_COMPLETE`, `SURFACED`, `SUCCEEDED`, `FAILED`, `INTERRUPTED`; events `SNAPSHOT`, `START_ACCEPTED`, `START_DUPLICATE`, `START_REJECTED`, `TERMINAL`).

## Services

- `AcknowledgeRingRepair.srv` — Confirms physical interaction with damaged ring before vehicle resumes pipeline traverse.
- `TriggerPipelineSurvey.srv` — Triggers generation and sealing of the final pipeline survey report.
- `TriggerCommand.srv` — Generic trigger service returning status and message.
- `ResetTask2.srv` — Resets Task 2 state between test runs.
