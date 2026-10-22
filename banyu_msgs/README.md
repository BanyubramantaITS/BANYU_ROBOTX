# banyu_msgs contracts

This package defines the core ROS 2 message, service, and action contracts for the BANYU ROV autonomy stack.

## Action Interfaces

Actions follow the ROS order `Goal --- Result --- Feedback`. Every terminal result begins with `bool success` and `string reason`, followed by action-specific fields. Feedback contains progress updates, including timestamps and distance/offset errors.

- **Proof of Readiness (POR)**:
  - `AcquireGate.action` — Acquire gate in field of view (defaults to dive in place).
  - `AlignGate.action` — Visually center and align vehicle heading and depth to the gate.
  - `PassGate.action` — Traverse through the gate; supports blind push on return leg if visual lock is temporarily lost.
  - `SearchMarker.action` — Search pattern to locate the yellow buoy.
  - `AlignMarker.action` — Visually center on the yellow buoy.
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

- `VehicleStatus.msg` — MAVROS-neutral health summary (`fcu_connected`, `rc_fresh`, `gps_fresh`, `calibrated_depth_m`, `battery_voltage`).
- `ActiveBuoyFix.msg` & `Task2BuoyTarget.msg` — USV pipeline buoy coordinates and locked target navigation state.
- `Detection.msg` — Single best visual detection per frame for POR (`object_id`: 0=gate, 1=yellow buoy; normalized `[-1, 1]` offsets).
- `PipelineObservation.msg` & `PipelineRingObservation.msg` — Line fit, lookahead angles, and detected rings from downward inspection camera.
- `PipelineRingState.msg` — Associated ring tracker state across temporal observations.
- `Task2SurveyEvidence.msg` & `Task2Report.msg` — Survey evidence assembled during pipeline inspection for RoboCommand / GCS reporting.
- `RunLifecycle.msg` — Epoch-tagged autonomy state transitions (`STATE_IDLE`, `STATE_PREPARED`, `STATE_ACTIVE`, `STATE_COMPLETED`).

## Services

- `AcknowledgeRingRepair.srv` — Confirms physical interaction with damaged ring before vehicle resumes pipeline traverse.
- `TriggerPipelineSurvey.srv` — Triggers generation and sealing of the final pipeline survey report.
- `TriggerCommand.srv` — Generic trigger service returning status and message.
- `ResetTask2.srv` — Resets Task 2 state between test runs.
