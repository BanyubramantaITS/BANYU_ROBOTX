# GCS ROS bridge — interface spec for the GCS/OCS team

The UUV publishes and subscribes the RobotX SoS `/system/...` topics (`rx_msgs`, team repo
`RobotX-SoS-communication-ITSN`). RoboCommand speaks MQTT + protobuf to the OCS only. Something on
the GCS has to translate between the two. `robocommand-rx-ui-itsn/report.py` (`GcsReporter`)
handles the MQTT side but has no ROS side; this document specifies the bridge between them.
A working reference implementation is `tools/sos_integration/gcs_ros_bridge.py`.

```text
UUV (ROS domain 40) --DDS Router: /system/* only--> GCS (ROS domain 10) --bridge--> GcsReporter --MQTT--> RoboCommand
```

## Topic mapping

| ROS topic (GCS domain) | Direction | RoboCommand message |
|---|---|---|
| `/system/mission/course` (`Course`, reliable, transient-local) | RC → vehicle | `RxCourse` (retained). `corners` → `boundary`; `uav_geofence` from the declaration |
| `/system/mission/command` (`Command`, reliable) | RC → vehicle | `RxCommand`; `command_seq = RxCommand.seq` (never renumber) |
| `/system/vehicle/uuv/heartbeat` (`Heartbeat`) | vehicle → RC | `RxReport.heartbeat`, resampled at 2 Hz by `GcsReporter` |
| `/system/vehicle/uuv/task2/pipeline_survey` (`PipelineSurvey`, transient-local) | vehicle → RC | `RxReport.pipeline_survey` |
| `/system/vehicle/uuv/task2/delivery` (`ResourceDelivery`, transient-local) | vehicle → RC | `RxReport.resource_delivery` |
| `/system/vehicle/uuv/task4/incident_ack` (`IncidentAck`) | vehicle → RC | `RxReport.incident_ack` |
| `/system/vehicle/uuv/task4/readiness` (`ReadinessReport`) | vehicle → RC | `RxReport.readiness` |
| `/system/mission/status` (`MissionStatus`) | vehicle → GCS | not sent to RoboCommand (GCS display/log only) |

All shared enum values (`RobotState`, `RxTask`, `VehicleType`, `TaskTier`, `Color`,
`PipelineSegmentStatus`, ...) are numerically identical between `rx_msgs` and the protobuf schema,
so enum fields copy across unchanged.

`RxCommand` body → `rx_msgs/Command`:

| Protobuf body | `command_type` | Fields |
|---|---|---|
| `run_start` | `CMD_RUN_START` | `run_id`; `target_vehicle = TYPE_UNKNOWN` (all). Forward only after `GcsReporter` verified `declaration_seq` |
| `assistance_request` | `CMD_TASK4_ASSISTANCE` | `target_vehicle = vehicle_type`, `position` |
| `keep_out_zone` | `CMD_TASK4_KEEP_OUT_ZONE` | `target_vehicle`, `center`, `radius_m` |
| `all_clear` | `CMD_TASK4_ALL_CLEAR` | `target_vehicle` |
| `moving_object_alert` | `CMD_TASK4_MOVING_OBJECT` | one command per `affected_vehicle_types` entry; `position`, `heading_deg`, `speed_mps` |
| `readiness_confirm` | `CMD_TASK4_READINESS_CONFIRM` | `report_seq`; target from `vehicle_id` |

## Required behaviour (found missing in `GcsReporter` during integration testing)

1. **Forward commands.** `GcsReporter._handle_command` only logs `RunStart` and Task 4 commands.
   Add a command callback (or subclass) and publish them on `/system/mission/command`.
2. **Do not replay stale telemetry.** `GcsReporter` keeps sending the last heartbeat at 2 Hz forever.
   The UUV cannot transmit while submerged, so RoboCommand would see a "live" UUV frozen at its last
   surface position. Pause heartbeats for a vehicle after a few seconds without fresh ROS
   heartbeats (reference: `--stale-timeout-s 3`). Whether RoboCommand prefers paused heartbeats or a
   flagged stale state is a **joint decision**; replaying old data as live is the wrong default.
3. **Survive a GCS restart.** A restarted `GcsReporter` restarts `RxReport.seq` at 1 and re-declares,
   violating "per-vehicle seq never decreases within a run". Persist report/request sequences,
   `declaration_seq` and `run_id` (reference: `--state-file`).
4. **Send one-shot reports once.** Survey/delivery topics are transient-local so a late GCS still
   receives them after the UUV surfaces (store-and-forward). The same property re-delivers them to
   a restarted bridge. De-duplicate per run, e.g. by payload hash (the public messages carry no
   report id — a stronger receipt protocol would need a joint `rx_msgs` change).

## Vehicle-side behaviour the GCS can rely on (BANYU UUV, Issue #13)

- Heartbeat at 2 Hz with GPS position, depth, attitude, `STATE_AUTO/MANUAL/KILLED` and
  authoritative `current_task` (`TASK_INFRA_SURVEY_REPAIR` during the run, `TASK_NONE` otherwise).
- Holds position until a valid `CMD_RUN_START`; duplicate starts are ignored without restarting;
  commands for other vehicle types are ignored.
- A new RoboCommand session may restart `RxCommand.seq` at 1: the UUV accepts it after the operator
  re-prepares autonomy. Each preparation opens a new run identity window (the console may also reset
  `run_id` to 1); within one window an already accepted `run_id` is never restarted, and reports stay
  tied to the run epoch.
- Task 4: `IncidentAck` with the original `command_seq` for assistance, keep-out and all-clear; no
  ACK for moving-object alerts; no `ReadinessReport` (Task 4 is not attempted yet).
- `PipelineSurvey` is published once per run, after surfacing, with the active (GREEN) buoy
  position and the four original segment states ordered from the active buoy end.

## Verification done

Local, single machine: UUV stack in Gazebo/ArduSub SITL on domain 40, `/system/*` relay to domain
10, reference bridge + `GcsReporter` (`robocommand-rx-ui-itsn` `18bcd00`), local MQTT broker,
RoboCommand's official test client and protobuf schema. Course, heartbeat (2.000 Hz), RunStart,
duplicate start, Task 4 ACK sequences, task transitions and `PipelineSurvey` were observed on the
RoboCommand side. Not yet done: two-machine test with the real DDS Router, the RoboCommand test
server/Team Village, and the GCS team's own bridge.
