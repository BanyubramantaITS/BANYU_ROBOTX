# Task 4 — Dynamic Incident Response (Advanced + Disruptive) for the UUV

Handbook §3.3.5 and §3.4.12, scoring "Dynamic Incident Response". **A UUV on the surface is treated as a USV.** The zones
are virtual; entering one at any time of the run is -25 and no Task 4 points. Underwater the UUV is outside the surface
domain, so only the time on the surface is constrained. Core tier (`AssistanceRequest`) is unchanged: acknowledged, not
executed.

- **Advanced:** a static keep-out zone (center + radius) until `AllClear`; acknowledge both.
- **Disruptive:** a moving surface object (position, heading, speed, radius defined by RoboCommand, handbook update
  2026-10-23); avoid its current **and projected** travel.

## Data flow

```
RoboCommand ─ gcs_ros_bridge ─ /system/mission/command (rx_msgs/Command)
   └─ uuv_comm_node  Task4ZoneTracker ── /system/vehicle/uuv/task4/incident_ack   (ack policy below)
        └─ /banyu/task4/zones (banyu_msgs/Task4Zones, 1 Hz + on change, transient_local, recorded in the bag)
             ├─ vehicle_execution_node: NavigateSurfaceGps, AlignHeading, Surface, idle guard
             └─ banyu_mission: KeepClearOfZones (end of banyu_mission.xml)
```

- Proto (robonation/robocommand `rx_commands.proto`): `KeepOutZone.radius_m`, `MovingObjectAlert.keep_out_radius_m` (5 m default). The bridge maps the latter to `Command.radius_m`. The proto does not say how `heading_deg` is measured; it is assumed compass like the heartbeat heading (confirm with RoboCommand).
- `Command.heading_deg` is a compass heading (clockwise from north); the tracker publishes ENU radians.
- Zones are kept per run: cleared when the lifecycle enters PREPARING (`prepare_autonomy`, which opens a new run
  identity window; RoboCommand may restart `command_seq` at 1). Duplicates are ignored by `(command_seq, command_type)`
  (the UAV does the same). The bridge sends one `Command` per addressed vehicle type with the same seq, so a zone sent to
  both USV and UUV is one zone, acknowledged once for the UUV.
- Static zones accumulate (`task4.max_static_zones`); a zone re-sent with the same center updates its radius.
  `ALL_CLEAR` clears the static zones. The moving object has no all-clear: the latest alert replaces the previous one (an
  older seq arriving late is ignored) and it is dropped `task4.moving_expiry_s` after the last update.
- Each change is logged (`Task 4 keep_out_zone (seq N, target T)`, `Task 4 zones (all_clear): ...`).

## Ack policy

| Command | Addressed to UUV / all | Addressed to USV |
|---|---|---|
| KEEP_OUT_ZONE, ALL_CLEAR | `IncidentAck(command_seq)` immediately, zone applied | no ack (the USV is the "correct system"), zone **still avoided** |
| ASSISTANCE (Core) | ack, "unsupported/not ready" status | ignored |
| MOVING_OBJECT | **no ack** by default; `task4.ack_moving_object: true` acks | no ack, still avoided |

An invalid zone (no radius, radius > `task4.max_radius_m`, bad position) is rejected with a warning but still acked.

**Open question for the team briefing (protocol vs scoring):** the protocol says a moving-object alert has no acknowledgement
and no all-clear, but the scoring awards 25 for acknowledging receipt. `IncidentAck` carries only a `command_seq`, so it can
carry the ack through the bridge (`on_incident_ack` → `RxReport.incident_ack`), but the GCS may reject an ack for a seq
that is not an ack-able command. Default is off; set `task4.ack_moving_object: true` in `task2_params.yaml` once agreed.
The GCS bridge also needs the `radius_m` fix (`tools/sos_integration/gcs_ros_bridge.py` `on_command`, below).

## Avoidance (velocity loop only, `zones.*` of `vehicle_execution_node`)

Geometry (`task4_zones.hpp`): a static zone is a circle, a moving object a capsule from its predicted position now to where
it will be after `zones.horizon_s` (so both current and projected travel are kept clear of); every radius grows by
`zones.margin_m` (GPS error + vehicle size). The vehicle position is its GPS fix.

| Surface phase | Behaviour |
|---|---|
| `WaitForBuoyTarget` standby (no goal running) | idle guard: inside an inflated zone → leave by the shortest way (`zones.exit_speed_mps`, to `zones.exit_clearance_m` outside), then hold outside |
| `NavigateSurfaceGps` (transit to the dive point, second-dive transit) | inside a zone → leave first, never hold inside; otherwise detour around zones on the path (shortest way round the ends/flanks), re-checked every tick, together with the buoy keep-out; no way round → hold |
| dive point inside a **static** zone | shifted along the buoy line to the nearest point outside (still `zones.dive_point_buoy_clear_m` from both buoys), else the nearest point outside anywhere |
| dive point inside a **moving** zone | waits outside, at the zone's edge, until it has passed |
| `AlignHeading` | a zone reaching the vehicle → move out (still turning), then hold the new position outside |
| `Surface` before the final ascent and before the second-dive surfacing (`hold_for_zones`) | the surfacing point is estimated by the last GPS fix (there is no underwater position): inside a zone → hold at depth until clear, for at most `zones.surface_hold_timeout_s` in all; then ascend anyway and exit at once (the report must not be lost) |
| after the report | BT `KeepClearOfZones`: while a zone is active the mission stays running (vehicle armed, idle guard moves it out and keeps it out); ends `mission.zones.release_s` after the last zone is gone, or `mission.zones.max_hold_s`. No zone → immediate (unchanged) |
| abort / teleop surfacing | not delayed: ascend (safety first). The idle guard exits the zone only while the vehicle is still under mission control; with teleop active the operator drives |

Limits to know: a zone that appears *after* the mission has ended (vehicle disarmed, MANUAL) is not avoided; the surfacing
point is the dive point, not the real position after the survey; GPS is only valid on the surface, so nothing is avoided
underwater (by design).

## Parameters

`task2_params.yaml` (also the right section for overlays):

- `vehicle_execution_node.zones`: `margin_m` 2.0, `horizon_s` 30, `exit_clearance_m` 0.5, `exit_speed_mps` 0.6,
  `idle_guard` true, `surface_depth_m` 0.5, `dive_point_buoy_clear_m` 1.5 (= `mission.buoy_standoff_m`),
  `surface_hold_timeout_s` 60. Topic `topics.task4_zones`.
- `uuv_comm_node.task4`: `max_static_zones`, `max_radius_m`, `moving_expiry_s`, `ack_moving_object`.
- `banyu_mission.mission.zones`: `release_s`, `max_hold_s`.
- Default run: `launch_task2.sh` with `sensors:=full`, `dive_point:=midpoint` (now the default for every strategy), the
  full survey tree and the light-cue search.

## How to test

Unit: `build/banyu_vehicle_execution/test_task4_zones` (geometry, `NavigateSurfaceGps` avoiding a zone, idle guard),
`build/banyu_communication/test_task4_zones` and the `Task4*` cases of `test_uuv_comm_node` (lifecycle, ALL_CLEAR, run
reset, duplicates, USV vs UUV targeting and acks). Run them with `ROS_DOMAIN_ID=42`, one package at a time.

Sim (`tools/scripts/run/task4_command.py`, UUV domain):

```
# terminal 1: MAVProxy + MAVROS as usual, then the Task 2 sim (midpoint is the default)
./tools/scripts/run/launch_task2.sh --sim --record
# terminal 2: operator start as usual (prepare_autonomy, run start via task2_test_waypoint.py or the GCS)
# static zone across the transit line: 12 m ahead of the UUV (north/east to match the buoy line), radius 8 m
ros2 topic echo /banyu/task4/zones &
./tools/scripts/run/task4_command.py zone --north 12 --east 0 --radius 8
# expect: the vehicle swings round it (preview_target velocity y), "Task 4: dive point inside ..." if it covers the dive point
./tools/scripts/run/task4_command.py all-clear      # the transit resumes straight
# moving object crossing the midpoint while AlignHeading runs: starts 30 m north of the UUV, heads south at 0.8 m/s
./tools/scripts/run/task4_command.py moving-sim --north 30 --east 0 --heading 180 --speed 0.8 --radius 6
# expect: "Task 4: inside a keep-out zone during AlignHeading, leaving" as the sweep reaches the vehicle, the dive
# after it has passed; /banyu/task4/zones and the acks are in the bag
```
