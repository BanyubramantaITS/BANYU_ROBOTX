# Task 2 simulation plan: Core → Advanced → Disruptive, minimal sensors

Status: **plan, nothing implemented yet** (October 2026). Related: `task2_underwater_position.md`,
`gcs_ros_bridge_spec.md`, `issue13.md`, `tools/sos_integration/README.md`.

## 1. Goal

With only the sensors below, the UUV must: transit to the active buoy, dive, find and follow the
pipeline, record every ring colour in order from the active buoy, repair the red ring, (Advanced and
Disruptive) read the flash code, then surface and report to RoboCommand through the GCS. The GCS relays
the resource request to the UAV. The end result is a **feasibility verdict per tier**: success rate,
the step where it fails, and why.

Sensors: Pixhawk IMU/compass, front camera, bottom camera, Bar30 depth, RTK GPS (USB to the UUV mini
PC, bridged to the Pixhawk as `GPS_INPUT`, valid **only at the surface**), hydrophone (undecided).
No DVL, no altimeter.

Stages: Core → Advanced → Disruptive. A tier is "done" when its test cases (section 8) pass.

## 2. What the documents say (constraints)

- Buoy colour indicator faces **up** and is meant for the UAV (FAQ, 2 Feb 2026). The UUV and USV are
  meant to use the **pinger**. The front camera at the waterline probably cannot read it.
- Pinger: Benthos ALP-365 or a replica. 25–40 kHz, 0.5 kHz steps, 4 ms pulse, 1 pulse per 0.5 / 1 / 2 s.
  Frequency comes from `rx_msgs/Course.pinger_freq_hz`, and may change daily. Only the green buoy pings.
  Other courses' pingers are ≥ 2 kHz apart.
- The UUV may surface for a GPS fix and to talk to the USV/UAV while staying autonomous (FAQ, 1 Oct 2026).
- Pipeline has 4 LED-ring nodes, 1–2 m above the seabed, depth and slope can vary. Buoys are only
  "near" the pipeline ends.
- Disruptive code, after repair: red → green for 5 s, then a two-colour code for 30 s: colour 1 (tin)
  1 s, off 1 s, colour 2 (circle) 1 s, black 2 s. Cycle is 5 s.
- Scoring (TeamTime5, 15 Oct 2026): Core 450 = heartbeat 25, correct start point 25, light order 150,
  repair 250. Advanced 700, Disruptive 1050. UAV receipt of tasking is judged from the UAV heartbeat.
- No run-time limit found in the 23 Oct 2026 handbook. The budget below is our own assumption.

## 3. What already exists

- Core flow: `bt/banyu_mission.xml` (surface transit, dive, acquire, follow, repair red ring, surface).
- `uuv_comm_node` + `SurveyLedger` accept tiers 2–4 and publish `PipelineSurvey` and `ResourceDelivery`.
- Sim lights: `PipelineLights.cc` and `robotx_light_controller.py` generate the Disruptive code
  (`main_launch.launch.py` has a `core | advanced | disruptive` option, colours can be set).
- `pipeline_buoys_node` takes the USV `PipelineBuoys` (including `active_buoy`).
- On `feat/dive-trajectory-path`: `underwater_odometry_node` (dead reckoning from commanded velocity,
  pinger bearing corrections).
- `tools/sos_integration/`: `gcs_ros_bridge.py`, `dds_relay.py`, `mock_usv.py`, `rc_observer.py`.

## 4. Gaps

| # | Gap |
|---|---|
| G1 | Sim hides GPS loss: SITL GPS keeps working underwater, so Task 2 passes in sim but not on the robot |
| G2 | GUIDED needs `position_ok()`; below the surface there is no GPS and no DVL |
| G3 | No flash-code decoder (nothing sets `resource_code_complete`, `resource_color`, `delivery_circle_color`) |
| G4 | Ring detector classifies only red and green, not blue |
| G5 | BT has no hold-and-decode step after repair (the code lasts only 30 s) |
| G6 | `task2_tier` is 2 in `mission` and `uuv_comm_node.survey` |
| G7 | Hydrophone gives azimuth only (no range/elevation), pulse detector must follow Course frequency and PRI |
| G8 | No mock GCS that sends the 4 team commands and relays the resource request to a UAV |

## 5. Architecture decisions

**D1. Control mode.** Surface transit in GUIDED (GPS). After the dive, switch to a mode that does not need
a position estimate. Candidate: **ALT_HOLD + RC override** (same as POR). Decided by experiment (section 6).

**D2. Underwater position is a coarse prior only.** Dead reckoning (`underwater_odometry_node`) says where to
search. Pipeline and rings are tracked by vision.

**D3. Surface-to-fix as drift reset.** If the pipeline is not acquired within a time budget or the drift
estimate is too large, surface, take a GPS fix, and re-dive with a corrected offset.

**D4. Heading from several sources.** Log gyro yaw and compass, fuse in the EKF, and use the pipeline direction
in the bottom camera and the `buoy_a`→`buoy_b` bearing as extra references. No single heading sensor is trusted.

**D5. Find the active buoy, in this order:**
1. `PipelineBuoys.active_buoy` (USV/UAV), when not `ACTIVE_UNKNOWN`.
2. Hydrophone: listen at each buoy for ≥ 4 s (two longest PRI), choose the buoy with valid pulses.
   Pulse detector: narrow bandpass at `pinger_freq_hz`, 4 ms gate, PRI from parameters.
3. Front camera, only if a pool test shows it can read the indicator.
4. Visit both buoys in GPS order and compare.

## 6. GPS-loss simulation and Pixhawk mode experiments

**`sim_gps_gate` (sim only).** When `calibrated_depth` exceeds `gps_loss_depth_m` (default 0.1 m, worst
case), stop the GPS feed to the Pixhawk (`SIM_GPS1_ENABLE 0`, or stop the `GPS_INPUT` publisher). Re-enable
it on surfacing after a reacquire delay (default 10 s, also run with 30 s). Equivalent on the robot: the u-blox
bridge on the mini PC stops delivering fixes, so the Pixhawk sees `GPS_INPUT` end.

**Mode experiments (record, per mode, 60 s each):** does the mode hold or get cancelled, `STATUSTEXT` and
`/rosout` messages, EKF status, depth and heading drift, and whether the transition back to GUIDED at the
surface is clean.

| # | Mode | Purpose |
|---|---|---|
| M1 | GUIDED, velocity setpoints | Confirm it is cancelled or drifts when GPS drops |
| M2 | GUIDED + ExternalNav beacon | Confirm it is unsafe (zero-displacement beacon), do not ship |
| M3 | ALT_HOLD + RC override | Main candidate |
| M4 | STABILIZE + RC override | Depth is not held, needs a Bar30 loop |
| M5 | MANUAL + RC override | Emergency only |
| M6 | POSHOLD | Confirm it needs position |

Also check `FS_EKF_ACTION`, `EK3_SRC*`, `FS_GCS_ENABLE` and `ARMING_CHECK` in `banyu_defaults.parm`.
Results go in `docs/task2_mode_experiments.md`.

## 7. Mission flow and work items

1. Preflight: wait for `CMD_RUN_START`, hold position, declare the tier.
2. Surface transit (GPS) to the active buoy. Find the active buoy as in D5.
3. Dive, switch to the underwater mode (D1).
4. Search and acquire the pipeline, with prior heading and a sweep box. Use apparent pipe width in the bottom
   camera as an altitude proxy, because there is no altimeter.
5. Traverse and follow. Record ring colours in order from the active buoy.
6. Red ring: centre, hold the probe, verify red → green.
7. **Advanced:** after repair, read the one-colour flash (delivery circle), `resource_color = COLOR_ANY (4)`.
8. **Disruptive: new `DecodeResourceCode`.** Hold station over the ring for the full 30 s window. Sample the
   ring region at ≥ 15 Hz. Find each cycle by its long dark gap (2 s versus 1 s). Majority vote over about 6
   cycles for colour 1 (tin, 1–3) and colour 2 (circle). If no valid pair, still send the survey.
9. Finish the pipeline so the ring list is complete.
10. Surface, hold station, check the WiFi link to the GCS (Ubiquiti Bullet AC), then send
    `PipelineSurvey` (+ `ResourceDelivery` for Advanced/Disruptive) and resend every 1 s until acked, up to 60 s.
    Do not dive again until acked or timed out.

Work items:
- W1 `sim_gps_gate` and the mode experiments (section 6).
- W2 Mode switch in the BT and `vehicle_execution`, and `task2_params.yaml` (`control.output_mode`).
- W3 Search prior from `underwater_odometry_node`, and the surface-to-fix BT branch (max 3 attempts).
- W4 Blue ring classifier and a per-frame ring-ROI hue topic (recorded in the bag for offline tuning).
- W5 `ring_code_decoder` node in `banyu_detection` feeding `survey_evidence_node`.
- W6 BT: `DecodeResourceCode`, link check, resend-until-ack.
- W7 Pulse detector parameters in the pinger node (`pinger_freq_hz`, `pulse_repetition_s`, `bandwidth_hz`).
- W8 `task2_tier` 2 → 3 → 4 per stage, in `mission` and `uuv_comm_node.survey`.
- W9 Operator launch script `tools/scripts/launch_task2.sh` kept in sync, per `AGENTS.md`.

Rule from `AGENTS.md`: every new or renamed node parameter goes in `task2_params.yaml`, is passed in
`mission.launch.py`, and the launch script and docs are updated in the same change.

## 8. Mock GCS (`tools/sos_integration/mock_gcs.py`)

Domain 10, uses `rx_msgs` from `RobotX-SoS-communication-ITSN` (pull `origin/main` first, the local copy is
1 commit behind). Reuses `dds_relay.py`, `gcs_ros_bridge.py`, `mock_usv.py`, `rc_observer.py`.

**Scripted commands** on `/system/mission/command` (`rx_msgs/Command`). `command_seq` increases on every send:

| Name | Fields |
|---|---|
| RUN-START | `command_type: 1`, `target_vehicle: 3`, `run_id: 1` |
| UAV-GO | `command_type: 8`, `target_vehicle: 3` |
| DELIVERY | `command_type: 2`, `target_vehicle: 3`, `task: 3`, `resource_color`, `delivery_circle_color` |
| MISSION-DONE | `command_type: 9`, `target_vehicle: 3` |

(`task: 3` is `TASK_INFRA_SURVEY_REPAIR`. For the UUV's own `CMD_RUN_START`, `target_vehicle` is 2 or 0.)

**Auto-relay:** on `/system/vehicle/uuv/task2/delivery` (`ResourceDelivery`), publish `CMD_DELIVERY` to the UAV
and log it. **Mock UAV:** on `CMD_DELIVERY`, echo `ResourceDelivery` on `/system/vehicle/uav/task2/delivery`
and publish heartbeats with the tasking, to match the scoring rule. It does not check whether the UAV was
already autonomous (decided: echo only).

Checks per run: survey order, tin/circle pair equals the sim flash, `CMD_DELIVERY` matches, seqs never repeat.

## 9. Assumptions (replace with real numbers when known)

| Item | Assumed | Source |
|---|---|---|
| GPS loss depth | 0.1 m (worst case) | user |
| GPS reacquire delay | 10 s, also 30 s | assumption |
| Hydrophone accuracy / rate | ±15°, one bearing per 1–2 s | assumption |
| Hydrophone ADC | unknown | open |
| Forward speed at safe thrust | 0.3 m/s | assumption, to measure with `TimedMove` |
| Run budget | 20 min total, 8 min per dive | assumption, no rule found |
| Max depth | to be set | open |
| Link | WiFi at the surface only | user |
| Water visibility | no data | open |

## 10. Test matrix (run with GPS cut underwater, several seeds)

| Case | Variation | Pass |
|---|---|---|
| S1 | Baseline, hydrophone on | Order correct, repair verified, report sent |
| S2 | Hydrophone off | Same, slower |
| S3 | Hydrophone dropout mid-dive | Falls back and completes |
| S4 | Buoy 2–5 m off the pipeline end | Pipeline found within the timeout |
| S5 | Red ring at each of 4 positions | Repair works every time |
| S6 | All 9 tin × circle pairs (Disruptive) | Decoder correct on all |
| S7 | Pipeline missed on first dive | Surface-to-fix and re-dive succeed |
| S8 | Heading or drift disturbance | Completes or fails safe to surface report |
| S9 | Decode fails | Survey-only report still sent |
| S10 | Duplicate/replayed `RUN_START`, new run window | Run identity unchanged |
| S11 | Reduced visibility | Reported as a limit, not a pass/fail |
| S12 | GPS reacquire 30 s | Report sent after reacquire, no dive loop |

Unit tests: decoder on synthetic ROI traces, pulse detector on synthetic pings, ledger tier rules.

## 11. Risks and open items

- **Mode choice (D1)** is not a team decision yet. Experiments in section 6 decide it.
- **Hydrophone** hardware, ADC rate, and the real pinger (a replica may be used) are unknown. The no-hydrophone path must work alone.
- No pinger for pool tests: pulse detection is validated only in sim until one is available.
- Real flash timing, exposure and colour shift are unknown. Record a real ring early.
- Seabed depth is unknown. Use the pipe's apparent width as an altitude proxy.
- Run-time limit unknown. Re-check the final handbook.

## 12. Per-tier specification

Each tier includes everything in the one before it. `task2_tier` follows the tier (2 Core, 3 Advanced, 4 Disruptive).

| | **Core** (450) | **Advanced** (700) | **Disruptive** (1050) |
|---|---|---|---|
| Objective | Report the start point, survey the pipeline, report light order, repair the red ring | Core + after repair, read the flash and request a supply delivery | Advanced + the flash carries two colours (tin, circle) |
| UUV receives | `Course`, `PipelineBuoys`, `CMD_RUN_START` | same | same |
| UUV sends (surface) | Heartbeat 2 Hz, mission status, `PipelineSurvey` (active buoy position + ordered segments) | Core + `ResourceDelivery` (`task=3`, `resource_color=COLOR_ANY`, `delivery_circle_color`) | Core + `ResourceDelivery` (`resource_color` = tin 1–3, `delivery_circle_color` = circle 1–3) |
| Destination | GCS → RoboCommand | GCS → RoboCommand, and GCS → UAV as `CMD_DELIVERY` | same as Advanced |
| After repair | nothing to read | Ring goes green 5 s, then flashes one colour: 1 s on, 1 s off, for 30 s. Read the circle colour | Ring goes green 5 s, then 5 s cycle (colour 1 for 1 s, off 1 s, colour 2 for 1 s, black 2 s) for 30 s. Read both |
| New work | Core flow (W2–W6 minus decode) | W4 (blue), W5 single-colour mode, W8 | W5 two-colour mode, W8 |
| Pass criteria | Light order matches the sim, repair verified, `PipelineSurvey` acked | Core + circle colour correct, request delivered to GCS and relayed as `CMD_DELIVERY` | Core + all 9 tin × circle pairs decoded correctly, relay matches |
| Fallback if the code cannot be read | n/a | Send the survey only (Core score still holds) | same |
| Main risk | Pipeline not found without position | Flash timing, blue vs green confusion | Two-colour timing, need to hold station 30 s over the ring |
| Points at risk from sensors | start point 25, order 150, repair 250 | +25 delivery location | +25 tin, +25 location |

UAV points (receipt, retrieve, deliver) depend on the UAV and the GCS relay, not on UUV sensors.

## 13. Sensor limitations (what each sensor cannot do)

Position is not measured underwater at all. Everything below is the reason for D1–D5 and the fallbacks.

| Sensor | Gives | Limitation | Effect on Task 2 | Mitigation in the plan |
|---|---|---|---|---|
| Pixhawk IMU (gyro, accel) | Roll, pitch, yaw rate | Accelerometer cannot give position (double integration drifts within seconds); gyro yaw drifts over minutes | No velocity or position underwater | Use for attitude only; yaw from the fused heading (D4) |
| Compass | Absolute heading | Disturbed by thrusters, battery and ferrous parts; stability in real use unknown | Heading error shifts the whole search and dead-reckoning track | Log gyro and compass, cross-check with pipeline direction in the camera and the `buoy_a`→`buoy_b` bearing |
| Bar30 | Depth (pressure) | No height above the seabed; needs a surface datum each run; baro drift (0.1–0.4 m in sim); water density differs (Marina Bay is brackish) | Cannot tell how far the pipeline is below, only the depth we are at | `calibrated_depth` datum at the surface; use pipe apparent width as the altitude proxy; set a max depth |
| RTK GPS (USB → mini PC → `GPS_INPUT` → Pixhawk) | Position, heading of motion | Surface only, lost at ≈ 0.1 m, reacquire delay after surfacing; RTK needs a correction link, otherwise it may fall back to plain GPS (accuracy unknown); depends on the mini PC and the bridge | No position underwater; GUIDED breaks (G2); drift reset needs a surface trip | `sim_gps_gate`, mode experiments, surface-to-fix, resend report only after the fix |
| Front camera | Forward image | No range; poor in turbidity; buoy indicator faces up, so likely unreadable at the waterline | Weak for buoy colour; weak for obstacles | Last-priority source for the active buoy; used for buoy structure and obstacles only |
| Bottom camera | Image of the pipeline and rings | Narrow field of view; unknown altitude above the pipeline; turbidity; auto-exposure shifts ring colour; blue ring not classified yet; frame rate and rolling shutter limit the flash decoding | Pipeline lost easily; wrong ring colour; missed flash cycles | Altitude proxy from pipe width, reacquire sweep, per-frame ROI hue log, ≥ 15 Hz sampling, majority vote |
| Hydrophone (undecided) | Azimuth only | No range or elevation; hardware, ADC rate and element count unknown; 4 ms pulses; marina reflections cause outliers; front/back ambiguity with few elements; thruster and electronics noise | Bearing, not a position; cannot say how far the buoy is | Bandpass + gate at Course frequency and PRI, soft corrections only, listen ≥ 4 s per buoy, drop to no-hydrophone mode when invalid |
| Dead reckoning (derived) | Position estimate | Uses commanded velocity, not measured; ignores currents; error grows with time; speed at safe thrust not measured yet | Only a coarse search prior | Short dives, surface-to-fix (max 3 attempts), calibrate speed with `TimedMove` |
| No DVL, no altimeter, no sonar | n/a | No speed over ground, no height above bottom, no range to obstacles | Open-loop motion, no underwater obstacle ranging | Vision-based following and centring only |
| WiFi (Ubiquiti Bullet AC) | Link to GCS | Surface only, may drop when moving | Report delivery can fail | Link check, resend every 1 s until acked (60 s), do not dive again before ack |

## 14. Red-ring scenarios (which node is damaged)

Handbook 3.3.3: when the challenge activates, **exactly one** of the four ring lights turns RED and the rest
GREEN. The red one is the damaged section to repair. Nodes sit at random distances along the pipe (FAQ), and
the pipeline can bend and change depth and slope (FAQ). The UUV does not know in advance which node is red.

Sim support already exists: `robotx_light_controller.py` takes `/robotx/pipeline/set_damaged_node` (1..4 or
`RANDOM`) and `BANYU_LIGHT_SEED`; `task2_sim_provider.py` uses `ring_colors` (default red at node 4).
Node numbers in the sim are not the same as the report order, which always starts at the active buoy.

**Base matrix (run for each tier, with the GPS gate on):**

| Case | Active end | Red node (in report order) | What it checks |
|---|---|---|---|
| R1 | start | 1 (first ring met) | Repair and decode happen right after acquire, before the ring list is complete |
| R2 | start | 2 | Normal case |
| R3 | start | 3 | Normal case |
| R4 | start | 4 (last) | Red ring is at the pipeline end, decode near the end condition |
| R5–R8 | other end | 1–4 | Same four, with the active buoy at the other end, so ordering and heading prior flip |

That is 8 cases per tier (24 total), each run with several seeds for spacing, bends and depth.

**Extra cases:**

| Case | Variation | Pass |
|---|---|---|
| R9 | Red ring missed on the first pass (turbidity, glare) | Backtrack and re-scan within the retry limit, or report the survey without repair |
| R10 | Probe hold fails, ring stays red | Retry the hold (limit), then continue and report `REPAIR_FAILED` |
| R11 | After repair the ring shows green, then flash colours red/green/blue | Ring tracker does not treat the flash as a new damaged ring; the repaired node is acknowledged (`AcknowledgeRingRepair`) and ignored for new "red" until the run ends |
| R12 | Repair attempted on the wrong ring (green) | Not counted as repair, no false `VERIFIED` |
| R13 | Unexpected input: no red ring, or two red rings | No crash; report what was seen, mark confidence low |
| R14 | Ring flickers or one ring is off (LED fault) | Colour unknown for that node, order still correct for the others |
| R15 | Pipeline bend or slope between rings | Follow stays locked, ring distance estimate stays ordered |
| R16 | Red node first seen while the UUV is re-acquiring | Repair still triggered once centred |

R11 is a real risk: the flash shows red for 1 s, which looks like a damaged ring. In the BT, `RedRingPending`
must stay false for a node after `AcknowledgeRedRing`, and the decoder must read the code on that node only.

Per case, log: which node was red, which end was active, time to acquire, time to repair, decoded tin/circle,
and the final report order, so the success rate per case is visible in the feasibility verdict.
