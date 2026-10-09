# Issue 13: reconciled rules, implementation plan, and acceptance gates

Reviewed 2026-10-09 against branch `13-featcomm-integrate-robotx-sos-communication-interface-rx_msgs-into-main_task`, commit `46d1f65aae33c78737b974f98a03603f3ce59961`. This document specifies work to do; it does not claim implementation or passing results for the proposed tests. Core implementation targets ROS 2 Jazzy, C++/rclcpp, and BehaviorTree.CPP v4, as requested.

## 1. Evidence and authority

| Reference | Authority and use |
| --- | --- |
| [Handbook §3.4](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol) | Official OCS/RoboCommand protocol, declaration/start, heartbeat and task semantics. Local readable copy: `/media/erlangga/DATA/RobotX/GitBook-Handbook/section-3-autonomy-challenge/3.4-communications-protocol.md`. |
| [Handbook §3.3.3](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.3-mission-task-2-infrastructure-survey-and-repair) | Official active buoy, survey order, tier requirements and optical timings. |
| [Handbook §3.3.5](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.5-mission-task-4-dynamic-incident-response) | Official Task 4, now titled Dynamic Incident Response. |
| [Handbook §3.5.5](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.5-task-build-specifications) | Five straight PVC sections; bends need not be coplanar. Pipe geometry does not determine electronic sensor count. |
| [Canonical RoboCommand schemas](https://github.com/robonation/robocommand/tree/f6457fa47489647a8650c1d3eb74efa84ce4b84a/RobotX_2026/proto/robotx) | Pinned commands, reports, enums and course wire definitions. Competition-approved release must be pinned again before deployment. |
| `UNDERWATER_MAGNETIC_ACTIVATED_LIGHT.md` and `Pipeline with UNDERWATER MAGNETIC ACTIVATED LIGHT.png` in `/media/erlangga/DATA/RobotX` | Supplied hardware summary and image; cross-checked against original human Q&A below rather than treating every summary statement as a final competition rule. |
| [Original Task 2 Q&A export](</media/erlangga/DATA/RobotX/[DiscordKit] Task 2_ Infrastructure Survey & Repair Questions_RoboNation Community_20260930_170619.json>) | Human RoboNation technical-forum export through September 29, 2026. Author roles, timestamps and message IDs establish provenance; it does not prove the absence of later updates. |
| Installed team interface at `../RobotX-SoS-communication-ITSN`, commit `2946a98a905e2e8fac779b4c02316c0e9dcd8f87` | Exact `/system/...` topics, ROS schemas and DDS domains. This is the team's internal contract, not the official external transport. |
| User-supplied `AGENTS.md` constraints | Velocity streaming, ENU/NED conversion, explicit yaw rate, parameter/config/launch/dependency consistency, C++ core nodes. Explicit Jazzy instruction supersedes the older Humble description. |

The local PDF basename ends `20260829`, but its printed revision is September 29, 2026 and its changelog includes the September 29 light-node quantity update. Use contents and revision rather than the basename to judge currency. The current official task diagram and supplied pipeline image support four electronic light nodes, corroborated by [David | RoboNation, September 28](https://discord.com/channels/969707043218935848/1437482903599841373/1554246751391846572). [Aamir | RoboNation, July 6](https://discord.com/channels/969707043218935848/1437482903599841373/1523760286899896451) confirms upward mounting. An older preliminary guide's one-node-per-section language conflicts with the later evidence; it must not produce a five-entry report. The September 29 team question about four versus five is a question, not an official counter-ruling.

The supplied hardware summary itself contains inconsistent magnetic timing: 0.5–2 seconds in one passage and Director guidance of 1–3 seconds in another. The official preliminary guide says two seconds; the prototype guide says 0.5 seconds. The original [Aamir | RoboNation September 2 reply](https://discord.com/channels/969707043218935848/1437482903599841373/1544719226516930561) anticipates 1–3 seconds but explicitly says timing was not finalized. Treat that as traceable Director guidance, configure for this range during testing, and record the final competition threshold before hardware acceptance. The same reply leaves magnet choice to teams. The prototype's 250 ms green feedback and immediate RGB flashing do not implement the competition's five-second green confirmation and tier waveforms.

## 2. Reconciled rules

### Start, run identity and task lifecycle

Official start sequence is: validated course → OCS RunDeclaration → per-vehicle autonomous-ready heartbeats and position hold → RoboCommand RunStart → OCS verifies `declaration_seq` → assigned `run_id` recorded → run movement authorized. Only the OCS connects to RoboCommand's MQTT/protobuf network. Vehicles communicate through the team interface.

`Heartbeat.current_task` starts an attempt when it enters a task and ends the attempt when it becomes `TASK_NONE`. Never use `TASK_UNKNOWN` as an intermediate or terminal task. Aborted/failed attempts therefore need a real lifecycle transition to NONE; retaining the old run ID for diagnostics is appropriate.

Neither §3.4 nor the canonical uint32 schema expressly forbids zero. Rejecting `run_id == 0` is the requested BANYU/team policy; the test server's allocation from one supports that convention. The all-zero UUID prohibition in the internal `Task2Report` is a different contract and does not establish a rule for the external uint32.

Idempotency is the selected engineering policy: retransmission of an accepted run must not restart it, reset observations, repeat task entry, or resurrect a terminal run. Invalid, stale, conflicting or unaddressed commands have no mission-data side effects. Do not infer numeric monotonicity of run IDs; use authoritative command sequences, declaration context and accepted-run records.

Installed `rx_msgs/Command` omits `declaration_seq`, `team_id`, and `vehicle_id`. The GCS must validate these before forwarding an addressed trusted command. Current BANYU fields alone cannot independently prove declaration matching. No canonical or installed `CMD_RUN_STOP` exists. `/banyu/mission/abort` remains an internal safety command; `CMD_MISSION_DONE` is a team extension whose terminal behavior must be explicitly defined.

### Survey meaning, cardinality and ordering

The [official report schema](https://github.com/robonation/robocommand/blob/f6457fa47489647a8650c1d3eb74efa84ce4b84a/RobotX_2026/proto/robotx/rx_reports.proto#L88) specifies the pipeline as surveyed, rather than its post-repair state; [Aamir's February 2 Q&A](https://discord.com/channels/969707043218935848/1437482903599841373/1467909205221708022) independently describes the complete sequence before repair. Preserve initial stable RED→DAMAGED and GREEN→INTACT observations. A repaired red node stays DAMAGED in that survey. BLUE and post-repair flashing are resource-code evidence, not INTACT classifications.

Use exactly four distinct upward-facing electronic nodes for the current task configuration: one initially solid red, three initially solid green, with 16 WS2812B LEDs and a central TLV493D magnetometer per node, daisy-chained in series in the supplied assembly. Wiring topology does not turn the five PVC sections into five electronic observations. The protobuf uses a repeated field; four is the course/hardware contract rather than a fixed wire array length. Do not force observations to match the expected one-red distribution by changing data.

Index zero is the node nearest the confirmed active GREEN buoy, then nodes follow the pipeline's traversal/topological order. Global sorting by straight-line distance to the buoy is wrong for a bent pipeline. Acceptance order is usable only when the confirmed starting end, forward progress and revisit deduplication are established. Vehicle pose at image time is useful evidence, but is not itself the node's measured position. [Aamir's August 3 reply](https://discord.com/channels/969707043218935848/1437482903599841373/1533906508264374394) says spacing is random; David's September 28 reply says buoy-to-end distance is not precise. No fixed spacing or buoy offset belongs in the ordering oracle.

An unobserved `[1,1,1,2]` fabricates measurements and can identify the wrong damaged segment; it cannot establish survey scoring. UNKNOWN is legal in the schema, but an incomplete array is not a successful complete BANYU survey. No official service response format is prescribed. The selected policy is `success=false`, an actionable diagnostic, no publication, and preserved observations until all four known conditions and their order are established.

Handbook wording permits surfacing rather than universally requiring it. BANYU's requested verified-surface guard is a stricter team policy appropriate to its store-and-forward link. Do not describe it as an explicit universal competition rule.

### Course and active buoy inputs

The actual topic is `/system/mission/course`. Official Course carries course ID, pinger frequency in Hz and a closed boundary; the team ROS type also carries `uav_geofence`. Neither contains active-buoy coordinates.

Provide a separate observed, timestamped, run-associated active-buoy fix from UUV green-indicator/acoustic localization or UAV/USV perception. Validate source, endpoint identity, coordinate validity, uncertainty and course consistency. Feed that fix to surface transit and survey orientation. Feed course frequency to the acoustic receiver/filter and homing consumer; feed boundary to navigation/geofence consumers. Static YAML positions are simulation fixture inputs, not competition observations. The current vision-only pipeline guidance is not an acoustic consumer.

### Task tiers and Task 4

Core's minimum is an ordered survey; repair is optional for extra points. Advanced/Disruptive require repair and resource decoding. After completed repair, the official five-second solid green precedes Advanced's one-second color/one-second OFF cycle for 30 seconds, or Disruptive's color1 1 s → OFF 1 s → color2 1 s → OFF 2 s cycle for 30 seconds. Advanced requests ANY resource and the observed destination color; Disruptive requests color1 resource and color2 destination. Image centering plus elapsed time does not prove magnetic repair. Interrupted magnetic proximity resets continuous dwell. Early proximity green must not be mistaken for completed repair.

Task 4 Core requires ACK, mission interruption, actual transit, loiter, readiness, and correctly correlated confirmation before resuming. `ReadinessReport.command_seq` references the assistance command; `ReadinessConfirm.report_seq` references the containing OCS `RxReport.seq`, additionally identified by vehicle ID. The GCS needs an explicit mapping or receipt so a vehicle can correlate that sequence. A surfaced UUV remains TYPE_UUV. Advanced keep-out restrictions persist until all-clear or run end, and both commands are ACKed. Disruptive alerts require avoidance using current and projected motion; there is no separate ACK/all-clear. A conservative acceptance margin greater than 10 m avoids the handbook's 10 m exclusion. Hazard handling need not interrupt an otherwise valid task if avoidance can continue it. Exact Task 4 interrupt/resume heartbeat sequencing is a team policy to document, not an unstated official rule.

## 3. Step 1: mission start and lifecycle hardening

### Ownership and C++ components

Use one authoritative `RunCoordinator` owned by `banyu_mission`. Make its transition/validation logic a testable C++ class without ROS side effects. `uuv_comm_node` serializes external telemetry and observes lifecycle; it must not independently mutate run/task state before mission acceptance.

Concrete files:

- Add `banyu_mission/include/banyu_mission/run_coordinator.hpp` and `src/run_coordinator.cpp`; integrate with `mission_state.hpp/.cpp`, `banyu_mission.hpp/.cpp` and their CMake/test targets.
- Add private `banyu_msgs/RunLifecycle` carrying official uint32 run ID, local epoch/attempt identity, command sequence, lifecycle phase, task and failure reason. Publish reliable/transient-local snapshots for comm restarts, with strict epoch validation.
- Make the Task 2 mission node consume trusted `/system/mission/command` directly for start, or an equivalently typed handoff that carries this context. Select direct consumption here; remove the comm-to-empty-Trigger start path in Task 2. Keep one mutator, not competing direct and service paths.
- Keep `/banyu/mission/start` for POR. In Task 2 it rejects direct calls with `GCS_START_REQUIRED` or is absent. Preserve the abort safety endpoint in both modes.
- Make `uuv_comm_node` derive `run_id`/`current_task` from lifecycle snapshots; publish `MissionStatus` on accepted/rejected start and every terminal transition.

`Task2Report.msg` already defines an internal UUID plus original ring records and repair status, but no producer/consumer was found in the current mission/comm source. Reuse or evolve this contract when adding the survey snapshot rather than creating conflicting report owners. Keep the official uint32 run ID, local attempt UUID and epoch explicitly distinct; never silently hash or cast between them.

### Transition contract

| Event/state | Result | Data effect |
| --- | --- | --- |
| Operator enables autonomy, safety/preflight valid | Prepare GUIDED/armed readiness and position hold; task NONE | No task survey reset or scoring attempt |
| Accepted trusted start, nonzero run ID, correct target/declaration context, ready, no active conflicting run | Record run/epoch and enter authorized initialization | Archive previous attempt; reset current tracker and ledger once |
| Reset consumers acknowledge current epoch | Enter Task 2 and allow run movement | Task becomes INFRA_SURVEY_REPAIR |
| Same accepted run retransmitted, including after terminal state | Return idempotent status | No reset, new BT start, report or task transition |
| Zero ID, stale/conflicting sequence/context, wrong target, not ready, active different run | Reject with reason | Preserve run, task, buffer and generation |
| Abort/internal stop, operator takeover, action/BT failure or readiness loss | Cancel current action/BT; terminal lifecycle; task NONE | Preserve diagnostic run and observations; invalidate callbacks |
| Survey/required repair complete | Enter surfacing/report phase | Freeze original survey; Task 2 remains active |
| Guarded report accepted into defined delivery mechanism, then BT complete | Succeeded; task NONE | Preserve immutable report and deduplication identity |

Pre-run preparation is necessary: currently an unarmed UUV is advertised KILLED and `ArmAndSetGuided` is behind `RequireMissionStarted`, while RoboCommand waits for AUTO before sending start. Split safe preparation from run movement. Require explicit operator autonomy enable and actual safety/FCU readiness; do not fake AUTO. Keep readiness maintenance and position hold active before start.

Serialize command and lifecycle transitions through a mutually exclusive rclcpp callback group/coordinator queue; the BT thread takes mutex-protected snapshots. Commit the accepted context before issuing resets, but permit no movement until reset acknowledgments complete. Retain old evidence in an archive if initialization fails. Reset ring tracker, end detector and comm ledger together; clearing only the comm map allows old observations to leak back. Tag accepted records and asynchronous action/service callbacks with epoch and reject old generations.

Use steady-clock deadlines for handoff, reset and safety expiry. A late callback cannot restore the task after abort or a subsequent run. Check duplicate identity before any clear or generation increment. Persist enough accepted-run/sequence context, or require a fresh trusted authorization handshake, to reject retained commands after process restart. A new run ID is not sufficient evidence by itself after restart.

Report task NONE by the next production heartbeat (nominal 0.5 s), with immediate lifecycle/status publication. Operator takeover cancels autonomy and yields outputs without forcing SURFACE over the pilot's selected mode. Fault-triggered emergency recovery has its own safety policy and does not keep the failed Task 2 attempt active. Treat `CMD_MISSION_DONE` as an internal stop/terminal request; it cannot manufacture survey or success when completion guards are absent.

## 4. Step 2: survey payload integrity and surfacing guard

### Validated ledger

Add a C++ `SurveyLedger`/report builder in communication, with a single authoritative snapshot from the tracking/mission side. Each of four physical nodes needs current epoch, stable node identity, first-pass pipe progression/topology, source frame/timestamp, observation quality and original stable condition. Keep repair verification and optical request fields separate. Repeated frames, backward revisits and changing post-repair light colors cannot append new physical nodes or overwrite initial survey conditions.

Extend `Task2RingRecord` as needed with stable association/progression and provenance. Do not repurpose its `vehicle_pose_map` as a node coordinate. Establish progression from validated pipeline traversal, or calibrated perception geometry; fail with uncertain ordering if that evidence is absent. `PipelineRingState.index` currently reflects acceptance order and alone cannot prove official ordering. Replace the current mutable color-to-map assembly in `uuv_comm_node.cpp` and remove the fallback entirely.

Use reliable, recoverable snapshot delivery for the final ledger. A best-effort stream of individual observations is insufficient evidence that all four were captured. Raw camera traffic can retain its existing sensor QoS. A reliable event/snapshot must still reject stale runs; transient-local QoS does not provide cross-run correctness by itself.

### Guard and service response

Evolve the private `TriggerPipelineSurvey` request to include run ID/epoch and a report identity, retaining boolean success/message in the response and adding a structured result code if useful. Update both service and BT client together. These are proposed private interfaces, not new official `rx_msgs` payload fields.

Publication requires all of:

1. Matching accepted run/epoch; Task 2 is in its authorized reporting phase and has not failed, aborted or been suspended.
2. Four distinct known original RED/GREEN conditions, validated active start buoy and unambiguous along-pipeline order.
3. Survey/traversal completion; required repair and optical decoding for the declared Advanced/Disruptive tier. Core survey-only remains valid.
4. Current Surface action success for this epoch plus independent fresh, finite, calibrated depth relative to water surface, held inside the configured surface tolerance for a configured duration. Reject missing/default/stale/future-dated telemetry. Use steady receipt deadlines so paused ROS time cannot keep proof fresh.
5. Snapshot accepted into the defined report outbox/publication mechanism; repeat requests for the same report identity return its prior status without another logical report.

Define `survey_complete` before surfacing and `report_ready` after verified surfacing. Do not require final BT SUCCESS before the report service: `SurfaceAndReport` calls that service before the tree can finish, creating a circular dependency otherwise.

Example failure diagnostics: `WRONG_RUN`, `RUN_NOT_ACTIVE`, `SURVEY_INCOMPLETE observed=3 expected=4 missing=node3`, `ORDER_UNVERIFIED`, `ACTIVE_BUOY_UNVERIFIED`, `REPAIR_UNVERIFIED`, `RESOURCE_CODE_INCOMPLETE`, `NOT_SURFACED`, `SURFACE_TELEMETRY_STALE`, `OUTBOX_UNAVAILABLE`. Failure publishes no successful scoring report and preserves evidence. Success contains exactly the observed four entries and observed buoy position. Protect guard/snapshot/outbox commit against concurrent abort/new-run changes using the serialized coordinator and epoch recheck.

Surface thresholds are calibration/engineering parameters, not handbook values. Choose and validate water-relative depth source, datum, age limit and continuous hold; do not infer surfacing solely from map z=0 or one tolerance-crossing sample. Extend the current `surface_server.cpp` completion logic accordingly while preserving velocity-loop control.

No official `PipelineSurveyAck` exists. ROS `publish()` only means local submission. Define service success precisely as accepted into a recoverable local outbox, and expose forwarded status separately if needed. The GCS must map report identity/run to its official envelope and avoid reuse across runs. To require actual onward transmission before Task 2 completion, add a documented team delivery receipt and wait for it; it is not a RoboCommand survey ACK. Do not invent such an ACK in tests. Reconnection may retry transport, but deduplicate the logical report and do not relabel old data as a new run.

The installed public `rx_msgs/PipelineSurvey` has only header, buoy and segments: the private service's run/epoch/report identity does not reach GCS. Separately received MissionStatus cannot atomically identify a delayed retained survey. A coordinated team-interface/GCS change is therefore required for reliable run association. Proposed solution: an agreed versioned submission envelope under `/system/vehicle/uuv/task2/submission`, containing uint32 official run ID, local attempt/report identity and the complete survey or resource-request payload in one message. GCS validates accepted-run context, deduplicates this envelope, and returns a team receipt carrying the same identity and its assigned OCS report sequence. The existing survey/delivery topics remain available with their exact installed types, but GCS must select one authoritative ingestion path and must not forward both copies. This extension is a proposal requiring matching interface/GCS changes, not an existing official or team schema. Do not encode run IDs in frame_id or assume DDS ordering across separate topics. Without an atomic agreed envelope/handshake, focused guard work can proceed but cross-run store-and-forward closure remains unproven.

Add the currently missing `rx_msgs/ResourceDelivery` publisher on `/system/vehicle/uuv/task2/delivery`, with `task=TASK_INFRA_SURVEY_REPAIR` and verified tier-specific colors. Feed it from the separate optical decoder, never from survey conditions. Apply the same run association/outbox rules to its authoritative submission, and verify GCS conversion to official ResourceDeliveryRequest. Full SoS acceptance also checks the UAV echo when it begins delivery.

### Downstream context wiring

Proposed private flow:

```text
GCS validated /system/mission/course
  -> retained validated course context
     -> navigation/geofence: closed boundary and course identity
     -> acoustic frontend/homing: pinger_freq_hz
Observed active-buoy fix (UUV or GCS-relayed UAV/USV observation)
  -> mission surface transit/acoustic approach
  -> survey endpoint/orientation and report buoy position
RunCoordinator epoch
  -> tracker reset + mission + comm/outbox + context validation
```

The current Course ROS type has no timestamp/header, unlike official RxCourse. Preserve freshness/version metadata in the trusted GCS adapter or an agreed internal context extension; do not pretend the vehicle received fields the type does not carry. Validate finite legal geographic coordinates, polygon closure/geometry and frequency support; unsupported frequency fails preparation rather than silently using a default. Preserve consistent ENU geodetic anchoring. Mid-run course/active-end changes require an explicit validated update policy; they cannot silently reverse the report.

Add a real C++ acoustic frontend/homing consumer when hardware is available; do not claim that forwarding a topic alone implements homing. Node parameters belong in the corresponding `task2_params.yaml` section and `mission.launch.py`, with installation and package dependencies updated. Production launch must not consume deterministic fixture buoy/ring data.

## 5. Headless/SITL safety

`rc_fresh` currently measures receipt of a valid `/mavros/rc/in` array with takeover channel 4 present, within three seconds. It does not measure joystick presence. The prior audit did not capture live RC channel count or protocol parameter values, so lack of joystick is not a demonstrated cause.

A strong source-backed candidate is the bare launch's missing frame defaults. ArduSub's [default `RC_PROTOCOLS=0`](https://github.com/ArduPilot/ardupilot/blob/755258dbb452f46dc7e1e47a36591f3b62e8830d/ArduSub/Parameters.h#L444) disables RC backends, whereas official [sub-6dof defaults](https://github.com/ArduPilot/ardupilot/blob/755258dbb452f46dc7e1e47a36591f3b62e8830d/Tools/autotest/default_params/sub-6dof.parm#L65) set `RC_PROTOCOLS=1`, `RC_OPTIONS=0`. The enabled SITL UDP backend supplies neutral RC without a physical joystick. Zero HAL channels produce empty MAVROS RC arrays, which this tracker rejects even if messages arrive.

First perform a clean bare-versus-wrapper A/B capture: firmware SHA, defaults/readback, `RC_PROTOCOLS`, RC channel count/rate/value, FCU pilot failsafe, connection, GUIDED, GPS/EKF readiness and preflight outcome. Apply the official frame RC configuration through a simulation-only overlay for the bare JSON path; preserve normal wrapper defaults. No real-FCU parameter changes are part of this plan. Do not disable `FS_PILOT_INPUT`, fabricate RCIn, or force `rc_fresh=true`. Neutral MANUAL_CONTROL can refresh ArduSub pilot input but does not necessarily create HAL RC channels; handle the two mechanisms separately.

Then add a C++ `sitl_safety_node`, launched only for explicit `use_sim:=True` and `headless_sitl:=True`. It grants an expiring private simulation safety lease based on owned SITL/Gazebo session, fresh connected FCU, valid RC and advancing physics evidence. Bind it to a local simulation session generation so a retained lease cannot authorize another process/run. Use steady-clock receive/expiry and physics-progress deadlines. A possible engineering starting point is 10 Hz renewal, 0.5 s expiry; validate actual latency.

Add explicit `safety_source` and `safety_fresh` to internal VehicleStatus. Hardware requires physical RC freshness and operator permission; headless simulation requires truthful RC freshness AND the live simulation lease. `use_sim:=False` rejects simulation safety sources/configuration and does not launch the supervisor. Shared C++ safety logic must gate mission preparation, action execution and MAVROS publication. Execution currently stores `rc_fresh` but omits it from its fault/publication predicates, which must be corrected.

Preserve operator takeover precedence, fresh FCU/status requirements and GUIDED-only autonomy. On lost permission stop nonzero autonomous output within lease/status deadline plus one publication cycle, with the defined controller cancellation/failsafe behavior. Do not force a mode change after pilot takeover. Add the missing Gazebo `/clock` bridge when using simulated ROS time; a stalled clock must not freeze safety expiry. Keep 20 Hz velocity streaming, mask `0x5C7`, ENU fields and explicit yaw rate zero except intentional turns/orbits.

## 6. Acceptance suite and proof obligations

Tests below specify outcomes, not current passing results. Use pure C++ gtests for coordinator/ledger/safety/decoder and rclcpp integration fixtures for real mission/comm nodes. Run tests in an isolated DDS domain, e.g. 42. End-to-end team routing uses UUV 40, GCS 10 and applicable USV 20/UAV 30 per team contract. Record timestamps and current configuration; do not speed the heartbeat to 10 Hz or replace the mission-start service when claiming production verification.

| ID / grounding | Exact probe | Passing evidence |
| --- | --- | --- |
| L1 / §3.4.10 | OCS sends mismatched declaration start; then a matching start | Mismatch never reaches authorized vehicle state; matching start does. Test GCS validation, not fields absent from Command. |
| L2 / team start gate | Call Task 2 `/banyu/mission/start` without command; separately exercise POR start | Task 2 rejects/is absent, no BT movement/task entry; POR retains its documented start path. |
| L3 / zero policy | Send run ID zero in READY and during active run | Explicit rejection, identical run/task/epoch/ledger afterward. |
| L4 / idempotency | Retransmit same run/command before initialization completes, while active and after terminal | Exactly one accepted initialization/reset/task entry; no new motion/report; original observations unchanged. |
| L5 / replay | Wrong vehicle, old command, same seq conflicting payload, different run while active, delayed previous-run callback, process restart replay | Reject without mutations; old callback/retained data cannot enter the new epoch. |
| L6 / transaction | Reset tracker unavailable; comm restarts while run active | No motion before reset acknowledgment; initialization error visible; restarted comm reconstructs authoritative lifecycle without resetting mission. |
| L7 / task semantics | Abort, internal stop, action failure, BT failure, readiness loss and operator takeover at each mission phase | Lifecycle reason immediately published; next 2 Hz heartbeat within nominal 0.5 s is NONE; run retained; stale callbacks cannot reenter Task 2. |
| L8 / readiness | Actual pre-run operator enable/preflight, no start for several seconds | Real AUTO/ready and position hold, TASK_NONE, no mission transit; valid start then begins task. No injected mock start hides readiness dependency. |
| S1 / four-node contract | Place initial red at each of four positions, test both active endpoints | Eight parameterized cases each yield exactly the four observed statuses in correct direction. No hardcoded status pattern. |
| S2 / integrity | Zero, one, three, duplicate-only, five ambiguous identities, BLUE-only, stale-run observations | Failure diagnostic and zero successful reports; no filler; retained evidence supports later legitimate retry. |
| S3 / §3.5.5 ordering | Bent/U-shaped pipe where distance sorting disagrees with arc order; backward revisit | Report follows pipe topology from active buoy; revisit neither appends nor reorders. |
| S4 / official original survey | Observe red, confirm repair, then emit green/blue/red optical flashes | Original red remains DAMAGED; other original conditions unchanged; repair/code records update separately. |
| S5 / surface policy | Invoke report before survey complete, submerged, during surfacing, after emergency surface on failed run | All fail without publication. Final BT SUCCESS is not required prematurely. |
| S6 / surface evidence | Missing/NaN/default depth, stale/future stamp, wrong datum, single near-surface sample, clock pause | Guard remains closed. Fresh calibrated sustained surface proof plus action result opens it only for current eligible epoch. |
| S7 / report transaction | Two simultaneous requests; retry; abort during guard evaluation; new run during delayed response | One logical report for accepted identity; response states consistent; no report attributed to wrong/terminal run. |
| S8 / delivery scope | Disconnect/reconnect GCS; inspect local outbox, forwarded receipt and MQTT separately | No false receipt/ACK claim; no cross-run replay; explicit distinction between local acceptance and onward transmission. |
| C1 / course propagation | Publish valid retained course before consumers start; capture acoustic/geofence consumer inputs | Late consumers receive validated frequency/boundary unchanged; unsupported frequency/open polygon prevents readiness. |
| C2 / buoy provenance | Course alone; then valid green/pinger buoy fix; then stale/conflicting fix | Course alone supplies no fabricated buoy; observed fix drives transit/order; inconsistent updates rejected/diagnosed. |
| R1 / magnetic hardware | Calibrated probe geometry, continuous hold at configured 1–3 s guidance; interrupt field before threshold | No repair for interruption; early green is insufficient. Real repair confirmation, not image timer, gates required tiers. Record actual hardware threshold/Q&A provenance. |
| R2 / optical tiers | Five-second green then all three Advanced colors and all nine Disruptive pairs with the official waveforms | Advanced resource ANY and observed destination; Disruptive first resource/second destination; no incomplete code fabricated. |
| R3 / tier behavior | Core survey-only; Advanced/Disruptive without repair or complete optical code | Core report eligible; higher-tier completion/report milestone remains guarded. |
| R4 / §3.4 Task 2 request | Observe completed Advanced/Disruptive decoding; capture UUV delivery topic, authoritative submission, GCS conversion and UAV start | UUV publishes ResourceDelivery with task INFRA_SURVEY_REPAIR and exact observed tier colors; wrong/incomplete/stale-run codes produce no request. GCS publishes canonical ResourceDeliveryRequest once logically; UAV echoes identical colors when delivery begins. |
| D1 / diagnosis | Bare launch vs wrapper with protocol/channel/failsafe captures | A/B establishes whether RC defaults explain the hang; default correction produces valid RC and readiness, without physical joystick assumption. |
| D2 / safety boundary | Hardware launch receives simulation lease or attempts simulation safety configuration | No simulated authorization, no arming/nonzero output from it; launch/constructor rejects inconsistent settings. |
| D3 / truthful freshness | Empty/undersized RC, valid neutral RC, stale RC, takeover channel over 1700 | Only valid fresh RC refreshes rc_fresh; takeover yields pilot control and no autonomy mode overwrite. |
| D4 / watchdog | Stop supervisor, stall physics/clock, disconnect FCU, age status, leave GUIDED while RC continues | Permission/actions/output revoke within configured timeout plus one execution period, measured in wall time. |
| D5 / recovery | Restore simulation after fault; replay old lease/start | No automatic resume; fresh session/readiness plus valid new authorization required. |
| D6 / control contract | Capture motion during pre-run hold, task navigation, turns and surface | Continuous configured velocity-loop rate, `0x5C7`, ENU axis signs, explicit zero yaw except commanded turn/orbit; no position-mask navigation regression. |
| T4-1 / §3.3.5 Core | Assistance far from current GPS; observe transit and station hold; deliver correct/incorrect confirmation | Immediate original-seq ACK; no readiness en route; readiness only at configured station tolerance/hold; resume only correct OCS report-seq/vehicle confirmation. |
| T4-2 / Advanced | Keep-out intersects planned route; then all-clear or run termination | Original command ACK, actual avoidance until cleared/end; all-clear ACK; no unnecessary task restart. |
| T4-3 / Disruptive | Moving object crossing route with position/heading/speed and affected domain | Current/projected safety separation greater than 10 m with chosen margin; no invented ACK; applicable domain only. |
| W1 / §3.4 wire | Real GCS serializes official pinned classes to test server; inspect decoded envelopes | Correct topic/team/vehicle IDs, oneof, sequences, synchronized sent_at, enum values and report fields; only OCS broker connection. |
| W2 / rates | Production run heartbeat measured for 30 s and OCS publication rate budget captured | Per-active-vehicle heartbeat nominal 2 Hz (60 intervals, tolerance documented for host load); at most five publications/s/system unless specified otherwise. No 10 Hz fixture substitution. |
| W3 / DDS team policy | Bridge domains 10/20/30/40 with allowlisted `/system/...` while private camera/tf/mission data active | Required system messages cross; private data does not; each vehicle remains isolated. This proves team isolation, not an official mandated domain number. |
| E1 / end-to-end | Clean headless Gazebo + SITL, real mission/tracker/comm, physics-backed motion and optical fixture | Ready/hold → trusted start → observed active end → four-node survey/required repair/code → verified surface → report → NONE; fault variants also pass. A telemetry-only mock is not this proof. |
| E2 / §3.1.5 SoS | Official `rc-test` start with at least two declared ready vehicles and actual GCS | Preserve server/OCS logs proving declaration/start chain. Single-UUV fixture cannot establish this requirement. |

For timing tests use controllable/injected steady time in pure classes, then wall-clock bounds in integration. Do not make sleep-heavy tests that merely mirror private methods. Capture externally observable commands, lifecycle, reports and setpoints. Include loss/reorder/reconnection and delayed callbacks because these are the defects that prior mock success missed.

Repair bench acceptance must record magnet size/orientation, probe-to-sensor geometry, continuous in-range evidence and observed completed state. No universal millimeter threshold is established by the supplied rules. Do not reuse prototype firmware flashing as the official-tier optical oracle.

## 7. Implementation order and closure

1. Implement the single start/lifecycle owner and pre-run readiness path; update comm to observe it; prove L1–L8.
2. Implement run-scoped original survey ledger, active-end ordering, completion/surface guard and defined local outbox semantics; prove S1–S8.
3. Verify/fix SITL RC defaults and implement explicit supervised simulation safety in mission and execution; prove D1–D6 and E1.
4. Implement actual course/acoustic/buoy consumers, physical repair verification and tier optical delivery; prove C/R cases.
5. Implement Task 4 transit/loiter/confirmation and hazard constraints, then GCS wire and DDS routing proofs; prove T4/W/E2 cases.

Build changed interfaces before dependents. Focused package set is `banyu_msgs banyu_guidance banyu_vehicle_interface banyu_vehicle_execution banyu_communication banyu_mission banyu_detection banyu_bringup`. Source `/opt/ros/jazzy/setup.bash` and workspace install via bash. Keep parameters synchronized with both applicable POR/Task 2 node sections, launch forwarding, install and dependencies. Preserve the BT phase chain's plain memory Sequence and reactive interruption branch. Never alter the user-configured front-camera sensor block.

The existing branch remains unresolved. The saved earlier verification at `/tmp/banyu-issue13-46d1f65.5O19Uc/VERIFICATION.md` recorded 137 passed and 63 skipped checks, but also demonstrated the start bypass, fabricated/sparse reports, destructive duplicate start and instant Task 4 readiness; the SITL run did not complete. Those results are historical baseline evidence, not validation of this plan. A skipped nonexistent RUN_STOP test is not compliance proof.

Passing Steps 1 and 2 closes their identified defects, not all Issue 13 behavior or full declared-tier competition compliance. Full closure requires the remaining consumer, repair/tier, Task 4, routing and official GCS wire evidence above, plus traceable confirmation of the final competition magnetic dwell threshold. Unit/ROS mock success alone cannot establish physical repair or official scoring acceptance.
