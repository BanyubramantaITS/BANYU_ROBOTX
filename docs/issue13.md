# Issue 13 — status, implementation plan and verification

This is the single maintained document for [Issue #13: integrate the RobotX SoS interface into main_task](https://github.com/BanyubramantaITS/BANYU_ROBOTX/issues/13). It consolidates the earlier rules, audits, corrections and coding handoffs. Superseded tracked documents remain recoverable from Git history.

**Status baseline:** October 11, 2026 update (§0) on top of `50dbb01`. Sections 1–10 below are the October 10 baseline; where they conflict, §0 is current.

**Verdict:** the UUV ⇄ GCS ⇄ RoboCommand communication path is verified end to end in simulation against the GCS team's `GcsReporter` and RoboCommand's official schemas and test client. A full simulated Task 2 run, started by RoboCommand, delivered a correct `PipelineSurvey` to RoboCommand (§0). What remains is hardware/field work and joint decisions (real DDS Router/two-machine test, GCS-side bridge, Task 4 scope, pinger use).

## 0. Update — October 11, 2026

Scope decisions from the UUV team: Task 2 is attempted at the **Disruptive** tier; Task 4 is **undecided** (keep the ACK-only stub); use of the acoustic pinger on the robot is **undecided**. (`pinger_localization` already turns hydrophone TDOA arrivals into `/banyu/pinger/observation` and `SearchPipeline` steers on it, but nothing filters on the course's pinger frequency yet.)

### Fixed (each with regression tests)

| Area | Problem found in integration/simulation | Fix |
|---|---|---|
| RoboCommand starts | Journal enforced a globally increasing `RxCommand.seq`; a restarted RoboCommand session (seq 1) was rejected as replay forever | `prepare_autonomy` opens a new sequence window; journal V2 stores each run's epoch (V1 still loads); within one preparation window accepted `run_id`s never restart (re-preparing forgets them, epochs keep increasing) |
| Heartbeat state | `MAV_STATE_CRITICAL` (ArduSub pilot-input failsafe) reported `STATE_KILLED` while driving | Only EMERGENCY/POWEROFF/TERMINATION count as killed |
| Action goals | Task 2 action nodes read run context from an unpublished topic; every goal was `(0, 0)` and rejected | Use `topics.run_lifecycle` |
| Simulation clock | No `/clock` bridge and `SetUseSimTime` does not reach launched nodes; sim-stamped camera vs wall-time nodes dropped every observation; MAVROS stamped zero | `/clock` bridge, `SetParameter(use_sim_time)` only with `use_sim:=True`, MAVROS from the same condition |
| Telemetry rate | MAVProxy default 4 Hz pose vs 1 s readiness window | `--streamrate=10` |
| Mission BT | Single-pass traversal; acquisition without retry; no traversal bound | Loop until `PipelineEnded`, retry acquisition via search, `mission.traverse_timeout_s` |
| Pipeline loss | One dropped frame aborted following; reacquire surges random-walked the vehicle off the pipe | Loss only after `pipeline_timeout_s` without a tracked frame; search retraces before sweeping |
| Readiness | Lost readiness only said `AUTONOMY_READINESS_LOST`; busy-executor clock lag made fresh poses look future-stamped | Reason names the failing input; 0.5 s future-skew tolerance |
| Ring repair | `CenterOnRing` drove depth from image-y and hung when the ring left view; hold commanded zero and drifted off | Surge from image-y, grace + backing-up search with timeout; centering continues during the probe hold |
| Detector | Bends and node housings made one line fit fail (NO_PIPE, ring dropped) | Local refit around the lookahead row |
| Red-ring repair loop | The tracker re-emits an accepted ring's state; the mission re-armed the repair for the already repaired ring and the second acknowledgement was rejected forever | A repaired ring index no longer re-arms the repair |
| Surface datum | The datum captured at the surface in READY was dropped at the run start when barometer drift moved raw `rel_alt` out of the surface band; `Surface` then held zero velocity at the bottom | Keep the pre-start datum while still at the surface relative to it; refresh it before each run; surfaced tolerance 0.5 m (sim EKF drift up to 0.38 m) |
| Survey evidence | Raw surface start-buoy fix overwrote the qualified buoy; rings passed at speed and rings closer than the camera footprint were never recorded | Qualified buoy wins; pose tolerance 0.12 s; ring position estimated from the camera model; centerline ordered from the active buoy |

Sim-only stand-ins (labelled, launched only with `use_sim:=True`): `task2_sim_provider.py` publishes a run-tagged GREEN-confirmed `ActiveBuoyFix` and `PipelineEndCandidate` from world ground truth, in place of the not-yet-built buoy localizer and end-of-pipe detector. `tools/sos_integration/sim_supervisor.py` provides the headless deadman RC (ArduSub SITL reports no RC channels).

### Integration evidence (local, one machine)

Stack: Gazebo Harmonic + ArduSub SITL + MAVROS + the full Task 2 launch on domain 40; `tools/sos_integration/dds_relay.py` (`/system/*` only) to domain 10; `tools/sos_integration/gcs_ros_bridge.py` around `GcsReporter` (`robocommand-rx-ui-itsn` `18bcd00`); local MQTT broker; RoboCommand's official test client and protobuf schemas.

- Course → UUV (boundary, 25 kHz pinger); heartbeat at 2.000 Hz (0.1 ms jitter) with GPS position, depth, attitude, `TASK_INFRA_SURVEY_REPAIR` during the run and `TASK_NONE` after; `STATE_AUTO` while ArduSub reports CRITICAL.
- `RunStart` → run ACTIVE in ~120 ms; duplicate start ignored; restarted RoboCommand session (seq 1) accepted after re-preparation; Task 4 `IncidentAck` with the original `command_seq`; no ACK for moving-object; no false readiness.
- `PipelineSurvey` delivered to RoboCommand from the real `uuv_comm_node` with injected evidence (`sos_gcs_mock.py`).
- Full simulated mission (run 102, started by RoboCommand `RunStart` seq 17): surface GPS transit → dive → acquire → follow → red-ring repair (one center/probe/acknowledge) → all four rings recorded in order (RED, GREEN, GREEN, GREEN at 1.39/2.55/3.20/4.39 m from the active-buoy end) → end of pipe → `SURVEY_COMPLETE` → surfaced → `SUCCEEDED / MISSION_COMPLETED`. RoboCommand received exactly one `RxReport.pipeline_survey` (active buoy lat/lon, segments `DAMAGED, INTACT, INTACT, INTACT`) 164 s after the start, and heartbeats returned to `TASK_NONE` after the run.

Open findings for the GCS team (spec: `docs/gcs_ros_bridge_spec.md`): command forwarding, stale-heartbeat policy, sequence persistence across GCS restarts and one-shot survey de-duplication are missing from `GcsReporter`; the reference bridge implements all four.

### Remaining for closure

1. Two-machine test with the team's real DDS Router (UUV domain 40, GCS domain 10) and the GCS team's own bridge.
2. RoboCommand test server / Team Village run.
3. Real perception producers replacing the sim stand-ins (active buoy, end of pipe); Disruptive-tier LED reading → `ResourceDelivery` on hardware.
4. Task 4 scope decision (stub vs full); pinger use decision; boundary enforcement in navigation.
5. A real deadman/RC source for headless hardware runs.

## 1. Read this status correctly

| Status | Meaning |
| --- | --- |
| IMPLEMENTED / LOCAL PASS | Production code exists and the stated local tests passed. This is scoped evidence, not proof of every operating condition. |
| PARTIAL | Some implementation exists, but important connections or guarantees remain incomplete. |
| NOT IMPLEMENTED | The identified production component/behavior was not found at the baseline. |
| NOT VERIFIED | No sufficient acceptance evidence is recorded; implementation may exist. |
| HARDWARE / JOINT PENDING | Requires physical measurements or another team's integration, separate from local software checks. |

Use separate milestones: **software verified**, **simulation verified**, **hardware verified**, and **joint GCS/OCS verified**. Do not describe synthetic inputs as physical validation or a local publish attempt as official receipt.

## 2. What is completed and what remains

| Area | Baseline status | Completed behavior / remaining task |
| --- | --- | --- |
| Workspace/shared interface | IMPLEMENTED / LOCAL PASS | `import.repos` includes the team interface; communication/mission depend on `rx_msgs`; legacy named CRC32 report/action artifacts are removed. |
| Public ROS communication | IMPLEMENTED / LOCAL PASS | Heartbeat, MissionStatus, command/course subscriptions, PipelineSurvey and ResourceDelivery interfaces. Default heartbeat is 2 Hz; a fresh production-rate/load measurement remains to be recorded. |
| Mission lifecycle / Step 1 | IMPLEMENTED / LOCAL PASS | Authoritative RunCoordinator, Task 2 GCS start gate, nonzero run policy, idempotent duplicate starts, rejected conflicting/foreign starts, run/epoch tracking and terminal TASK_NONE behavior. Full deployed pre-run hold/motion remains a simulation gate. |
| Survey ledger / Step 2 | IMPLEMENTED / LOCAL PASS | Four original conditions, incomplete/conflicting evidence rejection, no fabricated fallback, both active-end directions in synthetic fixtures, original-state preservation. Production node identity/order evidence remains partial. |
| Completion/surface guards | IMPLEMENTED / LOCAL PASS | Reporting phase, completion and synthetic calibrated-depth guards; missing/invalid/stale depth, excursion and failure cases covered. Defensible physical water datum is unresolved. |
| Durable reporting/recovery | IMPLEMENTED / LOCAL PASS | Private immutable submission, journal states, eligible replay, run/epoch publication gate, transactional authority replacement, terminal recovery and duplicate/conflict handling. No claim of receiver-confirmed/exactly-once delivery. |
| Runtime parameter integrity | IMPLEMENTED / LOCAL PASS | Side-effect-free validation and post-set application; rejected atomic updates no longer publish pending reports. |
| Production ROS boundary | IMPLEMENTED / LOCAL PASS, QUALIFIED | Actual BanyuMission/comm/adapter callbacks tested. Reduced BT, sensor/action/readiness fixtures and accelerated timing; qualifications in §5 remain. |
| Task 4 listener scope | IMPLEMENTED / LOCAL PASS | Assistance ACK and unsupported/not-ready diagnostic; no premature readiness. Full transit/hold/confirmation/avoidance is NOT IMPLEMENTED in the tested stub. |
| Topic separation | PARTIAL | Private submission is under `/banyu/...`; public reports use agreed `rx_msgs`. Actual domain-40/10 router isolation is NOT VERIFIED. |
| Course consumers | PARTIAL | Course/frequency stored and ledger updated. Navigation constraint and meaningful acoustic frequency consumer are NOT IMPLEMENTED in the inspected path. |
| Qualified buoy/end inputs | PARTIAL | Consumers exist; only test publishers of ActiveBuoyFix/PipelineEndCandidate were found. Production publisher/wiring is missing. |
| Physical node association | PARTIAL | Centerline/vehicle-pose projection exists. Calibrated node location, lateral bounds and ambiguous-segment rejection remain incomplete. |
| Water-datum provenance | PARTIAL | ACTIVE heartbeat no longer recalibrates datum. Pre-ACTIVE near-zero relative altitude may still auto-calibrate without independent surface proof. |
| Headless deadman/full SITL | NOT VERIFIED | Supervised simulation freshness and complete deployed Task 2 execution need evidence. Do not force hardware rc_fresh true. |
| Higher-tier repair/optical behavior | PARTIAL / HARDWARE PENDING | Select the supported tier explicitly; timer/image fixtures do not prove magnetic repair or actual light waveforms. |
| External RoboCommand receipt | JOINT PENDING | GCS/OCS owns MQTT/protobuf/official sequence and receiving persistence; not a local UUV implementation prerequisite. |

The issue permits Task 4 stub listeners in its checklist, while describing full incident behavior elsewhere. Record the accepted stub scope or a separate follow-up for full Task 4; do not silently claim full behavior. A communication integration milestone need not implement every perception algorithm, but must identify upstream contracts/dependencies and exercise the complete deployed reporting chain with honestly labeled inputs.

## 3. Last verified results and resolved history

The last local audit at the baseline freshly ran the focused build, registered checks and updated smoke. It did not run physical motion, full Gazebo, cross-domain routing, a race sanitizer or a new default-heartbeat timing measurement.

| Check | Result |
| --- | --- |
| Focused incremental build | Nine packages passed |
| CTest-registered C++ cases | **338 passed**, zero failures/errors |
| Aggregate colcon checks | **739**, zero failures/errors, **156 skipped** |
| Updated mock GCS smoke | **6/6 passed**, including ACK/diagnostic and absence of false readiness |
| Adapter / production boundary / comm node cases | **48 / 15 / 17 passed** |
| Working tree at final audit check | Clean; no source edits by the audit |

Registered C++ breakdown: communication 136, vehicle execution 74, mission 70, detection 42, guidance 8, vehicle interface 7, msgs 1; rx_msgs/bringup have no C++ cases in this count. Communication's 136 comprises adapter 48, production boundary 15, comm node 17, survey reporting 22, survey core 25, coordinator/comm integration 9.

Aggregate checks include linters and are not interchangeable with gtest cases. Skips are not executed passes; record their reasons in subsequent evidence. Count only CTest-registered result files, not obsolete XML. Older 330-case claims were inaccurate. Recompute counts after adding tests.

Audit artifact filenames were `banyu-issue13-50dbb01-{build,test,test-results,smoke}.log` and `banyu-issue13-50dbb01-local-audit.md`, stored in the system temporary directory. These may disappear. Archive future acceptance evidence durably with exact revision, commands, input fixtures and limitations; do not rely on temporary files alone.

| Earlier problem | Current disposition |
| --- | --- |
| Start bypass/invalid starts/duplicate reset and fabricated survey | Addressed in the tested lifecycle/ledger paths; do not reintroduce them. |
| Disabled forwarding falsely marked attempted; raw commands changed report ownership | Registered regressions pass. |
| Future-epoch publication under older authority; rejected replacement retired valid owner | Fixed and registered at 4d74257. |
| Rejected atomic parameter update leaked pending report | Fixed at 50dbb01 with validation/post-set callbacks; regression passes. |
| Immediate assistance readiness without transit | Replaced at 50dbb01 with a truthful unsupported/not-ready stub. |

Do not keep historical bugs listed as current failures. Likewise, do not convert a resolved fixture regression into proof of hardware or complete mission acceptance.

## 4. Authority, references and contracts

### Competition and team references

- [Handbook §3.4](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.4-communications-protocol): course/run/heartbeat/report semantics and OCS transport ownership.
- [Handbook §3.3.3](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.3-mission-task-2-infrastructure-survey-and-repair): Task 2 active buoy, survey order and selected-tier behavior.
- [Handbook §3.3.5](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.5-mission-task-4-dynamic-incident-response): Task 4 readiness/transit/hazards.
- [Task-build specifications, including §3.5.5](https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.5-task-build-specifications): pipeline assembly; five PVC sections do not imply five electronic nodes.
- [Pinned canonical RoboCommand schemas](https://github.com/robonation/robocommand/tree/f6457fa47489647a8650c1d3eb74efa84ce4b84a/RobotX_2026/proto/robotx): historical plan reference; recheck the competition-approved release before deployment.
- Installed team repository `../../RobotX-SoS-communication-ITSN` relative to this document (`../RobotX-SoS-communication-ITSN` from the repository root), previously checked at `2946a98a905e2e8fac779b4c02316c0e9dcd8f87`: team ROS fields/topics/domains, not the official external transport.
- Supplied `UNDERWATER_MAGNETIC_ACTIVATED_LIGHT.md`, pipeline image and human Q&A export are external reference files, not tracked in this repository. Their reference directory was unavailable in the latest inspection; locate originals before new hardware-specific decisions. The original export was `[DiscordKit] Task 2_ Infrastructure Survey & Repair Questions_RoboNation Community_20260930_170619.json`.

Provenance retained from the approved reconciliation: [David, September 28](https://discord.com/channels/969707043218935848/1437482903599841373/1554246751391846572) supports four nodes; [Aamir, July 6](https://discord.com/channels/969707043218935848/1437482903599841373/1523760286899896451) supports upward mounting; [February 2](https://discord.com/channels/969707043218935848/1437482903599841373/1467909205221708022) concerns the original survey; [August 3](https://discord.com/channels/969707043218935848/1437482903599841373/1533906508264374394) concerns random spacing; [September 2](https://discord.com/channels/969707043218935848/1437482903599841373/1544719226516930561) anticipates 1–3 s magnetic dwell but says timing was not finalized. An older hardware summary/prototype conflicts on timing; do not promote its values to final competition requirements.

Read original references when making new rule-dependent decisions. State whether a requirement is official, a shared team contract or a BANYU engineering policy. Do not implement from the issue description alone or invent absent payload fields.

### Responsibility boundary

| Owner | Work |
| --- | --- |
| UUV developer/AI agent | Autonomy, validation/consumers, private evidence/lifecycle/outbox, agreed public ROS interfaces, local software/simulation checks |
| GCS developer/OCS | External MQTT/protobuf, declaration validation, official envelopes/sequences/vehicle attribution, persistence and RoboCommand receipt |
| Joint | Shared schema/QoS, DDS allowlist, receipt/retry/run attribution, Task 4 confirmation mapping |
| Human with equipment | Receiver/profile/calibration measurements, physical sensor/repair/motion validation |

No OCS access is required to finish local UUV software or mock-GCS routing. Do not require GCS to consume private Task2Submission. Stronger receipt or atomic cross-run identity requires a coordinated shared-interface change, not a unilateral custom public envelope.

### Public ROS contract

| Topic | Type / direction relative to UUV |
| --- | --- |
| `/system/vehicle/uuv/heartbeat` | `rx_msgs/Heartbeat`, publish |
| `/system/mission/status` | `rx_msgs/MissionStatus`, publish |
| `/system/mission/command` | `rx_msgs/Command`, subscribe |
| `/system/mission/course` | `rx_msgs/Course`, subscribe |
| `/system/vehicle/uuv/task2/pipeline_survey` | `rx_msgs/PipelineSurvey`, publish |
| `/system/vehicle/uuv/task2/delivery` | `rx_msgs/ResourceDelivery`, publish |
| `/system/vehicle/uuv/task4/incident_ack` | `rx_msgs/IncidentAck`, publish |
| `/system/vehicle/uuv/task4/readiness` | `rx_msgs/ReadinessReport`, publish only after genuine readiness; current stub emits none |

UUV domain 40, GCS 10, USV 20, UAV 30 are the team convention; isolated local tests use domain 42. `/banyu/...`, MAVROS, camera and TF stay private. Topic names alone do not prove router isolation.

### Preserve these rules

1. One mission-owned RunCoordinator accepts/transitions runs. Communication observes reliable recoverable lifecycle, not raw-command ownership. Filter TYPE_UUV/TYPE_UNKNOWN; installed VehicleType has no TYPE_ALL.
2. Nonzero run_id is BANYU policy, not an explicit universal uint32 prohibition. Duplicate accepted starts—including after termination—do not reset observations, restart the BT or resurrect tasks. Reject invalid/conflicting/stale/unaddressed commands without mission-data mutation.
3. Pre-run operator-authorized preparation and position hold precede run motion. Accept context, reset current-epoch consumers once, and permit motion only after their acknowledgements. Task 2 local Trigger cannot bypass GCS start; preserve POR's documented local start.
4. Abort/stop/failure/readiness loss cancels autonomy, invalidates stale callbacks, preserves diagnostic evidence and publishes TASK_NONE. Do not use TASK_UNKNOWN or a literal zero as terminal task. Abort fallback is not proof of disarming; respect pilot takeover.
5. Four distinct initial RED/GREEN observations form the survey; preserve the original damaged node after repair. UNKNOWN/incomplete evidence fails the complete BANYU report. No fallback `[1,1,1,2]`, coercion to one-red distribution, or fifth electronic node from PVC count.
6. Order along pipeline topology from the confirmed active green buoy. Global distance sorting fails on bends. Course does not supply active-buoy coordinates; use separate qualified observation with provenance/uncertainty.
7. Reporting requires matching accepted identity, survey/selected-tier completion and current surface milestone plus fresh calibrated sustained depth. This verified-surface guard is BANYU policy. Final BT SUCCESS cannot be required before its own report action finishes.
8. Durable local acceptance, PENDING, ATTEMPTED and suppression are distinct. ATTEMPTED is not receipt. Preserve journal migration/storage diagnostics and conservative old-run recovery; public PipelineSurvey/ResourceDelivery lack atomic run/report identity.
9. Core permits survey-only completion. Higher selected tiers need their repair/resource evidence: completed-repair green 5 s precedes optical coding; Advanced color 1 s/OFF 1 s, Disruptive color1 1 s/OFF 1 s/color2 1 s/OFF 2 s, for 30 s. Advanced requests ANY resource/observed destination; Disruptive uses first/second colors. Magnetic dwell must be continuously evidenced and its final threshold verified separately.
10. Full Task 4 requires transit/hold before readiness and correct confirmation before resume. Readiness command sequence differs from official OCS report sequence. Keep-out persists until all-clear/run end; moving-object avoidance does not invent an ACK. The current listener stub is not that implementation.

## 5. Current problems and evidence limitations

| Problem | Why it matters | Correction |
| --- | --- | --- |
| Test fixture reads/writes `depth_datum_` on different threads without shared locking | A passing ordinary run does not prove thread safety | Synchronize all accesses or use immutable telemetry snapshots; optionally run a suitable race check. |
| Terminal-heartbeat assertions can accept an older TASK_NONE sample | Preflight data can masquerade as post-failure evidence | Require a newly received, run-matching sample after a recorded failure cutoff. |
| Wrong-epoch recovery test stops BanyuMission and injects matching lifecycle | It proves adapter recovery with fixture authority, not the full production transition | Disclose the substitute; add a real accepted-context transition case when claiming mission provenance. |
| External veto test's newly registered callback runs first | It does not isolate a veto after adapter validation | Control validator order if claiming that exact sequence. Post-set implementation itself is correct. |
| Course only stored; no effective navigation/acoustic consumer | Data has no required behavioral effect | Implement §6.2–6.4; assert actual motion/processing output. |
| Missing qualified buoy/end producers | Consumers cannot establish production completion from raw observations | Wire an upstream contract/producer; use labeled simulation equivalents now. |
| Vehicle pose projected onto configured centerline certifies identity/order | Projection alone cannot establish physical node location | Add calibrated association, distance/ambiguity bounds and provenance. |
| Near-zero pre-ACTIVE relative altitude can establish datum | Relative zero can be underwater startup, not water surface | Explicit validated calibration operation/profile with identity and validity. |
| Reduced test BT and accelerated timing | Callback proof does not prove deployed navigation/repair/physics or production timing | Run full selected Task 2 composition and capture default-rate/hold evidence. |

These test/evidence qualifications do not reproduce a new production atomic-update failure at 50dbb01. Do not label the entire stack broken, or all acceptance gates passed, from the same limited results.

## 6. Next software work, in implementation order

### 6.1 Fix fixture reliability first

Targets: [production mission boundary tests](../banyu_communication/test/test_production_mission_boundary.cpp) and existing adapter tests. Protect shared strings/state; strengthen heartbeat identity/time predicates; retain real seal service and successful-abort-before-response assertions. Preserve all passing epoch/authority/restart/atomic-update/Task 4 cases. Pass when the intended guards and terminal evidence are independently observed without fixture races or provenance overclaims.

### 6.2 Validate and distribute accepted Course

Current schema:

```text
string course_id
uint32 pinger_freq_hz
rx_msgs/LatLng[] boundary
rx_msgs/LatLng[] uav_geofence
```

Targets: [communication callback](../banyu_communication/src/uuv_comm_node.cpp), an appropriate reusable context/validation component, mission readiness and existing consumers. Search the current workspace before adding a new node/package; avoid dependency cycles.

1. Validate nonempty identity, finite/ranged coordinates, at least three usable distinct vertices, closed nondegenerate usable polygon and supported positive frequency. Document engineering geometry/support limits; do not silently treat the UAV polygon as the UUV boundary.
2. Keep immutable accepted context; invalid candidates diagnose rejection without overwriting it. Distinguish old accepted context from a newly rejected candidate in readiness/status.
3. Bind a validated snapshot to the authoritative run. Define identical replay, pre-run replacement and incompatible active-run update policy; do not silently switch pinned reporting/navigation context.
4. Course has no ROS header/run/version. Internal receipt time/revision is local bookkeeping, not official acquisition metadata. Do not fabricate buoy position or bearing.
5. Distribute via compatible direct subscriptions or a private validated snapshot. Inspect actual GCS ROS QoS: retained MQTT does not imply transient-local ROS. Test late consumers/restarts.

Pass: a valid course is applied by actual consumers; invalid/unsupported input cannot produce falsely ready configuration; update/restart behavior is deterministic.

### 6.3 Enforce boundary in navigation and actual motion

Targets: [GPS navigation server](../banyu_vehicle_execution/src/action_servers/navigate_surface_gps_server.cpp), [execution arbitration](../banyu_vehicle_execution/src/vehicle_execution_node.cpp), guidance and tests. Goal validation currently lacks the course polygon constraint.

1. Use consistent geodetic/local horizontal frames and compatible existing conversions. Validate coordinates and document origin/valid extent.
2. Reject outside-boundary targets and invalid transit segments. Inside endpoints can still connect through an outside part of a concave polygon.
3. Constrain actual selected autonomous horizontal intent—including pipeline following—so other action/intent sources cannot bypass enforcement. Subscribe to execution/MAVROS output in acceptance tests.
4. Define a bounded prediction/constraint policy, margins and uncertainty behavior. Document engineering/simulation settings rather than inventing competition tolerances.
5. Handle stale localization, boundary contact and outside-at-start explicitly. Do not command an unvalidated recovery route from unknown position.
6. Preserve operator/abort/watchdog priority and necessary vertical emergency surfacing. Keep velocity-only mask `0x5C7`, explicit yaw rate, ROS ENU/MAVROS NED conversion, continuous output and GUIDED arbitration.

Pass: valid inside motion succeeds; outside goal/path and outward crossing intents are rejected/constrained at the real output under the selected policy, with clear diagnostics and no safety bypass.

### 6.4 Apply pinger frequency to meaningful acoustic processing

No acoustic frontend/homing source was found in the inspected tracked path; search again first. Integrate an existing frontend if available. Otherwise implement a small production C++ processing component and labeled synthetic sample source.

1. Define raw samples, rate, channel layout, units/timing and a configurable capability profile. Keep unknown real hardware unconfigured; use an explicit simulation profile now.
2. Apply accepted Course frequency to real detection/filter/selection calculations. Changing a variable or unused topic alone does not pass.
3. Enforce profile/sampling limits, including Nyquist/guard-band requirements of the selected algorithm. Diagnose unsupported frequency and malformed/stale/missing samples.
4. Choose implementation from available libraries/primary DSP documentation; do not guess a real receiver sample rate or response band.
5. Test target/off-target/mixed/noise/silence buffers and changed frequency using actual computed outputs. Apply changes through synchronized immutable snapshots or defined buffer boundaries.
6. Keep frequency detection separate from bearing/homing. One-channel signal strength does not establish direction or buoy coordinates. Directional homing requires a calibrated sensor/array contract; software may test labeled synthetic directional observations without claiming physical localization.

Pass: frequency alters meaningful production processing and invalid input remains not-ready. Real underwater homing remains a separate hardware/software capability, not inferred from this milestone.

### 6.5 Finish upstream evidence, physical association and datum contracts

Targets: `survey_evidence_node.cpp`, `pipeline_end_node.cpp`, `ring_tracker.cpp`/node, `calibrated_depth_node.cpp`, their producers/config/launch.

1. Wire qualified ActiveBuoyFix and run-tagged PipelineEndCandidate producers. Active-end evidence includes observed green/pinger qualification, position, source/time/course/run association and uncertainty. Detection loss, elapsed travel or four counted nodes must not manufacture an end.
2. If another vehicle/GCS supplies observations, agree its public contract; adapt privately. Build simulation providers through the same declared interfaces instead of preassembling a successful report in the test.
3. Establish calibrated node association or a bounded, validated centered-observation method. Reject far-off projections and ambiguous bends/overlapping segments. Preserve identity through backtracking/reacquisition and original RED after repair.
4. Configure centerline frame/origin/source/calibration; fixed world geometry is simulation input, not actual node measurement. Order exactly four known originals from either active endpoint along topology.
5. Implement explicit water-datum calibration/configuration with source, offset, identity and validity. Underwater startup/restart and repeated lifecycle cannot establish surface merely from map/relative-altitude zero.
6. Keep Surface action/guard depth semantics consistent, with current-epoch completion and fresh continuous water-relative hold. Paused ROS time must not retain stale authorization indefinitely.

Pass software: raw/labeled upstream fixtures traverse the deployed consumers and guards; missing/conflicting/stale/wrong-run/uncalibrated input blocks reports. Physical sensor/calibration accuracy remains separately pending. New perception algorithms may be upstream follow-ups if the communication scope and dependency are explicitly documented.

### 6.6 Integrate readiness, launch and supervised headless safety

Readiness must reflect applied accepted consumer configuration, not merely arrival of a Course message. Define required capabilities per mode/tier; preserve POR and do not fabricate acoustic/physical readiness from a simulation profile.

Preserve the BT's plain memory Sequence for mission phases with reactive interruption at the root. Never modify the user-configured front-camera sensor block as part of this communication work.

Update every new node/message/parameter's package dependencies, CMake/interface generation/installation, matching YAML section and launch forwarding. Simulation sample/evidence sources launch only in simulation/test mode. Retain one lifecycle mutator, private/public separation, observations and durable seal identities.

Headless SITL needs an explicit simulation-only supervised freshness source selected by use_sim. Test source expiry/loss and hardware configuration refusing simulation authority. Never globally force rc_fresh=true, disable the watchdog or let simulation authorization leak into use_sim=False. Inspect existing RC/SITL configuration before selecting the source; do not blindly publish a gamepad stream as production safety.

Pass: valid configured consumers permit the selected pre-run hold/start path; unsupported inputs produce diagnostics; supervised simulation preflight progresses and stale-source faults remain effective.

### 6.7 Complete full simulation and DDS acceptance

Inspect `mission.launch.py` and `main_launch.launch.py` arguments. After wiring is ready, run the actual Task 2 BT with the selected tier in Gazebo/ArduSub SITL, UUV domain 40. Distinguish deterministic `task2_sim_provider.py`/fixture_route inputs from sensor/physics models; a direct evidence injection is software integration evidence, not physical sensor proof.

Capture: validated course/consumer configuration, pre-run hold, accepted start/run/epoch, action progression, four original observations/active end/completion, calibrated depth/hold, report payload/journal, terminal TASK_NONE and velocity outputs. Measure default 2 Hz heartbeat for at least 30 s and report jitter separately. Include abort/failure/restart variants and supervised deadman loss.

Separately run local mock GCS domain 10 with allowlisted routing to UUV 40. Verify course/commands/reports and QoS/late-reader behavior; prove private banyu/MAVROS/camera/TF exclusion. Domain-42 smoke alone does not pass this gate. Only stop identified test-owned stale processes; do not operate attached real hardware as a simulation test.

Pass: complete deployed mission/report behavior in the stated model plus real two-domain public delivery/private exclusion. Record missing sensor models explicitly. External OCS serialization/receipt is still a joint test.

### 6.8 Task 4 scope and higher-tier follow-ups

Preserve current truthful assistance ACK/diagnostic/no-readiness behavior and smoke expectations. If full Task 4 is selected, implement safe interruption, actual assistance transit/hold, readiness and correctly correlated confirmation/resume. Keep-out/all-clear and moving-object avoidance need separate constraints/tests; do not treat ACK as avoidance. Coordinate official report-sequence mapping with GCS.

For selected Advanced/Disruptive Task 2, validate continuous magnetic activation, completed-repair confirmation and optical decoding separately from original survey. Core-only software acceptance must not be blocked by undeclared optional tiers; full competition compliance must not be claimed without their measured evidence.

## 7. Acceptance checklist

Each row is an outcome to demonstrate, not a claim that it already passes. Preserve earlier passing cases while extending coverage.

| Gate group | Required probes / evidence |
| --- | --- |
| L: lifecycle | Direct Task 2 start blocked; zero/wrong-target/stale/conflicting/duplicate starts preserve state; current-epoch reset acknowledgement precedes motion; authoritative restart; readiness/hold before start; abort/stop/action/BT failure/takeover clears task and rejects late callbacks. |
| S: survey | Four original conditions with red at each position and both ends; zero/partial/duplicate/ambiguous/BLUE/stale evidence rejected; bent topology/revisits; original RED preserved after repair; qualified buoy/end; missing completion cannot report. |
| S: surfacing | Missing/NaN/stale/future/wrong-datum/uncalibrated/single-sample/excursion/paused-clock cases blocked; independently eligible survey plus current Surface completion and sustained depth permits reporting. Emergency surface after failed run does not authorize scoring. |
| S: transaction | Concurrent/repeated requests, conflicting payload/reused report ID, abort before/after durable seal, delayed response/new run, journal/storage failure, pending restart/late receiver and conservative old-run replay. No false receipt or cross-run attribution. |
| C: course | Closed valid/invalid polygons/frequencies, duplicate/changed course, mid-run policy, retained/late/restarted consumers, actual inside/outside/concave path and velocity constraints, frame tests, processing output changed by frequency. |
| C: acoustic/evidence | Synthetic target/off-target/mixed/noise/silence, unsupported profile/Nyquist/malformed/stale data, separate directional evidence, Course alone provides no buoy, qualified/stale/conflicting observation behavior. |
| D: simulation safety | Supervised source fresh/stale/missing; hardware rejects simulation authorization; deadman/FCU/mode/takeover loss cancels motion; ENU/0x5C7/yaw-rate continuity and depth hold preserved. |
| T4: selected scope | Stub ACK/diagnostic and no readiness; if full scope: arrival/hold, correct/wrong confirmation, duplicate/stale commands, aborted transit, keep-out/all-clear and moving hazards. |
| W/E: integration | Default heartbeat/telemetry correctness, complete selected Task 2 deployed BT in domain 40, actual public-domain-10 delivery/private exclusion, and separately agreed GCS/OCS receipt. |
| R: selected physical tiers | Recorded probe/magnet geometry and continuous activation, actual completed state and correct optical waveforms/colors; original survey unchanged. Never validate competition waveforms from conflicting prototype firmware alone. |

Use pure C++ tests for context/geometry/processing and isolated rclcpp fixtures for real callbacks/output. A successful test must reach its intended guard; incomplete survey cannot be used to prove an independent depth rejection. Use exact registered names, bounded waits, test-owned resources and synchronized fixtures.

## 8. Build, verify and record results

Check HEAD/worktree first. Markdown file links are relative to this document. Shell paths below assume you start at the repository root: the workspace is `../..`, and this repository is `src/BANYU_ROBOTX` from the workspace. The inspected environment uses ROS Jazzy/C++17/BT4. Older Humble/path text in AGENTS.md must be reconciled with the agreed environment; do not silently change distro. Start in a Bash shell with your installed ROS 2 Jazzy underlay already sourced.

```bash
# Start at the BANYU_ROBOTX repository root.
git status --short
git rev-parse HEAD

cd ../..
colcon build --packages-select rx_msgs banyu_msgs
source install/setup.bash
colcon build --packages-select \
  banyu_guidance banyu_vehicle_interface banyu_vehicle_execution \
  banyu_communication banyu_mission banyu_detection banyu_bringup
source install/setup.bash

ROS_DOMAIN_ID=42 colcon test --executor sequential \
  --packages-select rx_msgs banyu_msgs banyu_guidance banyu_vehicle_interface \
    banyu_vehicle_execution banyu_communication banyu_mission \
    banyu_detection banyu_bringup \
  --ctest-args -j1 --return-code-on-test-failure
colcon test-result --test-result-base build --verbose
python3 src/BANYU_ROBOTX/banyu_communication/test/sos_gcs_mock.py
ROS_DOMAIN_ID=42 ros2 launch banyu_bringup mission.launch.py --show-args
ROS_DOMAIN_ID=42 ros2 launch banyu_bringup main_launch.launch.py --show-args

cd src/BANYU_ROBOTX
git diff --check
git status --short
```

Add new packages if the design requires them. Run applicable focused tests first, then the registered stack/smoke; suites sharing a domain run sequentially. Source-built interfaces precede dependents. Launch --show-args is inspection, not execution. Label incremental versus clean builds and do not reuse stale XML after changes.

Record exact revision/environment, commands, registered counts/skips, inputs, observed outputs, timing and test doubles. Include authoritative transitions, reports, setpoints and diagnostics—not private counters alone. Keep accepted evidence after an abort/restart/storage fault; never delete journals to hide recovery failures.

After each milestone, update **this document** with its status/evidence. Keep historical baseline results labeled. Do not create another competing Issue 13 plan/verification file. Machine logs/fixtures may be archived separately with links and a clear scope.

## 9. What can be completed without the robot

| Work | AI/software now | Human/equipment later |
| --- | --- | --- |
| Course, boundary and readiness | Validation/context, constraints, mocks/SITL and actual software-output tests | Applicable frame/localization uncertainty and vehicle response measurements |
| Acoustic processing | Configurable simulation profile, synthetic signals, receiver interface and processing tests | Receiver/ADC band/sample rate, real recordings and underwater response |
| Directional homing | Contract and labeled directional simulation | Array geometry, clock/orientation calibration and actual bearing measurements |
| Survey evidence/order | Producer contracts, fixture/recorded-image path, ambiguity/revisit/topology tests | Camera/localization calibration and real buoy/light observations |
| Surface datum | Explicit calibration/validity contract and synthetic depth tests | Water-surface pressure/offset calibration and physical dive/resurface evidence |
| Full mission/routing | Robot-free SITL and local mock-domain router tests, subject to available models/tools | Real vehicle motion and joint deployed GCS/OCS receipt |
| Repair/optical tiers | State machine/waveform fixtures and original-survey preservation | Probe/magnet geometry, actual activation threshold and light code behavior |

If hardware specifications are absent, keep real capability unconfigured/not-ready and complete a clearly labeled simulation profile. Do not fabricate bearing, calibration, coordinates or success flags. GCS access and a physical robot are not reasons to defer independent software work.

## 10. Issue closure and agent assignment

Issue 13 closure needs its agreed software acceptance: shared interface migration, correct start/hold/lifecycle, production heartbeat, valid guarded report through the deployed selected Task 2 path, course-consumer behavior, full domain-40 simulation evidence, explicit Task 4 stub/full disposition and the agreed DDS boundary test. Upstream perception scope/dependencies must be explicit. Do not force every new sensing algorithm, optional tier or external OCS backend into this communication issue without agreement.

Hardware calibration/performance and full team competition/OCS acceptance are separately tracked milestones. Simulation with declared synthetic sensors can establish software integration before the robot exists; it cannot establish physical performance.

**Next coding assignment:**

> Read AGENTS.md and docs/issue13.md; check HEAD/worktree and preserve passing behavior. Fix fixture synchronization and fresh terminal-heartbeat assertions first. Implement validated, run-associated Course context, actual navigation boundary enforcement and meaningful acoustic frequency processing with labeled synthetic samples. Preserve ENU velocity streaming, GUIDED/deadman safety, authoritative lifecycle, public rx_msgs and private submissions. Add matching YAML/launch/install/dependencies and the acceptance probes in §7. Continue qualified upstream contracts/fixtures, bounded node association and explicit depth-datum validity, then supervised headless SITL/full Task 2 and domain-40/10 routing. Use unknown hardware profiles as not-ready rather than inventing specifications. Run isolated focused/stack/smoke checks, archive fresh evidence and update only this Issue 13 document. Report software, simulation, hardware and GCS/joint status separately; no OCS credentials or finished robot are required for the local software assignment.
