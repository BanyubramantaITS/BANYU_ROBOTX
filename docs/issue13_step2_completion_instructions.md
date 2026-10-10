# Issue 13 — instructions for completing Step 2

Latest baseline `886e17f`: start with [the next-step instructions](issue13_next_steps_886e17f.md). The earlier unsent-report replay case now passes, but three new adapter probes fail. Do not repeat completed fixes or interpret the historical findings below as the current verification record.

Current instructions: [corrected ownership and remaining UUV work](issue13_uuv_corrections_and_remaining_work.md), reviewed at `92cd50e`. The `588e211` findings below are historical: the three original regressions now pass. Preserve their coverage, and address the newly reproduced journal-before-publication recovery failure. GCS/OCS backend implementation is owned by the GCS developer; it is not a prerequisite for local UUV verification.

Baseline: local commit `588e211`, reviewed on October 10, 2026. This is an implementation checklist, not a completion claim. It follows Step 2 and acceptance gates S1–S8 in [the approved plan](issue13_rules_and_implementation_plan.md). The local audit is `/tmp/banyu-issue13-local-audit-588e211.md`; copy essential evidence into a durable verification record because `/tmp` is temporary.

The baseline builds all eight focused packages and passes 259 registered C++ cases and six communication smoke checks. The five earlier supplemental probes now pass. Three further probes fail: ACTIVE-heartbeat calibration, adapter restart deduplication, and recovery of a durably sealed report after abort.

## 1. Register the remaining failures before changing behavior

Add the ACTIVE calibration case to `banyu_vehicle_execution/test/test_calibrated_depth_node.cpp`, and the retained-report restart case to `banyu_communication/test/test_gcs_submission_adapter.cpp`.

Add a report transaction integration test using both real owners. Prefer a dedicated test package depending on mission and communication if adding a test dependency inside either package would create a cycle. Expose node library targets where necessary; avoid production dependency cycles. Start with the real `RunCoordinator` journal and `UuvCommNode`, then extend coverage to the actual mission ROS node and service callbacks.

Reproduce these sequences:

| Probe | Sequence | Required result |
| --- | --- | --- |
| ACTIVE calibration | Start ACTIVE, publish relative altitude zero, then publish another ACTIVE lifecycle heartbeat without a surface-calibration authorization | Calibration remains unproven; no surfacing/report authorization |
| Adapter restart | Publish a sealed report, receive it, restart the local adapter while its publisher retains the report | Selected local identity deduplication is preserved; separately test an unsent pending report and late receiver. No claim about official sequence uniqueness. |
| Seal then abort | Persist the report seal while Surfaced; delay its response; abort; deliver the response; restart comm | Durable acceptance and delivery intent remain recoverable under the original run identity |

Preserve the already passing original-condition, mixed-snapshot, single-sample, underwater-startup, and unsealed-submission regressions.

The supplemental sources and logs are named `/tmp/banyu-issue13-588e211-test_calibrated_depth_node.*`, `/tmp/banyu-issue13-588e211-test_gcs_submission_adapter.*`, and `/tmp/banyu-issue13-588e211-seal-recovery.*`.

## 2. Make water-surface calibration explicit and consistent

Targets: `banyu_vehicle_execution/src/calibrated_depth_node.cpp`, its header/tests, and its section in `banyu_bringup/config/task2_params.yaml`.

Currently `handleRelAlt()` blocks initial automatic calibration during ACTIVE, but `handleLifecycle()` enables it from a near-zero sample. Remove this second authorization path. Neither callback may infer a water-surface datum merely from relative altitude, map z, lifecycle phase, or a heartbeat.

Separate three states: accepted run context, valid depth telemetry, and established water-surface calibration. A run/epoch change updates the first; it must not silently overwrite or invalidate the physical datum without a documented calibration policy.

Use an explicit calibration operation supported by verified surface evidence, or a validated configured/persisted datum. If a new private service is needed, define its provenance and authorization requirements before adding it. A caller asserting `calibrated=true` is insufficient verification by itself.

Record the measured offset, source/datum, calibration identity, validity conditions, and evidence used to establish it. Validate finite values. Restart recovery must either restore a still-valid calibration or remain uncalibrated; it must never treat the current underwater position as a new surface.

If only headerless Float64 relative altitude is available, distinguish receipt time from sensor acquisition time. Do not manufacture a sensor timestamp or imply source freshness that the input cannot establish. Select and document an adequate stamped telemetry source for the final acceptance proof.

Keep water-relative depth separate from navigation depth. Preserve existing ENU velocity control and execution safety gating.

Acceptance: underwater startup/restart, zero relative altitude without calibration evidence, repeated ACTIVE heartbeats, invalid/stale inputs, changed calibration identity, and run changes must not open the surface guard. A verified calibration followed by descent and sustained resurfacing must work. Ensure both the Surface action and independent comm guard consume the same declared datum semantics.

## 3. Reconcile the mission seal and communication outbox

Targets: `banyu_communication/src/uuv_comm_node.cpp`, `survey_reporting.cpp/.hpp`, `banyu_mission/src/run_coordinator.cpp`, and their transaction tests.

Use one explicit acceptance rule: successful durable mission sealing is the acceptance point. Abort can prevent a new seal, but cannot erase a report already accepted before it. This is an engineering transaction policy; document it alongside the service response contract.

| Event ordering | Required behavior |
| --- | --- |
| Abort wins before seal | Reject submission; do not publish a successful report |
| Durable seal wins before abort | Preserve accepted report/delivery intent and original identity; abort remains terminal |
| New run begins while an old response is delayed | Reconcile the old report under its old identity; do not change the new run's task or ledger |
| Response is lost or comm crashes | Query/reconcile durable authority state; absence of a response is not proof of rejection |

Do not simply remove the abort cleanup or authorize every staged report. A staged report might never have been sealed. Replace unconditional `unstage()` on abort with reconciliation that distinguishes accepted, rejected, and unresolved transactions.

Provide an authoritative way to recover a seal for the exact `(run_id, epoch, kind, report_id, immutable payload)` identity. If a private query service or retained seal record is introduced, bind it to authority identity and durable journal state. Reuse an accepted immutable transaction on retry; reject conflicting payloads.

Preserve pending service waiters until they receive a defined response or a bounded timeout. Clearing the pending map must not strand service requests. A timeout must describe unresolved acceptance honestly and leave enough durable state for recovery.

A positive delayed response must not reactivate the mission, restore current_task, authorize motion, or place old observations in a newer epoch. Define response semantics consistently: locally accepted reports remain locally accepted even if the mission subsequently terminates; report terminal mission status separately.

Test crashes before/after staging, before/after durable seal, before/after response, and before/after publication. Verify disk state after restart, not only messages already present in the fixture's receive vector. Also test required SURVEY/DELIVERY pairs: partial acceptance must remain explicit and recoverable rather than being silently called complete.

## 4. Finish local publication recovery and agree the GCS boundary

UUV targets: `banyu_communication/src/gcs_submission_adapter.cpp`, its header/tests, local outbox interfaces and launch/configuration. The external GCS consumer is a separate developer's responsibility.

Keep `Task2Submission` private to the UUV, for example under `/banyu/communication/task2_submission`. Preserve its local identity and immutable payload. `rx_msgs/PipelineSurvey` and `rx_msgs/ResourceDelivery` are the agreed public reports, not legacy copies. Do not require the GCS to consume a custom `/system/...` envelope without a shared-interface agreement.

Use one production conversion path to those public types. Do not reconstruct missing report identity from MissionStatus arrival order or frame_id. Document the schemas' lack of run/report identifiers as a joint integration limitation; keep official sequence allocation and protobuf/MQTT forwarding in the GCS.

Verify the seal against the trusted authority/context. A nonzero `authorization_revision` is a useful field check but does not independently establish acceptance. Scope deduplication to the complete identity and immutable payload, while enforcing the chosen uniqueness rule for report_id.

Persist local acceptance, payload and recoverable publication intent. At `92cd50e`, acceptance journaling precedes publication and restored acceptance suppresses replay, so a crash can permanently lose an unsent report. Register that regression and fix recovery. Check storage errors and distinguish publication attempted from receiver receipt.

Local acceptance measures durable recoverability and correct public messages received by a mock under a documented replay policy. An unconditional no-second-publication assertion can hide report loss. Receiver-side logical deduplication and official sequence reuse require GCS cooperation; do not claim them from the local adapter alone.

Keep service success defined as durable local acceptance. If team delivery receipts are introduced, document their authority, identity fields, and whether they mean GCS durable receipt or onward publication. They are not an official PipelineSurvey scoring ACK.

Test duplicate/conflicting payloads, journal-before-publish recovery, storage failures, restart/late subscribers, unsealed submissions and stale-run replay. Verify delivery separately from survey. Preserve accepted old-run data without silently forwarding it as the new run. Record joint contract questions without requiring OCS access to finish local work.

## 5. Supply real physical identity and pipeline topology

Targets: `banyu_detection/src/ring_tracker.cpp`, `ring_tracker_node.cpp`, their headers/tests, `survey_evidence_node.cpp`, and message definitions if necessary.

Remove unconditional `physical_identity_verified=true`. An encounter counter, color, or temporal debounce alone cannot establish that two observations came from different physical light nodes.

Implement stable association using calibrated perception/localization and validated pipeline geometry. Keep node identity stable during forward traversal, backward revisits, loss/reacquisition, and post-repair color changes. Preserve the first qualified original condition.

Implement progression along the pipeline centerline/topology. Total vehicle travel distance is not pipeline arc progression: backtracking or looping increases travel distance without discovering a new node. Do not substitute Euclidean distance from the buoy, record count, vehicle pose, or an arbitrary increasing counter.

At `92cd50e`, centerline progression is wired into `RingTrackerNode`. Validate its frame, geometry and physical association; the earlier missing-`has_path_distance` finding is historical. Set verification flags only after their supporting checks succeed. Far-off projections and ambiguous association/topology must remain unverified and block reporting.

Verify the actual detector/tracker/aggregator path rather than injecting records with verification flags already set. Use deterministic source-level fixtures with known geometry, both active endpoints, a bent/U-shaped pipe, reverse revisits, missed frames, and overlapping observations. A revisit must reuse a physical identity and must not manufacture a fifth node.

## 6. Connect observed active-buoy, course, and end evidence

Targets: `survey_evidence_node.cpp`, `pipeline_end_node.cpp`, their upstream producers, private message contracts, and launch/configuration.

Add or connect a real producer of `ActiveBuoyFix`. Use observed buoy position and truthful green/pinger evidence under the approved identification policy. Preserve source timestamp, course association, run context, uncertainty, revision, and physical endpoint association. Raw NavSatFix alone must not establish an active green buoy. Keep any raw-fix trust override disabled in production and explicit in YAML.

Associate the observed active buoy with the correct pipeline endpoint; do not always assign the low-progress endpoint. Freeze survey orientation once qualified original observations are pinned. Reject conflicting mid-run endpoint/course updates unless an explicit validated policy handles them.

Add or connect a real run-scoped `PipelineEndCandidate` producer. End confirmation must follow observed task geometry; ordinary pipeline loss or simply collecting four records is insufficient. Match current run/epoch, source time, and confirmation continuity.

Make validated course context recoverable for late consumers. Ensure the publisher offers and the subscribers request compatible retained QoS, then test restart without republishing the course. Because the installed Course has no header, define trusted version/freshness handling explicitly; do not invent acquisition metadata.

Keep full geofence/acoustic homing implementation and hardware repair/optical decoding tracked under Step 4. Step 2 must consume truthful contracts and reject missing required evidence. A Core-only nominal test does not establish Advanced/Disruptive completion: keep higher-tier gates closed until their independent producers are implemented and verified.

## 7. Verify the actual reporting sequence

Use real mission, tracker/evidence, comm, depth, execution, and agreed submission-consumer nodes in a registered integration fixture. Mock only external sensor/FCU/GCS boundaries where necessary, and identify those substitutions in the results.

Nominal flow:

```text
Accepted run + consumer reset acknowledgments
  -> observed active endpoint + four qualified original nodes
  -> verified traversal completion
  -> survey_complete
  -> current-run Surface action + independent calibrated sustained depth
  -> report_ready
  -> durable seal/outbox acceptance
  -> agreed public rx_msgs received by a mock under the documented replay policy
  -> BT completion and task NONE
```

Preserve survey_complete before surfacing and report_ready afterward. Do not require final BT SUCCESS before the reporting service. Preserve velocity streaming, mask 0x5C7, ENU fields, explicit yaw rate, hardware RC protection, and run-authorized execution.

Negative cases must include direct Task 2 start, zero/duplicate/conflicting starts, incomplete/ambiguous survey, both endpoint directions, stale-run input, emergency surfacing on a failed run, paused clock, calibration loss, abort during sealing, consumer restarts, and delayed old-run replies. Duplicate accepted starts must preserve observations.

Use production heartbeat and surface settings for nominal integration. Pure tests may inject time; accelerated fixtures must not be labeled proof of production rates or physics. Fix receive-vector restart tests so old received messages cannot satisfy assertions about new replay.

## 8. Build, run, and record acceptance

Every new/renamed parameter needs matching YAML and launch forwarding. Every new node/interface needs installation and package dependency updates. Update POR sections only where the changed shared node behavior applies. Do not alter the user-configured front camera.

Run from the local workspace in bash:

```bash
cd /home/erlangga/Desktop/ros2_ws_robotx
source /opt/ros/jazzy/setup.bash

colcon build --packages-select banyu_msgs
source install/setup.bash

colcon build --packages-select \
  banyu_guidance banyu_vehicle_interface banyu_vehicle_execution \
  banyu_communication banyu_mission banyu_detection banyu_bringup
source install/setup.bash

ROS_DOMAIN_ID=42 colcon test --executor sequential \
  --packages-select banyu_msgs banyu_guidance banyu_vehicle_interface \
    banyu_vehicle_execution banyu_communication banyu_mission \
    banyu_detection banyu_bringup \
  --ctest-args -j1 --return-code-on-test-failure

colcon test-result --test-result-base build --verbose
```

Build/test any added integration package separately after its dependencies. Run the communication smoke script after registered tests, not concurrently on the same DDS domain. Its six checks alone remain insufficient acceptance evidence.

Create a durable verification record, suggested path `docs/issue13_step2_verification.md`. Record commit, configuration, commands, exact test counts, skipped gates, logs, actual nodes versus mocks, and S1–S8 results:

| Gate | Evidence required |
| --- | --- |
| S1 | Four original conditions; each possible RED position; both active endpoints |
| S2 | No filler or successful report for missing/unknown/ambiguous/stale evidence; valid evidence survives retry |
| S3 | Topology order from observed active endpoint; bent pipe and backward revisits |
| S4 | Original RED/GREEN frozen across repair and optical colors |
| S5 | Premature/failed-run surfacing and report calls rejected |
| S6 | Verified datum and sustained fresh depth plus current-run Surface success |
| S7 | Concurrent requests, abort/seal ordering, restart recovery, consistent responses and run identity |
| S8 | Durable local acceptance and recoverable public publication; GCS receipt/official attribution separately checked under an agreed contract |

Mark the appropriate hardware or joint-integration portion pending when evidence is unavailable. Do not convert an unperformed or mocked check into a pass, or label missing OCS access a UUV implementation defect. Steps 3–5, physics-backed E1, and official multi-vehicle E2 remain separate obligations with their respective owners.

## Completion gate

Local Step 2 closure requires passing registered regressions, real upstream identity/topology/buoy/end wiring, proven water-datum authorization, recoverable accepted reports and correct public publication to a mock. Receiver-confirmed delivery, official run attribution and GCS wire checks are separate joint/GCS milestones. Name the demonstrated local/Core subset explicitly and leave unperformed gates open.

Use the ordered work items in the current correction guide rather than repeating fixes already completed at `92cd50e`.
