# Issue 13: next implementation and verification steps after 886e17f

Current handoff: use [the 5f9932b AI coding agent instructions](issue13_next_steps_5f9932b.md). The three regressions described below now pass; the newer audit identifies two remaining adapter defects and the next production verification gates. This document retains its historical baseline.

Baseline: `886e17f00950cbab652956fcbed4c0b395589a6e`, reviewed October 10, 2026. These are instructions for future changes, not a claim that those changes exist. Use this guide first, then continue the broader [UUV correction guide](issue13_uuv_corrections_and_remaining_work.md). The [approved plan](issue13_rules_and_implementation_plan.md) remains the source for competition rules and safety constraints.

## 1. What is already done

Do not repeat completed work:

- The internal submission topic is `/banyu/communication/task2_submission`; public reports retain their agreed `rx_msgs` types.
- The adapter persists pending/attempted/suppressed publication states and active-run records.
- The original journal-before-first-publication recovery probe now passes for the registered replay path.
- The smoke test launches the adapter with direct report copies disabled and receives standard public reports.
- Fresh local build passed for nine focused packages. All 280 registered C++ cases and six smoke checks passed. The default heartbeat timer measured approximately 2 Hz over 30 seconds with checked synthetic telemetry fields passing.

Three additional probes failed. They identify two immediate defects: unauthorized run-context changes and false publication-attempt transitions. Fix these before claiming local recovery complete or spending effort on full mission acceptance.

The GCS developer owns OCS credentials, MQTT/protobuf, official sequences and RoboCommand forwarding. None of the immediate fixes requires OCS access. Keep Task2Submission private and retain the existing public ROS contract.

## 2. First commit: register the three failures

Targets: `banyu_communication/test/test_gcs_submission_adapter.cpp` and its existing test target. Add a real coordinator/adapter integration target if necessary to avoid a package dependency cycle. Do not introduce a production dependency cycle between communication and mission merely to link a test.

The temporary audit source is `/tmp/banyu_issue13_886_probes.cpp`, build script `/tmp/banyu_issue13_886_probe_build.py`, and results `/tmp/banyu-issue13-886e17f-probes.log`. Those files are temporary; move the useful scenarios into registered tests. They include the existing adapter fixture and, for the conflicting-start case, link the actual `RunCoordinator` class. The audit did not launch the full mission ROS node.

| Regression | Sequence to reproduce | Correct observable result | Current failure |
| --- | --- | --- | --- |
| `ForeignVehicleStartMustNotBlockUuvReport` | Deliver `CMD_RUN_START`, run 99, addressed to UAV; then submit a valid accepted UUV report for run 15 | Foreign command cannot change UUV eligibility; valid report reaches the public subscriber | Adapter switches to 99 and rejects report 15 |
| `RejectedConflictingStartMustNotBlockAcceptedRunReport` | Real coordinator accepts run 15 and enters ACTIVE; it rejects run 99; adapter sees the same raw command; run 15 later surfaces/seals/reports | Report 15 remains eligible; no suppression or context switch from the rejected command | Coordinator retains 15, adapter switches to 99, report is rejected |
| `DisabledForwardingMustPreservePendingPublication` | Start with forwarding disabled; accept report; restart with forwarding enabled and the same valid authority context | No first publication; report remains pending and can be recovered | It becomes ATTEMPTED without publication and never forwards |

When adopting the authoritative-lifecycle fix below, update these fixtures to publish valid accepted lifecycle context. A fixture must not depend on the adapter treating a report or a raw start command as independent mission authorization. Keep the bad-command sequences and output assertions unchanged.

Capture baseline failures before implementation. After the change, keep the existing 280 cases passing and add meaningful regressions; a particular final case count is not the acceptance criterion.

## 3. Second commit: use the mission's accepted run context

Targets:

- `banyu_communication/include/banyu_communication/gcs_submission_adapter.hpp`
- `banyu_communication/src/gcs_submission_adapter.cpp`
- Adapter tests and real mission/adapter integration
- `banyu_bringup/config/task2_params.yaml` and applicable launch/configuration sections

### 3.1 Remove the second start owner

`handleCommand()` currently changes `active_run_id_` on any nonzero raw start. It neither filters the target nor waits for mission acceptance. Filtering the target alone fixes only the first probe. It does not fix mission-rejected conflicting starts, replay or premature commands.

The selected architecture is one start owner: `banyu_mission/RunCoordinator`. The adapter observes its accepted lifecycle. Raw `/system/mission/command` must not authorize a run switch or permanently suppress a report.

Either remove the adapter's command subscription if it has no other purpose, or keep it for diagnostics only. If retained, filter UUV/broadcast targets and ensure it has no run-state side effects. Remove obsolete parameter entries if the subscription is removed.

### 3.2 Subscribe to the existing private lifecycle

Use `banyu_msgs/msg/RunLifecycle` on `/banyu/mission/run_lifecycle`. Add the include, subscription and a `topics.run_lifecycle` parameter. Use compatible reliable/transient-local QoS and update the adapter's YAML section. Check launch parameter forwarding and installation; the contract already exists, so a new public message is unnecessary.

Fields already published by `BanyuMission::publishLifecycle()` include `run_id`, `epoch`, `revision`, `authority_instance`, `phase`, `event`, `requested_run_id` and `accepted_at`.

Use **`run_id` and `epoch`**, which describe the coordinator's accepted snapshot. Do not switch to `requested_run_id`: rejected or duplicate events can describe an attempted command while retaining the accepted run context. In particular:

| Lifecycle input | Adapter action |
| --- | --- |
| START_ACCEPTED for a valid new context | Establish accepted context after validation/persistence |
| START_REJECTED while accepted run remains 15 and requested run is 99 | Preserve accepted run 15 and pending reports |
| START_DUPLICATE or newer SNAPSHOT of the same accepted context | Refresh valid authority/freshness; do not reset/suppress data |
| Retained valid snapshot after adapter restart | Reconstruct the accepted context under the authority/replay policy |
| Lower epoch, old revision, invalid metadata or retired authority | Reject with diagnostic; no run/report mutation |
| Failed/aborted/terminal lifecycle | Preserve durable accepted report data; do not authorize new mission activity or motion |

Validate source stamp, allowed phases, nonzero accepted identities where required, authority identity and monotonic revisions. Use the existing comm lifecycle validation as a reference for semantics rather than inventing a different owner. Account explicitly for mission restart/authority replacement. Do not treat every new process identifier as permission to resurrect an old live run.

A periodic valid snapshot has an increasing revision in the production mission publisher. Test fixtures must also increase revisions; replaying an identical revision forever should not keep authority fresh. Use steady-clock receipt deadlines as well as appropriate source-time validation.

### 3.3 Bind suppression to accepted transitions and durable recovery

Only an accepted new context can make an older pending report ineligible for automatic publication. Preserve its immutable payload and original identity. A rejected command must not append an active-run or suppression record.

Persist sufficient accepted context to apply the documented restart policy. Do not change in-memory authority and silently continue after a required persistence failure. Make the accepted-context/suppression update recoverable as a whole, or show how replay reconciles interrupted writes safely.

Existing journal `ACTIVE_RUN` records came from raw commands. During migration, treat them as historical hints, not proof of mission acceptance. Reconcile with authoritative lifecycle/seal evidence before authorizing a run or recovering a previously suppressed record. Preserve files and report data; do not silently erase suppression or relabel payloads as the current run.

Keep authority context separate from report transaction identity. A same-run epoch change and a new run ID are different events. Define recovery eligibility for a durably sealed report under its original `(run_id, epoch, kind, report_id, payload)` rather than forcing it into the newest epoch.

If no validated authority is available after restart, leave recoverable data pending and expose an actionable diagnostic. Avoid both unauthorized publication and permanently discarding the record. Accepted report recovery after terminal state must follow the existing seal policy; terminal state must never restore mission motion or `current_task`.

### 3.4 Acceptance checks

Run both foreign-vehicle and coordinator-rejected-start regressions. Also cover zero/stale/conflicting command sequences, duplicate starts, delayed old authority, valid retained lifecycle, unavailable authority and adapter restart. Observe public output, journal content and accepted context; do not rely only on private counters.

Extend to the real mission ROS node: issue a conflicting start through `/system/mission/command`, capture START_REJECTED with the original accepted snapshot, then demonstrate that the original run's properly sealed report still reaches the standard public subscriber.

Done means the adapter cannot independently start/switch runs and valid report publication remains possible after rejected/unaddressed commands.

## 4. Third commit: mark ATTEMPTED only when publication occurred

Targets: `publishPublicReport()`, both its callers, journal state handling and adapter tests.

### 4.1 Give publication an explicit outcome

The method currently returns `void`. It returns early when `forward_to_rx_topics=false`, but both fresh and recovered submission paths still mark the record ATTEMPTED.

Use a private outcome such as `Attempted`, `Disabled` or `Failed`; exact naming is an implementation choice. ATTEMPTED means the ROS publish call was actually issued. It does not mean a subscriber received, durably stored or forwarded the report.

| Outcome | Local record | Required behavior |
| --- | --- | --- |
| Forwarding disabled | PENDING | Preserve durable acceptance and publication intent; no attempt marker |
| ROS publish issued | ATTEMPTED under the chosen persistence strategy | Record the attempt; make no receipt/scoring claim |
| Invalid payload or unsupported kind | Reject/error | No fabricated report or successful attempt marker |
| Publish failed before issuance | PENDING plus diagnostic | Preserve retry intent; no false attempt |
| Publication issued, attempt-marker persistence fails | Explicit unresolved durable state/error | Acknowledge the possible replay window; do not claim exactly-once delivery |

Apply the outcome consistently in both the first-acceptance and pending-recovery branches. Avoid fixing only one caller.

### 4.2 Replay with forwarding enabled

On restart: load/validate journal → initialize publishers/subscriptions → establish accepted authority/context → reconcile eligible pending reports → attempt publication. Do not publish inside journal loading before publishers exist.

For recovery independent of the original sender, reconstruct the pending report from its persisted immutable payload and required metadata. Preserve authorization provenance; do not manufacture a seal from nonzero fields. If the chosen implementation relies on outbox retransmission, include that real outbox path in acceptance and document the dependency.

No runtime parameter callback currently exists for forwarding. Prove restart-based disabled-to-enabled recovery first. If adding runtime toggling, implement and test its parameter callback deliberately; do not assume `ros2 param set` changes the cached member automatically.

Keep SURVEY and DELIVERY separate. Document header timestamp semantics on recovery and retain the metadata needed by that policy. Avoid adding undocumented fields to the public schemas.

### 4.3 Acceptance checks

- Disabled forwarding preserves PENDING across restart; enabling it under valid authority delivers the expected public payload.
- Already-pending recovery with forwarding still disabled remains pending.
- SURVEY and DELIVERY each follow that behavior independently.
- Repeated submissions do not produce unbounded publications within a running process.
- Required storage failure does not claim durable acceptance, and attempt-marker failure is diagnosed.
- New accepted run, wrong epoch and unavailable authority cannot cause old data to be silently attributed to the new run.

The current late-receiver test expects no output after an ATTEMPTED record and adapter restart. That proves suppression, not guaranteed receipt. Keep that limitation explicit. Stronger cross-run receipt/deduplication needs an agreed GCS/shared-interface mechanism; do not invent one locally.

## 5. Fourth commit: verify the real mission/report path

Fixing the adapter regressions is necessary but does not complete Step 2. Create a registered integration fixture using real mission/coordinator, comm and adapter owners. Mock the FCU/sensor boundary and standard GCS ROS interface, not mission lifecycle or sealing.

Feed sufficient valid current-run evidence to exercise the actual reporting service and seal callbacks. Keep direct report copies disabled. The mock GCS subscribes to the agreed PipelineSurvey/ResourceDelivery types and does not consume Task2Submission.

Required cases: valid start/report; rejected conflicting start/report; duplicate start preserves observations; abort before seal; durable seal before delayed abort; adapter restart; disabled forwarding recovery; stale epoch and delayed previous-run response. Identify every substituted component in the result record.

This verifies node ownership and local transactions. It is still not a physical survey or full Gazebo acceptance.

## 6. Continue the remaining production work

Use Work C–H in the broader correction guide for detailed requirements. The next milestones are:

| Order | Targets | Required completion evidence |
| --- | --- | --- |
| 1 | Qualified ActiveBuoyFix and PipelineEndCandidate producers; detector/evidence/end nodes; launch | Real upstream current-run observations reach the ledger; no preassembled survey or fabricated verification flags |
| 2 | RingTracker, localization/geometry and config | Four distinct original nodes, stable revisits, active-end orientation, bounded physical association and ambiguity rejection |
| 3 | CalibratedDepthNode, Surface action and SurfaceGuard | Independent validated water datum; underwater startup/restart cannot establish a new surface; continuous fresh resurfacing proof |
| 4 | Course context, navigation/geofence and actual acoustic consumer | Received boundary/frequency affects its intended UUV consumer; late/restarted consumers receive context |
| 5 | Task 4 comm/mission/execution behavior | No premature readiness; either an explicitly scoped truthful stub or verified transit/hold/confirmation and hazard behavior |
| 6 | Full Gazebo/SITL and DDS boundary | Real hold/start/traverse/surface/report flow; UUV domain 40 to mock GCS domain 10; private topics remain local |

For each producer, list input, output type/topic, frame/datum, provenance, freshness, run/epoch behavior and launch ownership before implementation. Neither Course nor the existing public schema supplies active-buoy coordinates; agree an external observation interface if relying on another vehicle.

Preserve the approved four-node hardware/Q&A contract and original RED/GREEN meanings. Core versus Advanced/Disruptive repair and optical gates remain distinct. A timer or centered image alone cannot prove magnetic repair. Do not hardcode expected colors or count fixture passes as hardware evidence.

For headless SITL, complete the approved Step 3 safety diagnosis/simulation authorization if RC freshness blocks readiness. Do not force `rc_fresh=true` or weaken hardware deadman protection. Keep GUIDED arbitration, ENU velocity streaming, `0x5C7` and explicit yaw-rate behavior. Do not modify the user-configured front-camera sensor block.

## 7. Build and verify each milestone

Use bash and the actual local Jazzy workspace:

```bash
cd /home/erlangga/Desktop/ros2_ws_robotx
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rx_msgs banyu_msgs
source install/setup.bash
colcon build --packages-select banyu_guidance banyu_vehicle_interface \
  banyu_vehicle_execution banyu_communication banyu_mission \
  banyu_detection banyu_bringup
source install/setup.bash
ROS_DOMAIN_ID=42 colcon test --executor sequential \
  --packages-select rx_msgs banyu_msgs banyu_guidance banyu_vehicle_interface \
    banyu_vehicle_execution banyu_communication banyu_mission \
    banyu_detection banyu_bringup \
  --ctest-args -j1 --return-code-on-test-failure
colcon test-result --test-result-base build --verbose
```

Run the smoke test after the registered suite because the script forces domain 42:

```bash
python3 src/BANYU_ROBOTX/banyu_communication/test/sos_gcs_mock.py
```

Update the mock if adding adapter lifecycle requirements. Maintain valid increasing lifecycle revisions. State explicitly that its mission/seal/evidence are substituted and its surface hold is accelerated.

Run targeted regression checks while developing, then the required affected-package/full focused suite. Add tests for observable failures and ownership boundaries rather than mirroring private methods. Use unique temporary journal paths and isolated DDS domains; do not reuse real operational report files.

Synchronize every added/renamed parameter with matching YAML, launch forwarding, installation and dependencies. Apply POR updates only where shared-node behavior requires them. Check for stale old topic/parameter references and unintended changes outside this repository.

## 8. Update verification and stop claiming broader completion

Append a revision-scoped entry to `issue13_step2_verification.md` after each verified milestone. Record tested SHA, clean/modified working-tree state, build commands, exact C++ counts, aggregate checks/skips, three new regression outcomes, fixtures versus real owners, rates/payload samples and remaining gates.

At this baseline, the correct record is:

- Registered checks: 280 C++ passes; aggregate 675 checks, zero failures/errors, 154 skipped.
- Communication mock: six passes through the adapter; not a full mission proof.
- Supplemental probes: three failures, covering two defects.
- Public/private topic migration: complete locally.
- Original journal-before-first-publication regression: registered replay case passes.
- Production evidence, water datum, course consumers, truthful full Task 4 and complete Gazebo/routing: still open.

Keep older results labeled historical. Do not delete evidence of the earlier failure or carry its OPEN label forward after the matching regression passes. Archive essential results in the repository's verification record; `/tmp` paths are not permanent evidence.

Use three distinct milestones: adapter regressions fixed; local Step 2 acceptance complete; Issue 13 acceptance complete. Full declared-tier competition and GCS/RoboCommand field integration are further team milestones. Passing the next three tests alone does not close the issue.

Immediate next action: register the failures, replace raw-command run-context mutation with accepted lifecycle observation, and fix false ATTEMPTED transitions. Then verify those changes before continuing production sensor/mission integration.
