# Issue 13: AI coding agent handoff after 5f9932b

Current baseline handoff: [next steps at 4d74257](issue13_next_steps_4d74257.md). The three adapter regressions below now pass and production mission ROS tests exist. The new guide covers the remaining parameter-transaction defect, independent guard/failure coverage and production acceptance work.

Baseline: `5f9932b6d66814a816248595de71abfcc2e44cea`, audited locally on October 10, 2026. This document instructs future implementation; it does not claim the fixes below are implemented. Start here instead of repeating the failures from the older [886e17f guide](issue13_next_steps_886e17f.md), which now pass.

## 1. Assignment and scope

First complete Phases A–C: repair the remaining adapter authority defects, synchronize configuration, and verify the production mission ROS boundary. Then continue the production evidence and simulation gates in Phases D–E. Report each milestone separately. Do not equate a passing adapter fixture with a completed Task 2 mission or full competition compliance.

The current user request is for instructions. This document alone does not authorize the agent reading it to deploy, publish, or operate a physical vehicle. When assigned to implement this handoff, make the local changes and checks within that assignment without asking for routine implementation choices.

Work in `/home/erlangga/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX`; run colcon from `/home/erlangga/Desktop/ros2_ws_robotx`. Read `AGENTS.md` and check the actual environment before editing. This audited workspace uses ROS 2 Jazzy, C++/rclcpp and BehaviorTree.CPP v4; the approved session plan explicitly supersedes the stale Humble description in AGENTS.md. Keep changes inside this repository.

Before any change:

```bash
cd /home/erlangga/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX
git status --short
git rev-parse HEAD
git log -5 --oneline
```

If HEAD differs, inspect the intervening diff and reproduce the relevant behavior. Preserve user changes. Do not reset the branch or assume this audit still describes newer code.

## 2. Rules and ownership that constrain the implementation

Read [the approved rules and implementation plan](issue13_rules_and_implementation_plan.md) and [the ownership correction and remaining-work guide](issue13_uuv_corrections_and_remaining_work.md). Ground any new payload, transition, or sensor acceptance rule in those references and the locally available handbook/Q&A. The local reference root is `/media/erlangga/DATA/RobotX`:

- `GitBook-Handbook/section-3-autonomy-challenge/3.4-communications-protocol.md`.
- `GitBook-Handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.3-mission-task-2-infrastructure-survey-and-repair.md`.
- `GitBook-Handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.5-mission-task-4-dynamic-incident-response.md`.
- Task-build references for §3.5.5, `UNDERWATER_MAGNETIC_ACTIVATED_LIGHT.md`, the supplied pipeline image, and the original Q&A export identified in the approved plan.

Use these distinctions throughout the implementation:

| Area | Responsible party / constraint |
| --- | --- |
| UUV autonomy, evidence validation, lifecycle, private journal/outbox, public ROS reports | UUV developer; implement and verify locally |
| Agreed `/system/...` topics and `rx_msgs` schemas | Shared team contract; preserve the installed definitions |
| OCS credentials, MQTT/protobuf, official report sequence allocation, RoboCommand forwarding | GCS developer; no OCS access is required for local UUV fixes |
| DDS routing, receiving persistence, receipt/retry and cross-run attribution | Joint agreement and integration; use a local mock for the UUV boundary |
| `/banyu/...`, private Task2Submission, MAVROS, camera and TF | Internal UUV traffic; exclude from the public GCS router |

The installed course topic is `/system/mission/course`. It supplies boundary and pinger frequency, not Active Start Buoy coordinates. Active buoy localization needs a separate qualified observation. Do not implement against the earlier shorthand `/system/course`.

The four-node ordered survey records original observed conditions. Do not fabricate `[1,1,1,2]`, coerce measurements into the expected one-red/three-green distribution, or overwrite an original RED observation after repair. Order by pipeline topology from the confirmed active green buoy, not Euclidean distance on a bent pipe. Incomplete or unqualified surveys fail with actionable diagnostics and no public report.

Nonzero `run_id`, idempotent start handling and the verified-surface reporting guard are BANYU policies reconciled in the approved plan; do not label them explicit universal handbook requirements. Terminal heartbeat task is `rx_msgs::msg::RxTask::TASK_NONE`, not a literal zero or TASK_UNKNOWN. Never invent a public ACK, place report identity in `frame_id`, or claim receiver delivery from ROS `publish()`.

## 3. Verified baseline and what to preserve

Fresh local checks at the baseline passed: nine focused packages built, **308 registered C++ cases** passed, colcon reported **706 aggregate checks / zero errors / zero failures / 155 skipped**, and all six communication smoke checks passed. C++ breakdown: msgs 1, guidance 8, vehicle interface 7, execution 74, communication 106, mission 70, detection 42. Aggregate checks and C++ cases are different counts.

The older verification entry's 330 C++ count is inaccurate for this audit. The new nine-case integration suite uses the real coordinator/journal and communication components, but fixture-owned lifecycle/seal callbacks on a generic ROS node. It does not instantiate the production BanyuMission node.

Preserve these already passing behaviors:

- Foreign-vehicle commands and mission-rejected starts do not take adapter ownership away from the accepted run.
- Duplicate/invalid starts preserve observations and accepted mission state.
- Direct report copies are disabled in the smoke fixture; the adapter supplies standard public reports.
- Disabled forwarding keeps reports PENDING; runtime enabling reconciles them.
- Retained same-revision lifecycle snapshots can reconstruct authority after adapter restart.
- A first START_REJECTED snapshot describes its accepted `run_id`/epoch, not the rejected requested ID.
- Valid terminal context can recover a durably sealed report under the existing recovery policy.
- Journal migration, suppression persistence, payload/report-ID conflict rejection and storage-error handling remain intact.
- No mutex recursion is reintroduced: locked callbacks must call helpers with the correct locking contract.

Do not remove journal durability or globally tighten terminal timeouts to hide the new failures. No new physical motion or safety relaxation is needed for Phases A–C.

## 4. Phase A: register and fix the three reproduced failures

Primary targets:

- `banyu_communication/src/gcs_submission_adapter.cpp`.
- `banyu_communication/include/banyu_communication/gcs_submission_adapter.hpp`.
- `banyu_communication/test/test_gcs_submission_adapter.cpp` and its existing CMake test target.

Register the following scenarios as normal tests before repairing the implementation. Use the existing fixture, test-owned journals and bounded executor waits. Assert public subscriber observations as well as journal/publication state; internal counters alone do not establish correct output.

| Test | Input sequence | Required result |
| --- | --- | --- |
| `FutureEpochSubmissionCannotPublishUnderOlderAuthority` | Valid lifecycle: run 15, epoch 2, SURFACED. Submit valid survey: run 15, epoch 3. | No public survey; no ATTEMPTED transition. Preserve the unpublishable submission as PENDING under the documented staging policy. |
| `PendingFutureEpochCannotRecoverUnderOlderAuthority` | Stage run-15/epoch-3 survey before any lifecycle. Then provide run-15/epoch-2 SURFACED lifecycle. | No public survey; report remains PENDING awaiting matching validated authority. |
| `RejectedAuthorityMustNotRetireCurrentMissionOwner` | Owner A has run 15, epoch 2, SUCCEEDED. Owner B sends a fresh-stamped run-14/epoch-1 ACTIVE snapshot. Then A sends valid run-16/epoch-3 SURFACED lifecycle and its survey. | Reject B without changing A's ownership. Accept A's next valid context and publish its valid survey once. |

At the baseline all three fail. The first two currently emit a report under epoch-2 authority; the third leaves run 15 active and blocks A's next report. Temporary reproductions were `/tmp/banyu_issue13_5f_probes.cpp` and `/tmp/banyu-issue13-5f9932b-probes.log`. Those files are optional aids, not durable dependencies: implement the scenarios directly in repository tests; do not include an absolute `/tmp` or developer-home source path.

### A1. Bind publication authority to the full eligible report context

`hasValidAuthorityLocked` currently checks `run_id` but not epoch. `handleSubmission` only rejects older epochs; a future epoch therefore passes. Pending reconciliation has the same run-only gate. Identical retransmission is another publication path that must use the same rule.

Implement one explicit publication-eligibility decision using report identity and validated lifecycle context. For the normal path, matching run ID **and epoch** is necessary. Check authority validity/freshness under the existing policy, forwarding state, report validity and seal authorization requirements already defined by the private contract. Update all publication paths: new submission, identical PENDING retransmission, lifecycle reconciliation, restart recovery and runtime forwarding enablement.

Keep durable acceptance distinct from permission to publish: valid future data may be stored while awaiting matching authority, but must not be marked ATTEMPTED or silently discarded. When matching eligible epoch-3 authority arrives, publish once and persist the actual attempt. A different run or superseding context must follow the existing documented suppression policy and retain an unresolved-delivery diagnostic.

Do not derive authority from a raw GCS command, a submission's self-reported revision, or the journal's historical active-run hint. If an older sealed-report recovery exception is required, prove it from recorded accepted/sealed provenance and a separate explicit policy. Do not introduce a blanket exception that permits arbitrary future epochs, and do not erase valid sealed recovery to make a strict-equality test pass. Public schemas lack enough identity to safely reinterpret an old report as a newer run's report.

Extend the two regression tests with matching-authority arrival, retransmission, restart, forwarding-off/on and same-run/different-epoch cases. Require no premature output followed by one eligible attempt, with existing deduplication intact.

### A2. Make rejected authority replacements free of state changes

`handleLifecycle` currently inserts the current owner into `retired_authorities_` before lower-epoch/conflicting-context validation completes. Rejection can therefore poison the valid owner.

Refactor into a validation decision followed by an accepted transition:

1. Validate timestamp, phase/context, owner eligibility, epoch, run consistency and revision without mutating ownership, report state or journal.
2. Build the proposed accepted context. Persist required context changes with the existing durability strategy before exposing them as accepted in memory.
3. Retire the previous authority only as part of successfully accepting its replacement. Apply phase/revision/freshness updates and report reconciliation after acceptance.
4. On rejection or required context-storage failure, preserve the previous accepted owner, run, epoch, phase, revision and report states. Diagnostics may change; authority must not.

Keep revision semantics scoped to authority identity; a genuinely new authority may start its own revision sequence. Preserve same-revision retained-snapshot startup reconstruction and the current takeover eligibility rules. Document any necessary policy change instead of treating numeric run-ID order as authority.

Add observable coverage for lower-epoch replacement, conflicting same-epoch run, stale same-owner revision and required storage-write failure. After each rejection/failure, prove the previously accepted owner can still progress and publish an otherwise eligible report. Add an accepted replacement case proving the old owner is rejected only after replacement succeeds. If new durable records are needed, test their restart/migration behavior rather than deleting old journals.

**Phase A passes when:** all three registered reproductions pass, matching-context positive paths pass, storage failure leaves the accepted context usable, and the existing adapter/recovery suite still passes. Review every `publishPublicReport` call for the shared eligibility gate.

## 5. Phase B: synchronize timeout configuration

The adapter declares `lifecycle_timeout_s` with default 5.0 seconds, but its `gcs_submission_adapter.ros__parameters` section in `banyu_bringup/config/task2_params.yaml` omits it. The `uuv_comm_node` parameter `timing.lifecycle_timeout_s` is a separate parameter and does not configure the adapter.

Add an explicit adapter `lifecycle_timeout_s: 5.0` to preserve baseline behavior unless a documented timing decision justifies a different value. Confirm Task 2 launch passes this YAML to the adapter. Do not add unrelated POR settings when that mode does not launch the adapter.

Ensure timeout input is finite and positive; reject invalid startup configuration with a clear diagnostic. If runtime updates are supported, validate them before state mutation, update the cached timeout consistently and define the effect on pending eligibility. Otherwise make the timeout explicitly read-only rather than accepting ineffective runtime changes. Preserve the existing runtime forwarding toggle.

Test invalid values and authority freshness at meaningful timeout boundaries using bounded waits or an existing injectable clock. Test stale active lifecycle versus allowed terminal recovery separately. Update node/config documentation and any parameter descriptor. Run applicable config/launch checks. Every added or renamed parameter must follow AGENTS.md's YAML/launch/install/dependency rule.

**Phase B passes when:** the launched adapter receives the intended value, invalid timeout configuration is rejected, and freshness/terminal recovery/toggle tests pass.

## 6. Phase C: verify the production BanyuMission ROS boundary

Keep the current coordinator/communication integration suite. Add integration that launches the production `banyu_mission` executable or instantiates `BanyuMission` through its real initialization path, together with `UuvCommNode` and `GcsSubmissionAdapter`. Inspect `banyu_mission/src/main.cpp`, `src/banyu_mission.cpp`, its header, selected BT and launch/config before choosing a harness.

Use test-owned journals, isolated DDS, deterministic bounded ROS mocks for external sensors/MAVROS/action servers, and unique resources. Avoid a production package dependency cycle merely to link tests. The test may mock physical hardware; it must exercise production mission command, lifecycle and sealing callbacks rather than replacing them with a generic node or directly calling coordinator transitions to establish acceptance.

Required production-boundary scenarios:

1. Ready/preflight setup through real interfaces; addressed nonzero CMD_RUN_START accepted and lifecycle/report attribution correct.
2. Direct local start cannot bypass the configured Task 2 GCS start gate. Zero, foreign-target, duplicate and conflicting starts have the documented outcomes; duplicate/rejected starts preserve observations.
3. Abort, internal stop/mission-done where defined, BT/action failure and readiness loss clear `current_task` to TASK_NONE and do not permit stale callbacks to restore activity. Do not invent a nonexistent CMD_RUN_STOP enum.
4. Missing/ambiguous/stale node evidence, incomplete survey, missing completion, or unverified surfacing returns failure diagnostics and emits no public survey.
5. Complete qualified four-node evidence and verified surfacing allow the real seal/report path; public payload preserves original conditions and active-end topology order. Cover both active ends.
6. Abort before durable sealing rejects reporting; abort after durable sealing preserves the sealed immutable record under the existing send/recovery policy.
7. Distinguish seal-then-abort from abort while a durably sealed response is delayed. Exercise the latter with controlled response scheduling at the transport boundary while retaining real sealing; do not rename the existing seal-then-abort test as proof of that race.
8. Adapter restart and matching retained mission snapshot recover eligible PENDING data; ATTEMPTED data and conflicting/reused report IDs follow the documented policy. Wrong epoch/rejected replacement remain blocked in the integrated path.

Use generated enum constants in fixtures, including TASK_NONE. A mock-completed surface action and injected depth are useful for ROS integration, but do not claim physical surfacing from them.

**Phase C passes when:** real mission callbacks are exercised and the new matrix passes. State which external nodes are mocks, how mission actions progress, what was measured, and what physics remains unverified. Correct the verification record using actual counts generated at the resulting commit.

## 7. Phase D: finish the remaining production evidence and consumers

Follow Work C–G in [the UUV correction guide](issue13_uuv_corrections_and_remaining_work.md), using the following implementation order. These are open production gates, not demonstrated failures of every existing helper.

| Work | Concrete implementation | Required acceptance evidence |
| --- | --- | --- |
| C: active buoy / pipeline end | Identify and wire production publishers of qualified ActiveBuoyFix and run-tagged PipelineEndCandidate. Carry source/time/run/epoch/endpoint provenance as defined by the private contracts. Keep raw GPS and fixture-only assertions distinguishable from observed green/pinger and endpoint proof. | Valid producer-to-consumer chain in launch; missing, stale, wrong-run, contradictory or unqualified observations block completion. Both active ends tested. If upstream sensing is unavailable, retain the guard and report the dependency. |
| D: physical node identity/order | Inspect RingTracker projection and configured centerline. Use calibrated transforms/node localization and bounded association. Reject far-off geometry, ambiguous bent/crossing segments and revisits that cannot establish a distinct node. Static world points and vehicle pose alone must not certify physical node location. | Exactly four distinct original conditions ordered along topology from the observed active end; both ends, bends, revisits, uncertainty, missing and conflicting detections tested. |
| E: water datum/surfacing | Require a defensible water-surface calibration source. Near-zero relative altitude before ACTIVE is insufficient physical proof by itself. Preserve calibration across allowed epoch changes and keep active heartbeat from recalibrating depth. | Stale/uncalibrated/bad datum rejected; calibrated physical depth satisfies continuous surface hold and freshness. Oscillation resets hold. Simulation and hardware datum provenance documented separately. |
| F: course consumers | Forward validated course boundary to guidance/geofence consumers and frequency in Hz to the acoustic receiver/homing consumer. Deliver qualified active-buoy fix separately to Task 2 transit/orientation. | Consumers actually use changing course/frequency/fix; reject invalid/stale inputs and boundary violations. Merely storing private variables does not pass. If an acoustic component is missing, implement a meaningful consumer or record the unmet dependency. |
| G: truthful Task 4 | Resolve the intended Issue 13 scope: a listener stub or full task behavior. Keep command ACK separate from readiness. A stub must not claim arrived/ready immediately. Full Core behavior needs actual transit, station hold and correlated readiness confirmation before resume; higher tiers require their specified hazards. | No readiness before physical arrival/hold. Full behavior needs transit/hold/interrupt/resume and selected-tier tests. Official readiness-report sequence mapping is a GCS/joint integration item. |

For Advanced/Disruptive Task 2, separately verify magnetic proximity/dwell, five-second completed-repair green and optical resource decoding against the approved references. A timer/image-centering fixture is not proof of magnetic activation. The Q&A's 1–3 second dwell guidance was anticipated rather than finalized; keep the chosen setting traceable and do not invent a final competition threshold.

Choose the supported task tier explicitly in the completion record. Do not make unimplemented optional tiers or the GCS backend a hidden prerequisite for a narrowly scoped local milestone; do not advertise full competition compliance while those gates are open.

## 8. Phase E: headless SITL, full mission and DDS routing

Apply the approved headless safety architecture: a simulation-only, explicitly configured and observable source of supervised deadman freshness, with stale-input handling. Select it through `use_sim`; keep real-hardware `use_sim:=False` RC deadman protection and GUIDED arbitration intact. Do not globally force `rc_fresh=true` or disable the watchdog. Test simulation-source loss and the hardware configuration's refusal to accept simulation authority.

Inspect launch arguments before starting simulations. Use the actual launch files in this checkout; the existing full composition entrypoint is `main_launch.launch.py`. Inspect test-owned processes/ports and stop only identified stale test processes. Do not kill unrelated user sessions or attach motion tests to real hardware.

Run full Task 2 Gazebo/SITL after the producer/consumer wiring is ready. The deterministic `task2_sim_provider.py` is fixture evidence: disable or distinguish direct evidence injection before calling a run physics-backed. Record how detections, repair/end evidence and calibrated depth arise from the simulation. Identify any unavailable physical model honestly.

Capture at least: accepted start, run/epoch/phase sequence, task clearing, action progress, four original observations, active endpoint, end proof, calibrated depth/hold, public report payload, journal/restart behavior and setpoint streams. Measure the default heartbeat for at least 30 seconds; state nominal frequency and observed jitter separately. Accelerated test timing does not establish production timing.

Preserve ROS ENU conversion, continuous velocity-only control (`0x5C7`), explicit yaw rate and GUIDED arbitration. Confirm headless preflight completes with the supervised simulation source and abort/watchdog paths remain effective.

Test DDS routing separately: UUV domain 40, local mock GCS domain 10, router allowlisting agreed public topics/types/QoS. Demonstrate delivered commands/course/reports and exclusion of private `/banyu/...`, camera, TF and MAVROS traffic. The existing domain-42 smoke test alone does not establish this topology. OCS access is unnecessary; actual GCS/OCS receipt is a separate joint acceptance item.

## 9. Build, test and evidence workflow

Use Bash for ROS setup. These commands assume the audited workspace/environment; verify them if paths or distro change. Build interfaces first, then affected dependents. For Phase A/B, start with the registered adapter target; broaden after the fixes to the focused stack below.

```bash
cd /home/erlangga/Desktop/ros2_ws_robotx
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rx_msgs banyu_msgs
source install/setup.bash
colcon build --packages-select \
  banyu_guidance banyu_vehicle_interface banyu_vehicle_execution \
  banyu_communication banyu_mission banyu_detection banyu_bringup
source install/setup.bash

# First inspect the registered regression names after building.
ROS_DOMAIN_ID=42 ./build/banyu_communication/test_gcs_submission_adapter \
  --gtest_list_tests
ROS_DOMAIN_ID=42 ./build/banyu_communication/test_gcs_submission_adapter

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

Use a free isolated DDS domain for local tests; domain 42 is the established example. Do not run independent ROS fixture suites concurrently in the same domain. Tests should clean up their own processes and temporary journals.

Archive relevant command output, exact code revision, environment, fresh result-file counts and failed/passed probes in a durable verification record. Label incremental build versus clean build; record skips with reasons rather than treating them as executed cases. Baseline logs in `/tmp/banyu-issue13-5f9932b-*` are temporary aids, not the final evidence archive.

Keep counts tied to the tested revision. New tests will change 308/706; never copy those numbers into a new completion claim without recounting. Correct the existing verification record's fixture scope and count rather than deleting historical entries. Explain any remaining limitations.

## 10. Completion contract and suggested change boundaries

Make changes reviewable in this sequence (commits only when the coding assignment requests them):

1. Register the three regressions and repair publication identity plus transactional authority replacement.
2. Synchronize/validate timeout configuration, preserving runtime forwarding behavior.
3. Add production BanyuMission integration and correct the evidence record.
4. Implement qualified evidence producers, physical association/datum and course consumers with focused acceptance checks.
5. Implement the selected Task 4 scope and simulation deadman path; perform full simulation and DDS acceptance.

Deliver a final status table with each gate marked **PASS**, **FAIL**, or **NOT VERIFIED**, linked to code and fresh evidence. Distinguish:

- Adapter/lifecycle milestone: Phases A–B, all reproduced regressions resolved.
- Production mission ROS integration milestone: Phase C, real callbacks verified with external mocks clearly identified.
- Task 2 production evidence/reporting milestone: applicable Phase D evidence and Phase E execution gates.
- Team/competition acceptance: selected tiers, cross-domain handoff and GCS/OCS-owned confirmation.

List unresolved dependencies by owner and the exact evidence needed. Do not state “Issue 13 fully solved” merely because unit/smoke checks pass, or “OCS access required” for work that can be completed entirely on the UUV side.

## 11. Copyable assignment for the next coding agent

> Read `AGENTS.md`, `docs/issue13_next_steps_5f9932b.md`, the approved rules plan and ownership correction guide. Check HEAD/worktree and implement Phases A–C first. Register and fix all three baseline adapter regressions; gate fresh/replayed publication by eligible run and epoch; validate/persist lifecycle replacement before retiring the valid owner; synchronize and validate the adapter timeout; preserve existing restart, terminal recovery, disabled-forwarding and runtime-toggle behavior. Add production BanyuMission integration rather than substituting fixture lifecycle/seal callbacks. Run isolated focused tests, the nine-package verification and communication smoke, then update the verification record with actual counts, evidence and limitations. Keep public rx_msgs and internal Task2Submission boundaries unchanged. Ground new rules in local competition references and the approved plan. Report remaining Work C–H separately; no OCS credentials or GCS backend implementation is required for this assignment.
