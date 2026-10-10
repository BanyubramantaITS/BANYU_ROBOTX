# Issue 13: next coding-agent assignment at 4d74257

Baseline: `4d74257521889690494930b7594e8b1573618b79`. Use this guide before [the 5f9932b handoff](issue13_next_steps_5f9932b.md). The earlier epoch/authority regressions now pass; do not reimplement their fixes. This guide specifies future changes, not completed work.

## 1. Correct starting assessment

The core interface and mission architecture have substantial working implementation. The previous local audit passed nine-package build, **321 registered C++ cases**, **722 aggregate checks with zero failures/errors and 156 skipped**, and six communication smoke checks. Eight new tests instantiate production BanyuMission and exercise its ROS callbacks. These counts belong to this baseline and must be regenerated after changes.

The conclusion “100% complete and fully verified” is unsupported. A supplemental audit probe reproduced a rejected atomic parameter transaction publishing a pending survey. Some requested production-boundary failure/guard probes are missing or do not isolate the intended guard. Qualified upstream evidence, physical water datum, course consumers, full simulation and multi-domain routing remain open.

Correct the implementor's statements as follows:

| Statement | Accurate qualification |
| --- | --- |
| Entire handbook/state machine is complete | The tested local paths pass; the reproduced parameter defect and remaining acceptance gates prevent this claim. |
| Command handles TYPE_ALL | Installed `rx_msgs/msg/VehicleType.msg` has TYPE_UNKNOWN, TYPE_USV, TYPE_UUV and TYPE_UAV; no TYPE_ALL. Use the agreed TYPE_UNKNOWN broadcast convention. |
| Private isolation verified by smoke | Private/public topic placement is implemented. A single-domain smoke test does not prove router isolation between domains 40 and 10. |
| Abort disarms the vehicle | Current code requests fallback behavior/mode. Prove an actual disarm command/acknowledgement before claiming disarm; do not add underwater disarming merely to match that claim. Preserve the approved safety policy. |
| Software enforces WS2812B/TLV493D hardware | Software enforces the four-node evidence contract. Tests do not establish the physical ring/magnetometer assembly or magnetic activation. |
| Surfacing is fully proven | Synthetic calibrated depth exercises selected guards. A physically defensible water datum and full motion remain unverified. |
| Task 4 is a compliant stub | ACK/listener behavior may satisfy the issue's stub allowance. Immediate ReadinessReport without arrival/hold is misleading and must be corrected. |
| No lifecycle/seal mocks in new suite | Correct for the new production callback suite, but its BT is reduced and its hardware, evidence, resets and readiness are fixtures. |

## 2. Scope, references and ownership

Read `AGENTS.md`, [the approved rules plan](issue13_rules_and_implementation_plan.md), [the UUV correction guide](issue13_uuv_corrections_and_remaining_work.md), and the installed shared `rx_msgs` definitions before editing. Compare actual Issue 13 acceptance criteria separately from broader competition/tier goals.

Work inside `/home/erlangga/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX`; colcon workspace is `/home/erlangga/Desktop/ros2_ws_robotx`. The audited environment is ROS 2 Jazzy, C++/rclcpp, BehaviorTree.CPP v4. Check HEAD, working tree, installed distro and intervening changes first. Preserve user edits and do not reset journals or the branch to make tests pass.

Official handbook/hardware references and their provenance are identified in the approved plan. The previously used `/media/erlangga/DATA/RobotX` reference directory is unavailable in the current environment inspection. Locate accessible copies before making new competition-specific architecture/payload/timing decisions; if the originals cannot be accessed, report exactly which rule-dependent work lacks grounding. The reproduced parameter fix and tests can proceed from local code/rclcpp APIs without inventing competition rules.

Responsibilities:

- **UUV:** local autonomy, lifecycle, evidence qualification, reporting guards, journal/outbox, public ROS publishers/subscribers and local verification.
- **GCS:** OCS credentials, MQTT/protobuf/RoboCommand conversion, official report envelope/sequence allocation and receiving persistence.
- **Joint:** agreed ROS schemas/QoS, DDS routing, receipt/retry and cross-run attribution. Use local mock GCS where actual access is unavailable.

Keep `/banyu/communication/task2_submission` private. Keep agreed public `rx_msgs` types and `/system/...` topics. Do not create a custom public envelope, put identity in `frame_id`, invent a scoring ACK, or equate ROS publication with receiver receipt. No OCS credentials are needed for this assignment's local UUV fixes.

## 3. First change: repair parameter transaction integrity

Priority: fix this before claiming the software reporting path complete.

Targets:

- `banyu_communication/src/gcs_submission_adapter.cpp`: constructor callback registration and `handleParameters`.
- `banyu_communication/include/banyu_communication/gcs_submission_adapter.hpp`: callback declarations and lifetime handles.
- `banyu_communication/test/test_gcs_submission_adapter.cpp`: registered regression coverage.

### Reproduce first

Add `RejectedAtomicParameterBatchCannotPublishPendingReport` using the existing adapter fixture:

1. Restart with `forward_to_rx_topics=false`.
2. Publish accepted run 15, epoch 2, SURFACED lifecycle; use the existing fixture's valid authority metadata.
3. Submit a valid matching survey, wait for acceptance, and assert PENDING=1, ATTEMPTED=0 and no public survey.
4. Call `set_parameters_atomically` with this ordered batch:

   ```cpp
   {
     rclcpp::Parameter("forward_to_rx_topics", true),
     rclcpp::Parameter("lifecycle_timeout_s", -1.0)
   }
   ```

5. Assert unsuccessful result and readback `forward_to_rx_topics=false`, timeout unchanged.
6. Spin the bounded fixture executor; assert no public report, PENDING=1, ATTEMPTED=0.
7. Retransmit the report; it must still remain pending with no public output.

Baseline behavior: result/readback indicate rejection, but the on-set callback has already changed its forwarding cache and reconciled/published the report. PENDING becomes zero and ATTEMPTED becomes one. The previous probe files were temporary and are no longer present; reconstruct this test directly in the repository rather than depending on `/tmp`.

### Implementation contract

Use separate validation and committed-application callbacks. The installed Jazzy `rclcpp/node.hpp` provides `add_on_set_parameters_callback` and `add_post_set_parameters_callback`.

1. On-set validates types and the complete candidate batch only. Reject nonfinite/nonpositive timeout and incorrect types with useful diagnostics. It must not change caches, freshness, report state, journal or public outputs.
2. Post-set applies successfully committed values. Update forwarding and timeout together under the node's existing synchronization; reconcile pending reports once using the final committed configuration.
3. Retain both callback handles for node lifetime. Register application callbacks only after required members/publishers are initialized; account for parameter declaration invoking callbacks.
4. Keep accesses to mutable timeout/forwarding caches synchronized with lifecycle and submission callbacks. Audit reads outside the mutex and avoid recursive locking of existing `...Locked` helpers.
5. Do not recursively set parameters from the post-set callback. Keep existing source-age, terminal-recovery and publication-eligibility policies intact.
6. Keep normal single-parameter enable/disable behavior working. Any required journal-write failure retains the documented pending/recovery behavior.

Validating the whole batch and then mutating inside on-set is insufficient: another registered validator can reject afterward. Side effects belong after successful commit.

### Required regression matrix

| Probe | Expected result |
| --- | --- |
| Forwarding=true followed by invalid timeout, atomic batch | Rejected; unchanged readback/cache; no publication; PENDING retained |
| Same invalid batch with reversed parameter order | Same result; no order-dependent effects |
| Valid timeout followed by an invalid parameter in an atomic batch | No timeout cache/readback divergence; test actual freshness behavior, not only readback |
| Another on-set validator rejects an otherwise valid batch | No post-set application or publication; retain the veto handle and control callback order |
| Accepted timeout+forwarding batch | Both committed values used; eligible pending report published once |
| Accepted disable plus timeout update | Forwarding remains off; no transient publication during application |
| Accepted enable under wrong epoch/stale active authority | PENDING retained until an eligible matching context arrives |
| NaN, infinity, zero, negative timeout and wrong parameter types | Rejected without state or publication effects |
| Parameter update concurrent with lifecycle/submission processing | No race, mutex recursion or deadlock; use bounded waits and observable outcomes |

**Pass:** the new baseline reproduction passes as a registered test, the matrix passes, and existing 39 adapter cases—including epoch, authority replacement, restart, terminal recovery and toggling—continue to pass. This is the first reviewable software milestone.

## 4. Second change: finish independent production-boundary probes

Targets: `banyu_communication/test/test_production_mission_boundary.cpp`, its existing target, and production code only where an independently reproduced failure requires it.

Keep production BanyuMission initialization and its real command/lifecycle/seal callbacks. Existing hardware mocks are appropriate for ROS boundary tests. Document the reduced BT and accelerated timing instead of presenting them as full Task 2 execution.

### Isolate reporting guards

The current submerged subcase retains the earlier two-node evidence and expects SURVEY_INVALID. That proves rejection of incomplete evidence, not an independent surface guard.

Construct separate valid baselines for each negative test: four distinct qualified original conditions, valid current run/epoch/course/active-end identity, unambiguous topology and completion evidence. Change **one prerequisite at a time**. For depth tests, ensure completion is established through the real mission path without relying on a surface condition that prevents the test from ever reaching the intended guard. State which gate is expected to reject and assert its diagnostic.

Required cases:

- Complete survey, but submerged depth; no public report or successful seal authorization.
- Complete survey with missing, stale, invalid, uncalibrated or wrong-datum depth; all blocked independently.
- Complete survey and valid near-surface samples, but missing survey-completion/current-epoch surface milestone; blocked.
- One qualifying surface sample followed by excursion; continuous hold resets and publication remains blocked until a fresh full hold completes.
- Missing, stale, ambiguous, conflicting or wrong-run node/buoy/end evidence; preserve accepted observations and emit no public report.
- Qualified successful paths for both active ends; preserve original RED across repair/revisits and actual topology direction.
- Wrong-epoch private submission delivered over ROS through the production stack; no premature public output, then eligible-context recovery under the defined policy.
- Rejected foreign authority delivered over ROS, followed by valid accepted-owner progression; owner remains usable.

Do not weaken guards or inject success flags into production code to get the harness through them. Lower-level unit tests are supporting evidence, not replacements for the real callback path.

### Isolate terminal failures and transport races

Add explicit action-result failure and BT failure through test-owned action servers or deterministic test BTs that use production actions. Assert authoritative terminal phase/reason, TASK_NONE in lifecycle **and public heartbeat**, and no resurrection by delayed action/result callbacks. Keep existing abort, mission-done and readiness-loss cases.

Preserve the real-service delayed-seal proxy test. Assert abort actually completed before forwarding the seal response, instead of ignoring wait failures. Prove the sealed immutable record survives, no forbidden immediate public report occurs, and eligible recovery follows the existing policy. Keep abort-before-seal independently rejected.

**Pass:** each intended guard/failure is reached independently, with exact registered names and observable public outputs. Do not label a blanket “failure/surfacing PASS” from tests rejected earlier by another prerequisite.

## 5. Third change: remove false Task 4 readiness claims

Targets: `UuvCommNode::commandCallback`, `sos_gcs_mock.py`, related tests/config/status diagnostics. Current assistance behavior immediately publishes IncidentAck and ReadinessReport without transit.

Issue 13 permits stub listeners in its implementation checklist, while its detailed requirements describe full behavior. Choose and document the implemented scope; default this local correction to a truthful stub rather than quietly expanding it into a complete Task 4 planner.

- For a stub: ACK supported addressed commands with the correct sequence; expose “assistance execution unsupported/not ready” through the existing diagnostic/status contract. Do not emit readiness or change mission state as though transit succeeded. Update the smoke test to require ACK and absence of readiness for an unexecuted assistance request.
- For full Core behavior: interrupt safely, transit to assistance coordinates, establish station hold, then publish readiness; resume only after correctly correlated confirmation. Verify actual motion and hold. Readiness-report/OCS sequence mapping is a joint/GCS contract; command sequence and official report sequence must not be conflated.
- Keep-out and moving-obstacle behaviors need their own implementation/evidence for full supported tiers. Do not call an ACK-only stub proof of avoidance.

**Pass:** no assistance readiness is published before confirmed arrival/hold; stub/full capability is explicit and smoke expectations reflect the selected behavior.

## 6. Fourth change: complete course propagation

The issue's checked acoustic-guidance item is not established by storing `Course` and updating the survey ledger. Use `/system/mission/course`; Course has boundary/frequency but no active-buoy coordinates.

Inspect the actual geofence field semantics and shared QoS before wiring consumers. Connect validated course identity/boundary to the appropriate UUV navigation constraint component, and frequency in Hz to the acoustic frontend/homing consumer. Treat UAV-specific fields according to their actual contract; do not assume every polygon applies to the UUV.

Required evidence:

1. Consumer receives a valid course, including retained/late-subscriber/restart cases under agreed QoS.
2. Frequency changes affect the actual configured acoustic processing/selection path; unsupported values diagnose failure rather than silently retaining a default.
3. Applicable boundary constraints affect planned/output motion; invalid geometry blocks readiness or yields the defined safe behavior.
4. Mid-run updates cannot silently reverse pinned survey orientation or switch course attribution.
5. Active-buoy coordinates enter through a separate qualified observation.

A mock receiver verifies forwarding only. If actual homing/frontend is unavailable, preserve the qualification and report the named dependency; do not claim an unused private context topic completes acoustic integration.

## 7. Remaining production evidence, safety and full acceptance

Continue Work C–H in [the UUV correction guide](issue13_uuv_corrections_and_remaining_work.md), with the approved handbook/hardware grounding. Preserve the already implemented guards while filling missing inputs.

| Work | Required implementation/evidence |
| --- | --- |
| Qualified active buoy and pipeline end | Production publishers and launch wiring with observed provenance/time/run/epoch/uncertainty; green active buoy and active pinger evidence. Detection loss or four counted nodes alone must not manufacture an end. |
| Node association and ordering | Calibrated node location or a validated bounded observation method; lateral-distance/ambiguous-segment rejection; both ends, bent pipes, revisits and fifth-node ambiguity tested. Vehicle pose projected onto fixed YAML geometry alone is insufficient physical proof. |
| Water datum | Verified calibration/configured datum with provenance; underwater startup/restart cannot declare the water surface from near-zero relative altitude. Continuous fresh depth and current-run Surface completion use consistent semantics. |
| Task 2 tiers | Explicitly select Core/Advanced/Disruptive scope. Preserve four original observations and magnetic repair/optical timing distinctions. No fabricated fallback, coerced one-red distribution or timer-only hardware compliance claim. |
| Headless deadman | Simulation-only supervised freshness source selected through use_sim; stale-source loss must stop authorization. Hardware use_sim=False retains real RC protection. Never globally force rc_fresh=true. |
| Full Task 2 Gazebo/SITL | Run the actual Task 2 BT and deployed producers/consumers in domain 40. Record which evidence originates in sensor/physics models versus direct fixtures. Capture start/hold, dive, traversal, repair as applicable, surfacing and correct public report. |
| DDS routing | UUV domain 40, local mock GCS domain 10 and an allowlisted router. Prove agreed public traffic crosses and private banyu/MAVROS/camera/TF does not. No OCS access needed for this test. |

Preserve continuous ENU velocity-loop streaming (`0x5C7`), explicit yaw rate, MAVROS NED conversion, GUIDED arbitration and the user-configured camera block. Stop only identified test-owned stale processes; do not disrupt unrelated sessions or operate attached real hardware under a simulation test assignment.

Measure the production 2 Hz heartbeat over at least 30 seconds with raw timestamps and telemetry inputs. Report observed jitter separately from nominal frequency. Tests overriding heartbeat to 10 Hz and surface hold to 0.05 seconds do not establish production timing or physical proof.

Every new/renamed parameter must have matching YAML/config forwarding, launch wiring, installation and dependencies as required by AGENTS.md. Prefer C++ core nodes; use Python fixtures only where appropriate.

## 8. Verification commands and completion record

From the audited workspace, use Bash with the installed distro:

```bash
cd /home/erlangga/Desktop/ros2_ws_robotx
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rx_msgs banyu_msgs
source install/setup.bash
colcon build --packages-select \
  banyu_guidance banyu_vehicle_interface banyu_vehicle_execution \
  banyu_communication banyu_mission banyu_detection banyu_bringup
source install/setup.bash

ROS_DOMAIN_ID=42 ./build/banyu_communication/test_gcs_submission_adapter
ROS_DOMAIN_ID=42 ./build/banyu_communication/test_production_mission_boundary
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

Run fixture suites sequentially in a free isolated domain. Use test-owned journals/resources and bounded waits. Launch --show-args is inspection, not end-to-end execution. Rebuild and rerun after relevant source changes; do not reuse stale XML or copy baseline counts.

Update `docs/issue13_step2_verification.md` with exact code revision, commands, environment, actual registered test names/counts, skip reasons, logs and mock/physical boundaries. Retain historical entries but explicitly supersede inaccurate completion claims. Archive useful evidence durably; temporary audit paths are not the sole evidence record.

Deliver separate milestone verdicts:

1. Adapter transaction correctness: reproduced bug fixed and regression matrix passes.
2. Production ROS boundary: independent guard, failure and delayed-response probes pass with mocks disclosed.
3. UUV issue acceptance: course consumers, truthful selected Task 4 scope, production heartbeat, valid survey and full domain-40 simulation demonstrated.
4. Joint/team acceptance: domain-40/10 routing and GCS/OCS-owned receipt/transport checks separately recorded.

Use PASS / FAIL / NOT VERIFIED per gate. A missing sensor module is a named dependency, not permission to fabricate evidence; external GCS ownership is not a reason to defer independently implementable UUV fixes. Do not close the overall issue on unit/smoke results alone.

## 9. Copyable first assignment

> At local baseline 4d74257521889690494930b7594e8b1573618b79, read AGENTS.md and docs/issue13_next_steps_4d74257.md. Check the working tree and preserve passing epoch/authority/recovery behavior. First register and fix RejectedAtomicParameterBatchCannotPublishPendingReport: on-set validation must have no side effects; update caches/reconcile only after successful parameter commit. Test reversed order, later-validator veto, accepted batches, invalid values, stale/wrong-epoch authority and concurrent callbacks. Then strengthen real BanyuMission tests so each surfacing/completion guard and action/BT failure is independently reached. Preserve real sealing and delayed-response race coverage. Correct immediate Task 4 readiness to reflect a truthful stub or verified arrival/hold, and update smoke expectations. Run isolated focused and nine-package checks; update the verification record with measured counts and limitations. Keep public rx_msgs unchanged and private submissions private. Track course consumers, qualified producers, physical datum/association, supervised SITL and full domain-40/10 acceptance as remaining reviewable milestones. No OCS backend or credentials are required for the local fixes.
