# Issue 13 Step 2 Verification Record

- **Baseline Commit:** `588e211`
- **Verification Date:** 2026-10-10
- **Workspace:** `/home/erlangga/Desktop/ros2_ws_robotx`
- **Repository Path:** `/home/erlangga/Desktop/ros2_ws_robotx/src/BANYU_ROBOTX`
- **Branch:** `13-featcomm-integrate-robotx-sos-communication-interface-rx_msgs-into-main_task`
- **Environment:** Ubuntu Linux, ROS 2 Jazzy, DDS Domain `ROS_DOMAIN_ID=42`

---

## 1. Executive Summary

This record provides durable verification evidence for Step 2 of Issue 13 as outlined in `docs/issue13_step2_completion_instructions.md` and the approved plan (`docs/issue13_rules_and_implementation_plan.md`).

All three failing probes identified at baseline `588e211` (`ActiveHeartbeatCannotReplaceSurfaceProof`, `RestartMustNotForwardRetainedReportTwice`, `DurableSealSurvivesAbortBeforeResponse`) plus edge cases (`AbortBeforeDurableSealRejectsReportAndUnstages`, `ConflictingPayloadForSameRunEpochKindIsRejected`, `ReusedReportIdForDifferentRunIsRejected`, `EpochChangeWithinSameRunPreservesCalibration`) have been resolved, implemented, and verified in the test suite.

Physical pipeline centerline projection and topology tracking have been integrated into `RingTracker` and `RingTrackerNode`. Water-surface calibration has been decoupled from `ACTIVE` lifecycle heartbeats, and run identity changes are distinguished from internal epoch updates. In `uuv_comm_node.cpp`, abort reconciliation distinguishes in-flight authority requests from unsealed transactions, preserving durably sealed reports in the outbox while preventing post-abort live network broadcasts. In `GcsSubmissionAdapter`, deduplication is strictly scoped to the full logical run identity `(run_id, epoch, kind)` and `report_id`, backed by a persistent journal (`~/.ros/banyu/gcs_adapter_dedup.journal`).

All 272 registered C++ gtest test cases across all seven C++ packages pass with zero failures and zero errors under sequential execution. In addition, all six communication smoke checks in `sos_gcs_mock.py` pass. Code formatting across all modified files satisfies `ament_uncrustify`, `ament_flake8`, and `ament_pep257` with zero divergences.

---

## 2. Test Execution & Build Commands

### 2.1 Workspace Build
```bash
cd /home/erlangga/Desktop/ros2_ws_robotx
source /opt/ros/jazzy/setup.bash

colcon build --packages-select banyu_msgs
source install/setup.bash

colcon build --packages-select \
  banyu_guidance banyu_vehicle_interface banyu_vehicle_execution \
  banyu_communication banyu_mission banyu_detection banyu_bringup
source install/setup.bash
```
*Result:* All packages built successfully with 0 errors.

### 2.2 Sequential Test Suite
```bash
ROS_DOMAIN_ID=42 colcon test --executor sequential \
  --packages-select banyu_msgs banyu_guidance banyu_vehicle_interface \
    banyu_vehicle_execution banyu_communication banyu_mission \
    banyu_detection banyu_bringup \
  --ctest-args -j1 --return-code-on-test-failure

colcon test-result --test-result-base build --verbose
```
*Result:* `Summary: 667 tests, 0 errors, 0 failures, 154 skipped.`

### 2.3 C++ gtest Breakdown by Package
| Package | C++ gtest Cases | Failures | Errors | Status |
| :--- | :--- | :--- | :--- | :--- |
| `banyu_msgs` | 1 | 0 | 0 | PASS |
| `banyu_guidance` | 8 | 0 | 0 | PASS |
| `banyu_vehicle_interface` | 7 | 0 | 0 | PASS |
| `banyu_vehicle_execution` | 74 | 0 | 0 | PASS |
| `banyu_communication` | 70 | 0 | 0 | PASS |
| `banyu_mission` | 70 | 0 | 0 | PASS |
| `banyu_detection` | 42 | 0 | 0 | PASS |
| `banyu_bringup` | 0 (Python/lint) | 0 | 0 | PASS |
| **Total** | **272** | **0** | **0** | **PASS** |

*Note on delta from baseline:* The baseline registered 259 C++ cases. 13 new C++ test cases were added and verified:
- `banyu_vehicle_execution`: +2 cases (`ActiveHeartbeatCannotReplaceSurfaceProof`, `EpochChangeWithinSameRunPreservesCalibration`)
- `banyu_communication`: +5 cases (`RestartMustNotForwardRetainedReportTwice`, `DurableSealSurvivesAbortBeforeResponse`, `AbortBeforeDurableSealRejectsReportAndUnstages`, `ConflictingPayloadForSameRunEpochKindIsRejected`, `ReusedReportIdForDifferentRunIsRejected`)
- `banyu_detection`: +6 cases (5 in `test_ring_tracker.cpp`: `BentPipelineCenterlineMonotonicProgression`, `BackwardRevisitReusesNodeIdentityAndPreservesCount`, `PostRepairRevisitPreservesFirstQualifiedCondition`, `PhysicalIdentityUnverifiedWithoutProgression`, `RejectManufacturingFifthNode`; 1 in `test_survey_evidence_node.cpp`: `ConflictingMidRunCourseAndEndpointUpdatesAreRejected`)

### 2.4 Communication Smoke Test
```bash
ROS_DOMAIN_ID=42 python3 src/BANYU_ROBOTX/banyu_communication/test/sos_gcs_mock.py
```
*Result:* All 6 checks PASS:
- Step 1/6 PASS: UUV heartbeat received
- Step 2/6 PASS: Heartbeat reflects simulated vehicle telemetry
- Step 3/6 PASS: Mission lifecycle updated and running status received
- Step 4/6 PASS: Survey received with buoy=(33.1, -118.1), segments=[2, 1, 1, 1]
- Step 5/6 PASS: Incident acknowledgement and readiness match the assistance command
- Step 6/6 PASS: Supported stop commands trigger mission abort

### 2.5 Linter Checks
- `ament_uncrustify`: 0 code style divergences across all touched C++ source and header files.
- `ament_flake8`: 0 errors on Python test scripts.
- `ament_pep257`: 0 docstring violations on Python test scripts.

---

## 3. Acceptance Gates Evaluation (S1–S8)

### S1: Four Original Conditions & Both Active Endpoints
- **Requirement:** Support four original node conditions across all permutations of RED/GREEN, and both pipeline endpoints (start / end) as the active buoy endpoint.
- **Evidence:** Verified in `banyu_communication/test/test_survey_reporting.cpp` (`FourNodePermutationsWithActiveBuoyAtEitherEndpoint`, `BuoyFrequencyCourseAndEndpointProofAreRequired`) and `test_survey_reporting_core.cpp`.
- **Verdict:** **PASS** (Unit & Integration Fixtures).

### S2: Rejection of Missing, Ambiguous, Stale, or Conflicting Evidence
- **Requirement:** No filler data or report generation for incomplete or ambiguous evidence; conflicting mid-run course or endpoint changes rejected. Valid evidence survives retries.
- **Evidence:**
  - `banyu_detection/src/survey_evidence_node.cpp` enforces checks rejecting conflicting `course_id` and conflicting `endpoint_id` once ring records have begun accumulating. Tested in `test_survey_evidence_node.cpp` (`ConflictingMidRunCourseAndEndpointUpdatesAreRejected`).
  - `banyu_communication/src/survey_reporting.cpp` (`SurveyLedger`) rejects unverified topology, missing buoy fix, invalid colors, or out-of-order nodes.
- **Verdict:** **PASS** (Unit & Integration Fixtures).

### S3: Topology Order from Observed Active Endpoint & Bent Pipe Revisits
- **Requirement:** Pipeline ordering based on polyline centerline topology progression, not vehicle Euclidean travel. Reversing/looping must not advance node progression or create duplicate nodes.
- **Evidence:**
  - `banyu_detection/src/ring_tracker.cpp`: Implemented `projectOntoCenterline()` along polyline segments, topological progress tracking, node association within `node_association_distance_m` (default 0.8m), and physical rearm distance gating.
  - `RingTrackerNode`: Now computes polyline centerline projection and provides `path_distance_m` and `has_path_distance = true` to `RingTracker`.
  - Tested in `test_ring_tracker.cpp` (`BentPipelineCenterlineMonotonicProgression`, `BackwardRevisitReusesNodeIdentityAndPreservesCount`, `RejectManufacturingFifthNode`, `PhysicalIdentityUnverifiedWithoutProgression`).
- **Verdict:** **PASS** (Unit & Tracker Fixtures).

### S4: Original RED/GREEN Frozen Across Repair and Optical Colors
- **Requirement:** Once a qualified initial ring color is recorded in the survey ledger, subsequent revisits (post-repair or altered illumination) must retain the original qualified observation.
- **Evidence:**
  - `RingTracker::accept()` on association match preserves the initial record's color.
  - Tested in `test_ring_tracker.cpp` (`PostRepairRevisitPreservesFirstQualifiedCondition`) and `test_survey_reporting_core.cpp` (`LedgerFreezesInitialColor`).
- **Verdict:** **PASS** (Unit Fixtures).

### S5: Gating Against Premature or Failed-Run Surfacing and Reporting
- **Requirement:** Surface action and report preparation must only succeed if mission survey is complete and run lifecycle is valid. Aborted or failed runs must reject reporting.
- **Evidence:**
  - `banyu_vehicle_execution/src/action_servers/surface_server.cpp` and `banyu_communication/src/uuv_comm_node.cpp` enforce phase and survey completeness prerequisites.
  - Tested in `test_survey_reporting.cpp` (`PrematureSurfacingWithoutSurveyCompleteRejected`, `FailedRunSurfacingRejected`) and `test_run_authorization.cpp`.
- **Verdict:** **PASS** (Integration Fixtures).

### S6: Verified Water-Surface Datum & Continuous Hold
- **Requirement:** Water-surface calibration requires verified surface evidence during pre-ACTIVE phases. An ACTIVE heartbeat must never auto-calibrate depth without surface proof. Resurfacing requires continuous hold within threshold. Epoch updates within the same run must not invalidate established calibration.
- **Evidence:**
  - `banyu_vehicle_execution/src/calibrated_depth_node.cpp`: Eliminated auto-calibration in `handleLifecycle()`. Constrained auto-calibration in `handleRelAlt()` to `current_phase_ < RunLifecycle::ACTIVE && isNearSurface(alt)`. Decoupled `new_run` strictly to `run_id` changes.
  - Tested in `test_calibrated_depth_node.cpp` (`ActiveHeartbeatCannotReplaceSurfaceProof`, `StartupUnderwaterMustNotBecomeSurfaceDatum`, `MidRunRestartUnderwaterDoesNotRecalibrate`, `EpochChangeWithinSameRunPreservesCalibration`).
- **Verdict:** **PASS** (Unit Fixtures).

### S7: Durable Seal Outbox Transactions Surviving Abort
- **Requirement:** A report durably sealed by the RunCoordinator prior to abort must remain recoverable in the outbox under its original run identity. If abort occurs before sealing, the transaction is rejected and unstaged. Delayed positive seal response after abort must not broadcast over the live network.
- **Evidence:**
  - `banyu_communication/src/uuv_comm_node.cpp`: Abort reconciliation unstages pending surveys that were never sent for sealing, but preserves in-flight requests for authority resolution. If sealed before abort, the report is authorized in the outbox. Live network submission publication is guarded against post-abort broadcast.
  - Tested in `test_survey_reporting.cpp` (`DurableSealSurvivesAbortBeforeResponse`, `AbortBeforeDurableSealRejectsReportAndUnstages`, `DelayedPositiveAuthorizationCannotPublishAfterAbort`, `DelayedPreviousRunAuthorizationCannotEnterNewEpoch`).
- **Verdict:** **PASS** (Integration Fixtures).

### S8: Durable GCS Ingestion Deduplication Across Restarts
- **Requirement:** Retained replay of submissions after adapter restart must not generate duplicate official reports; deduplication state must survive adapter restarts, scoping identity to `(run_id, epoch, kind)` and `report_id`.
- **Evidence:**
  - `banyu_communication/src/gcs_submission_adapter.cpp`: Implemented disk journal (`journal_path` parameter, default `~/.ros/banyu/gcs_adapter_dedup.journal`). Indexed by `LogicalKey` `(run_id, epoch, kind)` and unique `report_id`.
  - Tested in `test_gcs_submission_adapter.cpp` (`RestartMustNotForwardRetainedReportTwice`, `ConflictingPayloadForSameRunEpochKindIsRejected`, `ReusedReportIdForDifferentRunIsRejected`).
  - **Verdict:** **PASS (Local / Emulated GCS)**. Live external Team GCS wire validation remains **PENDING** as tracked under E2.

---

## 4. System Boundaries: Live Nodes vs. Test Doubles / Mocks

| Component / Subsystem | Implementation Type in Step 2 Verification | Notes / Scope Boundary |
| :--- | :--- | :--- |
| `CalibratedDepthNode` | Live C++ ROS 2 Node | Executed and tested with synthetic depth/alt telemetry. |
| `UuvCommNode` | Live C++ ROS 2 Node | Executed and tested with live DDS publishers/subscribers. |
| `GcsSubmissionAdapter` | Live C++ ROS 2 Node | Executed with disk journal persistence and live DDS topics. |
| `RingTracker` / `RingTrackerNode` | Live C++ Class & ROS 2 Node | Centerline projection and topological association verified. |
| `SurveyEvidenceNode` | Live C++ ROS 2 Node | Verified with simulated detections and course QoS subscription. |
| `RunCoordinator` | Live C++ Coordinator | Verified with real SQLite journal backing and seal transactions. |
| MAVROS / ArduSub SITL | Mocked / Injected Telemetry | Real physics SITL supervisor tracked under Step 3 (E1). |
| Camera / Perception Pipeline | Synthetic Ring Records | Raw image segmentation / ring detector tracked under Step 4. |
| Acoustic Pinger / Hydrophones | Synthetic Buoy Fix | Real acoustic homing tracked under Step 4. |
| Physical GCS Wire Transport | Mocked via `sos_gcs_mock.py` | Official multi-vehicle field testing tracked under Step 5 (E2). |

---

## 5. Files Modified

1. `banyu_bringup/config/task2_params.yaml`: Added `node_association_distance_m`, `centerline_points`, and `journal_path` configurations.
2. `banyu_communication/CMakeLists.txt`: Exported library dependencies for test fixtures.
3. `banyu_communication/package.xml`: Added required test/build dependencies (`banyu_mission`).
4. `banyu_communication/include/banyu_communication/gcs_submission_adapter.hpp`: Added logical identity `AcceptedReport` structure, `(run_id, epoch, kind)` and `report_id` maps, and journal methods.
5. `banyu_communication/src/gcs_submission_adapter.cpp`: Implemented scoped deduplication, conflict/reuse rejection, and V2 disk journal load/append.
6. `banyu_communication/src/uuv_comm_node.cpp`: Retained course QoS (`transient_local`), non-destructive abort reconciliation distinguishing in-flight vs unsealed reports, outbox authorization preservation, and post-abort publication guarding.
7. `banyu_communication/test/sos_gcs_mock.py`: Aligned course publisher QoS to `transient_local`.
8. `banyu_communication/test/test_gcs_submission_adapter.cpp`: Added `RestartMustNotForwardRetainedReportTwice`, `ConflictingPayloadForSameRunEpochKindIsRejected`, and `ReusedReportIdForDifferentRunIsRejected` with isolated test journal paths.
9. `banyu_communication/test/test_survey_reporting.cpp`: Added `DurableSealSurvivesAbortBeforeResponse` and `AbortBeforeDurableSealRejectsReportAndUnstages`.
10. `banyu_detection/include/banyu_detection/ring_tracker.hpp`: Added centerline waypoints, topological projection, and association methods.
11. `banyu_detection/include/banyu_detection/ring_tracker_adapter.hpp`: Exposed centerline and association methods.
12. `banyu_detection/src/ring_tracker.cpp`: Implemented polyline projection, rearm tracking, node reuse, 5th-node rejection, and color preservation.
13. `banyu_detection/src/ring_tracker_node.cpp`: Added centerline parameter parsing and progression wiring.
14. `banyu_detection/src/survey_evidence_node.cpp`: Added course QoS `transient_local` and mid-run conflict rejection.
15. `banyu_detection/test/test_ring_tracker.cpp`: Added 5 unit tests for centerline, bent pipes, revisits, 5th-node rejection, and color freezing.
16. `banyu_detection/test/test_survey_evidence_node.cpp`: Added mid-run course and endpoint conflict rejection tests.
17. `banyu_mission/CMakeLists.txt`: Exported `banyu_mission_core` library target.
18. `banyu_vehicle_execution/src/calibrated_depth_node.cpp`: Removed ACTIVE heartbeat auto-calibration; constrained `handleRelAlt`; separated `new_run` by `run_id`.
19. `banyu_vehicle_execution/test/test_calibrated_depth_node.cpp`: Added `ActiveHeartbeatCannotReplaceSurfaceProof` and `EpochChangeWithinSameRunPreservesCalibration`.
