#include "banyu_detection/ring_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{

banyu_msgs::msg::PipelineRingObservation ring(
  uint64_t sequence, uint8_t color,
  bool visible = true)
{
  banyu_msgs::msg::PipelineRingObservation result;
  result.frame_sequence = sequence;
  result.header.stamp.sec = static_cast<int32_t>(sequence);
  result.visible = visible;
  result.color = visible ? color :
    banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  result.confidence = visible ? 0.9F : 0.0F;
  result.offset_x = visible ? 0.1F : std::numeric_limits<float>::quiet_NaN();
  result.offset_y = visible ? -0.1F : std::numeric_limits<float>::quiet_NaN();
  return result;
}

banyu_detection::RingTrackerUpdate update(
  banyu_detection::RingTracker & tracker,
  uint64_t sequence, uint8_t color,
  bool visible = true,
  double path_distance = 0.0)
{
  banyu_detection::RingTrackerInput input;
  input.observation = ring(sequence, color, visible);
  input.has_path_distance = true;
  input.path_distance_m = path_distance;
  input.has_vehicle_pose = true;
  input.vehicle_pose_map.header.frame_id = "map";
  input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
  input.vehicle_pose_map.pose.position.x = path_distance;
  input.vehicle_pose_map.pose.orientation.w = 1.0;
  return tracker.update(input);
}

banyu_detection::RingTrackerConfig trackerConfig(
  std::size_t entry,
  std::size_t exit,
  std::size_t debounce)
{
  banyu_detection::RingTrackerConfig config;
  config.entry_confirm_frames = entry;
  config.exit_confirm_frames = exit;
  config.debounce_frames = debounce;
  return config;
}

TEST(RingTracker, EntryExitDebounceAppendsPhysicalRingOnce) {
  banyu_detection::RingTracker tracker(
    trackerConfig(3, 2, 2));

  EXPECT_FALSE(update(tracker, 1, 1).accepted);
  EXPECT_FALSE(update(tracker, 2, 1).accepted);
  EXPECT_TRUE(update(tracker, 3, 1).accepted);
  EXPECT_EQ(tracker.records().size(), 1U);

  EXPECT_FALSE(update(tracker, 4, 1).accepted);
  EXPECT_EQ(
    update(tracker, 5, 1, false).phase,
    banyu_detection::RingTrackerPhase::EXIT);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 1U);
  EXPECT_EQ(
    update(tracker, 6, 1, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 0U);
  EXPECT_EQ(
    update(tracker, 7, 1, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 1U);
  EXPECT_EQ(
    update(tracker, 8, 1, false).phase,
    banyu_detection::RingTrackerPhase::IDLE);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 0U);
  EXPECT_EQ(tracker.records().size(), 1U);
}

TEST(RingTracker, NewRingRequiresFreshEntryAfterDebounce) {
  auto config = trackerConfig(2, 1, 1);
  config.minimum_rearm_distance_m = 1.0;
  banyu_detection::RingTracker tracker(config);

  EXPECT_FALSE(update(tracker, 1, 2, true, 0.0).accepted);
  EXPECT_TRUE(update(tracker, 2, 2, true, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 3, 2, false, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 4, 2, false, 0.0).accepted);

  EXPECT_FALSE(update(tracker, 5, 3, true, 0.5).accepted);
  EXPECT_FALSE(update(tracker, 6, 3, true, 0.5).accepted);
  EXPECT_FALSE(update(tracker, 7, 3, true, 1.0).accepted);
  EXPECT_TRUE(update(tracker, 8, 3, true, 1.0).accepted);
  EXPECT_EQ(tracker.records().size(), 2U);
}

TEST(RingTracker, RedPendingIsClearedOnlyByExplicitRepair) {
  banyu_detection::RingTracker tracker(
    trackerConfig(1, 1, 1));
  EXPECT_TRUE(update(tracker, 1, banyu_msgs::msg::PipelineRingObservation::COLOR_RED).accepted);
  EXPECT_TRUE(tracker.redPending());
  EXPECT_FALSE(tracker.repaired());

  EXPECT_TRUE(tracker.markRedRepaired());
  EXPECT_FALSE(tracker.redPending());
  EXPECT_TRUE(tracker.repaired());
  EXPECT_FALSE(tracker.markRedRepaired());
}

TEST(RingTracker, DuplicateRedIsNotAppended) {
  auto config = trackerConfig(1, 1, 1);
  config.minimum_rearm_distance_m = 1.0;
  banyu_detection::RingTracker tracker(config);
  EXPECT_TRUE(update(tracker, 1, 1, true, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 2, 1, false, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 3, 1, false, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 4, 1, true, 1.0).accepted);
  EXPECT_EQ(tracker.records().size(), 1U);
}

TEST(RingTracker, SameRingReacquisitionWithoutSeparationIsRejected) {
  auto config = trackerConfig(1, 1, 1);
  config.minimum_rearm_distance_m = 0.25;
  banyu_detection::RingTracker tracker(config);
  EXPECT_TRUE(update(tracker, 1, 2, true, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 2, 2, false, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 3, 2, false, 0.0).accepted);
  EXPECT_FALSE(update(tracker, 4, 2, true, 0.0).accepted);
  EXPECT_EQ(tracker.records().size(), 1U);
}

TEST(RingTracker, MalformedAndUncertainFramesDoNotCountAsExit) {
  auto config = trackerConfig(1, 2, 2);
  banyu_detection::RingTracker tracker(config);
  EXPECT_TRUE(update(tracker, 1, 2).accepted);

  auto malformed = ring(2, 2, false);
  malformed.color = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  EXPECT_EQ(
    tracker.update(malformed).phase,
    banyu_detection::RingTrackerPhase::TRACKING);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 0U);

  auto uncertain = ring(3, 2, true);
  uncertain.confidence = 0.01F;
  EXPECT_EQ(
    tracker.update(uncertain).phase,
    banyu_detection::RingTrackerPhase::TRACKING);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 0U);

  EXPECT_EQ(
    update(tracker, 4, 2, false).phase,
    banyu_detection::RingTrackerPhase::EXIT);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 1U);
}

TEST(RingTracker, MissingOrMisalignedPoseCannotAppend) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  auto observation = ring(1, 2);
  banyu_detection::RingTrackerInput missing_pose;
  missing_pose.observation = observation;
  EXPECT_FALSE(tracker.update(missing_pose).accepted);

  banyu_detection::RingTrackerInput misaligned_pose;
  misaligned_pose.observation = ring(2, 2);
  misaligned_pose.has_vehicle_pose = true;
  misaligned_pose.vehicle_pose_map.header.frame_id = "map";
  misaligned_pose.vehicle_pose_map.header.stamp.sec = 99;
  misaligned_pose.vehicle_pose_map.pose.orientation.w = 1.0;
  EXPECT_FALSE(tracker.update(misaligned_pose).accepted);
  EXPECT_TRUE(tracker.records().empty());
}

TEST(RingTracker, RegressedSequenceIsIgnored) {
  banyu_detection::RingTracker tracker(
    trackerConfig(2, 1, 1));
  EXPECT_FALSE(update(tracker, 10, 3).accepted);
  EXPECT_TRUE(update(tracker, 9, 3).ignored);
  EXPECT_TRUE(update(tracker, 11, 3).accepted);
  EXPECT_EQ(tracker.records().size(), 1U);
}

TEST(RingTracker, SequenceGapRestartsExitWithoutCountingAbsence) {
  auto config = trackerConfig(1, 2, 2);
  banyu_detection::RingTracker tracker(config);
  EXPECT_TRUE(update(tracker, 1, 2).accepted);

  EXPECT_EQ(
    update(tracker, 3, 2, false).phase,
    banyu_detection::RingTrackerPhase::EXIT);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 0U);
  EXPECT_EQ(
    update(tracker, 4, 2, false).phase,
    banyu_detection::RingTrackerPhase::EXIT);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 1U);
  EXPECT_EQ(
    update(tracker, 5, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
}

TEST(RingTracker, VisibleInterruptionRestartsDebounce) {
  auto config = trackerConfig(1, 1, 2);
  banyu_detection::RingTracker tracker(config);
  EXPECT_TRUE(update(tracker, 1, 2).accepted);
  EXPECT_EQ(
    update(tracker, 2, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(
    update(tracker, 3, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 1U);

  EXPECT_EQ(
    update(tracker, 4, 3, true).phase,
    banyu_detection::RingTrackerPhase::EXIT);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 0U);
  EXPECT_EQ(
    update(tracker, 5, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(
    update(tracker, 6, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 1U);
  EXPECT_EQ(
    update(tracker, 7, 2, false).phase,
    banyu_detection::RingTrackerPhase::IDLE);
}

TEST(RingTracker, MalformedGapCannotEnableRearm) {
  auto config = trackerConfig(1, 1, 2);
  config.minimum_rearm_distance_m = 0.25;
  banyu_detection::RingTracker tracker(config);
  EXPECT_TRUE(update(tracker, 1, 2).accepted);
  EXPECT_EQ(
    update(tracker, 2, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);

  auto malformed_gap = ring(4, 2, false);
  malformed_gap.color = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  EXPECT_EQ(
    tracker.update(malformed_gap).phase,
    banyu_detection::RingTrackerPhase::EXIT);
  EXPECT_EQ(tracker.snapshot().invisible_frames, 0U);

  EXPECT_EQ(
    update(tracker, 5, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(
    update(tracker, 6, 2, false).phase,
    banyu_detection::RingTrackerPhase::DEBOUNCE);
  EXPECT_EQ(
    update(tracker, 7, 2, false).phase,
    banyu_detection::RingTrackerPhase::IDLE);
  EXPECT_FALSE(update(tracker, 8, 2, true, 0.0).accepted);
  EXPECT_EQ(tracker.records().size(), 1U);
}

TEST(RingTracker, BentPipelineCenterlineMonotonicProgression) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  tracker.setCenterline({
      {0.0, 0.0},
      {10.0, 0.0},
      {10.0, 5.0},
      {0.0, 5.0}
  });
  EXPECT_TRUE(tracker.hasCenterline());

  // Segment 0: (0,0) to (10,0)
  EXPECT_NEAR(tracker.projectOntoCenterline(2.0, 0.0), 2.0, 1e-4);
  // Segment 1: (10,0) to (10,5)
  EXPECT_NEAR(tracker.projectOntoCenterline(10.0, 2.0), 12.0, 1e-4);
  // Segment 2: (10,5) to (0,5)
  EXPECT_NEAR(tracker.projectOntoCenterline(8.0, 5.0), 17.0, 1e-4);
  EXPECT_NEAR(tracker.projectOntoCenterline(2.0, 5.0), 23.0, 1e-4);
}

TEST(RingTracker, BackwardRevisitReusesNodeIdentityAndPreservesCount) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  tracker.setCenterline({
      {0.0, 0.0},
      {10.0, 0.0}
  });

  auto make_input = [](uint64_t seq, uint8_t color, double x, bool visible = true) {
      banyu_detection::RingTrackerInput input;
      input.observation = ring(seq, color, visible);
      input.has_vehicle_pose = true;
      input.vehicle_pose_map.header.frame_id = "map";
      input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
      input.vehicle_pose_map.pose.position.x = x;
      input.vehicle_pose_map.pose.position.y = 0.0;
      input.vehicle_pose_map.pose.orientation.w = 1.0;
      return input;
    };

  // 1. Accept first ring (GREEN) at x=1.0
  auto u1 = tracker.update(make_input(1, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN,
      1.0));
  ASSERT_TRUE(u1.accepted);
  EXPECT_EQ(u1.record.node_id, 0U);
  EXPECT_EQ(u1.record.color, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN);
  EXPECT_TRUE(u1.record.physical_identity_verified);
  EXPECT_TRUE(u1.record.ordering_verified);
  EXPECT_EQ(tracker.records().size(), 1U);

  // Exit and debounce Ring 0
  EXPECT_FALSE(tracker.update(make_input(2, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN,
      1.0, false)).accepted);
  EXPECT_FALSE(tracker.update(make_input(3, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN,
      1.0, false)).accepted);

  // 2. Accept second ring (RED) at x=3.0
  auto u2 = tracker.update(make_input(4, banyu_msgs::msg::PipelineRingObservation::COLOR_RED, 3.0));
  ASSERT_TRUE(u2.accepted);
  EXPECT_EQ(u2.record.node_id, 1U);
  EXPECT_EQ(u2.record.color, banyu_msgs::msg::PipelineRingObservation::COLOR_RED);
  EXPECT_TRUE(u2.record.physical_identity_verified);
  EXPECT_TRUE(u2.record.ordering_verified);
  EXPECT_EQ(tracker.records().size(), 2U);

  // Exit and debounce Ring 1
  EXPECT_FALSE(tracker.update(make_input(5, banyu_msgs::msg::PipelineRingObservation::COLOR_RED,
      3.0, false)).accepted);
  EXPECT_FALSE(tracker.update(make_input(6, banyu_msgs::msg::PipelineRingObservation::COLOR_RED,
      3.0, false)).accepted);

  // 3. Vehicle backs up to x=1.0 and revisits Ring 0
  auto u3 = tracker.update(make_input(7, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN,
      1.0));
  ASSERT_TRUE(u3.accepted);
  EXPECT_EQ(u3.record.node_id, 0U);
  EXPECT_EQ(u3.record.color, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN);
  EXPECT_TRUE(u3.record.physical_identity_verified);
  // Records count must NOT increase (no 3rd node manufactured on revisit)
  EXPECT_EQ(tracker.records().size(), 2U);
}

TEST(RingTracker, PostRepairRevisitPreservesFirstQualifiedCondition) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  tracker.setCenterline({
      {0.0, 0.0},
      {10.0, 0.0}
  });

  auto make_input = [](uint64_t seq, uint8_t color, double x, bool visible = true) {
      banyu_detection::RingTrackerInput input;
      input.observation = ring(seq, color, visible);
      input.has_vehicle_pose = true;
      input.vehicle_pose_map.header.frame_id = "map";
      input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
      input.vehicle_pose_map.pose.position.x = x;
      input.vehicle_pose_map.pose.position.y = 0.0;
      input.vehicle_pose_map.pose.orientation.w = 1.0;
      return input;
    };

  // Ring 0 is originally RED
  auto u1 = tracker.update(make_input(1, banyu_msgs::msg::PipelineRingObservation::COLOR_RED, 1.0));
  ASSERT_TRUE(u1.accepted);
  EXPECT_EQ(u1.record.node_id, 0U);
  EXPECT_EQ(u1.record.color, banyu_msgs::msg::PipelineRingObservation::COLOR_RED);
  EXPECT_TRUE(tracker.redPending());

  // Explicit repair succeeds
  EXPECT_TRUE(tracker.markRedRepaired());
  EXPECT_TRUE(tracker.repaired());

  // Vehicle exits ring
  EXPECT_FALSE(tracker.update(make_input(2, banyu_msgs::msg::PipelineRingObservation::COLOR_RED,
      1.0, false)).accepted);
  EXPECT_FALSE(tracker.update(make_input(3, banyu_msgs::msg::PipelineRingObservation::COLOR_RED,
      1.0, false)).accepted);

  // Move away
  EXPECT_FALSE(tracker.update(make_input(4, banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN,
      2.0, false)).accepted);

  // Return to x=1.0: physically repaired, so observation is now GREEN
  auto u2 = tracker.update(make_input(5, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN,
      1.0));
  ASSERT_TRUE(u2.accepted);
  EXPECT_EQ(u2.record.node_id, 0U);
  // Crucial: original condition remains RED
  EXPECT_EQ(u2.record.color, banyu_msgs::msg::PipelineRingObservation::COLOR_RED);
  EXPECT_EQ(tracker.records().size(), 1U);
}

TEST(RingTracker, PhysicalIdentityUnverifiedWithoutProgression) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  // No centerline configured, and input does NOT supply has_path_distance
  banyu_detection::RingTrackerInput input;
  input.observation = ring(1, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN);
  input.has_vehicle_pose = true;
  input.vehicle_pose_map.header.frame_id = "map";
  input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
  input.vehicle_pose_map.pose.position.x = 1.0;
  input.vehicle_pose_map.pose.orientation.w = 1.0;
  input.has_path_distance = false;

  auto update = tracker.update(input);
  ASSERT_TRUE(update.accepted);
  EXPECT_FALSE(update.record.physical_identity_verified);
  EXPECT_FALSE(update.record.ordering_verified);
}

TEST(RingTracker, RejectManufacturingFifthNode) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  tracker.setCenterline({
      {0.0, 0.0},
      {20.0, 0.0}
  });

  auto make_input = [](uint64_t seq, uint8_t color, double x, bool visible = true) {
      banyu_detection::RingTrackerInput input;
      input.observation = ring(seq, color, visible);
      input.has_vehicle_pose = true;
      input.vehicle_pose_map.header.frame_id = "map";
      input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
      input.vehicle_pose_map.pose.position.x = x;
      input.vehicle_pose_map.pose.position.y = 0.0;
      input.vehicle_pose_map.pose.orientation.w = 1.0;
      return input;
    };

  uint64_t seq = 1;
  const std::vector<double> positions = {1.0, 3.0, 5.0, 7.0};
  for (double pos : positions) {
    EXPECT_TRUE(tracker.update(make_input(seq++,
        banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN, pos)).accepted);
    EXPECT_FALSE(tracker.update(make_input(seq++,
        banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN, pos, false)).accepted);
    EXPECT_FALSE(tracker.update(make_input(seq++,
        banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN, pos, false)).accepted);
  }
  EXPECT_EQ(tracker.records().size(), 4U);

  // Now attempt to record a 5th node at x=9.0
  auto u5 = tracker.update(make_input(seq++, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN,
      9.0));
  EXPECT_FALSE(u5.accepted);
  EXPECT_EQ(tracker.records().size(), 4U);
}

}  // namespace

TEST(RingTracker, NextRingVisibleBeforePreviousLeavesIsAcceptedAfterMoving) {
  auto config = trackerConfig(3, 2, 2);
  config.minimum_rearm_distance_m = 0.25;
  config.node_association_distance_m = 0.4;
  banyu_detection::RingTracker tracker(config);
  using Obs = banyu_msgs::msg::PipelineRingObservation;
  uint64_t seq = 1;
  for (int i = 0; i < 3; ++i) {
    update(tracker, seq++, Obs::COLOR_RED, true, 0.0);
  }
  ASSERT_EQ(tracker.records().size(), 1U);
  // Green comes into view while red is still tracked and never leaves the frame.
  EXPECT_FALSE(update(tracker, seq++, Obs::COLOR_GREEN, true, 0.05).accepted);
  EXPECT_FALSE(update(tracker, seq++, Obs::COLOR_GREEN, true, 0.1).accepted);
  EXPECT_EQ(tracker.records().size(), 1U);  // not yet moved past the red record
  bool accepted = false;
  for (double distance = 0.3; distance <= 0.9; distance += 0.05) {
    accepted = update(tracker, seq++, Obs::COLOR_GREEN, true, distance).accepted || accepted;
  }
  EXPECT_TRUE(accepted);
  ASSERT_EQ(tracker.records().size(), 2U);
  EXPECT_EQ(tracker.records().back().color, Obs::COLOR_GREEN);
}

TEST(RingTracker, CameraModelSeparatesSameColorRingsSeenBackToBack) {
  auto config = trackerConfig(3, 2, 2);
  config.minimum_rearm_distance_m = 0.25;
  config.node_association_distance_m = 0.4;
  config.camera_height_m = 1.7;
  config.camera_forward_m = 0.2;
  config.camera_tilt_from_vertical_rad = 0.227;
  config.camera_half_vfov_rad = 0.408;
  banyu_detection::RingTracker tracker(config);
  using Obs = banyu_msgs::msg::PipelineRingObservation;
  const std::vector<double> rings{1.0, 1.85};
  uint64_t seq = 1;
  for (double p = -0.6; p <= 2.2; p += 0.02) {
    // Image row of each ring as seen from vehicle position p; feed the one nearest center.
    double best_offset = 2.0;
    for (double ring_position : rings) {
      const double angle = std::atan((ring_position - p - 0.2) / 1.7);
      const double offset = (0.227 - angle) / 0.408;
      if (std::abs(offset) <= 1.0 && std::abs(offset) < std::abs(best_offset)) {
        best_offset = offset;
      }
    }
    banyu_detection::RingTrackerInput input;
    const bool visible = std::abs(best_offset) <= 1.0;
    input.observation = ring(seq, Obs::COLOR_GREEN, visible);
    if (visible) {input.observation.offset_y = static_cast<float>(best_offset);}
    input.has_path_distance = true;
    input.path_distance_m = p;
    input.has_vehicle_pose = true;
    input.vehicle_pose_map.header.frame_id = "map";
    input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
    input.vehicle_pose_map.pose.position.x = p;
    input.vehicle_pose_map.pose.orientation.w = 1.0;
    tracker.update(input);
    ++seq;
  }
  ASSERT_EQ(tracker.records().size(), 2U);
  EXPECT_NEAR(tracker.records()[0].pipeline_progress_m, 1.0, 0.1);
  EXPECT_NEAR(tracker.records()[1].pipeline_progress_m, 1.85, 0.1);
}

TEST(RingTracker, GreenThenRedAtTheSameNodeIsOneRedRecord) {
  // The damaged node can first look green and then red: one node, one slot, and the record must end up red.
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  banyu_detection::RingTrackerConfig config = trackerConfig(1, 1, 1);
  config.node_association_distance_m = 0.8;
  banyu_detection::RingTracker wide(config);

  EXPECT_TRUE(update(wide, 1, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN, true, 1.00).accepted);
  EXPECT_EQ(wide.records().size(), 1U);
  EXPECT_EQ(wide.records()[0].color, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN);

  const auto red = update(wide, 2, banyu_msgs::msg::PipelineRingObservation::COLOR_RED, true, 1.60);
  EXPECT_TRUE(red.accepted);
  EXPECT_EQ(wide.records().size(), 1U) << "the same node must not take a second slot";
  EXPECT_EQ(wide.records()[0].color, banyu_msgs::msg::PipelineRingObservation::COLOR_RED);
  EXPECT_TRUE(wide.snapshot().red_pending);
}

TEST(RingTracker, RedRecordIsNeverDowngradedToGreen) {
  banyu_detection::RingTrackerConfig config = trackerConfig(1, 1, 1);
  config.node_association_distance_m = 0.8;
  banyu_detection::RingTracker tracker(config);

  EXPECT_TRUE(update(tracker, 1, banyu_msgs::msg::PipelineRingObservation::COLOR_RED, true, 1.00).accepted);
  update(tracker, 2, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN, true, 1.50);
  EXPECT_EQ(tracker.records().size(), 1U);
  EXPECT_EQ(tracker.records()[0].color, banyu_msgs::msg::PipelineRingObservation::COLOR_RED);
}

TEST(RingTracker, PipeMetFromTheFarEndKeepsOrderingVerifiedInThatDirection) {
  // The UUV may meet the pipe from the end away from the active buoy: the progress then falls from ring to ring. The order is
  // verified in that direction, and a ring that breaks it is still unverified.
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  const auto green = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  uint64_t seq = 1;
  auto meet = [&](double progress) {
      auto u = update(tracker, seq++, green, true, progress);
      update(tracker, seq++, green, false, progress);
      update(tracker, seq++, green, false, progress);
      return u;
    };
  EXPECT_TRUE(meet(4.0).record.ordering_verified);
  EXPECT_TRUE(meet(2.9).record.ordering_verified);   // falls: fixes the direction
  EXPECT_TRUE(meet(1.5).record.ordering_verified);   // keeps falling
  EXPECT_FALSE(meet(6.5).record.ordering_verified);  // jumps back up past the first ring: not in order
}

TEST(RingTracker, MeasuredCameraHeightPlacesRingsWhenTheVehicleIsLowerThanTheConfiguredHeight) {
  // The vehicle follows 1.15 m above the rings, the configured height is 1.7 m; the measured height (pipe width) is given.
  auto config = trackerConfig(3, 2, 2);
  config.minimum_rearm_distance_m = 0.25;
  config.node_association_distance_m = 0.4;
  config.camera_height_m = 1.7;
  config.camera_forward_m = 0.2;
  config.camera_tilt_from_vertical_rad = 0.227;
  config.camera_half_vfov_rad = 0.408;
  banyu_detection::RingTracker tracker(config);
  using Obs = banyu_msgs::msg::PipelineRingObservation;
  const double height = 1.15;
  const std::vector<double> rings{1.0, 1.85};
  uint64_t seq = 1;
  for (double p = -0.6; p <= 2.2; p += 0.02) {
    double best_offset = 2.0;
    for (double ring_position : rings) {
      const double angle = std::atan((ring_position - p - 0.2) / height);
      const double offset = (0.227 - angle) / 0.408;
      if (std::abs(offset) <= 1.0 && std::abs(offset) < std::abs(best_offset)) {best_offset = offset;}
    }
    banyu_detection::RingTrackerInput input;
    const bool visible = std::abs(best_offset) <= 1.0;
    input.observation = ring(seq, Obs::COLOR_GREEN, visible);
    if (visible) {input.observation.offset_y = static_cast<float>(best_offset);}
    input.has_path_distance = true;
    input.path_distance_m = p;
    input.has_vehicle_pose = true;
    input.vehicle_pose_map.header.frame_id = "map";
    input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
    input.vehicle_pose_map.pose.position.x = p;
    input.vehicle_pose_map.pose.orientation.w = 1.0;
    input.camera_height_m = height;
    tracker.update(input);
    ++seq;
  }
  ASSERT_EQ(tracker.records().size(), 2U);
  EXPECT_NEAR(tracker.records()[0].pipeline_progress_m, 1.0, 0.1);
  EXPECT_NEAR(tracker.records()[1].pipeline_progress_m, 1.85, 0.1);
}

TEST(RingTracker, FifthRingMergesADuplicateRecordSoTheRedNodeIsKept) {
  // Disruptive seed 24: one node recorded twice (0.03 and 0.85) filled the four slots before the RED node.
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  const auto green = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  const auto red = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
  uint64_t seq = 1;
  auto meet = [&](uint8_t color, double progress) {
      auto u = update(tracker, seq++, color, true, progress);
      update(tracker, seq++, color, false, progress);
      update(tracker, seq++, color, false, progress);
      return u;
    };
  for (double p : {0.03, 0.85, 2.05, 3.19}) {
    EXPECT_TRUE(meet(green, p).accepted);
  }
  const auto u = meet(red, 4.4);
  ASSERT_TRUE(u.accepted);
  EXPECT_TRUE(u.merged);
  EXPECT_EQ(u.merged_record.node_id, 0U);
  EXPECT_EQ(u.superseded_record.node_id, 1U);
  EXPECT_TRUE(u.superseded_record.superseded);
  // A new identity: the survey ledger keeps the first sighting of each node id.
  EXPECT_EQ(u.record.node_id, 4U);
  EXPECT_TRUE(u.record.ordering_verified);
  ASSERT_EQ(tracker.records().size(), 5U);
  EXPECT_EQ(tracker.liveRecordCount(), 4U);
  EXPECT_TRUE(tracker.records()[1].superseded);
  EXPECT_EQ(tracker.records()[4].color, red);
  EXPECT_NEAR(tracker.records()[4].pipeline_progress_m, 4.4, 1e-9);
  EXPECT_TRUE(tracker.redPending());
}

TEST(RingTracker, MergedPairKeepsTheRedRecordInItsSlot) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  const auto green = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  const auto red = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
  uint64_t seq = 1;
  auto meet = [&](uint8_t color, double progress) {
      auto u = update(tracker, seq++, color, true, progress);
      update(tracker, seq++, color, false, progress);
      update(tracker, seq++, color, false, progress);
      return u;
    };
  meet(green, 0.0);
  meet(green, 1.5);
  meet(red, 2.4);     // 0.9 m from the GREEN at 1.5: the same node, now RED
  meet(green, 3.9);
  const auto u = meet(green, 5.3);
  ASSERT_TRUE(u.merged);
  EXPECT_EQ(u.merged_record.node_id, 2U);
  EXPECT_EQ(u.merged_record.color, red);
  EXPECT_EQ(u.superseded_record.node_id, 1U);
  EXPECT_EQ(u.record.node_id, 4U);
  EXPECT_EQ(tracker.records()[2].color, red);
  EXPECT_FALSE(tracker.records()[2].superseded);
}

TEST(RingTracker, FifthRingIsIgnoredWhenNoRecordsAreCloserThanTheNodeSpacing) {
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  const auto green = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  uint64_t seq = 1;
  auto meet = [&](double progress) {
      auto u = update(tracker, seq++, green, true, progress);
      update(tracker, seq++, green, false, progress);
      update(tracker, seq++, green, false, progress);
      return u;
    };
  for (double p : {0.0, 1.3, 2.6, 3.9}) {
    meet(p);
  }
  EXPECT_FALSE(meet(5.2).accepted);
  EXPECT_EQ(tracker.records().size(), 4U);
}

TEST(RingTracker, NoNewRecordWhileTheCameraIsHighAboveThePipe)
{
  // Midpoint seed 2: three records of one or two nodes in 6 s from 2.8 m above them during the descent.
  banyu_detection::RingTracker tracker(trackerConfig(1, 1, 1));
  const auto green = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  uint64_t seq = 1;
  auto meet = [&](double progress, double height) {
      banyu_detection::RingTrackerInput input;
      input.observation = ring(seq++, green);
      input.has_path_distance = true;
      input.path_distance_m = progress;
      input.has_vehicle_pose = true;
      input.vehicle_pose_map.header.frame_id = "map";
      input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
      input.vehicle_pose_map.pose.position.x = progress;
      input.vehicle_pose_map.pose.orientation.w = 1.0;
      input.camera_height_m = height;
      const auto u = tracker.update(input);
      update(tracker, seq++, green, false, progress);
      update(tracker, seq++, green, false, progress);
      return u;
    };
  EXPECT_FALSE(meet(1.0, 2.8).accepted);
  EXPECT_FALSE(meet(2.5, 2.4).accepted);
  EXPECT_TRUE(meet(4.0, 1.1).accepted);
  EXPECT_TRUE(meet(5.5, 0.0).accepted);   // height unknown: recorded as before
  EXPECT_EQ(tracker.records().size(), 2U);
}
