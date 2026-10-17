#include "banyu_detection/ring_tracker_adapter.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{

banyu_detection::RingTrackerInput coloredInput(uint64_t sequence, uint8_t color, double x)
{
  banyu_detection::RingTrackerInput input;
  input.observation.header.stamp.sec = static_cast<int32_t>(sequence);
  input.observation.frame_sequence = sequence;
  input.observation.visible = true;
  input.observation.color = color;
  input.observation.confidence = 0.9F;
  input.observation.offset_x = 0.1F;
  input.observation.offset_y = -0.1F;
  input.vehicle_pose_map.header.frame_id = "map";
  input.vehicle_pose_map.header.stamp = input.observation.header.stamp;
  input.vehicle_pose_map.pose.position.x = x;
  input.vehicle_pose_map.pose.orientation.w = 1.0;
  input.has_vehicle_pose = true;
  return input;
}

banyu_detection::RingTrackerInput redInput(uint64_t sequence, double x = 0.0)
{
  return coloredInput(
    sequence, banyu_msgs::msg::PipelineRingObservation::COLOR_RED, x);
}

TEST(RingTrackerAdapter, AcknowledgesOneAcceptedRedRecordOnce)
{
  banyu_detection::RingTrackerConfig config;
  config.entry_confirm_frames = 1;
  banyu_detection::RingTrackerAdapter adapter(config);
  EXPECT_TRUE(adapter.update(redInput(1)).accepted);

  const auto first = adapter.acknowledgeRingRepair(0);
  EXPECT_TRUE(first.success);
  EXPECT_EQ(first.accepted_ring_index, 0U);
  EXPECT_TRUE(adapter.tracker().repaired());
  EXPECT_FALSE(adapter.tracker().redPending());
  EXPECT_FALSE(adapter.update(redInput(2)).accepted);
  EXPECT_FALSE(adapter.tracker().redPending());

  const auto second = adapter.acknowledgeRingRepair(0);
  EXPECT_FALSE(second.success);
}

TEST(RingTrackerAdapter, RejectsWrongOrStaleRepairIndex)
{
  banyu_detection::RingTrackerConfig config;
  config.entry_confirm_frames = 1;
  config.exit_confirm_frames = 1;
  config.debounce_frames = 1;
  banyu_detection::RingTrackerAdapter adapter(config);
  EXPECT_TRUE(
    adapter.update(
      coloredInput(
        1, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN, 0.0)).accepted);

  auto invisible = coloredInput(2, banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN, 0.0);
  invisible.observation.visible = false;
  invisible.observation.confidence = 0.0F;
  invisible.observation.offset_x = std::numeric_limits<float>::quiet_NaN();
  invisible.observation.offset_y = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(adapter.update(invisible).accepted);
  invisible.observation.frame_sequence = 3;
  invisible.observation.header.stamp.sec = 3;
  invisible.vehicle_pose_map.header.stamp = invisible.observation.header.stamp;
  EXPECT_FALSE(adapter.update(invisible).accepted);
  EXPECT_TRUE(adapter.update(redInput(4, 1.0)).accepted);

  const auto wrong = adapter.acknowledgeRingRepair(0);
  EXPECT_FALSE(wrong.success);
  EXPECT_TRUE(adapter.tracker().redPending());

  const auto stale = adapter.acknowledgeRingRepair(banyu_detection::kInvalidRingIndex);
  EXPECT_FALSE(stale.success);
  EXPECT_TRUE(adapter.tracker().redPending());
}

TEST(RingTrackerAdapter, ResetClearsRecordsSequenceAndRepairAcknowledgement)
{
  banyu_detection::RingTrackerConfig config;
  config.entry_confirm_frames = 1;
  banyu_detection::RingTrackerAdapter adapter(config);
  auto first = redInput(1);
  first.run_id = 7;
  first.epoch = 12;
  ASSERT_TRUE(adapter.update(first).accepted);
  ASSERT_TRUE(adapter.acknowledgeRingRepair(0).success);
  EXPECT_EQ(adapter.tracker().records().front().color,
    banyu_msgs::msg::PipelineRingObservation::COLOR_RED);

  adapter.reset();
  EXPECT_TRUE(adapter.tracker().records().empty());
  EXPECT_FALSE(adapter.tracker().repaired());
  EXPECT_FALSE(adapter.tracker().redPending());
  auto next = redInput(1);
  next.run_id = 3;
  next.epoch = 13;
  const auto accepted = adapter.update(next);
  ASSERT_TRUE(accepted.accepted);
  EXPECT_EQ(accepted.record.run_id, 3U);
  EXPECT_EQ(accepted.record.epoch, 13U);
  EXPECT_EQ(accepted.record.node_id, 0U);
  EXPECT_FALSE(accepted.record.ordering_verified);
  EXPECT_TRUE(adapter.acknowledgeRingRepair(0).success);
}

}  // namespace
