#include "banyu_detection/pipeline_end_adapter.hpp"

#include <gtest/gtest.h>

namespace
{

banyu_msgs::msg::PipelineObservation observation(
  uint64_t sequence,
  uint8_t status = banyu_msgs::msg::PipelineObservation::TRACKED,
  float offset_x = 0.1F,
  float offset_y = 0.2F,
  float heading = 0.03F,
  float confidence = 0.9F)
{
  banyu_msgs::msg::PipelineObservation obs;
  obs.frame_sequence = sequence;
  obs.status = status;
  obs.offset_x = offset_x;
  obs.offset_y = offset_y;
  obs.heading_error_rad = heading;
  obs.confidence = confidence;
  return obs;
}

banyu_detection::PipelineEndConfirmerConfig endConfig(std::size_t required_frames)
{
  banyu_detection::PipelineEndConfirmerConfig config;
  config.required_frames = required_frames;
  return config;
}

TEST(PipelineEndAdapter, StartsUnconfirmed)
{
  banyu_detection::PipelineEndAdapter adapter(endConfig(3));
  EXPECT_FALSE(adapter.isConfirmed());
  EXPECT_EQ(adapter.confirmer().state(), banyu_detection::PipelineEndState::WAITING);
}

TEST(PipelineEndAdapter, ConfirmsOnStableCandidateFramesAndLatches)
{
  banyu_detection::PipelineEndAdapter adapter(endConfig(3));

  auto res1 = adapter.update(observation(1), true);
  EXPECT_FALSE(res1.confirmed);
  EXPECT_FALSE(res1.confirmed_now);
  EXPECT_EQ(res1.stable_frames, 1U);

  auto res2 = adapter.update(observation(2), true);
  EXPECT_FALSE(res2.confirmed);
  EXPECT_FALSE(res2.confirmed_now);
  EXPECT_EQ(res2.stable_frames, 2U);

  auto res3 = adapter.update(observation(3), true);
  EXPECT_TRUE(res3.confirmed);
  EXPECT_TRUE(res3.confirmed_now);
  EXPECT_TRUE(adapter.isConfirmed());

  // Subsequent frame latches confirmed state
  auto res4 = adapter.update(observation(4), false);
  EXPECT_TRUE(res4.confirmed);
  EXPECT_FALSE(res4.confirmed_now);
  EXPECT_TRUE(adapter.isConfirmed());
}

TEST(PipelineEndAdapter, ResetClearsConfirmation)
{
  banyu_detection::PipelineEndAdapter adapter(endConfig(2));
  adapter.update(observation(1), true);
  auto res = adapter.update(observation(2), true);
  EXPECT_TRUE(res.confirmed);
  EXPECT_TRUE(adapter.isConfirmed());

  adapter.reset();
  EXPECT_FALSE(adapter.isConfirmed());
  EXPECT_EQ(adapter.confirmer().state(), banyu_detection::PipelineEndState::WAITING);
}

TEST(PipelineEndAdapter, PipelineLossCannotConfirm)
{
  banyu_detection::PipelineEndAdapter adapter(endConfig(2));
  auto loss1 = observation(1, banyu_msgs::msg::PipelineObservation::NO_PIPE);
  auto loss2 = observation(2, banyu_msgs::msg::PipelineObservation::NO_PIPE);

  EXPECT_FALSE(adapter.update(loss1, false).confirmed);
  EXPECT_FALSE(adapter.update(loss2, false).confirmed);
  EXPECT_FALSE(adapter.isConfirmed());
}

}  // namespace
