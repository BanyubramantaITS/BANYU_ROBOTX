#include "banyu_guidance/pipeline_follower.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace banyu_guidance
{
namespace
{

PipelineObservation trackedObservation()
{
  PipelineObservation observation;
  observation.frame_sequence = 1;
  observation.status = PipelineStatus::Tracked;
  observation.stamp_ns = 1'000'000'000;
  observation.offset_x = 0.5;
  observation.offset_y = -0.1;
  observation.heading_error_rad = 0.25;
  observation.confidence = 0.8;
  return observation;
}

TEST(PipelineFollower, ProducesBoundedBodyFrameIntent)
{
  GuidanceConfig config;
  config.surge_gain_mps = 1.0;
  config.max_surge_mps = 0.5;
  config.sway_gain_mps = 1.0;
  config.max_sway_mps = 0.2;
  config.yaw_gain_rad_s = 1.0;
  config.max_yaw_rate_rad_s = 0.1;
  PipelineFollower follower(config);

  const auto intent = follower.compute(trackedObservation(), 1'100'000'000);

  EXPECT_EQ(intent.status, IntentStatus::Tracking);
  EXPECT_DOUBLE_EQ(intent.surge_mps, 0.5);
  EXPECT_DOUBLE_EQ(intent.sway_mps, -0.2);
  EXPECT_DOUBLE_EQ(intent.yaw_rate_rad_s, -0.1);
  EXPECT_TRUE(intent.yaw_control_active);
}

TEST(PipelineFollower, InactiveYawControlPublishesExplicitZero)
{
  GuidanceConfig config;
  config.yaw_control_enabled = false;
  PipelineFollower follower(config);

  const auto intent = follower.compute(trackedObservation(), 1'100'000'000);

  EXPECT_EQ(intent.status, IntentStatus::Tracking);
  EXPECT_FALSE(intent.yaw_control_active);
  EXPECT_DOUBLE_EQ(intent.yaw_rate_rad_s, 0.0);
}

TEST(PipelineFollower, LostPipelineStopsAllMotion)
{
  auto observation = trackedObservation();
  observation.status = PipelineStatus::NoPipe;
  const auto intent = PipelineFollower().compute(observation, 1'100'000'000);

  EXPECT_EQ(intent.status, IntentStatus::Lost);
  EXPECT_DOUBLE_EQ(intent.surge_mps, 0.0);
  EXPECT_DOUBLE_EQ(intent.sway_mps, 0.0);
  EXPECT_DOUBLE_EQ(intent.yaw_rate_rad_s, 0.0);
}

TEST(PipelineFollower, StaleObservationStopsForwardMotion)
{
  auto observation = trackedObservation();
  const auto intent = PipelineFollower().compute(observation, 1'500'000'000);

  EXPECT_EQ(intent.status, IntentStatus::Stale);
  EXPECT_DOUBLE_EQ(intent.surge_mps, 0.0);
  EXPECT_DOUBLE_EQ(intent.sway_mps, 0.0);
  EXPECT_DOUBLE_EQ(intent.yaw_rate_rad_s, 0.0);
}

TEST(PipelineFollower, InvalidObservationStopsForwardMotion)
{
  auto observation = trackedObservation();
  observation.offset_x = std::numeric_limits<double>::quiet_NaN();
  const auto intent = PipelineFollower().compute(observation, 1'100'000'000);

  EXPECT_EQ(intent.status, IntentStatus::Invalid);
  EXPECT_DOUBLE_EQ(intent.surge_mps, 0.0);
  EXPECT_DOUBLE_EQ(intent.sway_mps, 0.0);
  EXPECT_DOUBLE_EQ(intent.yaw_rate_rad_s, 0.0);
}

TEST(PipelineFollower, LowConfidenceIsLostAndCannotAdvance)
{
  auto observation = trackedObservation();
  observation.confidence = 0.1;
  const auto intent = PipelineFollower().compute(observation, 1'100'000'000);

  EXPECT_EQ(intent.status, IntentStatus::Lost);
  EXPECT_DOUBLE_EQ(intent.surge_mps, 0.0);
}

TEST(PipelineFollower, FutureObservationBeyondSkewIsStale)
{
  auto observation = trackedObservation();
  observation.stamp_ns = 2'000'000'000;
  const auto intent = PipelineFollower().compute(observation, 1'000'000'000);

  EXPECT_EQ(intent.status, IntentStatus::Stale);
  EXPECT_DOUBLE_EQ(intent.surge_mps, 0.0);
}

TEST(PipelineFollower, ConfigLimitsAndInvalidValuesAreSafe)
{
  GuidanceConfig config;
  config.surge_gain_mps = -1.0;
  config.max_sway_mps = std::numeric_limits<double>::quiet_NaN();
  config.min_confidence = 2.0;
  PipelineFollower follower(config);

  EXPECT_DOUBLE_EQ(follower.config().surge_gain_mps, 0.0);
  EXPECT_DOUBLE_EQ(follower.config().max_sway_mps, 0.0);
  EXPECT_DOUBLE_EQ(follower.config().min_confidence, 1.0);
}

}  // namespace
}  // namespace banyu_guidance
