#include "banyu_detection/pipeline_end_confirmer.hpp"

#include <gtest/gtest.h>

namespace
{

banyu_detection::PipelineEndEvidence evidence(uint64_t sequence, bool end_candidate)
{
  banyu_detection::PipelineEndEvidence result;
  result.end_candidate = end_candidate;
  result.observation.frame_sequence = sequence;
  result.observation.status = banyu_msgs::msg::PipelineObservation::TRACKED;
  result.observation.offset_x = 0.1F;
  result.observation.offset_y = 0.2F;
  result.observation.heading_error_rad = 0.03F;
  result.observation.confidence = 0.9F;
  return result;
}

banyu_detection::PipelineEndConfirmerConfig endConfig(std::size_t required_frames)
{
  banyu_detection::PipelineEndConfirmerConfig config;
  config.required_frames = required_frames;
  return config;
}

TEST(PipelineEndConfirmer, RequiresStableExplicitEvidence) {
  banyu_detection::PipelineEndConfirmer confirmer(
    endConfig(3));

  EXPECT_EQ(
    confirmer.update(evidence(1, true)).state,
    banyu_detection::PipelineEndState::CANDIDATE);
  EXPECT_EQ(confirmer.update(evidence(2, true)).stable_frames, 2U);
  const auto update = confirmer.update(evidence(3, true));

  EXPECT_TRUE(update.confirmed_now);
  EXPECT_EQ(update.state, banyu_detection::PipelineEndState::CONFIRMED);
}

TEST(PipelineEndConfirmer, PipelineLossCannotConfirmEnd) {
  banyu_detection::PipelineEndConfirmer confirmer(
    endConfig(2));
  auto first = evidence(1, false);
  first.observation.status = banyu_msgs::msg::PipelineObservation::NO_PIPE;
  auto second = first;
  second.observation.frame_sequence = 2;

  EXPECT_FALSE(confirmer.update(first).confirmed_now);
  EXPECT_FALSE(confirmer.update(second).confirmed_now);
  EXPECT_EQ(confirmer.state(), banyu_detection::PipelineEndState::WAITING);
}

TEST(PipelineEndConfirmer, JitterAndFrameRegressionBreakCandidate) {
  auto config = endConfig(3);
  config.maximum_offset_delta = 0.05;
  banyu_detection::PipelineEndConfirmer confirmer(config);
  EXPECT_EQ(confirmer.update(evidence(1, true)).stable_frames, 1U);

  auto jitter = evidence(2, true);
  jitter.observation.offset_x = 0.3F;
  EXPECT_EQ(
    confirmer.update(jitter).state,
    banyu_detection::PipelineEndState::WAITING);

  EXPECT_EQ(
    confirmer.update(evidence(1, true)).state,
    banyu_detection::PipelineEndState::WAITING);
}

TEST(PipelineEndConfirmer, FrameGapBreaksStableEvidence) {
  auto config = endConfig(2);
  banyu_detection::PipelineEndConfirmer confirmer(config);
  EXPECT_EQ(confirmer.update(evidence(1, true)).stable_frames, 1U);
  EXPECT_EQ(
    confirmer.update(evidence(3, true)).state,
    banyu_detection::PipelineEndState::WAITING);
}

TEST(PipelineEndConfirmer, ReplayAfterLossAndResetCannotConfirm) {
  banyu_detection::PipelineEndConfirmer confirmer(endConfig(2));
  EXPECT_FALSE(confirmer.update(evidence(10, true)).confirmed_now);

  auto loss = evidence(11, false);
  loss.observation.status = banyu_msgs::msg::PipelineObservation::NO_PIPE;
  EXPECT_FALSE(confirmer.update(loss).accepted);
  confirmer.reset();

  EXPECT_FALSE(confirmer.update(evidence(10, true)).accepted);
  EXPECT_EQ(confirmer.state(), banyu_detection::PipelineEndState::WAITING);
  EXPECT_FALSE(confirmer.update(evidence(12, true)).confirmed_now);
  EXPECT_TRUE(confirmer.update(evidence(13, true)).confirmed_now);
}

}  // namespace
