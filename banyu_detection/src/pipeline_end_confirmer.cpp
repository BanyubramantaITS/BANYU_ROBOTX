#include "banyu_detection/pipeline_end_confirmer.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_detection
{

PipelineEndConfirmer::PipelineEndConfirmer(PipelineEndConfirmerConfig config)
: config_(config)
{
  config_.required_frames = std::max<std::size_t>(2, config_.required_frames);
  config_.maximum_sequence_gap = std::max<uint64_t>(1, config_.maximum_sequence_gap);
  config_.minimum_confidence = std::clamp(config_.minimum_confidence, 0.0, 1.0);
  config_.maximum_offset_delta = std::max(0.0, config_.maximum_offset_delta);
  config_.maximum_heading_delta_rad = std::max(0.0, config_.maximum_heading_delta_rad);
}

bool PipelineEndConfirmer::finiteTrackedObservation(
  const banyu_msgs::msg::PipelineObservation & observation)
{
  return observation.status == banyu_msgs::msg::PipelineObservation::TRACKED &&
         std::isfinite(observation.offset_x) && std::isfinite(observation.offset_y) &&
         std::isfinite(observation.heading_error_rad) &&
         std::isfinite(observation.confidence) && observation.offset_x >= -1.0F &&
         observation.offset_x <= 1.0F && observation.offset_y >= -1.0F &&
         observation.offset_y <= 1.0F && observation.confidence >= 0.0F &&
         observation.confidence <= 1.0F;
}

bool PipelineEndConfirmer::stableWithPrevious(
  const banyu_msgs::msg::PipelineObservation & observation) const
{
  if (!has_previous_) {
    return true;
  }
  return std::abs(observation.offset_x - previous_.offset_x) <=
         config_.maximum_offset_delta &&
         std::abs(observation.offset_y - previous_.offset_y) <=
         config_.maximum_offset_delta &&
         std::abs(observation.heading_error_rad - previous_.heading_error_rad) <=
         config_.maximum_heading_delta_rad;
}

void PipelineEndConfirmer::clearCandidate()
{
  state_ = PipelineEndState::WAITING;
  stable_frames_ = 0;
  has_previous_ = false;
}

PipelineEndUpdate PipelineEndConfirmer::update(const PipelineEndEvidence & evidence)
{
  PipelineEndUpdate result;
  result.state = state_;
  result.stable_frames = stable_frames_;

  const auto & observation = evidence.observation;
  const bool newer_than_watermark = !watermark_seen_ ||
    observation.frame_sequence > sequence_watermark_;
  if (!newer_than_watermark) {
    return result;
  }
  watermark_seen_ = true;
  sequence_watermark_ = observation.frame_sequence;

  const bool new_sequence = !has_previous_ || observation.frame_sequence > previous_sequence_;
  const bool sequence_gap = has_previous_ && observation.frame_sequence > previous_sequence_ &&
    observation.frame_sequence - previous_sequence_ > config_.maximum_sequence_gap;
  const bool valid_candidate = evidence.end_candidate && new_sequence &&
    !sequence_gap &&
    finiteTrackedObservation(observation) &&
    observation.confidence >= config_.minimum_confidence &&
    stableWithPrevious(observation);

  // NO_PIPE, INVALID, stale frames, and a false end marker are not evidence.
  // In particular, pipeline loss cannot complete this state machine.
  if (!valid_candidate) {
    clearCandidate();
    if (new_sequence) {
      previous_sequence_ = observation.frame_sequence;
    }
    result.state = state_;
    result.stable_frames = stable_frames_;
    return result;
  }

  previous_ = observation;
  previous_sequence_ = observation.frame_sequence;
  has_previous_ = true;
  ++stable_frames_;
  state_ = stable_frames_ >= config_.required_frames ? PipelineEndState::CONFIRMED :
    PipelineEndState::CANDIDATE;
  result.state = state_;
  result.stable_frames = stable_frames_;
  result.accepted = true;
  result.confirmed_now = state_ == PipelineEndState::CONFIRMED &&
    stable_frames_ == config_.required_frames;
  return result;
}

PipelineEndUpdate PipelineEndConfirmer::update(
  const banyu_msgs::msg::PipelineObservation & observation, bool end_candidate)
{
  PipelineEndEvidence evidence;
  evidence.observation = observation;
  evidence.end_candidate = end_candidate;
  return update(evidence);
}

void PipelineEndConfirmer::reset()
{
  clearCandidate();
}

}  // namespace banyu_detection
