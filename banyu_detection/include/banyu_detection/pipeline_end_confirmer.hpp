#ifndef BANYU_DETECTION_PIPELINE_END_CONFIRMER_HPP_
#define BANYU_DETECTION_PIPELINE_END_CONFIRMER_HPP_

#include <cstddef>
#include <cstdint>

#include <banyu_msgs/msg/pipeline_observation.hpp>

namespace banyu_detection
{

// End evidence is deliberately separate from NO_PIPE.  A detector may fill
// this flag from a geometric end marker or another explicit end feature.
struct PipelineEndEvidence
{
  banyu_msgs::msg::PipelineObservation observation;
  bool end_candidate{false};
};

struct PipelineEndConfirmerConfig
{
  std::size_t required_frames{3};
  uint64_t maximum_sequence_gap{1};
  double minimum_confidence{0.5};
  double maximum_offset_delta{0.15};
  double maximum_heading_delta_rad{0.20};
};

enum class PipelineEndState
{
  WAITING,
  CANDIDATE,
  CONFIRMED,
};

struct PipelineEndUpdate
{
  PipelineEndState state{PipelineEndState::WAITING};
  std::size_t stable_frames{0};
  bool confirmed_now{false};
  bool accepted{false};
};

class PipelineEndConfirmer
{
public:
  explicit PipelineEndConfirmer(PipelineEndConfirmerConfig config = {});

  PipelineEndUpdate update(const PipelineEndEvidence & evidence);
  PipelineEndUpdate update(
    const banyu_msgs::msg::PipelineObservation & observation, bool end_candidate);

  // Starts a new confirmation window.  This is useful when a new pipeline
  // run begins; confirmation is never carried between runs.
  void reset();

  PipelineEndState state() const {return state_;}
  std::size_t stableFrames() const {return stable_frames_;}

private:
  static bool finiteTrackedObservation(
    const banyu_msgs::msg::PipelineObservation & observation);
  bool stableWithPrevious(
    const banyu_msgs::msg::PipelineObservation & observation) const;
  void clearCandidate();

  PipelineEndConfirmerConfig config_;
  PipelineEndState state_{PipelineEndState::WAITING};
  std::size_t stable_frames_{0};
  bool has_previous_{false};
  uint64_t previous_sequence_{0};
  bool watermark_seen_{false};
  uint64_t sequence_watermark_{0};
  banyu_msgs::msg::PipelineObservation previous_{};
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_PIPELINE_END_CONFIRMER_HPP_
