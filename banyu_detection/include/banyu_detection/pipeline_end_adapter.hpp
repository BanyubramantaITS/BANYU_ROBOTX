#ifndef BANYU_DETECTION_PIPELINE_END_ADAPTER_HPP_
#define BANYU_DETECTION_PIPELINE_END_ADAPTER_HPP_

#include <cstddef>

#include <banyu_msgs/msg/pipeline_observation.hpp>

#include "banyu_detection/pipeline_end_confirmer.hpp"

namespace banyu_detection
{

struct PipelineEndAdapterResult
{
  bool confirmed{false};
  bool confirmed_now{false};
  PipelineEndState state{PipelineEndState::WAITING};
  std::size_t stable_frames{0};
  bool accepted{false};
};

class PipelineEndAdapter
{
public:
  explicit PipelineEndAdapter(PipelineEndConfirmerConfig config = {});

  PipelineEndAdapterResult update(
    const banyu_msgs::msg::PipelineObservation & observation, bool end_candidate);

  PipelineEndAdapterResult update(const PipelineEndEvidence & evidence);

  void reset();

  bool isConfirmed() const noexcept {return confirmed_;}
  const PipelineEndConfirmer & confirmer() const noexcept {return confirmer_;}

private:
  PipelineEndConfirmer confirmer_;
  bool confirmed_{false};
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_PIPELINE_END_ADAPTER_HPP_
