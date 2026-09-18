#include "banyu_detection/pipeline_end_adapter.hpp"

namespace banyu_detection
{

PipelineEndAdapter::PipelineEndAdapter(PipelineEndConfirmerConfig config)
: confirmer_(config)
{
}

PipelineEndAdapterResult PipelineEndAdapter::update(
  const banyu_msgs::msg::PipelineObservation & observation, bool end_candidate)
{
  const auto update_result = confirmer_.update(observation, end_candidate);
  if (update_result.confirmed_now || update_result.state == PipelineEndState::CONFIRMED) {
    confirmed_ = true;
  }
  return PipelineEndAdapterResult{
    confirmed_,
    update_result.confirmed_now,
    update_result.state,
    update_result.stable_frames,
    update_result.accepted
  };
}

PipelineEndAdapterResult PipelineEndAdapter::update(const PipelineEndEvidence & evidence)
{
  return update(evidence.observation, evidence.end_candidate);
}

void PipelineEndAdapter::reset()
{
  confirmer_.reset();
  confirmed_ = false;
}

}  // namespace banyu_detection
