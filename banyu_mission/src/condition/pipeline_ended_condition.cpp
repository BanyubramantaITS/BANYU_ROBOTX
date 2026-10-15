#include "banyu_mission/condition/pipeline_ended_condition.hpp"

#include <utility>

namespace banyu_mission
{

PipelineEndedCondition::PipelineEndedCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus PipelineEndedCondition::tick()
{
  return state_->pipelineEnded() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
