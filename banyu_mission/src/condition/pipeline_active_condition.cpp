#include "banyu_mission/condition/pipeline_active_condition.hpp"

#include <utility>

namespace banyu_mission
{

PipelineActiveCondition::PipelineActiveCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus PipelineActiveCondition::tick()
{
  return state_->pipelineEnded() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission
