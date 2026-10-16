#include "banyu_mission/condition/pipeline_lost_condition.hpp"

#include <utility>

namespace banyu_mission
{

PipelineLostCondition::PipelineLostCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus PipelineLostCondition::tick()
{
  return state_->pipelineLost() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
