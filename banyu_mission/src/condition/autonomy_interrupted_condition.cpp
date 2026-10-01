#include "banyu_mission/condition/autonomy_interrupted_condition.hpp"

#include <utility>

namespace banyu_mission
{

AutonomyInterruptedCondition::AutonomyInterruptedCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus AutonomyInterruptedCondition::tick()
{
  return state_->autonomyInterrupted() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
