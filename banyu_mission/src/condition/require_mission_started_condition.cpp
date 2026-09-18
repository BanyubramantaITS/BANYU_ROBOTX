#include "banyu_mission/condition/require_mission_started_condition.hpp"

#include <utility>

namespace banyu_mission
{

RequireMissionStartedCondition::RequireMissionStartedCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus RequireMissionStartedCondition::tick()
{
  return state_->snapshot().mission_started ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
