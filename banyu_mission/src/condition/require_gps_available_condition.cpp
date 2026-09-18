#include "banyu_mission/condition/require_gps_available_condition.hpp"

#include <utility>

namespace banyu_mission
{

RequireGpsAvailableCondition::RequireGpsAvailableCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus RequireGpsAvailableCondition::tick()
{
  return state_->gpsAvailable() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
