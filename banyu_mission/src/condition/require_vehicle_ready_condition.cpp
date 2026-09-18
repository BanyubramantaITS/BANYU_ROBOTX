#include "banyu_mission/condition/require_vehicle_ready_condition.hpp"

#include <utility>

namespace banyu_mission
{

RequireVehicleReadyCondition::RequireVehicleReadyCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus RequireVehicleReadyCondition::tick()
{
  return state_->vehicleReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
