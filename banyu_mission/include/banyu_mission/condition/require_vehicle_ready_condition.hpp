#ifndef BANYU_MISSION__CONDITION__REQUIRE_VEHICLE_READY_CONDITION_HPP_
#define BANYU_MISSION__CONDITION__REQUIRE_VEHICLE_READY_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "banyu_mission/mission_state.hpp"

namespace banyu_mission
{

class RequireVehicleReadyCondition : public BT::ConditionNode
{
public:
  RequireVehicleReadyCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<MissionState> state);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> state_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__CONDITION__REQUIRE_VEHICLE_READY_CONDITION_HPP_
