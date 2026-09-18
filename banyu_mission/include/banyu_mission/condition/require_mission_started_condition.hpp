#ifndef BANYU_MISSION__CONDITION__REQUIRE_MISSION_STARTED_CONDITION_HPP_
#define BANYU_MISSION__CONDITION__REQUIRE_MISSION_STARTED_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "banyu_mission/mission_state.hpp"

namespace banyu_mission
{

class RequireMissionStartedCondition : public BT::ConditionNode
{
public:
  RequireMissionStartedCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<MissionState> state);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> state_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__CONDITION__REQUIRE_MISSION_STARTED_CONDITION_HPP_
