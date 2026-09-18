#ifndef BANYU_MISSION__CONDITION__REGISTER_CONDITIONS_HPP_
#define BANYU_MISSION__CONDITION__REGISTER_CONDITIONS_HPP_

#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "banyu_mission/mission_state.hpp"

namespace banyu_mission
{

void registerConditionNodes(
  BT::BehaviorTreeFactory & factory,
  const std::shared_ptr<MissionState> & state);

}  // namespace banyu_mission

#endif  // BANYU_MISSION__CONDITION__REGISTER_CONDITIONS_HPP_
