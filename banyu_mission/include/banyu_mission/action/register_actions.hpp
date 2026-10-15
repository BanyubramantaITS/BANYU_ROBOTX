#ifndef BANYU_MISSION__ACTION__REGISTER_ACTIONS_HPP_
#define BANYU_MISSION__ACTION__REGISTER_ACTIONS_HPP_

#include <functional>
#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "banyu_mission/mission_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace banyu_mission
{

void registerActionNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MissionState> & state,
  std::function<void()> on_abort = nullptr);

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__REGISTER_ACTIONS_HPP_
