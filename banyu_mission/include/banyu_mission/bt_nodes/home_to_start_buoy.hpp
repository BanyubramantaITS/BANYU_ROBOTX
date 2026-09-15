#ifndef BANYU_MISSION_BT_NODES_HOME_TO_START_BUOY_HPP_
#define BANYU_MISSION_BT_NODES_HOME_TO_START_BUOY_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class HomeToStartBuoy : public BT::StatefulActionNode {
 public:
  HomeToStartBuoy(const std::string& name, const BT::NodeConfig& config,
                  const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  rclcpp::Time hold_start_time_;
  bool holding_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_HOME_TO_START_BUOY_HPP_
