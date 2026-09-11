#ifndef BANYU_MISSION_BT_NODES_RETURN_GATE_PASS_HPP_
#define BANYU_MISSION_BT_NODES_RETURN_GATE_PASS_HPP_

#include <limits>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class ReturnGatePass : public BT::StatefulActionNode {
 public:
  ReturnGatePass(const std::string& name, const BT::NodeConfig& config,
                 const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  double min_distance_{std::numeric_limits<double>::infinity()};
  double loss_distance_{0.0};
  bool after_loss_{false};
  geometry_msgs::msg::Point loss_start_pose_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_RETURN_GATE_PASS_HPP_
