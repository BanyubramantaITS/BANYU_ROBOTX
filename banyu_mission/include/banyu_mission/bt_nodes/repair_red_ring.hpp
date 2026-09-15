#ifndef BANYU_MISSION_BT_NODES_REPAIR_RED_RING_HPP_
#define BANYU_MISSION_BT_NODES_REPAIR_RED_RING_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/bt_nodes/task2_camera_common.hpp"
#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class RepairRedRing : public BT::StatefulActionNode {
 public:
  RepairRedRing(const std::string& name, const BT::NodeConfig& config,
                const rclcpp::Node::SharedPtr& node,
                std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  rclcpp::Time valid_hold_start_;
  bool hold_started_{false};
  geometry_msgs::msg::PoseStamped previous_pose_{};
  bool has_previous_pose_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_REPAIR_RED_RING_HPP_
