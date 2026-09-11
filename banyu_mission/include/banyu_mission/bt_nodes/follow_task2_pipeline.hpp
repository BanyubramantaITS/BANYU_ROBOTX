#ifndef BANYU_MISSION_BT_NODES_FOLLOW_TASK2_PIPELINE_HPP_
#define BANYU_MISSION_BT_NODES_FOLLOW_TASK2_PIPELINE_HPP_

#include <array>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"
#include "banyu_mission/bt_nodes/task2_common.hpp"

namespace banyu_mission {

class FollowTask2Pipeline : public BT::StatefulActionNode {
 public:
  FollowTask2Pipeline(const std::string& name, const BT::NodeConfig& config,
                      const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  std::array<geometry_msgs::msg::PoseStamped, bt_nodes::task2_common::kWaypointCount> route_;
  std::array<banyu_msgs::msg::PipelineRingState, bt_nodes::task2_common::kRingCount> ring_states_;
  rclcpp::Time start_time_;
  rclcpp::Time segment_start_time_;
  rclcpp::Time hold_start_time_;
  std::size_t waypoint_index_{0};
  bool holding_{false};
  bool entry_phase_{true};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_FOLLOW_TASK2_PIPELINE_HPP_
