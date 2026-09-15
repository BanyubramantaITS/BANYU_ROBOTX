#ifndef BANYU_MISSION_BT_NODES_REVERSE_TO_RED_RING_HPP_
#define BANYU_MISSION_BT_NODES_REVERSE_TO_RED_RING_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/bt_nodes/task2_camera_common.hpp"
#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class ReverseToRedRing : public BT::StatefulActionNode {
 public:
  ReverseToRedRing(const std::string& name, const BT::NodeConfig& config,
                   const rclcpp::Node::SharedPtr& node,
                   std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  bool validateAndBuildReversePathLocked();
  bool poseAtWaypoint(const geometry_msgs::msg::PoseStamped& pose,
                      const geometry_msgs::msg::PoseStamped& waypoint) const;
  bool freshMatchingRingLocked(
      const bt_nodes::task2_camera::PipelineFrame& frame, const rclcpp::Time& now) const;
  void failLocked(const std::string& reason);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  rclcpp::Time reacquire_start_time_;
  rclcpp::Time pairing_start_time_;
  std::vector<geometry_msgs::msg::PoseStamped> reverse_waypoints_;
  std::size_t waypoint_index_{0};
  std::size_t reacquire_frames_{0};
  uint64_t last_reacquire_sequence_{0};
  rclcpp::Time last_reacquire_source_stamp_{static_cast<int64_t>(0), RCL_ROS_TIME};
  uint64_t last_reacquire_ring_sequence_{0};
  rclcpp::Time last_reacquire_ring_source_stamp_{static_cast<int64_t>(0), RCL_ROS_TIME};
  bool has_reacquire_baseline_{false};
  bool pairing_candidate_{false};
  bool target_reached_{false};
  bool has_previous_pose_{false};
  geometry_msgs::msg::PoseStamped previous_pose_{};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_REVERSE_TO_RED_RING_HPP_
