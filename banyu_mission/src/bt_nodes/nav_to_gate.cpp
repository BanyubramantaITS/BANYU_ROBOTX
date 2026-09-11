#include "banyu_mission/bt_nodes/nav_to_gate.hpp"

#include <algorithm>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

NavToGate::NavToGate(const std::string& name, const BT::NodeConfig& config,
                     const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus NavToGate::onStart() {
  start_time_ = node_->now();

  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  if (!ctx_->mission_origin) {
    ctx_->mission_origin = ctx_->current_pose.pose.position;
  }
  const auto& origin = *ctx_->mission_origin;
  geometry_msgs::msg::Point target;
  // Parameters are expressed as NED mission offsets, while MAVROS local
  // pose/setpoint fields use ROS ENU: east -> x, north -> y, down -> -z.
  target.x = origin.x + node_->get_parameter("gate_offset_east").as_double();
  target.y = origin.y + node_->get_parameter("gate_offset_north").as_double();
  target.z = origin.z - node_->get_parameter("gate_offset_down").as_double();
  ctx_->gate_target = target;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "NavToGate: origin=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
                       origin.x, origin.y, origin.z, target.x, target.y, target.z);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavToGate::onRunning() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }

  if (!ctx_->gate_target) {
    return BT::NodeStatus::RUNNING;
  }

  // Closed-loop navigation with velocity-only setpoints: ArduSub's AC_WPNav
  // position controller is unreliable in this sim, so this uses ROS ENU
  // velocity components directly.
  const auto& target = *ctx_->gate_target;
  const double dx = target.x - pose.pose.position.x;
  const double dy = target.y - pose.pose.position.y;
  const double dz = target.z - pose.pose.position.z;

  if (bt_nodes::common::norm2(pose.pose.position, target) <
      node_->get_parameter("arrival_tolerance").as_double()) {
    bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "NavToGate: arrived at gate");
    return BT::NodeStatus::SUCCESS;
  }

  const double kp_h = node_->get_parameter("nav_kp_h").as_double();
  const double kp_v = node_->get_parameter("nav_kp_v").as_double();
  const double max_h = node_->get_parameter("nav_max_speed_h").as_double();
  const double max_v = node_->get_parameter("nav_max_speed_v").as_double();

  const double vx = std::clamp(kp_h * dx, -max_h, max_h);
  const double vy = std::clamp(kp_h * dy, -max_h, max_h);
  const double vz = std::clamp(kp_v * dz, -max_v, max_v);

  ctx_->setpoint_pub->publish(velocitySetpoint(vx, vy, vz, node_->now()));

  if ((node_->now() - start_time_).seconds() > node_->get_parameter("nav_timeout").as_double()) {
    RCLCPP_ERROR(node_->get_logger(), "NavToGate: timeout reaching gate");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void NavToGate::onHalted() {
  bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
