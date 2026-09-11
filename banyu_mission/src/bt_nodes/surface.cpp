#include "banyu_mission/bt_nodes/surface.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

Surface::Surface(const std::string& name, const BT::NodeConfig& config,
                 const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus Surface::onStart() {
  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Surface::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }

  const double surface_depth = node_->get_parameter("surface_depth").as_double();
  const double surface_tolerance = node_->get_parameter("surface_tolerance").as_double();

  if (std::abs(pose.pose.position.z - surface_depth) < surface_tolerance) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "Surface: at surface, mission complete");
    return BT::NodeStatus::SUCCESS;
  }

  const double kp_v = node_->get_parameter("nav_kp_v").as_double();
  const double max_v = node_->get_parameter("nav_max_speed_v").as_double();
  const double dz = surface_depth - pose.pose.position.z;
  ctx_->setpoint_pub->publish(
      velocitySetpoint(0.0, 0.0, std::clamp(kp_v * dz, -max_v, max_v), now));

  if ((now - start_time_).seconds() > node_->get_parameter("surface_timeout").as_double()) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "Surface: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void Surface::onHalted() {
  bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
