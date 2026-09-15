#include "banyu_mission/bt_nodes/home_to_start_buoy.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"
#include "banyu_mission/bt_nodes/task2_common.hpp"

namespace banyu_mission {

HomeToStartBuoy::HomeToStartBuoy(const std::string& name, const BT::NodeConfig& config,
                                 const rclcpp::Node::SharedPtr& node,
                                 std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus HomeToStartBuoy::onStart() {
  start_time_ = node_->now();
  hold_start_time_ = start_time_;
  holding_ = false;
  bt_nodes::common::publishZeroVelocity(ctx_, start_time_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HomeToStartBuoy::onRunning() {
  const rclcpp::Time now = node_->now();
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  mavros_msgs::msg::State state;
  geometry_msgs::msg::Point target;
  bool ready = false;
  bool has_anchor = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    has_anchor = ctx_->has_task2_surface_anchor;
    target = ctx_->task2_target_map_pose;
    ready = has_anchor && bt_nodes::task2_common::vehicleReady(
                            *ctx_, now,
                            node_->get_parameter("task2_pose_timeout_s").as_double(),
                            node_->get_parameter("task2_state_timeout_s").as_double(), pose, state);
  }

  if (!has_anchor || !ready) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "HomeToStartBuoy: local pose/state or FCU readiness lost");
    return BT::NodeStatus::FAILURE;
  }

  if ((now - start_time_).seconds() >= node_->get_parameter("task2_home_timeout_s").as_double()) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "HomeToStartBuoy: timeout");
    return BT::NodeStatus::FAILURE;
  }

  const double dx = target.x - pose.pose.position.x;
  const double dy = target.y - pose.pose.position.y;
  const double dz = target.z - pose.pose.position.z;
  const double horizontal_error = std::hypot(dx, dy);
  const double surface_tolerance =
      node_->get_parameter("task2_gps_surface_tolerance").as_double();
  const bool arrived =
      horizontal_error <= node_->get_parameter("task2_home_horizontal_tolerance").as_double() &&
      std::abs(dz) <= surface_tolerance;

  if (arrived) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    if (!holding_) {
      holding_ = true;
      hold_start_time_ = now;
    }
    if ((now - hold_start_time_).seconds() >=
        node_->get_parameter("task2_home_hold_s").as_double()) {
      RCLCPP_INFO(node_->get_logger(), "HomeToStartBuoy: reached start buoy");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  holding_ = false;
  double vx = node_->get_parameter("task2_home_kp_h").as_double() * dx;
  double vy = node_->get_parameter("task2_home_kp_h").as_double() * dy;
  const double max_horizontal =
      node_->get_parameter("task2_home_max_speed_h").as_double();
  const double horizontal_speed = std::hypot(vx, vy);
  if (horizontal_speed > max_horizontal && horizontal_speed > 0.0) {
    const double scale = max_horizontal / horizontal_speed;
    vx *= scale;
    vy *= scale;
  }
  const double vz = std::clamp(
      node_->get_parameter("task2_home_kp_v").as_double() * dz,
      -node_->get_parameter("task2_home_max_speed_v").as_double(),
      node_->get_parameter("task2_home_max_speed_v").as_double());
  ctx_->setpoint_pub->publish(velocitySetpoint(vx, vy, vz, now));
  return BT::NodeStatus::RUNNING;
}

void HomeToStartBuoy::onHalted() {
  bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
