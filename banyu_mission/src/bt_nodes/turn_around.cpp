#include "banyu_mission/bt_nodes/turn_around.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

TurnAround::TurnAround(const std::string& name, const BT::NodeConfig& config,
                       const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus TurnAround::onStart() {
  start_time_ = node_->now();
  accumulated_yaw_ = 0.0;
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  start_yaw_ = bt_nodes::common::yawFromQuaternion(pose.pose.orientation);
  last_yaw_ = start_yaw_;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnAround::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  const double yaw = bt_nodes::common::yawFromQuaternion(pose.pose.orientation);
  accumulated_yaw_ += bt_nodes::common::wrapToPi(yaw - last_yaw_);
  last_yaw_ = yaw;

  // Phase A: rotate at turn_yaw_rate until the gate enters the camera.
  // Phase B (capture): once sighted, close the loop on offset_x so the
  // rotation stops exactly when the gate is centered.
  banyu_msgs::msg::Detection detection;
  const bool have_gate =
      detectionIsFresh(*ctx_, now, node_->get_parameter("detection_timeout_s").as_double(), 0);
  if (have_gate) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    detection = ctx_->latest_detection;
  }

  if (have_gate && std::isfinite(detection.offset_x)) {
    const double deadband = node_->get_parameter("turn_capture_deadband").as_double();
    if (std::abs(detection.offset_x) <= deadband) {
      bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
      RCLCPP_INFO(node_->get_logger(), "TurnAround: aligned with gate after %.1f deg",
                  accumulated_yaw_ * 180.0 / M_PI);
      return BT::NodeStatus::SUCCESS;
    }

    // Same sign convention as the MarkerOrbit yaw-follow: yaw_rate =
    // +kp*offset_x, offset_x>0 means the gate is to the right.
    const double max_rate = node_->get_parameter("turn_yaw_rate").as_double();
    const double yaw_cmd = std::clamp(
        node_->get_parameter("turn_capture_gain").as_double() * static_cast<double>(detection.offset_x),
        -max_rate, max_rate);
    ctx_->setpoint_pub->publish(bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, yaw_cmd, now));
  } else {
    // Keep rotating until the gate appears.
    ctx_->setpoint_pub->publish(bodyVelocityYawRateSetpoint(
        0.0, 0.0, 0.0, node_->get_parameter("turn_yaw_rate").as_double(), now));
  }

  if (std::fabs(accumulated_yaw_) >= node_->get_parameter("turn_angle_rad").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "TurnAround: turned %.1f deg (cap)",
                accumulated_yaw_ * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }

  if ((now - start_time_).seconds() > node_->get_parameter("turn_timeout").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "TurnAround: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TurnAround::onHalted() {
  bt_nodes::common::publishZeroBodyVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
