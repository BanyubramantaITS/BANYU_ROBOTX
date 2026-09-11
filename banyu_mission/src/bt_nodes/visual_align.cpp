#include "banyu_mission/bt_nodes/visual_align.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

VisualAlign::VisualAlign(const std::string& name, const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx,
                         int object_id, const std::string& prefix)
    : BT::StatefulActionNode(name, config),
      node_(node),
      ctx_(std::move(ctx)),
      object_id_(object_id),
      prefix_(prefix) {}

double VisualAlign::param(const std::string& name) const {
  return node_->get_parameter(prefix_ + name).as_double();
}

BT::NodeStatus VisualAlign::onStart() {
  start_time_ = node_->now();
  alignment_started_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VisualAlign::onRunning() {
  const rclcpp::Time now = node_->now();
  const char* label = object_id_ == 0 ? "GateAlign" : "MarkerAlign";

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  if ((now - start_time_).seconds() > param("timeout")) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "%s: timeout", label);
    return BT::NodeStatus::FAILURE;
  }

  if (!detectionIsFresh(*ctx_, now, node_->get_parameter("detection_timeout_s").as_double(), object_id_)) {
    // Never command from an old image. Hold safely while waiting for a fresh
    // detection; the action timeout handles a permanently lost target.
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "%s: waiting for fresh detection", label);
    return BT::NodeStatus::RUNNING;
  }

  banyu_msgs::msg::Detection detection;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    detection = ctx_->latest_detection;
  }

  if (!std::isfinite(detection.offset_x) || !std::isfinite(detection.offset_y)) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::RUNNING;
  }

  const bool aligned = std::abs(detection.offset_x) <= param("deadband_x") &&
                       std::abs(detection.offset_y) <= param("deadband_y");

  if (aligned) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    if (!alignment_started_) {
      aligned_since_ = now;
      alignment_started_ = true;
    }

    if ((now - aligned_since_).seconds() >= param("hold_time")) {
      RCLCPP_INFO(node_->get_logger(), "%s: object centered", label);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  alignment_started_ = false;

  const double lateral_speed = std::clamp(
      param("gain_x") * static_cast<double>(detection.offset_x),
      -param("max_speed_y"), param("max_speed_y"));
  const double vertical_speed = std::clamp(
      param("gain_y") * static_cast<double>(detection.offset_y),
      -param("max_speed_z"), param("max_speed_z"));

  // Assumes the camera is aligned with base_link: image right maps to body
  // right, and image down maps to body down. The helper converts these
  // physical body-NED commands to the ROS base_link convention.
  if (ctx_->setpoint_pub) {
    ctx_->setpoint_pub->publish(bodyVelocitySetpoint(0.0, lateral_speed, vertical_speed, now));
  }
  return BT::NodeStatus::RUNNING;
}

void VisualAlign::onHalted() {
  bt_nodes::common::publishZeroBodyVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
