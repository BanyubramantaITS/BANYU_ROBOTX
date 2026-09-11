#include "banyu_mission/bt_nodes/return_gate_pass.hpp"

#include <algorithm>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

ReturnGatePass::ReturnGatePass(const std::string& name, const BT::NodeConfig& config,
                               const rclcpp::Node::SharedPtr& node,
                               std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus ReturnGatePass::onStart() {
  start_time_ = node_->now();
  min_distance_ = std::numeric_limits<double>::infinity();
  after_loss_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnGatePass::onRunning() {
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

  const double detection_timeout = node_->get_parameter("detection_timeout_s").as_double();
  const double lost_distance = node_->get_parameter("gate_pass_lost_distance").as_double();

  if (after_loss_) {
    // Blind final push to clear the gate plane after the gate left the frame.
    const double travelled = bt_nodes::common::norm2(pose.pose.position, loss_start_pose_);
    const double push_distance = loss_distance_ + node_->get_parameter("gate_pass_clearance_m").as_double();
    if (travelled >= push_distance) {
      bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
      RCLCPP_INFO(node_->get_logger(), "ReturnGatePass: through the gate (%.2f m past loss)", travelled);
      return BT::NodeStatus::SUCCESS;
    }
    ctx_->setpoint_pub->publish(
        bodyVelocitySetpoint(node_->get_parameter("gate_pass_speed").as_double(), 0.0, 0.0, now));
  } else if (detectionIsFresh(*ctx_, now, detection_timeout, 0)) {
    banyu_msgs::msg::Detection detection;
    {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      detection = ctx_->latest_detection;
    }

    const double distance = static_cast<double>(detection.distance);
    if (distance > 0.0f) {
      min_distance_ = std::min(min_distance_, distance);
      // Reached the gate while still seeing it: done (safety net).
      if (min_distance_ <= node_->get_parameter("gate_pass_min_distance").as_double()) {
        bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
        RCLCPP_INFO(node_->get_logger(), "ReturnGatePass: at gate (%.2f m)", min_distance_);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // Drive forward while keeping the gate centered in the camera.
    const double lateral = std::clamp(
        node_->get_parameter("gate_align_gain_x").as_double() * static_cast<double>(detection.offset_x),
        -node_->get_parameter("gate_align_max_speed_y").as_double(),
        node_->get_parameter("gate_align_max_speed_y").as_double());
    const double vertical = std::clamp(
        node_->get_parameter("gate_align_gain_y").as_double() * static_cast<double>(detection.offset_y),
        -node_->get_parameter("gate_align_max_speed_z").as_double(),
        node_->get_parameter("gate_align_max_speed_z").as_double());

    ctx_->setpoint_pub->publish(bodyVelocitySetpoint(
        node_->get_parameter("gate_pass_speed").as_double(), lateral, vertical, now));
  } else if (min_distance_ <= lost_distance) {
    // Gate was seen close and is now out of frame: start the blind push.
    after_loss_ = true;
    loss_distance_ = min_distance_;
    loss_start_pose_ = pose.pose.position;
    RCLCPP_INFO(node_->get_logger(), "ReturnGatePass: gate lost at %.2f m, pushing through",
                min_distance_);
  } else {
    // Not seen close yet and no detection: hold and wait for re-acquisition.
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "ReturnGatePass: waiting for re-acquisition");
  }

  if ((now - start_time_).seconds() > node_->get_parameter("gate_pass_timeout").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "ReturnGatePass: timeout (min distance %.2f m)", min_distance_);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void ReturnGatePass::onHalted() {
  bt_nodes::common::publishZeroBodyVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
