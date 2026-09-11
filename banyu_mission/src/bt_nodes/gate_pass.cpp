#include "banyu_mission/bt_nodes/gate_pass.hpp"

#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

GatePass::GatePass(const std::string& name, const BT::NodeConfig& config,
                   const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus GatePass::onStart() {
  start_time_ = node_->now();
  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  start_pose_ = ctx_->current_pose.pose.position;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GatePass::onRunning() {
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

  const double travelled = bt_nodes::common::norm2(pose.pose.position, start_pose_);
  const double target_distance = node_->get_parameter("gate_pass_distance").as_double();
  const double min_distance = node_->get_parameter("gate_pass_min_distance").as_double();

  // Through once the required distance is covered, or once the gate leaves
  // the camera after a minimum distance (the sub is inside/past it).
  const bool gate_lost = travelled >= min_distance &&
                         !detectionIsFresh(*ctx_, now,
                                           node_->get_parameter("detection_timeout_s").as_double(), 0);
  if (travelled >= target_distance || gate_lost) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "GatePass: through the gate (%.2f m)", travelled);
    return BT::NodeStatus::SUCCESS;
  }

  ctx_->setpoint_pub->publish(
      bodyVelocitySetpoint(node_->get_parameter("gate_pass_speed").as_double(), 0.0, 0.0, now));

  if ((now - start_time_).seconds() > node_->get_parameter("gate_pass_timeout").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "GatePass: timeout (travelled %.2f m)", travelled);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void GatePass::onHalted() {
  bt_nodes::common::publishZeroBodyVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
