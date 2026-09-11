#include "banyu_mission/bt_nodes/search_gate.hpp"

#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

SearchGate::SearchGate(const std::string& name, const BT::NodeConfig& config,
                       const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus SearchGate::onStart() {
  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchGate::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  if (detectionIsFresh(*ctx_, now, node_->get_parameter("detection_timeout_s").as_double(), 0)) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "SearchGate: gate found");
    return BT::NodeStatus::SUCCESS;
  }

  // Slow forward sweep while scanning for the gate.
  ctx_->setpoint_pub->publish(
      bodyVelocitySetpoint(node_->get_parameter("search_speed").as_double(), 0.0, 0.0, now));

  if ((now - start_time_).seconds() > node_->get_parameter("search_timeout").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "SearchGate: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SearchGate::onHalted() {
  bt_nodes::common::publishZeroBodyVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
