#include "banyu_mission/bt_nodes.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission {

namespace {
double norm2(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void publishZeroVelocity(const std::shared_ptr<MissionContext>& ctx,
                         const rclcpp::Time& stamp) {
  if (ctx->setpoint_pub) {
    ctx->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, stamp));
  }
}
}  // namespace

AbortedOrTeleop::AbortedOrTeleop(const std::string& name, const BT::NodeConfig& config,
                                 std::shared_ptr<MissionContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus AbortedOrTeleop::tick() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

MissionActive::MissionActive(const std::string& name, const BT::NodeConfig& config,
                             std::shared_ptr<MissionContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus MissionActive::tick() {
  return ctx_->mission_active.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

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
  target.x = origin.x + node_->get_parameter("gate_offset_north").as_double();
  target.y = origin.y + node_->get_parameter("gate_offset_east").as_double();
  target.z = origin.z + node_->get_parameter("gate_offset_down").as_double();
  ctx_->gate_target = target;

  RCLCPP_INFO(node_->get_logger(), "NavToGate: origin=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
              origin.x, origin.y, origin.z, target.x, target.y, target.z);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavToGate::onRunning() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroVelocity(ctx_, node_->now());
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

  ctx_->setpoint_pub->publish(positionSetpoint(*ctx_->gate_target, node_->now()));

  if (norm2(pose.pose.position, *ctx_->gate_target) < node_->get_parameter("arrival_tolerance").as_double()) {
    RCLCPP_INFO(node_->get_logger(), "NavToGate: arrived at gate");
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > node_->get_parameter("nav_timeout").as_double()) {
    RCLCPP_ERROR(node_->get_logger(), "NavToGate: timeout reaching gate");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void NavToGate::onHalted() {
  publishZeroVelocity(ctx_, node_->now());
}

StubHold::StubHold(const std::string& name, const BT::NodeConfig& config,
                   const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx,
                   const std::string& phase)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)), phase_(phase) {}

BT::NodeStatus StubHold::onStart() {
  start_time_ = node_->now();
  RCLCPP_WARN(node_->get_logger(), "%s: phase not implemented yet (Phase B/D work), holding position",
              phase_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StubHold::onRunning() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroVelocity(ctx_, node_->now());
    return BT::NodeStatus::SUCCESS;
  }

  // Safe hold: zero-velocity setpoint keeps GUIDED satisfied.
  ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, node_->now()));

  if ((node_->now() - start_time_).seconds() > node_->get_parameter("stub_hold_time").as_double()) {
    RCLCPP_INFO(node_->get_logger(), "%s: stub complete", phase_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void StubHold::onHalted() {
  publishZeroVelocity(ctx_, node_->now());
}

ReturnToStart::ReturnToStart(const std::string& name, const BT::NodeConfig& config,
                             const rclcpp::Node::SharedPtr& node,
                             std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus ReturnToStart::onStart() {
  start_time_ = node_->now();
  if (!ctx_->mission_origin) {
    RCLCPP_ERROR(node_->get_logger(), "ReturnToStart: no mission origin captured");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToStart::onRunning() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroVelocity(ctx_, node_->now());
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }

  ctx_->setpoint_pub->publish(positionSetpoint(*ctx_->mission_origin, node_->now()));

  if (norm2(pose.pose.position, *ctx_->mission_origin) < node_->get_parameter("arrival_tolerance").as_double()) {
    RCLCPP_INFO(node_->get_logger(), "ReturnToStart: mission complete");
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > node_->get_parameter("return_timeout").as_double()) {
    RCLCPP_ERROR(node_->get_logger(), "ReturnToStart: timeout returning home");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void ReturnToStart::onHalted() {
  publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
