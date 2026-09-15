#include "banyu_mission/bt_nodes/follow_task2_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

FollowTask2Pipeline::FollowTask2Pipeline(const std::string& name, const BT::NodeConfig& config,
                                         const rclcpp::Node::SharedPtr& node,
                                         std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus FollowTask2Pipeline::onStart() {
  const rclcpp::Time now = node_->now();
  start_time_ = now;
  segment_start_time_ = now;
  hold_start_time_ = now;
  waypoint_index_ = 0;
  holding_ = false;
  entry_phase_ = true;
  bt_nodes::common::publishZeroVelocity(ctx_, now);

  const double max_spacing = node_->get_parameter("task2_max_waypoint_spacing").as_double();
  bool valid = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    valid = max_spacing > 0.0 &&
            bt_nodes::task2_common::snapshotInputsLocked(
                *ctx_, now, node_->get_parameter("pipeline_route_timeout_s").as_double(),
                node_->get_parameter("pipeline_ring_state_timeout_s").as_double(),
                node_->get_parameter("task2_ring_confidence_min").as_double(),
                node_->get_parameter("task2_route_min_z").as_double(),
                node_->get_parameter("task2_route_max_z").as_double(),
                node_->get_parameter("task2_min_segment_spacing").as_double(), max_spacing,
                node_->get_parameter("task2_min_path_length").as_double(),
                node_->get_parameter("task2_max_path_length").as_double(), route_, ring_states_);
  }

  if (!valid) {
    RCLCPP_ERROR(node_->get_logger(), "FollowTask2Pipeline: invalid route or ring snapshot");
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowTask2Pipeline::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  if (!bt_nodes::task2_common::dvlReady(
          *ctx_, now, node_->get_parameter("task2_use_dvl").as_bool(),
          node_->get_parameter("task2_dvl_timeout_s").as_double())) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "FollowTask2Pipeline: DVL live pulse stale");
    return BT::NodeStatus::FAILURE;
  }

  if ((now - start_time_).seconds() >= node_->get_parameter("task2_total_timeout_s").as_double() ||
      (now - segment_start_time_).seconds() >=
          node_->get_parameter("task2_segment_timeout_s").as_double()) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "FollowTask2Pipeline: timeout");
    return BT::NodeStatus::FAILURE;
  }

  const double pose_timeout = node_->get_parameter("task2_pose_timeout_s").as_double();
  const double state_timeout = node_->get_parameter("task2_state_timeout_s").as_double();
  geometry_msgs::msg::PoseStamped pose;
  mavros_msgs::msg::State state;
  bool vehicle_ready = false;
  bool pose_fresh = false;
  bool state_fresh = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose_fresh = ctx_->has_pose && bt_nodes::task2_common::receiptIsFresh(now, ctx_->last_pose_received, pose_timeout);
    state_fresh = ctx_->has_state && bt_nodes::task2_common::receiptIsFresh(now, ctx_->last_state_received, state_timeout);
    vehicle_ready = bt_nodes::task2_common::vehicleReady(*ctx_, now, pose_timeout, state_timeout, pose, state);
  }
  if (!vehicle_ready) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    if (!pose_fresh || !state_fresh) {
      if (!pose_fresh && !state_fresh) {
        RCLCPP_ERROR(node_->get_logger(),
                     "FollowTask2Pipeline: pose and state observations stale");
      } else if (!pose_fresh) {
        RCLCPP_ERROR(node_->get_logger(), "FollowTask2Pipeline: pose observation stale");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "FollowTask2Pipeline: state observation stale");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "FollowTask2Pipeline: FCU not ready for Task 2");
    }
    return BT::NodeStatus::FAILURE;
  }

  if (!state.connected || !state.armed || state.mode != "GUIDED" || !bt_nodes::task2_common::finitePose(pose)) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "FollowTask2Pipeline: FCU not ready for Task 2");
    return BT::NodeStatus::FAILURE;
  }

  if (entry_phase_) {
    const double dz = route_.front().pose.position.z - pose.pose.position.z;
    const double vertical_tolerance =
        node_->get_parameter("task2_waypoint_vertical_tolerance").as_double();
    if (std::abs(dz) > vertical_tolerance) {
      const double kp_v = node_->get_parameter("nav_kp_v").as_double();
      const double max_v = std::max(0.0, node_->get_parameter("nav_max_speed_v").as_double());
      ctx_->setpoint_pub->publish(
          velocitySetpoint(0.0, 0.0, std::clamp(kp_v * dz, -max_v, max_v), now));
      return BT::NodeStatus::RUNNING;
    }
    entry_phase_ = false;
    segment_start_time_ = now;
  }

  if (waypoint_index_ >= bt_nodes::task2_common::kWaypointCount) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  const auto& target = route_[waypoint_index_].pose.position;
  const double dx = target.x - pose.pose.position.x;
  const double dy = target.y - pose.pose.position.y;
  const double dz = target.z - pose.pose.position.z;
  const double horizontal_error = std::hypot(dx, dy);
  const bool arrived = horizontal_error <=
                           node_->get_parameter("task2_waypoint_horizontal_tolerance").as_double() &&
                       std::abs(dz) <= node_->get_parameter("task2_waypoint_vertical_tolerance").as_double();

  if (arrived) {
    bt_nodes::common::publishZeroVelocity(ctx_, now);
    if (!holding_) {
      holding_ = true;
      hold_start_time_ = now;
    }

    if ((now - hold_start_time_).seconds() < node_->get_parameter("task2_waypoint_hold_s").as_double()) {
      return BT::NodeStatus::RUNNING;
    }

    if (waypoint_index_ < bt_nodes::task2_common::kRingCount) {
      std_msgs::msg::String command;
      command.data = "ring_index=" + std::to_string(waypoint_index_) +
                     " color=" + bt_nodes::task2_common::task2RingColorName(ring_states_[waypoint_index_].color);
      if (ctx_->robocommand_pub) {
        ctx_->robocommand_pub->publish(command);
        RCLCPP_INFO(node_->get_logger(), "Task2 ring command published: %s", command.data.c_str());
      }
      ++waypoint_index_;
      holding_ = false;
      segment_start_time_ = now;
      return BT::NodeStatus::RUNNING;
    }

    RCLCPP_INFO(node_->get_logger(), "FollowTask2Pipeline: completed");
    return BT::NodeStatus::SUCCESS;
  }

  holding_ = false;
  const double kp_h = node_->get_parameter("nav_kp_h").as_double();
  const double kp_v = node_->get_parameter("nav_kp_v").as_double();
  const double max_h = std::max(0.0, node_->get_parameter("nav_max_speed_h").as_double());
  const double max_v = std::max(0.0, node_->get_parameter("nav_max_speed_v").as_double());
  double vx = kp_h * dx;
  double vy = kp_h * dy;
  const double horizontal_command = std::hypot(vx, vy);
  if (horizontal_command > max_h && horizontal_command > 0.0) {
    const double scale = max_h / horizontal_command;
    vx *= scale;
    vy *= scale;
  }
  const double vz = std::clamp(kp_v * dz, -max_v, max_v);
  ctx_->setpoint_pub->publish(velocitySetpoint(vx, vy, vz, now));
  return BT::NodeStatus::RUNNING;
}

void FollowTask2Pipeline::onHalted() {
  bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
