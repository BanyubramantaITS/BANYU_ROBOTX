#include "banyu_mission/bt_nodes/repair_red_ring.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission {

RepairRedRing::RepairRedRing(const std::string& name, const BT::NodeConfig& config,
                             const rclcpp::Node::SharedPtr& node,
                             std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus RepairRedRing::onStart() {
  start_time_ = node_->now();
  valid_hold_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  hold_started_ = false;
  has_previous_pose_ = false;
  bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);

  std::string failure;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    auto& run = ctx_->task2_camera_run;
    if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
      failure = "RepairRedRing was started while inactive or aborted";
    } else {
      ctx_->task2_repair_started = true;
      ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::NOT_ATTEMPTED;
      ctx_->task2_repair_hold_duration_s = 0.0;
    }
    if (failure.empty() && (!run.reverse_target_selected || !run.reverse_reacquired ||
        run.reverse_target_pose.header.frame_id != "map" ||
        !bt_nodes::task2_common::finitePose(run.reverse_target_pose))) {
      ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::REPAIR_FAILED;
      failure = "red-ring reacquisition did not provide a valid map target";
      run.failure_reason = failure;
      run.input_failure = true;
    }
    if (!failure.empty()) {
      run.failure_reason = failure;
      run.input_failure = true;
    }
  }
  if (!failure.empty()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);
    RCLCPP_ERROR(node_->get_logger(), "RepairRedRing: %s", failure.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RepairRedRing::onRunning() {
  const rclcpp::Time now = node_->now();
  bt_nodes::task2_camera::publishZeroBody(ctx_, now);
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->task2_camera_run.failure_reason = "RepairRedRing was cancelled before completion";
    ctx_->task2_camera_run.input_failure = true;
    ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::REPAIR_FAILED;
    return BT::NodeStatus::FAILURE;
  }

  const double timeout_s = node_->get_parameter("task2_camera_repair_timeout_s").as_double();
  const double duration_s = node_->get_parameter("task2_camera_repair_duration_s").as_double();
  const double kp_h = node_->get_parameter("task2_camera_repair_kp_h").as_double();
  const double kp_v = node_->get_parameter("task2_camera_repair_kp_v").as_double();
  const double max_h = node_->get_parameter("task2_camera_repair_max_speed_h").as_double();
  const double max_v = node_->get_parameter("task2_camera_repair_max_speed_v").as_double();
  const double horizontal_tolerance =
      node_->get_parameter("task2_camera_repair_horizontal_tolerance_m").as_double();
  const double vertical_tolerance =
      node_->get_parameter("task2_camera_repair_vertical_tolerance_m").as_double();
  const double max_pose_jump =
      node_->get_parameter("task2_camera_repair_max_pose_jump_m").as_double();

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  bool command_velocity = false;
  bool complete = false;
  std::string failure;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    auto& run = ctx_->task2_camera_run;
    geometry_msgs::msg::PoseStamped pose;
    mavros_msgs::msg::State state;
    const bool ready =
        bt_nodes::task2_common::dvlReadyLocked(
            *ctx_, now, node_->get_parameter("task2_use_dvl").as_bool(),
            node_->get_parameter("task2_dvl_timeout_s").as_double()) &&
        bt_nodes::task2_common::vehicleReady(
            *ctx_, now, node_->get_parameter("task2_pose_timeout_s").as_double(),
            node_->get_parameter("task2_state_timeout_s").as_double(), pose, state);
    if (run.input_failure) {
      failure = run.failure_reason;
    } else if (!ready) {
      failure = "vehicle readiness lost during red-ring repair";
    } else if (pose.header.frame_id != "map" || !bt_nodes::task2_common::finitePose(pose)) {
      failure = "repair pose is invalid or not in map frame";
    } else if (has_previous_pose_ &&
               bt_nodes::task2_camera::poseDistance(previous_pose_, pose) > max_pose_jump) {
      failure = "repair pose jump exceeded the safety limit";
    } else if ((now - start_time_).seconds() >= timeout_s) {
      failure = "red-ring repair timeout";
    } else {
      previous_pose_ = pose;
      has_previous_pose_ = true;
      const auto& target = run.reverse_target_pose.pose.position;
      const double dx = target.x - pose.pose.position.x;
      const double dy = target.y - pose.pose.position.y;
      const double dz = target.z - pose.pose.position.z;
      const double horizontal_error = std::hypot(dx, dy);
      const bool within_tolerance = horizontal_error <= horizontal_tolerance &&
                                    std::abs(dz) <= vertical_tolerance;
      if (within_tolerance) {
        if (!hold_started_) {
          hold_started_ = true;
          valid_hold_start_ = now;
        }
        if ((now - valid_hold_start_).seconds() >= duration_s) {
          ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::HOLD_COMPLETED_UNVERIFIED;
          ctx_->task2_repair_hold_duration_s =
              std::max(duration_s, (now - valid_hold_start_).seconds());
          complete = true;
        }
      } else {
        hold_started_ = false;
        const double horizontal_command = kp_h * horizontal_error;
        if (horizontal_command > 0.0) {
          const double scale = std::min(1.0, max_h / horizontal_command);
          vx = kp_h * dx * scale;
          vy = kp_h * dy * scale;
        }
        vz = std::clamp(kp_v * dz, -max_v, max_v);
        command_velocity = true;
      }
    }
    if (!failure.empty()) {
      ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::REPAIR_FAILED;
      run.failure_reason = failure;
      run.input_failure = true;
    }
  }

  if (!failure.empty()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "RepairRedRing: %s", failure.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (complete) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "RepairRedRing: hold completed unverified");
    return BT::NodeStatus::SUCCESS;
  }
  if (command_velocity) {
    if (!ctx_->setpoint_pub) {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      ctx_->task2_camera_run.failure_reason = "RepairRedRing setpoint publisher unavailable";
      ctx_->task2_camera_run.input_failure = true;
      ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::REPAIR_FAILED;
      bt_nodes::task2_camera::publishZeroBody(ctx_, now);
      RCLCPP_ERROR(node_->get_logger(), "RepairRedRing: setpoint publisher unavailable");
      return BT::NodeStatus::FAILURE;
    }
    ctx_->setpoint_pub->publish(velocitySetpoint(vx, vy, vz, now));
  }
  return BT::NodeStatus::RUNNING;
}

void RepairRedRing::onHalted() {
  bt_nodes::task2_camera::publishZeroBody(ctx_, node_->now());
}

}  // namespace banyu_mission
