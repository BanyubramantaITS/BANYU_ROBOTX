#include "banyu_mission/bt_nodes/surface_at_red_target.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission {

SurfaceAtRedTarget::SurfaceAtRedTarget(const std::string& name, const BT::NodeConfig& config,
                                       const rclcpp::Node::SharedPtr& node,
                                       std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus SurfaceAtRedTarget::onStart() {
  start_time_ = node_->now();
  has_previous_pose_ = false;
  bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);

  std::string failure;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    const auto& run = ctx_->task2_camera_run;
    if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
      failure = "SurfaceAtRedTarget was started while inactive or aborted";
    } else if (!run.reverse_target_selected ||
        ctx_->task2_repair_status != banyu_msgs::msg::Task2Report::HOLD_COMPLETED_UNVERIFIED ||
        run.reverse_target_pose.header.frame_id != "map" ||
        !bt_nodes::task2_common::finitePose(run.reverse_target_pose)) {
      failure = "repair did not complete at a valid red-ring map target";
    }
    if (!failure.empty()) {
      ctx_->task2_camera_run.failure_reason = failure;
      ctx_->task2_camera_run.input_failure = true;
    }
  }
  if (!failure.empty()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);
    RCLCPP_ERROR(node_->get_logger(), "SurfaceAtRedTarget: %s", failure.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SurfaceAtRedTarget::onRunning() {
  const rclcpp::Time now = node_->now();
  bt_nodes::task2_camera::publishZeroBody(ctx_, now);
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->task2_camera_run.failure_reason = "SurfaceAtRedTarget was cancelled before completion";
    ctx_->task2_camera_run.input_failure = true;
    return BT::NodeStatus::FAILURE;
  }

  const double timeout_s = node_->get_parameter("task2_camera_surface_timeout_s").as_double();
  const double surface_depth = node_->get_parameter("task2_camera_surface_depth").as_double();
  const double kp_h = node_->get_parameter("task2_camera_surface_kp_h").as_double();
  const double kp_v = node_->get_parameter("task2_camera_surface_kp_v").as_double();
  const double max_h = node_->get_parameter("task2_camera_surface_max_speed_h").as_double();
  const double max_v = node_->get_parameter("task2_camera_surface_max_speed_v").as_double();
  const double horizontal_tolerance =
      node_->get_parameter("task2_camera_surface_horizontal_tolerance_m").as_double();
  const double vertical_tolerance =
      node_->get_parameter("task2_camera_surface_vertical_tolerance_m").as_double();
  const double max_pose_jump =
      node_->get_parameter("task2_camera_surface_max_pose_jump_m").as_double();
  const double source_timeout_s =
      node_->get_parameter("task2_camera_source_timeout_s").as_double();
  const double future_tolerance_s =
      node_->get_parameter("task2_camera_future_tolerance_s").as_double();

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
      failure = "vehicle readiness lost while surfacing at red target";
    } else if (!bt_nodes::task2_common::poseSourceIsFresh(
                   pose, now, source_timeout_s, future_tolerance_s)) {
      failure = "surface pose is invalid, stale, future-dated, or not in map frame";
    } else if (has_previous_pose_ &&
               bt_nodes::task2_camera::poseDistance(previous_pose_, pose) > max_pose_jump) {
      failure = "surface pose jump exceeded the safety limit";
    } else if ((now - start_time_).seconds() >= timeout_s) {
      failure = "surface-at-red-target timeout";
    } else {
      previous_pose_ = pose;
      has_previous_pose_ = true;
      const auto& target = run.reverse_target_pose.pose.position;
      const double dx = target.x - pose.pose.position.x;
      const double dy = target.y - pose.pose.position.y;
      const double dz = surface_depth - pose.pose.position.z;
      const double horizontal_error = std::hypot(dx, dy);
      const bool horizontal_ready = horizontal_error <= horizontal_tolerance;
      const bool vertical_ready = std::abs(dz) <= vertical_tolerance;
      if (horizontal_ready && vertical_ready) {
        ctx_->task2_final_surface_pose = pose;
        ctx_->has_task2_final_surface_pose = true;
        complete = true;
      } else {
        if (!horizontal_ready) {
          const double horizontal_command = kp_h * horizontal_error;
          const double scale = horizontal_command > max_h ? max_h / horizontal_command : 1.0;
          vx = kp_h * dx * scale;
          vy = kp_h * dy * scale;
        }
        if (!vertical_ready) {
          vz = std::clamp(kp_v * dz, -max_v, max_v);
        }
        command_velocity = true;
      }
    }
    if (!failure.empty()) {
      run.failure_reason = failure;
      run.input_failure = true;
    }
  }

  if (!failure.empty()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "SurfaceAtRedTarget: %s", failure.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (complete) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "SurfaceAtRedTarget: valid surface pose recorded");
    return BT::NodeStatus::SUCCESS;
  }
  if (command_velocity) {
    if (!ctx_->setpoint_pub) {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      ctx_->task2_camera_run.failure_reason = "SurfaceAtRedTarget setpoint publisher unavailable";
      ctx_->task2_camera_run.input_failure = true;
      bt_nodes::task2_camera::publishZeroBody(ctx_, now);
      RCLCPP_ERROR(node_->get_logger(), "SurfaceAtRedTarget: setpoint publisher unavailable");
      return BT::NodeStatus::FAILURE;
    }
    ctx_->setpoint_pub->publish(velocitySetpoint(vx, vy, vz, now));
  }
  return BT::NodeStatus::RUNNING;
}

void SurfaceAtRedTarget::onHalted() {
  bt_nodes::task2_camera::publishZeroBody(ctx_, node_->now());
}

}  // namespace banyu_mission
