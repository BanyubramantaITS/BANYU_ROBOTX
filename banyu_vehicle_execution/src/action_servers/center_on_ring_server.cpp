#include "banyu_vehicle_execution/action_servers/center_on_ring_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

CenterOnRingServer::CenterOnRingServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double ring_kp_sway,
  double ring_kp_surge,
  double search_speed,
  double search_timeout_s,
  double lost_grace_s)
: TypedActionServer(node, action_name, coordinator),
  ring_kp_sway_(ring_kp_sway), ring_kp_surge_(ring_kp_surge),
  search_speed_(std::abs(search_speed)), search_timeout_s_(search_timeout_s),
  lost_grace_s_(std::max(0.0, lost_grace_s))
{
}

bool CenterOnRingServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.tolerance <= 0.0) {
    reason = "tolerance must be positive";
    return false;
  }
  return true;
}

void CenterOnRingServer::onGoalStarted()
{
  search_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_visible_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_surge_ = 0.0;
  last_sway_ = 0.0;
}

MotionIntent CenterOnRingServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double /*dt*/)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    search_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    cancelActive();
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  const bool visible = telemetry.ring_obs_valid && telemetry.ring_visible;
  const double offset_x = visible ? telemetry.ring_offset_x : 0.0;
  const double offset_y = visible ? telemetry.ring_offset_y : 0.0;
  const double alignment_error = visible ? std::hypot(offset_x, offset_y) : 1.0;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->ring_visible = visible;
  feedback->offset_x = offset_x;
  feedback->offset_y = offset_y;
  feedback->alignment_error = alignment_error;
  active_handle_->publish_feedback(feedback);

  if (visible && alignment_error <= goal->tolerance) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Centered on ring";
    result->ring_color = goal->ring_color;
    result->final_offset_x = offset_x;
    result->final_offset_y = offset_y;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    search_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return zeroIntent();
  }

  if (!telemetry.pose_valid) {
    return zeroIntent();
  }

  // Bottom camera: image +x is body right, image +y (down) is body aft. Depth is held.
  if (visible) {
    search_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_visible_ = now;
    last_sway_ = -std::clamp(ring_kp_sway_ * offset_x, -0.5, 0.5);
    last_surge_ = -std::clamp(ring_kp_surge_ * offset_y, -0.3, 0.3);
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(last_surge_, last_sway_, 0.0, 0.0), telemetry.yaw);
    return local_intent.value_or(zeroIntent());
  }

  // Ring detections flicker; keep the last centering command through short dropouts.
  if (last_visible_.nanoseconds() != 0 && (now - last_visible_).seconds() < lost_grace_s_) {
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(last_surge_, last_sway_, 0.0, 0.0), telemetry.yaw);
    return local_intent.value_or(zeroIntent());
  }

  // The vehicle coasts past the ring after following stops, so search by backing up.
  if (search_started_.nanoseconds() == 0) {
    search_started_ = now;
  }
  if (search_timeout_s_ > 0.0 && (now - search_started_).seconds() >= search_timeout_s_) {
    abortWithReason("Ring not reacquired within search timeout", offset_x, offset_y);
    return zeroIntent();
  }
  const auto local_intent = bodyFluToLocalEnu(
    MotionIntent::bodyFlu(-search_speed_, 0.0, 0.0, 0.0), telemetry.yaw);
  return local_intent.value_or(zeroIntent());
}

void CenterOnRingServer::abortWithReason(
  const std::string & reason, double offset_x, double offset_y)
{
  auto result = std::make_shared<Result>();
  result->success = false;
  result->reason = reason;
  result->ring_color = active_handle_ ? active_handle_->get_goal()->ring_color : "";
  result->final_offset_x = offset_x;
  result->final_offset_y = offset_y;
  search_started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  abortActiveWithResult(result);
}

}  // namespace banyu_vehicle_execution
