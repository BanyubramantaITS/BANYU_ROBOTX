#include "banyu_vehicle_execution/action_servers/dive_to_depth_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

DiveToDepthServer::DiveToDepthServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double depth_kp,
  double max_depth_rate,
  double yaw_kp,
  double max_yaw_rate)
: TypedActionServer(node, action_name, coordinator),
  depth_kp_(depth_kp), max_depth_rate_(max_depth_rate),
  yaw_kp_(yaw_kp), max_yaw_rate_(max_yaw_rate)
{
}

bool DiveToDepthServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.depth_tolerance_m <= 0.0 || goal.heading_tolerance_rad <= 0.0) {
    reason = "tolerances must be positive";
    return false;
  }
  return true;
}

MotionIntent DiveToDepthServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double /*dt*/)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    cancelActive();
    return zeroIntent();
  }
  if (!telemetry.depth_valid || !telemetry.pose_valid) {
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  const double depth_err = goal->target_depth_m - telemetry.depth_m;
  const double heading_err = normalizeAngle(goal->target_heading_rad - telemetry.yaw);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_depth_m = telemetry.depth_m;
  feedback->current_heading_rad = telemetry.yaw;
  feedback->depth_error_m = depth_err;
  feedback->heading_error_rad = heading_err;
  active_handle_->publish_feedback(feedback);

  if (std::abs(depth_err) <= goal->depth_tolerance_m &&
    std::abs(heading_err) <= goal->heading_tolerance_rad)
  {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Target depth and heading reached";
    result->final_depth_m = telemetry.depth_m;
    result->final_heading_rad = telemetry.yaw;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  const double vz = -std::clamp(depth_kp_ * depth_err, -max_depth_rate_, max_depth_rate_);
  const double yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);
  return MotionIntent::localEnu(0.0, 0.0, vz, yaw_rate);
}

}  // namespace banyu_vehicle_execution
