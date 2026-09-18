#include "banyu_vehicle_execution/action_servers/surface_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

SurfaceServer::SurfaceServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double depth_kp,
  double max_depth_rate)
: TypedActionServer(node, action_name, coordinator),
  depth_kp_(depth_kp), max_depth_rate_(max_depth_rate)
{
}

bool SurfaceServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.depth_tolerance_m <= 0.0) {
    reason = "depth_tolerance_m must be positive";
    return false;
  }
  return true;
}

MotionIntent SurfaceServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double /*dt*/)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    cancelActive();
    return zeroIntent();
  }
  if (!telemetry.depth_valid) {
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  const double depth_err = goal->target_depth_m - telemetry.depth_m;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_depth_m = telemetry.depth_m;
  feedback->depth_error_m = depth_err;
  active_handle_->publish_feedback(feedback);

  if (std::abs(depth_err) <= goal->depth_tolerance_m) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Surface depth reached";
    result->final_depth_m = telemetry.depth_m;
    result->final_error_m = depth_err;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  const double vz = -std::clamp(depth_kp_ * depth_err, -max_depth_rate_, max_depth_rate_);
  return MotionIntent::localEnu(0.0, 0.0, vz, 0.0);
}

}  // namespace banyu_vehicle_execution
