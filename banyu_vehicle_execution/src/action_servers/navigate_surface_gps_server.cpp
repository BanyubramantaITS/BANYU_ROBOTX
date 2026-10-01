#include "banyu_vehicle_execution/action_servers/navigate_surface_gps_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

NavigateSurfaceGpsServer::NavigateSurfaceGpsServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double nav_kp,
  double nav_max_speed,
  double yaw_kp,
  double max_yaw_rate)
: TypedActionServer(node, action_name, coordinator),
  nav_kp_(nav_kp), nav_max_speed_(nav_max_speed),
  yaw_kp_(yaw_kp), max_yaw_rate_(max_yaw_rate)
{
}

bool NavigateSurfaceGpsServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.tolerance_m <= 0.0) {
    reason = "tolerance_m must be positive";
    return false;
  }
  return true;
}

MotionIntent NavigateSurfaceGpsServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double /*dt*/)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    cancelActive();
    return zeroIntent();
  }
  if (!telemetry.gps_valid || !telemetry.pose_valid) {
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  double dx = 0.0;
  double dy = 0.0;
  latLonToEnuOffsets(
    telemetry.latitude_deg, telemetry.longitude_deg,
    goal->target_latitude_deg, goal->target_longitude_deg,
    dx, dy);

  const double distance = std::hypot(dx, dy);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_latitude_deg = telemetry.latitude_deg;
  feedback->current_longitude_deg = telemetry.longitude_deg;
  feedback->distance_remaining_m = distance;
  active_handle_->publish_feedback(feedback);

  if (distance <= goal->tolerance_m) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Target GPS coordinates reached";
    result->final_latitude_deg = telemetry.latitude_deg;
    result->final_longitude_deg = telemetry.longitude_deg;
    result->final_error_m = distance;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  const double bearing = std::atan2(dy, dx);
  const double heading_err = normalizeAngle(bearing - telemetry.yaw);
  const double speed = std::clamp(nav_kp_ * distance, 0.0, nav_max_speed_);
  const double yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);

  const double surge = speed * std::cos(heading_err);
  const double sway = speed * std::sin(heading_err);
  const auto local_intent = bodyFluToLocalEnu(
    MotionIntent::bodyFlu(surge, sway, 0.0, yaw_rate), telemetry.yaw);
  return local_intent.value_or(zeroIntent());
}

}  // namespace banyu_vehicle_execution
