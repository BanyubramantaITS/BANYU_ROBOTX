#include "banyu_vehicle_execution/action_servers/align_heading_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

AlignHeadingServer::AlignHeadingServer(
  rclcpp::Node * node, const std::string & action_name, ActionServerCoordinator * coordinator,
  double yaw_kp,
  double max_yaw_rate)
: TypedActionServer(node, action_name, coordinator), yaw_kp_(yaw_kp), max_yaw_rate_(max_yaw_rate) {}

bool AlignHeadingServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.tolerance_rad <= 0.0) {
    reason = "tolerance_rad must be positive";
    return false;
  }
  return true;
}

void AlignHeadingServer::onGoalStarted()
{
  hold_captured_ = false;
}

MotionIntent AlignHeadingServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    cancelActive();
    return zeroIntent();
  }
  if (!telemetry.pose_valid) {
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  const double heading_err = normalizeAngle(goal->target_heading_rad - telemetry.yaw);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_heading_rad = telemetry.yaw;
  feedback->heading_error_rad = heading_err;
  active_handle_->publish_feedback(feedback);

  if (std::abs(heading_err) <= goal->tolerance_rad) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Heading aligned";
    result->final_heading_rad = telemetry.yaw;
    result->final_error_rad = heading_err;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle {active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  const double yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);
  double vx = 0.0;
  double vy = 0.0;
  if (telemetry.gps_valid) {
    if (!hold_captured_) {
      hold_latitude_deg_ = telemetry.latitude_deg;
      hold_longitude_deg_ = telemetry.longitude_deg;
      hold_captured_ = true;
    }
    double dx = 0.0;
    double dy = 0.0;
    latLonToEnuOffsets(telemetry.latitude_deg, telemetry.longitude_deg, hold_latitude_deg_, hold_longitude_deg_, dx, dy);
    constexpr double kHoldKp = 0.5;
    constexpr double kHoldMaxSpeed = 0.2;
    const double speed = std::min(kHoldKp * std::hypot(dx, dy), kHoldMaxSpeed);
    const double dist = std::hypot(dx, dy);
    if (dist > 1e-6) {
      vx = speed * dx / dist;
      vy = speed * dy / dist;
    }
  }
  return MotionIntent::localEnu(vx, vy, 0.0, yaw_rate);
}

}  // namespace banyu_vehicle_execution
