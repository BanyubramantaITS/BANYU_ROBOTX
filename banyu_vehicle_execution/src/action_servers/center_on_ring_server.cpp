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
  double ring_kp_z)
: TypedActionServer(node, action_name, coordinator),
  ring_kp_sway_(ring_kp_sway), ring_kp_z_(ring_kp_z)
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

MotionIntent CenterOnRingServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double /*dt*/)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
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
    return zeroIntent();
  }

  if (visible) {
    if (!telemetry.pose_valid) {
      return zeroIntent();
    }
    const double sway = -std::clamp(ring_kp_sway_ * offset_x, -0.5, 0.5);
    const double vz = -std::clamp(ring_kp_z_ * offset_y, -0.5, 0.5);
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(0.0, sway, vz, 0.0), telemetry.yaw);
    return local_intent.value_or(zeroIntent());
  }

  return zeroIntent();
}

}  // namespace banyu_vehicle_execution
