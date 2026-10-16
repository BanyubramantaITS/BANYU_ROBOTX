#include "banyu_vehicle_execution/action_servers/hold_probe_on_ring_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

HoldProbeOnRingServer::HoldProbeOnRingServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double ring_kp_sway,
  double ring_kp_surge)
: TypedActionServer(node, action_name, coordinator),
  ring_kp_sway_(ring_kp_sway), ring_kp_surge_(ring_kp_surge)
{
}

void HoldProbeOnRingServer::onGoalStarted()
{
  elapsed_aligned_s_ = 0.0;
  max_alignment_error_ = 0.0;
}

bool HoldProbeOnRingServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.hold_duration_s <= 0.0 || goal.alignment_tolerance <= 0.0) {
    reason = "hold_duration_s and alignment_tolerance must be positive";
    return false;
  }
  return true;
}

MotionIntent HoldProbeOnRingServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double dt)
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
  const double error = visible ? std::hypot(offset_x, offset_y) : 1.0;
  const bool aligned = visible && (error <= goal->alignment_tolerance);

  if (aligned) {
    elapsed_aligned_s_ += dt;
  } else {
    elapsed_aligned_s_ = 0.0;
  }
  max_alignment_error_ = std::max(max_alignment_error_, error);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->elapsed_s = elapsed_aligned_s_;
  feedback->alignment_error = error;
  feedback->aligned = aligned;
  active_handle_->publish_feedback(feedback);

  if (elapsed_aligned_s_ >= goal->hold_duration_s) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Probe held on ring for required duration";
    result->held_duration_s = elapsed_aligned_s_;
    result->maximum_alignment_error = max_alignment_error_;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  // Keep centering while holding, as the alignment actions do: the vehicle coasts after
  // CenterOnRing and a zero command would let the ring drift out of tolerance.
  if (visible && telemetry.pose_valid) {
    const double sway = -std::clamp(ring_kp_sway_ * offset_x, -0.5, 0.5);
    const double surge = -std::clamp(ring_kp_surge_ * offset_y, -0.3, 0.3);
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(surge, sway, 0.0, 0.0), telemetry.yaw);
    return local_intent.value_or(zeroIntent());
  }
  return zeroIntent();
}

}  // namespace banyu_vehicle_execution
