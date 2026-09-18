#include "banyu_vehicle_execution/action_servers/acquire_pipeline_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

AcquirePipelineServer::AcquirePipelineServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double yaw_kp,
  double max_yaw_rate,
  double min_confidence)
: TypedActionServer(node, action_name, coordinator),
  yaw_kp_(yaw_kp), max_yaw_rate_(max_yaw_rate), min_confidence_(min_confidence)
{
}

MotionIntent AcquirePipelineServer::tick(
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
  double yaw_rate = 0.0;
  if (goal->use_heading_prior) {
    const double heading_err = normalizeAngle(goal->heading_prior_rad - telemetry.yaw);
    if (std::abs(heading_err) > goal->heading_prior_tolerance_rad) {
      yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);
    }
  }

  const bool is_tracked = telemetry.pipeline_obs_valid &&
    telemetry.pipeline_status == banyu_msgs::msg::PipelineObservation::TRACKED;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->pipeline_visible = is_tracked;
  feedback->confidence = telemetry.pipeline_confidence;
  feedback->distance_m = 0.0;
  feedback->heading_error_rad = telemetry.pipeline_heading_error_rad;
  active_handle_->publish_feedback(feedback);

  if (is_tracked && telemetry.pipeline_confidence >= min_confidence_) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Pipeline acquired";
    result->acquired = true;
    result->pipeline_id = "pipeline_1";
    result->pipeline_heading_rad = normalizeAngle(
      telemetry.yaw + telemetry.pipeline_heading_error_rad);
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  return MotionIntent::localEnu(0.0, 0.0, 0.0, yaw_rate);
}

}  // namespace banyu_vehicle_execution
