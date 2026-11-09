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
  double min_confidence,
  double timeout_s,
  double surge_speed,
  int confirm_frames)
: TypedActionServer(node, action_name, coordinator),
  yaw_kp_(yaw_kp), max_yaw_rate_(max_yaw_rate), min_confidence_(min_confidence),
  timeout_s_(timeout_s), surge_speed_(surge_speed), confirm_frames_(confirm_frames)
{
}

void AcquirePipelineServer::onGoalStarted()
{
  start_time_ = node_->now();
  consecutive_frames_ = 0;
  light_cue_used_s_ = 0.0;
  light_cue_logged_ = false;
}

MotionIntent AcquirePipelineServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double dt)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    cancelActive();
    start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    consecutive_frames_ = 0;
    return zeroIntent();
  }
  if (!telemetry.pose_valid) {
    return zeroIntent();
  }

  if (start_time_.nanoseconds() == 0) {
    start_time_ = now;
  }

  if (timeout_s_ > 0.0 && (now - start_time_).seconds() >= timeout_s_) {
    auto result = std::make_shared<Result>();
    result->success = false;
    result->reason = "Acquisition timed out without acquiring pipeline";
    result->acquired = false;
    result->pipeline_id = "";
    result->pipeline_heading_rad = telemetry.yaw;
    active_handle_->abort(result);
    coordinator_->faultGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    consecutive_frames_ = 0;
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  double yaw_rate = 0.0;
  double surge = surge_speed_;
  if (goal->use_heading_prior) {
    const double heading_err = normalizeAngle(goal->heading_prior_rad - telemetry.yaw);
    if (std::abs(heading_err) > goal->heading_prior_tolerance_rad) {
      yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);
    }
    const double align_gate = std::max(goal->heading_prior_tolerance_rad * 2.0, 0.25);
    if (std::abs(heading_err) > align_gate) {
      surge = 0.0;
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

  bool heading_matches = true;
  if (goal->use_heading_prior) {
    const double detected_pipeline_heading = normalizeAngle(
      telemetry.yaw + telemetry.pipeline_heading_error_rad);
    double heading_diff = std::abs(
      normalizeAngle(detected_pipeline_heading - goal->heading_prior_rad));
    heading_diff = std::min(heading_diff, std::abs(normalizeAngle(heading_diff - M_PI)));
    const double tol = goal->heading_prior_tolerance_rad > 0.0 ?
      goal->heading_prior_tolerance_rad : 0.4;
    if (heading_diff > tol) {
      heading_matches = false;
    }
  }

  if (is_tracked && telemetry.pipeline_confidence >= min_confidence_ && heading_matches) {
    ++consecutive_frames_;
    if (consecutive_frames_ >= confirm_frames_) {
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
      start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      consecutive_frames_ = 0;
      return zeroIntent();
    }
  } else {
    consecutive_frames_ = 0;
  }

  // No pipe yet but a light in view: steer over it (the node sits on the pipe) instead of driving on.
  double sway = 0.0;
  const auto cue = is_tracked ? std::nullopt : lightCueVelocity(telemetry, light_cue_);
  if (cue && light_cue_used_s_ < light_cue_.budget_s) {
    light_cue_used_s_ += std::max(0.0, dt);
    surge = cue->surge;
    sway = cue->sway;
    if (!light_cue_logged_) {
      light_cue_logged_ = true;
      RCLCPP_INFO(node_->get_logger(), "AcquirePipeline: light in view, steering over it to find the pipe");
    }
  }

  const auto local_intent = bodyFluToLocalEnu(
    MotionIntent::bodyFlu(surge, sway, 0.0, yaw_rate), telemetry.yaw);
  return local_intent.value_or(zeroIntent());
}

}  // namespace banyu_vehicle_execution
