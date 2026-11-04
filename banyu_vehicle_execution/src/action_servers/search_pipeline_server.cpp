#include "banyu_vehicle_execution/action_servers/search_pipeline_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

SearchPipelineServer::SearchPipelineServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double default_sweep_rate,
  double min_confidence,
  double retrace_distance_m,
  double retrace_speed,
  double pinger_timeout_s,
  double pinger_kp,
  double pinger_forward_speed,
  bool use_pinger)
: TypedActionServer(node, action_name, coordinator),
  default_sweep_rate_(default_sweep_rate), min_confidence_(min_confidence),
  retrace_distance_m_(std::max(0.0, retrace_distance_m)), retrace_speed_(std::abs(retrace_speed)),
  pinger_timeout_s_(std::max(0.0, pinger_timeout_s)), pinger_kp_(pinger_kp),
  pinger_forward_speed_(pinger_forward_speed), use_pinger_(use_pinger)
{
}

void SearchPipelineServer::onGoalStarted()
{
  sweep_progress_rad_ = 0.0;
  retrace_started_ = false;
  retrace_done_ = retrace_distance_m_ <= 0.0 || retrace_speed_ <= 0.0;
  retrace_elapsed_s_ = 0.0;
}

MotionIntent SearchPipelineServer::tick(
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
  const double sweep_rate = (goal->sweep_rate_rad_s != 0.0) ?
    goal->sweep_rate_rad_s : default_sweep_rate_;

  const bool pinger_fresh = use_pinger_ && telemetry.pinger_obs_valid &&
    telemetry.pinger_obs_stamp.nanoseconds() > 0 &&
    (now - telemetry.pinger_obs_stamp).seconds() >= 0.0 &&
    (now - telemetry.pinger_obs_stamp).seconds() <= pinger_timeout_s_;

  // The pipe is usually lost by overrunning an end or a bend, which a sweep in place cannot
  // see past the camera footprint; retrace along the entry heading before sweeping (unless homing on pinger).
  bool retracing = false;
  if (!pinger_fresh && !retrace_done_ && telemetry.pose_valid) {
    if (!retrace_started_) {
      retrace_started_ = true;
      retrace_origin_x_ = telemetry.pos_x;
      retrace_origin_y_ = telemetry.pos_y;
      retrace_heading_ = telemetry.yaw;
    }
    if (dt > 0.0) {
      retrace_elapsed_s_ += dt;
    }
    const double travelled = std::hypot(
      telemetry.pos_x - retrace_origin_x_, telemetry.pos_y - retrace_origin_y_);
    retrace_done_ = travelled >= retrace_distance_m_ ||
      (retrace_speed_ > 0.0 && retrace_elapsed_s_ >= 2.0 * retrace_distance_m_ / retrace_speed_);
    retracing = !retrace_done_;
  }

  // Only accumulate visual sweep progress when actually performing the yaw sweep
  if (!retracing && !pinger_fresh && dt > 0.0) {
    sweep_progress_rad_ += std::abs(sweep_rate) * dt;
  }

  const bool is_tracked = telemetry.pipeline_obs_valid &&
    telemetry.pipeline_status == banyu_msgs::msg::PipelineObservation::TRACKED;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_heading_rad = telemetry.yaw;
  feedback->sweep_progress_rad = sweep_progress_rad_;
  feedback->pipeline_visible = is_tracked;
  active_handle_->publish_feedback(feedback);

  if (is_tracked && telemetry.pipeline_confidence >= min_confidence_) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Pipeline found during search";
    result->found = true;
    result->pipeline_id = "pipeline_1";
    result->found_heading_rad = telemetry.yaw;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  if (!pinger_fresh && sweep_progress_rad_ >= goal->search_span_rad &&
    goal->search_span_rad > 0.0)
  {
    auto result = std::make_shared<Result>();
    result->success = false;
    result->reason = "Search span exhausted without acquiring pipeline";
    result->found = false;
    result->pipeline_id = "";
    result->found_heading_rad = telemetry.yaw;
    active_handle_->abort(result);
    coordinator_->faultGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  if (pinger_fresh) {
    const double max_rate = (sweep_rate != 0.0) ? std::abs(sweep_rate) : default_sweep_rate_;
    const double pinger_yaw_rate = std::clamp(
      pinger_kp_ * telemetry.pinger_bearing_rad, -max_rate, max_rate);
    const double forward = (std::abs(telemetry.pinger_bearing_rad) <
      0.35) ? pinger_forward_speed_ : 0.0;
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(forward, 0.0, 0.0, pinger_yaw_rate), telemetry.yaw);
    return local_intent.value_or(zeroIntent());
  }

  if (retracing) {
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(-retrace_speed_, 0.0, 0.0, 0.0), retrace_heading_);
    return local_intent.value_or(zeroIntent());
  }
  return MotionIntent::localEnu(0.0, 0.0, 0.0, sweep_rate);
}

}  // namespace banyu_vehicle_execution
