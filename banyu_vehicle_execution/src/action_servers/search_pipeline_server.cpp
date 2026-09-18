#include "banyu_vehicle_execution/action_servers/search_pipeline_server.hpp"

#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

SearchPipelineServer::SearchPipelineServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double default_sweep_rate,
  double min_confidence)
: TypedActionServer(node, action_name, coordinator),
  default_sweep_rate_(default_sweep_rate), min_confidence_(min_confidence)
{
}

void SearchPipelineServer::onGoalStarted()
{
  sweep_progress_rad_ = 0.0;
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
  sweep_progress_rad_ += std::abs(sweep_rate) * dt;

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

  if (sweep_progress_rad_ >= goal->search_span_rad && goal->search_span_rad > 0.0) {
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

  return MotionIntent::localEnu(0.0, 0.0, 0.0, sweep_rate);
}

}  // namespace banyu_vehicle_execution
