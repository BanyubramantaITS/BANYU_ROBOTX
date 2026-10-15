#include "banyu_vehicle_execution/action_servers/follow_pipeline_server.hpp"

#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

FollowPipelineServer::FollowPipelineServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, action_name, coordinator)
{
}

MotionIntent FollowPipelineServer::tick(
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
  if (!goal->enable) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Pipeline following stopped";
    result->enabled = false;
    result->final_status = "disabled";
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  const bool tracking = telemetry.guidance_intent_valid &&
    telemetry.guidance_status == banyu_guidance::msg::VelocityIntent::TRACKING;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->following = tracking;
  feedback->status = tracking ? "tracking" : "searching";
  const bool pipe_tracked = telemetry.pipeline_obs_valid &&
    telemetry.pipeline_status == banyu_msgs::msg::PipelineObservation::TRACKED &&
    std::isfinite(telemetry.pipeline_offset_x);
  feedback->lateral_error_m = pipe_tracked ? telemetry.pipeline_offset_x : 0.0;
  feedback->distance_error_m = 0.0;
  active_handle_->publish_feedback(feedback);

  if (tracking && telemetry.pose_valid) {
    const double surge = telemetry.guidance_surge_mps;
    const double sway = telemetry.guidance_sway_mps;
    const double yaw_rate =
      telemetry.guidance_yaw_control_active ? telemetry.guidance_yaw_rate_rad_s : 0.0;
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(surge, sway, 0.0, yaw_rate), telemetry.yaw);
    return local_intent.value_or(zeroIntent());
  }

  return zeroIntent();
}

}  // namespace banyu_vehicle_execution
