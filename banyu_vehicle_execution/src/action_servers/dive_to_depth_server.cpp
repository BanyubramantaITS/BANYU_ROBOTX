#include "banyu_vehicle_execution/action_servers/dive_to_depth_server.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include "banyu_msgs/msg/pipeline_observation.hpp"

namespace banyu_vehicle_execution
{

DiveToDepthServer::DiveToDepthServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double depth_kp,
  double max_depth_rate,
  double yaw_kp,
  double max_yaw_rate)
: TypedActionServer(node, action_name, coordinator),
  depth_kp_(depth_kp), max_depth_rate_(max_depth_rate),
  yaw_kp_(yaw_kp), max_yaw_rate_(max_yaw_rate)
{
}

void DiveToDepthServer::setCameraDescent(double rate_mps, double lock_s, double max_time_s, double surge_mps)
{
  camera_surge_mps_ = std::isfinite(surge_mps) ? std::max(0.0, surge_mps) : 0.0;
  if (!(rate_mps > 0.0) || !(lock_s > 0.0) || !(max_time_s > 0.0)) {
    throw std::invalid_argument("camera descent rate, lock time and maximum time must be positive");
  }
  camera_rate_mps_ = rate_mps;
  camera_lock_s_ = lock_s;
  camera_max_time_s_ = max_time_s;
}

void DiveToDepthServer::onGoalStarted()
{
  elapsed_s_ = 0.0;
  locked_s_ = 0.0;
}

bool DiveToDepthServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.depth_tolerance_m <= 0.0 || goal.heading_tolerance_rad <= 0.0) {
    reason = "tolerances must be positive";
    return false;
  }
  return true;
}

MotionIntent DiveToDepthServer::cameraDescentTick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double dt, const Goal & goal)
{
  elapsed_s_ += std::max(0.0, dt);
  const bool pipe = telemetry.pipeline_obs_valid &&
    telemetry.pipeline_status == banyu_msgs::msg::PipelineObservation::TRACKED && telemetry.camera_range_valid;
  // Only the pipe (with a range from its width) ends the descent: a lit light alone can be the start buoy's indicator at the
  // surface (a dive ended at 0.2 m depth on it).
  const bool light = false;
  locked_s_ = pipe ? locked_s_ + std::max(0.0, dt) : 0.0;
  const bool heading_valid = telemetry.pose_valid || telemetry.imu_valid;
  const double yaw = telemetry.pose_valid ? telemetry.yaw : telemetry.imu_yaw;
  const double heading_err = heading_valid ? normalizeAngle(goal.target_heading_rad - yaw) : 0.0;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_depth_m = telemetry.heave_odom_m;
  feedback->current_heading_rad = yaw;
  feedback->depth_error_m = std::numeric_limits<double>::quiet_NaN();
  feedback->heading_error_rad = heading_err;
  active_handle_->publish_feedback(feedback);

  if (locked_s_ >= camera_lock_s_) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = pipe ? "Pipe in view of the bottom camera" : "Light in view of the bottom camera";
    result->final_depth_m = telemetry.heave_odom_m;
    result->final_heading_rad = yaw;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }
  if (elapsed_s_ >= camera_max_time_s_) {
    auto result = std::make_shared<Result>();
    result->success = false;
    result->reason = "Descent time limit reached with nothing in view of the bottom camera";
    result->final_depth_m = telemetry.heave_odom_m;
    result->final_heading_rad = yaw;
    abortActiveWithResult(result);
    return zeroIntent();
  }
  // Something in view: stop sinking while the lock is confirmed.
  const double vz = (pipe || light) ? 0.0 : -camera_rate_mps_;
  const double yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);
  const double surge = (pipe || light || !heading_valid) ? 0.0 : camera_surge_mps_;
  const auto local = bodyFluToLocalEnu(MotionIntent::bodyFlu(surge, 0.0, vz, yaw_rate), yaw);
  return local.value_or(MotionIntent::localEnu(0.0, 0.0, vz, yaw_rate));
}

MotionIntent DiveToDepthServer::tick(
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
  if (telemetry.depth_free) {
    return cameraDescentTick(telemetry, now, dt, *goal);
  }
  if (!telemetry.depth_valid || !telemetry.pose_valid) {
    return zeroIntent();
  }

  const double depth_err = goal->target_depth_m - telemetry.depth_m;
  const double heading_err = normalizeAngle(goal->target_heading_rad - telemetry.yaw);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_depth_m = telemetry.depth_m;
  feedback->current_heading_rad = telemetry.yaw;
  feedback->depth_error_m = depth_err;
  feedback->heading_error_rad = heading_err;
  active_handle_->publish_feedback(feedback);

  if (std::abs(depth_err) <= goal->depth_tolerance_m &&
    std::abs(heading_err) <= goal->heading_tolerance_rad)
  {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Target depth and heading reached";
    result->final_depth_m = telemetry.depth_m;
    result->final_heading_rad = telemetry.yaw;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  const double vz = -std::clamp(depth_kp_ * depth_err, -max_depth_rate_, max_depth_rate_);
  const double yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);
  return MotionIntent::localEnu(0.0, 0.0, vz, yaw_rate);
}

}  // namespace banyu_vehicle_execution
