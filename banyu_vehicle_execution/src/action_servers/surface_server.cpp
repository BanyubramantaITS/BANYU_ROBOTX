#include "banyu_vehicle_execution/action_servers/surface_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "banyu_vehicle_execution/task4_zones.hpp"

namespace banyu_vehicle_execution
{

SurfaceServer::SurfaceServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double depth_kp,
  double max_depth_rate,
  bool require_calibrated_depth,
  double hold_time_s,
  double depth_timeout_s,
  double max_surface_depth_m)
: TypedActionServer(node, action_name, coordinator),
  depth_kp_(depth_kp), max_depth_rate_(max_depth_rate),
  require_calibrated_depth_(require_calibrated_depth), hold_time_s_(hold_time_s),
  depth_timeout_s_(depth_timeout_s), max_surface_depth_m_(max_surface_depth_m)
{
  if (!std::isfinite(hold_time_s_) || hold_time_s_ <= 0.0 ||
    !std::isfinite(depth_timeout_s_) || depth_timeout_s_ <= 0.0 ||
    !std::isfinite(max_surface_depth_m_) || max_surface_depth_m_ <= 0.0)
  {
    throw std::invalid_argument("Surface hold, depth timeout and maximum depth must be positive");
  }
}

bool SurfaceServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (!std::isfinite(goal.target_depth_m) || !std::isfinite(goal.depth_tolerance_m) ||
    goal.depth_tolerance_m <= 0.0)
  {
    reason = "target_depth_m must be finite and depth_tolerance_m positive and finite";
    return false;
  }
  if (require_calibrated_depth_ && std::abs(goal.target_depth_m) > 1.0e-6) {
    reason = "Calibrated Surface target must be the water surface (0 m)";
    return false;
  }
  if (!std::isfinite(goal.watchdog_timeout_s) || goal.watchdog_timeout_s < 0.0) {
    reason = "watchdog_timeout_s must be finite and non-negative";
    return false;
  }
  return true;
}

void SurfaceServer::setBlindRise(double rate_mps, double timeout_s)
{
  if (!std::isfinite(rate_mps) || rate_mps < 0.0 || !std::isfinite(timeout_s) || timeout_s < 0.0) {
    throw std::invalid_argument("Surface blind rise rate and timeout must not be negative");
  }
  blind_rise_rate_ = rate_mps;
  blind_timeout_s_ = timeout_s;
}

void SurfaceServer::onGoalStarted()
{
  blind_s_ = 0.0;
  last_valid_depth_time_ = std::chrono::steady_clock::now();
  hold_active_ = false;
  zone_hold_s_ = 0.0;
}

MotionIntent SurfaceServer::tick(
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
  // Surfacing point (the last fix) inside a Task 4 zone: stay at depth until it clears, within the time limit. With a GPS
  // fix the vehicle is already on the surface and there is nothing to wait for.
  if (goal->hold_for_zones && !telemetry.gps_valid && telemetry.zones.any() &&
    telemetry.gps_stamp.nanoseconds() != 0 && zone_hold_s_ < zone_hold_timeout_s_ &&
    insideAny(Vec2{0.0, 0.0}, telemetry.zonesAroundVehicle(now)))
  {
    zone_hold_s_ += std::max(0.0, dt);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Task 4: surfacing point inside a keep-out zone, holding at depth (%.0f s)", zone_hold_s_);
    return zeroIntent();
  }
  const auto steady_now = std::chrono::steady_clock::now();
  const auto & water = telemetry.water_depth;
  const double depth_m = require_calibrated_depth_ ? water.depth_m : telemetry.depth_m;
  const auto & source_stamp = require_calibrated_depth_ ? water.stamp : telemetry.depth_stamp;
  const bool valid = require_calibrated_depth_ ? water.valid : telemetry.depth_valid;
  const bool receipt_valid = require_calibrated_depth_ ?
    water.receipt_valid : telemetry.depth_receipt_valid;
  const auto & receipt = require_calibrated_depth_ ? water.receipt_time :
    telemetry.depth_receipt_time;
  const uint64_t continuity = require_calibrated_depth_ ? water.continuity_generation :
    telemetry.depth_continuity_generation;
  const bool source_fresh = valid && std::isfinite(depth_m) &&
    source_stamp.nanoseconds() != 0 && (now - source_stamp).seconds() >= 0.0 &&
    (now - source_stamp).seconds() <= depth_timeout_s_;
  const double receipt_age = receipt_valid ?
    std::chrono::duration<double>(steady_now - receipt).count() : 0.0;
  const bool receipt_fresh = !receipt_valid ||
    (receipt_age >= 0.0 && receipt_age <= depth_timeout_s_);
  const bool calibrated = water.calibrated && water.receipt_valid && water.run_id != 0 &&
    water.epoch != 0 && water.run_id == telemetry.current_run_id &&
    water.epoch == telemetry.current_run_epoch && water.datum == "water_surface" &&
    !water.source_id.empty() && !water.calibration_id.empty();
  const bool depth_fresh = source_fresh && receipt_fresh &&
    (!require_calibrated_depth_ || calibrated);
  if (!depth_fresh && blind_rise_rate_ > 0.0) {
    hold_active_ = false;
    blind_s_ += std::max(0.0, dt);
    if (blind_s_ >= blind_timeout_s_) {
      auto result = std::make_shared<Result>();
      result->success = false;
      result->reason = "Surface not proven by a GPS fix within the blind rise time";
      result->final_depth_m = depth_m;
      result->final_error_m = goal->target_depth_m - depth_m;
      abortActiveWithResult(result);
      return zeroIntent();
    }
    return MotionIntent::localEnu(0.0, 0.0, blind_rise_rate_, 0.0);
  }
  if (!depth_fresh) {
    hold_active_ = false;
    if (goal->watchdog_timeout_s > 0.0 &&
      std::chrono::duration<double>(steady_now - last_valid_depth_time_).count() >=
      goal->watchdog_timeout_s)
    {
      auto result = std::make_shared<Result>();
      result->success = false;
      result->reason = "Surface water-depth input unavailable or timed out";
      result->final_depth_m = depth_m;
      result->final_error_m = goal->target_depth_m - depth_m;
      abortActiveWithResult(result);
    }
    return zeroIntent();
  }
  last_valid_depth_time_ = steady_now;
  const double depth_err = goal->target_depth_m - depth_m;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_depth_m = depth_m;
  feedback->depth_error_m = depth_err;
  active_handle_->publish_feedback(feedback);

  const double tolerance = require_calibrated_depth_ ?
    std::min(goal->depth_tolerance_m, max_surface_depth_m_) : goal->depth_tolerance_m;
  const bool near_surface = std::abs(depth_err) <= tolerance;
  if (hold_active_ && continuity != hold_depth_generation_) {
    hold_active_ = false;
  }
  if (hold_active_) {
    if (require_calibrated_depth_) {
      for (const auto & sample : water.samples_since_last_tick) {
        if (!sample.valid || std::abs(goal->target_depth_m - sample.depth_m) > tolerance) {
          hold_active_ = false;
          break;
        }
      }
    } else {
      for (double intermediate_depth : telemetry.depth_samples_since_last_tick) {
        if (std::abs(goal->target_depth_m - intermediate_depth) > tolerance) {
          hold_active_ = false;
          break;
        }
      }
    }
  }
  if (!near_surface) {
    hold_active_ = false;
  } else if (!hold_active_) {
    hold_active_ = true;
    hold_start_time_ = steady_now;
    hold_depth_generation_ = continuity;
    hold_first_source_stamp_ns_ = source_stamp.nanoseconds();
  }
  if (near_surface && hold_active_ &&
    (!require_calibrated_depth_ ||
    source_stamp.nanoseconds() > hold_first_source_stamp_ns_) &&
    std::chrono::duration<double>(steady_now - hold_start_time_).count() >= hold_time_s_)
  {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Surface depth verified continuously";
    result->final_depth_m = depth_m;
    result->final_error_m = depth_err;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  const double vz = -std::clamp(depth_kp_ * depth_err, -max_depth_rate_, max_depth_rate_);
  return MotionIntent::localEnu(0.0, 0.0, vz, 0.0);
}

}  // namespace banyu_vehicle_execution
