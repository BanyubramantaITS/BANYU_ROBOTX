#include "banyu_vehicle_execution/action_servers/por_action_servers.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <utility>

namespace banyu_vehicle_execution
{
namespace
{
constexpr double kDetectionTimeout = 0.5;

double positiveOr(double value, double fallback)
{
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

double clampFinite(double value, double low, double high)
{
  return std::isfinite(value) ? std::clamp(value, low, high) : 0.0;
}

MotionIntent bodyCommand(
  const VehicleTelemetry & telemetry, double x, double y, double z,
  double yaw_rate = 0.0)
{
  // Body-frame commands only need a heading: prefer the local pose yaw and fall
  // back to IMU yaw when no position estimate exists (real vehicle without DVL).
  double heading = 0.0;
  if (telemetry.pose_valid) {
    heading = telemetry.yaw;
  } else if (telemetry.imu_valid) {
    heading = telemetry.imu_yaw;
  } else {
    return zeroIntent();
  }
  const auto converted = bodyFluToLocalEnu(MotionIntent::bodyFlu(x, y, z, yaw_rate), heading);
  return converted ? *converted : zeroIntent();
}

double travelledFrom(double x, double y, double z, double sx, double sy, double sz)
{
  return std::hypot(std::hypot(x - sx, y - sy), z - sz);
}

double detectionAgeOrInf(const VehicleTelemetry & telemetry, const rclcpp::Time & now, int64_t id)
{
  return detectionIsFresh(telemetry, now, kDetectionTimeout, id) ?
         std::max(0.0, detectionAge(telemetry, now, id)) :
         std::numeric_limits<double>::infinity();
}

bool freshDepth(const VehicleTelemetry & telemetry, const rclcpp::Time & now, double & depth)
{
  if (telemetry.depth_valid && std::isfinite(telemetry.depth_m) &&
    telemetry.depth_stamp.nanoseconds() != 0)
  {
    const double age = (now - telemetry.depth_stamp).seconds();
    if (age >= 0.0 && age <= kDetectionTimeout) {
      depth = telemetry.depth_m;
      return true;
    }
  }
  if (telemetry.pose_valid && std::isfinite(telemetry.pos_z) &&
    telemetry.pose_stamp.nanoseconds() != 0)
  {
    const double age = (now - telemetry.pose_stamp).seconds();
    if (age >= 0.0 && age <= kDetectionTimeout) {
      depth = -telemetry.pos_z;
      return true;
    }
  }
  return false;
}

bool freshPose(const VehicleTelemetry & telemetry, const rclcpp::Time & now)
{
  if (!telemetry.pose_valid || telemetry.pose_stamp.nanoseconds() == 0 ||
    !std::isfinite(telemetry.pos_x) || !std::isfinite(telemetry.pos_y) ||
    !std::isfinite(telemetry.pos_z) || !std::isfinite(telemetry.yaw))
  {
    return false;
  }
  const double age = (now - telemetry.pose_stamp).seconds();
  return age >= 0.0 && age <= kDetectionTimeout;
}

bool freshHeading(const VehicleTelemetry & telemetry, const rclcpp::Time & now, double * yaw)
{
  if (freshPose(telemetry, now)) {
    if (yaw != nullptr) {*yaw = telemetry.yaw;}
    return true;
  }
  if (!telemetry.imu_valid || telemetry.imu_stamp.nanoseconds() == 0 ||
    !std::isfinite(telemetry.imu_yaw))
  {
    return false;
  }
  const double age = (now - telemetry.imu_stamp).seconds();
  if (age < 0.0 || age > kDetectionTimeout) {return false;}
  if (yaw != nullptr) {*yaw = telemetry.imu_yaw;}
  return true;
}

bool nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool validValues(
  std::string & reason, std::initializer_list<std::pair<const char *, double>> values)
{
  for (const auto & value : values) {
    if (!nonnegative(value.second)) {
      reason = std::string(value.first) + " must be finite and non-negative";
      return false;
    }
  }
  return true;
}

}

AcquireGateServer::AcquireGateServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
SearchMarkerServer::SearchMarkerServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
SearchGateServer::SearchGateServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
AlignGateServer::AlignGateServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
AlignMarkerServer::AlignMarkerServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
PassGateServer::PassGateServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
OrbitMarkerServer::OrbitMarkerServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
TurnAroundServer::TurnAroundServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}
TimedMoveServer::TimedMoveServer(
  rclcpp::Node * node, const std::string & name,
  ActionServerCoordinator * coordinator)
: TypedActionServer(node, name, coordinator) {}

bool AcquireGateServer::validateGoal(const Goal & goal, std::string & reason)
{
  return validValues(
    reason, {
      {"target_depth_m", goal.target_depth_m},
      {"depth_tolerance_m", goal.depth_tolerance_m}, {"timeout_s", goal.timeout_s}});
}

bool SearchMarkerServer::validateGoal(const Goal & goal, std::string & reason)
{
  return validValues(
    reason, {
      {"search_speed_mps", goal.search_speed_mps}, {"timeout_s", goal.timeout_s}});
}

bool SearchGateServer::validateGoal(const Goal & goal, std::string & reason)
{
  return validValues(
    reason, {
      {"search_speed_mps", goal.search_speed_mps}, {"timeout_s", goal.timeout_s}});
}

bool AlignGateServer::validateGoal(const Goal & goal, std::string & reason)
{
  return validValues(
    reason, {
      {"deadband_x", goal.deadband_x}, {"deadband_y", goal.deadband_y},
      {"gain_x", goal.gain_x}, {"gain_y", goal.gain_y},
      {"max_speed_y", goal.max_speed_y}, {"max_speed_z", goal.max_speed_z},
      {"hold_time_s", goal.hold_time_s}, {"timeout_s", goal.timeout_s}});
}

bool AlignMarkerServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (!validValues(
      reason, {
      {"deadband_x", goal.deadband_x}, {"deadband_y", goal.deadband_y},
      {"gain_x", goal.gain_x}, {"gain_y", goal.gain_y},
      {"max_speed_y", goal.max_speed_y}, {"max_speed_z", goal.max_speed_z},
      {"minimum_distance_m", goal.minimum_distance_m},
      {"standoff_min_m", goal.standoff_min_m}, {"standoff_max_m", goal.standoff_max_m},
      {"hold_time_s", goal.hold_time_s}, {"timeout_s", goal.timeout_s}}))
  {
    return false;
  }
  const double min_range = positiveOr(goal.standoff_min_m, 2.2);
  const double max_range = positiveOr(goal.standoff_max_m, 2.5);
  if (max_range < min_range) {
    reason = "standoff_max_m must be at least standoff_min_m";
    return false;
  }
  if (positiveOr(goal.minimum_distance_m, 1.8) > min_range) {
    reason = "minimum_distance_m must not exceed standoff_min_m";
    return false;
  }
  return true;
}

bool PassGateServer::validateGoal(const Goal & goal, std::string & reason)
{
  return validValues(
    reason, {
      {"speed_mps", goal.speed_mps}, {"target_distance_m", goal.target_distance_m},
      {"minimum_distance_m", goal.minimum_distance_m},
      {"blind_duration_s", goal.blind_duration_s}, {"timeout_s", goal.timeout_s}});
}

bool OrbitMarkerServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (!validValues(
      reason, {
      {"orbit_speed_mps", goal.orbit_speed_mps}, {"radius_m", goal.radius_m},
      {"range_gain", goal.range_gain}, {"max_range_speed_mps", goal.max_range_speed_mps},
      {"yaw_gain", goal.yaw_gain}, {"max_yaw_rate_rad_s", goal.max_yaw_rate_rad_s},
      {"vertical_gain", goal.vertical_gain},
      {"max_vertical_speed_mps", goal.max_vertical_speed_mps},
      {"angle_rad", goal.angle_rad}, {"timeout_s", goal.timeout_s}}))
  {
    return false;
  }
  if (goal.radius_m < kPorSafeOrbitRadiusM) {
    reason = "radius_m must be at least the 2.2 m safety floor";
    return false;
  }
  return true;
}

bool TurnAroundServer::validateGoal(const Goal & goal, std::string & reason)
{
  return validValues(
    reason, {
      {"turn_yaw_rate_rad_s", goal.turn_yaw_rate_rad_s},
      {"capture_gain", goal.capture_gain}, {"capture_deadband", goal.capture_deadband},
      {"max_angle_rad", goal.max_angle_rad}, {"timeout_s", goal.timeout_s}});
}

void AcquireGateServer::onGoalStarted()
{
  watchdog_.reset(node_->now());
}

MotionIntent AcquireGateServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  const double target = positiveOr(goal->target_depth_m, 2.5);
  const double tolerance = positiveOr(goal->depth_tolerance_m, 0.15);
  const bool gate = detectionIsFresh(telemetry, now, kDetectionTimeout, 0);
  double depth = -1.0;
  const bool depth_fresh = freshDepth(telemetry, now, depth);
  const bool pose_fresh = freshHeading(telemetry, now, nullptr);
  if (pose_fresh && depth_fresh && gate) {watchdog_.reset(now);}

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->depth_m = depth;
  feedback->gate_visible = gate;
  feedback->detection_age_s = detectionAgeOrInf(telemetry, now, 0);
  active_handle_->publish_feedback(feedback);

  if (pose_fresh && depth_fresh && std::abs(depth - target) <= tolerance && gate) {
    auto result = std::make_shared<Result>();
    result->success = true; result->reason = "Gate acquired at target depth";
    result->final_depth_m = depth; result->gate_visible = true;
    succeedActive(result);
    return zeroIntent();
  }
  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>();
    result->success = false; result->reason = "Gate acquisition timed out";
    result->final_depth_m = depth; result->gate_visible = gate;
    abortActiveWithResult(result);
    return zeroIntent();
  }
  if (!pose_fresh || !depth_fresh) {return zeroIntent();}
  const double error = target - depth;
  const double vertical = -clampFinite(0.7 * error, -0.4, 0.4);
  return bodyCommand(telemetry, 0.0, 0.0, std::abs(error) <= tolerance ? 0.0 : vertical);
}

void SearchMarkerServer::onGoalStarted()
{
  watchdog_.reset(node_->now());
}

MotionIntent SearchMarkerServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  const double speed = positiveOr(goal->search_speed_mps, 0.4);
  banyu_msgs::msg::Detection detection;
  const bool visible = detectionIsFresh(telemetry, now, kDetectionTimeout, 1, &detection);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now; feedback->marker_visible = visible;
  feedback->distance_m = visible ? detection.distance : 0.0;
  feedback->detection_age_s = detectionAgeOrInf(telemetry, now, 1);
  active_handle_->publish_feedback(feedback);

  if (visible) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Marker detected";
    result->final_distance_m = std::isfinite(detection.distance) ? detection.distance : 0.0;
    succeedActive(result);
    return zeroIntent();
  }

  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>();
    result->success = false;
    result->reason = "Marker search timed out without detection";
    result->final_distance_m = 0.0;
    abortActiveWithResult(result);
    return zeroIntent();
  }
  return bodyCommand(telemetry, speed, 0.0, 0.0);
}

void SearchGateServer::onGoalStarted()
{
  watchdog_.reset(node_->now());
}

MotionIntent SearchGateServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  const bool visible = detectionIsFresh(telemetry, now, kDetectionTimeout, 0);
  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now; feedback->gate_visible = visible;
  feedback->detection_age_s = detectionAgeOrInf(telemetry, now, 0);
  active_handle_->publish_feedback(feedback);
  if (visible) {
    auto result = std::make_shared<Result>();
    result->success = true; result->reason = "Gate found"; result->gate_found = true;
    succeedActive(result); return zeroIntent();
  }
  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>();
    result->success = false; result->reason = "Gate search timed out"; result->gate_found = false;
    abortActiveWithResult(result); return zeroIntent();
  }
  return bodyCommand(telemetry, positiveOr(goal->search_speed_mps, 0.4), 0.0, 0.0);
}

void AlignGateServer::onGoalStarted()
{
  watchdog_.reset(node_->now()); aligned_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

MotionIntent AlignGateServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  banyu_msgs::msg::Detection detection;
  const bool visible = detectionIsFresh(telemetry, now, kDetectionTimeout, 0, &detection);
  const double dx = visible ? detection.offset_x : 0.0;
  const double dy = visible ? detection.offset_y : 0.0;
  const bool requirement_valid = freshHeading(telemetry, now, nullptr) && visible &&
    std::isfinite(dx) && std::isfinite(dy);
  if (requirement_valid) {watchdog_.reset(now);}
  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now; feedback->gate_visible = visible;
  feedback->offset_x = dx; feedback->offset_y = dy;
  feedback->detection_age_s = detectionAgeOrInf(telemetry, now, 0);
  active_handle_->publish_feedback(feedback);
  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>(); result->success = false;
    result->reason = "Gate alignment timed out"; result->final_offset_x = dx;
    result->final_offset_y = dy; abortActiveWithResult(result); return zeroIntent();
  }
  const bool aligned = requirement_valid &&
    std::abs(dx) <= positiveOr(goal->deadband_x, 0.03) &&
    std::abs(dy) <= positiveOr(goal->deadband_y, 0.03);
  if (aligned) {
    if (aligned_since_.nanoseconds() == 0) {aligned_since_ = now;}
    if ((now - aligned_since_).seconds() >= positiveOr(goal->hold_time_s, 0.5)) {
      auto result = std::make_shared<Result>(); result->success = true;
      result->reason = "Gate centered"; result->final_offset_x = dx;
      result->final_offset_y = dy; succeedActive(result);
      return zeroIntent();
    }
  } else {
    aligned_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  if (!requirement_valid) {return zeroIntent();}
  const auto correction = visualCorrection(
    dx, dy, positiveOr(goal->gain_x, 1.2), positiveOr(goal->gain_y, 0.8),
    positiveOr(goal->max_speed_y, 0.3), positiveOr(goal->max_speed_z, 0.2));
  return bodyCommand(telemetry, 0.0, correction.lateral_flu_mps, correction.vertical_flu_mps);
}

void AlignMarkerServer::onGoalStarted()
{
  watchdog_.reset(node_->now());
  aligned_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

MotionIntent AlignMarkerServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  banyu_msgs::msg::Detection detection;
  const bool visible = detectionIsFresh(telemetry, now, kDetectionTimeout, 1, &detection);
  const double dx = visible ? detection.offset_x : 0.0;
  const double dy = visible ? detection.offset_y : 0.0;
  const double distance = visible ? detection.distance : 0.0;
  const bool offsets_valid = visible && std::isfinite(dx) && std::isfinite(dy);
  const bool pose_fresh = freshHeading(telemetry, now, nullptr);
  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now; feedback->marker_visible = visible;
  feedback->offset_x = dx; feedback->offset_y = dy; feedback->distance_m = distance;
  feedback->detection_age_s = detectionAgeOrInf(telemetry, now, 1);
  active_handle_->publish_feedback(feedback);
  const double min_range = positiveOr(goal->standoff_min_m, 2.2);
  const double max_range = positiveOr(goal->standoff_max_m, 2.5);
  const double minimum_distance = positiveOr(goal->minimum_distance_m, 1.8);
  const bool lateral_only = goal->lateral_only;
  const bool range_measured = offsets_valid && std::isfinite(distance) && distance > 0.0;
  // Lateral-only mode needs just the bearing: monocular pole range is too noisy
  // to hold a standoff, so the timed legs keep owning the range.
  const bool range_valid = pose_fresh && (lateral_only ? offsets_valid : range_measured);
  if (range_valid) {watchdog_.reset(now);}
  if (pose_fresh && range_measured && distance < minimum_distance) {
    auto result = std::make_shared<Result>();
    result->success = false;
    result->reason = "Marker range is below safety minimum";
    result->final_offset_x = dx;
    result->final_offset_y = dy;
    result->final_distance_m = distance;
    abortActiveWithResult(result);
    return zeroIntent();
  }
  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>(); result->success = false;
    result->reason = "Marker alignment stalled"; result->final_distance_m = distance;
    abortActiveWithResult(result); return zeroIntent();
  }
  const bool centered = std::abs(dx) <= positiveOr(goal->deadband_x, 0.03) &&
    (lateral_only || std::abs(dy) <= positiveOr(goal->deadband_y, 0.03));
  const bool in_range = lateral_only || (distance >= min_range && distance <= max_range);
  if (range_valid && centered && in_range) {
    if (aligned_since_.nanoseconds() == 0) {aligned_since_ = now;}
    if ((now - aligned_since_).seconds() >= positiveOr(goal->hold_time_s, 0.5)) {
      auto result = std::make_shared<Result>(); result->success = true;
      result->reason = "Marker centered at safe standoff";
      result->final_offset_x = dx; result->final_offset_y = dy;
      result->final_distance_m = distance; succeedActive(result);
      return zeroIntent();
    }
  } else {
    aligned_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  if (!range_valid) {return zeroIntent();}
  const double midpoint = 0.5 * (min_range + max_range);
  const double x = lateral_only ? 0.0 : clampFinite(0.35 * (distance - midpoint), -0.25, 0.25);
  const auto correction = visualCorrection(
    dx, dy, positiveOr(goal->gain_x, 1.2), positiveOr(goal->gain_y, 0.8),
    positiveOr(goal->max_speed_y, 0.3), positiveOr(goal->max_speed_z, 0.2));
  return bodyCommand(
    telemetry, x, correction.lateral_flu_mps, lateral_only ? 0.0 : correction.vertical_flu_mps);
}

void PassGateServer::onGoalStarted()
{
  watchdog_.reset(node_->now());
  min_distance_ = std::numeric_limits<double>::infinity();
  blind_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  blind_mode_ = false;
  gate_seen_ = false;
  start_pose_set_ = false;
  start_x_ = start_y_ = start_z_ = 0.0;
}

MotionIntent PassGateServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  // Travelled distance needs a position; steering and the bounded blind pass
  // only need a heading, so the pass also works on IMU yaw alone.
  const bool position_fresh = freshPose(telemetry, now);
  const bool pose_fresh = freshHeading(telemetry, now, nullptr);
  if (position_fresh && !start_pose_set_) {
    start_x_ = telemetry.pos_x; start_y_ = telemetry.pos_y; start_z_ = telemetry.pos_z;
    start_pose_set_ = true;
  }
  const double travelled = travelledFrom(
    telemetry.pos_x, telemetry.pos_y, telemetry.pos_z,
    start_x_, start_y_, start_z_);
  banyu_msgs::msg::Detection detection;
  const bool visible = detectionIsFresh(telemetry, now, kDetectionTimeout, 0, &detection);
  if (visible && std::isfinite(detection.distance) && detection.distance > 0.0) {
    min_distance_ = std::min(min_distance_, static_cast<double>(detection.distance));
  }
  if (!gate_seen_ && pose_fresh && visible) {gate_seen_ = true;}
  if (pose_fresh && gate_seen_) {watchdog_.reset(now);}
  if (blind_mode_ && visible) {
    blind_mode_ = false;
  }
  if (!blind_mode_ && gate_seen_ && !visible) {
    blind_mode_ = true;
    blind_start_time_ = now;
    RCLCPP_INFO(
      node_->get_logger(),
      "Executing: PassGate Detection Lost, Continue pushing through");
  }
  if (blind_mode_ && !pose_fresh) {
    auto result = std::make_shared<Result>(); result->success = false;
    result->reason = "Pose lost during blind gate pass"; result->travelled_m = travelled;
    result->minimum_observed_distance_m = std::isfinite(min_distance_) ? min_distance_ : 0.0;
    abortActiveWithResult(result); return zeroIntent();
  }
  const double target = positiveOr(goal->target_distance_m, 3.0);
  if (position_fresh && start_pose_set_ && travelled >= target) {
    auto result = std::make_shared<Result>(); result->success = true;
    result->reason = "Gate pass distance reached"; result->travelled_m = travelled;
    result->minimum_observed_distance_m = std::isfinite(min_distance_) ? min_distance_ : 0.0;
    succeedActive(result); return zeroIntent();
  }
  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>(); result->success = false;
    result->reason = "Gate pass timed out"; result->travelled_m = travelled;
    result->minimum_observed_distance_m = std::isfinite(min_distance_) ? min_distance_ : 0.0;
    abortActiveWithResult(result); return zeroIntent();
  }
  if (blind_mode_ && pose_fresh &&
    (now - blind_start_time_).seconds() >= goal->blind_duration_s)
  {
    auto result = std::make_shared<Result>(); result->success = true;
    result->reason = "Gate passed after bounded blind pass"; result->travelled_m = travelled;
    result->minimum_observed_distance_m = std::isfinite(min_distance_) ? min_distance_ : 0.0;
    succeedActive(result); return zeroIntent();
  }
  auto feedback = std::make_shared<Feedback>(); feedback->stamp = now;
  feedback->gate_visible = visible; feedback->distance_m = visible ? detection.distance : 0.0;
  feedback->travelled_m = travelled;
  active_handle_->publish_feedback(feedback);
  if (blind_mode_) {
    return pose_fresh ?
           bodyCommand(telemetry, positiveOr(goal->speed_mps, 0.4), 0.0, 0.0) : zeroIntent();
  }
  if (!pose_fresh || !visible) {return zeroIntent();}
  const auto correction = visualCorrection(
    detection.offset_x, detection.offset_y, 0.6, 0.4, 0.3, 0.2);
  return bodyCommand(
    telemetry, positiveOr(goal->speed_mps, 0.4), correction.lateral_flu_mps,
    correction.vertical_flu_mps);
}

void OrbitMarkerServer::onGoalStarted()
{
  watchdog_.reset(node_->now());
  accumulated_yaw_ = 0.0; last_yaw_ = 0.0;
  yaw_initialized_ = false;
  last_distance_ = 0.0;
}

MotionIntent OrbitMarkerServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  double heading = 0.0;
  const bool pose_fresh = freshHeading(telemetry, now, &heading);
  if (pose_fresh && !yaw_initialized_) {
    last_yaw_ = heading; yaw_initialized_ = true;
  } else if (pose_fresh) {
    accumulated_yaw_ += normalizeAngle(heading - last_yaw_); last_yaw_ = heading;
  }
  banyu_msgs::msg::Detection detection;
  const bool visible = detectionIsFresh(telemetry, now, kDetectionTimeout, 1, &detection);
  const bool marker_valid = visible && std::isfinite(detection.offset_x) &&
    std::isfinite(detection.offset_y) && std::isfinite(detection.distance) &&
    detection.distance > 0.0;
  if (pose_fresh && marker_valid) {watchdog_.reset(now);}
  const double angle = std::max(2.0 * M_PI, positiveOr(goal->angle_rad, 2.0 * M_PI));
  if (pose_fresh && marker_valid && std::abs(accumulated_yaw_) >= angle) {
    auto result = std::make_shared<Result>(); result->success = true;
    result->reason = "Marker orbit complete"; result->accumulated_yaw_rad = accumulated_yaw_;
    result->final_distance_m = last_distance_;
    succeedActive(result); return zeroIntent();
  }
  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>(); result->success = false;
    result->reason = "Marker orbit timed out"; result->accumulated_yaw_rad = accumulated_yaw_;
    abortActiveWithResult(result); return zeroIntent();
  }
  auto feedback = std::make_shared<Feedback>(); feedback->stamp = now;
  feedback->marker_visible = visible; feedback->accumulated_yaw_rad = accumulated_yaw_;
  feedback->detection_age_s = detectionAgeOrInf(telemetry, now, 1);
  feedback->distance_m = visible ? detection.distance : 0.0; active_handle_->publish_feedback(
    feedback);
  if (!pose_fresh || !marker_valid) {
    return zeroIntent();
  }
  last_distance_ = detection.distance;
  const double radius = positiveOr(goal->radius_m, 3.0);
  const double x = clampFinite(
    positiveOr(goal->range_gain, 0.5) *
    (static_cast<double>(detection.distance) - radius),
    -positiveOr(goal->max_range_speed_mps, 0.3), positiveOr(goal->max_range_speed_mps, 0.3));
  const double y = positiveOr(goal->orbit_speed_mps, 0.3);
  const auto correction = visualCorrection(
    detection.offset_x, detection.offset_y, 0.0, positiveOr(goal->vertical_gain, 0.4),
    0.0, positiveOr(goal->max_vertical_speed_mps, 0.2),
    positiveOr(goal->yaw_gain, 0.6), positiveOr(goal->max_yaw_rate_rad_s, 0.6));
  const double feedforward_yaw_rate = -y / std::max(radius, 0.5);
  const double max_yaw_rate = positiveOr(goal->max_yaw_rate_rad_s, 0.6);
  const double commanded_yaw_rate = clampFinite(
    feedforward_yaw_rate + correction.yaw_rate_rad_s, -max_yaw_rate, max_yaw_rate);
  return bodyCommand(telemetry, x, y, correction.vertical_flu_mps, commanded_yaw_rate);
}

void TurnAroundServer::onGoalStarted()
{
  watchdog_.reset(node_->now());
  accumulated_yaw_ = 0.0; last_yaw_ = 0.0;
  yaw_initialized_ = false;
}

MotionIntent TurnAroundServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double /*dt*/)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  double heading = 0.0;
  const bool pose_fresh = freshHeading(telemetry, now, &heading);
  if (pose_fresh) {watchdog_.reset(now);}
  if (pose_fresh && !yaw_initialized_) {
    last_yaw_ = heading; yaw_initialized_ = true;
  } else if (pose_fresh) {
    accumulated_yaw_ += normalizeAngle(heading - last_yaw_); last_yaw_ = heading;
  }
  banyu_msgs::msg::Detection detection;
  const bool visible = detectionIsFresh(telemetry, now, kDetectionTimeout, 0, &detection);
  const double max_angle = std::min(positiveOr(goal->max_angle_rad, M_PI), M_PI);
  auto feedback = std::make_shared<Feedback>(); feedback->stamp = now;
  feedback->gate_visible = visible; feedback->offset_x = visible ? detection.offset_x : 0.0;
  feedback->accumulated_yaw_rad = accumulated_yaw_; active_handle_->publish_feedback(feedback);
  const bool gate_valid = visible && std::isfinite(detection.offset_x);
  if (pose_fresh && gate_valid &&
    std::abs(detection.offset_x) <= positiveOr(goal->capture_deadband, 0.2))
  {
    auto result = std::make_shared<Result>(); result->success = true;
    result->reason = "Gate visually captured"; result->accumulated_yaw_rad = accumulated_yaw_;
    result->gate_captured = true; succeedActive(result); return zeroIntent();
  }
  if (pose_fresh && std::abs(accumulated_yaw_) >= max_angle) {
    auto result = std::make_shared<Result>(); result->success = true;
    result->reason = "Turn angle cap reached"; result->accumulated_yaw_rad = accumulated_yaw_;
    result->gate_captured = false; succeedActive(result); return zeroIntent();
  }
  if (watchdog_.expired(now, goal->timeout_s)) {
    auto result = std::make_shared<Result>(); result->success = false;
    result->reason = "Turn timed out"; result->accumulated_yaw_rad = accumulated_yaw_;
    result->gate_captured = false; abortActiveWithResult(result); return zeroIntent();
  }
  const auto correction = visualCorrection(
    gate_valid ? detection.offset_x : 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    positiveOr(goal->capture_gain, 0.5), positiveOr(goal->turn_yaw_rate_rad_s, 0.3));
  if (!pose_fresh) {return zeroIntent();}
  const double rate = gate_valid ? correction.yaw_rate_rad_s :
    positiveOr(goal->turn_yaw_rate_rad_s, 0.3);
  return bodyCommand(telemetry, 0.0, 0.0, 0.0, rate);
}

bool TimedMoveServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (!nonnegative(goal.duration_s)) {
    reason = "duration_s must be finite and non-negative";
    return false;
  }
  if (!std::isfinite(goal.speed_x_mps) || !std::isfinite(goal.speed_y_mps) ||
    !std::isfinite(goal.speed_z_mps) || !std::isfinite(goal.yaw_rate_rad_s))
  {
    reason = "speeds and yaw_rate_rad_s must be finite";
    return false;
  }
  if (goal.use_baro_depth &&
    (!nonnegative(goal.target_depth_m) || !nonnegative(goal.depth_kp) ||
    !nonnegative(goal.max_depth_speed_mps)))
  {
    reason = "target_depth_m, depth_kp, and max_depth_speed_mps must be finite and non-negative";
    return false;
  }
  if (goal.use_imu_yaw_cap && !nonnegative(goal.target_yaw_delta_rad)) {
    reason = "target_yaw_delta_rad must be finite and non-negative";
    return false;
  }
  return true;
}

void TimedMoveServer::onGoalStarted()
{
  start_time_ = node_->now();
  last_yaw_ = 0.0;
  accumulated_yaw_ = 0.0;
  yaw_initialized_ = false;
  if (active_handle_) {
    const auto goal = active_handle_->get_goal();
    RCLCPP_INFO(
      node_->get_logger(),
      "[TimedMove] Starting '%s' (duration=%.2fs, vx=%.2f, vy=%.2f, vz=%.2f, wz=%.2f, "
      "baro_depth=%s target_depth=%.2fm)",
      goal->segment_name.c_str(), goal->duration_s,
      goal->speed_x_mps, goal->speed_y_mps, goal->speed_z_mps, goal->yaw_rate_rad_s,
      goal->use_baro_depth ? "true" : "false", goal->target_depth_m);
  }
}

MotionIntent TimedMoveServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now,
  double dt)
{
  if (!active_handle_) {return zeroIntent();}
  if (active_handle_->is_canceling()) {cancelActive(); return zeroIntent();}
  const auto goal = active_handle_->get_goal();
  const double elapsed_s = std::max(0.0, (now - start_time_).seconds());
  const bool pose_fresh = freshPose(telemetry, now);
  if (pose_fresh && !yaw_initialized_) {
    last_yaw_ = telemetry.yaw;
    dead_reckon_yaw_ = telemetry.yaw;
    yaw_initialized_ = true;
  } else if (pose_fresh) {
    accumulated_yaw_ += normalizeAngle(telemetry.yaw - last_yaw_);
    last_yaw_ = telemetry.yaw;
    dead_reckon_yaw_ = telemetry.yaw;
  } else if (std::isfinite(dt) && dt > 0.0) {
    const double delta_yaw = goal->yaw_rate_rad_s * dt;
    accumulated_yaw_ += delta_yaw;
    dead_reckon_yaw_ = normalizeAngle(dead_reckon_yaw_ + delta_yaw);
  }
  double depth = -1.0;
  const bool depth_fresh = freshDepth(telemetry, now, depth);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->elapsed_s = elapsed_s;
  feedback->depth_m = depth;
  feedback->accumulated_yaw_rad = accumulated_yaw_;
  active_handle_->publish_feedback(feedback);

  if (goal->duration_s <= 0.0 || elapsed_s >= goal->duration_s) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Timed segment completed: " + goal->segment_name;
    result->elapsed_s = elapsed_s;
    result->final_depth_m = depth;
    result->accumulated_yaw_rad = accumulated_yaw_;
    succeedActive(result);
    return zeroIntent();
  }
  if (goal->use_imu_yaw_cap && goal->target_yaw_delta_rad > 0.0 &&
    pose_fresh && std::abs(accumulated_yaw_) >= goal->target_yaw_delta_rad)
  {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Target yaw delta reached: " + goal->segment_name;
    result->elapsed_s = elapsed_s;
    result->final_depth_m = depth;
    result->accumulated_yaw_rad = accumulated_yaw_;
    succeedActive(result);
    return zeroIntent();
  }

  double vz = clampFinite(goal->speed_z_mps, -0.6, 0.6);
  if (goal->use_baro_depth && depth_fresh) {
    const double kp = positiveOr(goal->depth_kp, 0.7);
    const double max_vz = positiveOr(goal->max_depth_speed_mps, 0.4);
    const double error = goal->target_depth_m - depth;
    vz = -clampFinite(kp * error, -max_vz, max_vz);
  }
  const double yaw = (pose_fresh && std::isfinite(telemetry.yaw)) ?
    telemetry.yaw : dead_reckon_yaw_;
  const auto converted = bodyFluToLocalEnu(
    MotionIntent::bodyFlu(goal->speed_x_mps, goal->speed_y_mps, vz, goal->yaw_rate_rad_s), yaw);
  return converted ? *converted : zeroIntent();
}

}  // namespace banyu_vehicle_execution
