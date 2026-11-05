#include "banyu_vehicle_execution/action_servers/navigate_surface_gps_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace banyu_vehicle_execution
{

std::pair<double, double> keepOutAim(
  std::pair<double, double> goal, const std::vector<std::pair<double, double>> & obstacles, double radius)
{
  if (!(radius > 0.0)) {return goal;}
  const double detour = 1.5 * radius;
  for (const auto & [ox, oy] : obstacles) {
    const double d = std::hypot(ox, oy);
    if (d < radius && d > 1e-6) {return {ox - ox / d * detour, oy - oy / d * detour};}
  }
  auto aim = goal;
  for (int pass = 0; pass < 3; ++pass) {
    const double len = std::hypot(aim.first, aim.second);
    if (len < 1e-6) {break;}
    const double ux = aim.first / len;
    const double uy = aim.second / len;
    double best_t = 2.0;
    const std::pair<double, double> * blocking = nullptr;
    for (const auto & o : obstacles) {
      const double t = (o.first * ux + o.second * uy) / len;
      const double cx = t * aim.first - o.first;
      const double cy = t * aim.second - o.second;
      if (t > 0.0 && t < 1.0 && std::hypot(cx, cy) < radius && t < best_t) {
        best_t = t;
        blocking = &o;
      }
    }
    if (!blocking) {break;}
    // Pass on the side the line is already on (left when it runs straight through).
    const double cx = best_t * aim.first - blocking->first;
    const double cy = best_t * aim.second - blocking->second;
    double nx = -uy;
    double ny = ux;
    if (std::hypot(cx, cy) > 1e-6 && cx * nx + cy * ny < 0.0) {
      nx = -nx;
      ny = -ny;
    }
    aim = {blocking->first + nx * detour, blocking->second + ny * detour};
  }
  return aim;
}

NavigateSurfaceGpsServer::NavigateSurfaceGpsServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double nav_kp,
  double nav_max_speed,
  double yaw_kp,
  double max_yaw_rate)
: TypedActionServer(node, action_name, coordinator),
  nav_kp_(nav_kp), nav_max_speed_(nav_max_speed),
  yaw_kp_(yaw_kp), max_yaw_rate_(max_yaw_rate)
{
}

bool NavigateSurfaceGpsServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.tolerance_m <= 0.0) {
    reason = "tolerance_m must be positive";
    return false;
  }
  return true;
}

MotionIntent NavigateSurfaceGpsServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double /*dt*/)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    cancelActive();
    return zeroIntent();
  }
  if (!telemetry.gps_valid || !telemetry.pose_valid) {
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  double dx = 0.0;
  double dy = 0.0;
  latLonToEnuOffsets(
    telemetry.latitude_deg, telemetry.longitude_deg,
    goal->target_latitude_deg, goal->target_longitude_deg,
    dx, dy);

  const double distance = std::hypot(dx, dy);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_latitude_deg = telemetry.latitude_deg;
  feedback->current_longitude_deg = telemetry.longitude_deg;
  feedback->distance_remaining_m = distance;
  active_handle_->publish_feedback(feedback);

  if (distance <= goal->tolerance_m) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Target GPS coordinates reached";
    result->final_latitude_deg = telemetry.latitude_deg;
    result->final_longitude_deg = telemetry.longitude_deg;
    result->final_error_m = distance;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  // Keep clear of the buoys on the way (a target inside a keep-out circle drops that circle).
  std::vector<std::pair<double, double>> obstacles;
  const std::size_t count = std::min(goal->keep_out_latitude_deg.size(), goal->keep_out_longitude_deg.size());
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(goal->keep_out_latitude_deg[i]) || !std::isfinite(goal->keep_out_longitude_deg[i])) {continue;}
    double ox = 0.0;
    double oy = 0.0;
    latLonToEnuOffsets(
      telemetry.latitude_deg, telemetry.longitude_deg,
      goal->keep_out_latitude_deg[i], goal->keep_out_longitude_deg[i], ox, oy);
    if (std::hypot(ox - dx, oy - dy) > goal->keep_out_radius_m) {obstacles.emplace_back(ox, oy);}
  }
  const auto [ax, ay] = keepOutAim({dx, dy}, obstacles, goal->keep_out_radius_m);
  const double bearing = std::atan2(ay, ax);
  const double heading_err = normalizeAngle(bearing - telemetry.yaw);
  const double speed = std::clamp(nav_kp_ * distance, 0.0, nav_max_speed_);
  const double yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);

  const double surge = speed * std::cos(heading_err);
  const double sway = speed * std::sin(heading_err);
  const auto local_intent = bodyFluToLocalEnu(
    MotionIntent::bodyFlu(surge, sway, 0.0, yaw_rate), telemetry.yaw);
  return local_intent.value_or(zeroIntent());
}

}  // namespace banyu_vehicle_execution
