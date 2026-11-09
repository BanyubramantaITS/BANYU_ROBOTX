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

MotionIntent zoneExitIntent(const Exit & exit, double max_speed_mps, double yaw_rate)
{
  const double speed = std::clamp(0.5 + exit.distance_m, 0.2, std::max(0.2, max_speed_mps));
  return MotionIntent::localEnu(exit.direction.x * speed, exit.direction.y * speed, 0.0, yaw_rate);
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

  // Buoys as ENU offsets from the vehicle.
  coordinator_->setBuoyKeepOut(
    goal->keep_out_latitude_deg, goal->keep_out_longitude_deg, goal->keep_out_radius_m);
  std::vector<std::pair<double, double>> buoys;
  std::vector<Vec2> buoy_points;
  const std::size_t count = std::min(goal->keep_out_latitude_deg.size(), goal->keep_out_longitude_deg.size());
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(goal->keep_out_latitude_deg[i]) || !std::isfinite(goal->keep_out_longitude_deg[i])) {continue;}
    double ox = 0.0;
    double oy = 0.0;
    latLonToEnuOffsets(
      telemetry.latitude_deg, telemetry.longitude_deg,
      goal->keep_out_latitude_deg[i], goal->keep_out_longitude_deg[i], ox, oy);
    buoys.emplace_back(ox, oy);
    buoy_points.push_back(Vec2{ox, oy});
  }

  // Task 4 zones: a dive point inside a static zone moves along the buoy line to the nearest point outside; one inside
  // the moving object's projected zone is waited for at its edge.
  const auto zones = telemetry.zonesAroundVehicle(now);
  std::vector<Capsule> static_zones;
  std::vector<Capsule> moving_zones;
  for (const auto & z : zones) {(z.moving ? moving_zones : static_zones).push_back(z);}
  bool waiting_for_moving = false;
  if (!static_zones.empty()) {
    std::optional<std::pair<Vec2, Vec2>> line;
    if (buoy_points.size() >= 2) {line = std::make_pair(buoy_points[0], buoy_points[1]);}
    if (const auto shift = shiftDivePoint(Vec2{dx, dy}, static_zones, buoy_points, zone_buoy_clear_m_, line);
      shift && shift->shifted)
    {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Task 4: dive point inside a keep-out zone, shifted %.1f m %s", std::hypot(shift->point.x - dx, shift->point.y - dy),
        shift->along_line ? "along the buoy line" : "off the buoy line");
      dx = shift->point.x;
      dy = shift->point.y;
    }
  }
  if (insideAny(Vec2{dx, dy}, moving_zones)) {waiting_for_moving = true;}

  // Keep clear of the buoys on the way (a dive point inside a keep-out circle drops that circle unless the vehicle
  // itself is currently inside that buoy's keep-out circle).
  std::vector<std::pair<double, double>> obstacles;
  std::vector<Vec2> obstacle_points;
  bool inside_buoy = false;
  for (const auto & [ox, oy] : buoys) {
    if (goal->keep_out_radius_m > 0.0 && std::hypot(ox, oy) < goal->keep_out_radius_m) {
      inside_buoy = true;
    }
    if (std::hypot(ox - dx, oy - dy) > goal->keep_out_radius_m ||
      (goal->keep_out_radius_m > 0.0 && std::hypot(ox, oy) < goal->keep_out_radius_m))
    {
      obstacles.emplace_back(ox, oy);
      obstacle_points.push_back(Vec2{ox, oy});
    }
  }

  // Inside a zone (published on top of the vehicle, or a moving object arriving): leave along the shortest buoy-clear way
  // first. If buoys block every exit, ignore the zone rather than risk colliding with a buoy.
  if (const auto exit = exitVector(
      Vec2{0.0, 0.0}, zones, telemetry.zones.exit_clearance_m, obstacle_points, goal->keep_out_radius_m))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "Task 4: inside a keep-out zone, leaving %.1f m", exit->distance_m);
    return zoneExitIntent(*exit, telemetry.zones.exit_speed_mps, 0.0);
  }

  const double distance = std::hypot(dx, dy);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->current_latitude_deg = telemetry.latitude_deg;
  feedback->current_longitude_deg = telemetry.longitude_deg;
  feedback->distance_remaining_m = distance;
  active_handle_->publish_feedback(feedback);

  if (distance <= goal->tolerance_m && !waiting_for_moving && !inside_buoy) {
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

  if (waiting_for_moving) {
    // Wait outside the projected zone until the object has passed.
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Task 4: dive point in the moving object's path, waiting");
    const auto edge = truncateAtZone(Vec2{dx, dy}, moving_zones, telemetry.zones.exit_clearance_m);
    dx = edge ? edge->x : 0.0;
    dy = edge ? edge->y : 0.0;
  }

  auto [ax, ay] = keepOutAim({dx, dy}, obstacles, goal->keep_out_radius_m);
  if (!zones.empty() && !inside_buoy) {
    // Around a zone on the way while respecting buoy keep-out; with no clear way (boxed in) hold here rather than cross a buoy.
    const auto aim = detourAim(Vec2{ax, ay}, zones, obstacle_points, goal->keep_out_radius_m);
    ax = aim ? aim->x : 0.0;
    ay = aim ? aim->y : 0.0;
  }
  if (std::hypot(ax, ay) < 1e-3) {return zeroIntent();}
  const double bearing = std::atan2(ay, ax);
  const double heading_err = normalizeAngle(bearing - telemetry.yaw);
  const double speed = std::clamp(nav_kp_ * std::max(distance, std::hypot(ax, ay)), 0.0, nav_max_speed_);
  const double yaw_rate = std::clamp(yaw_kp_ * heading_err, -max_yaw_rate_, max_yaw_rate_);

  const double surge = speed * std::cos(heading_err);
  const double sway = speed * std::sin(heading_err);
  const auto local_intent = bodyFluToLocalEnu(
    MotionIntent::bodyFlu(surge, sway, 0.0, yaw_rate), telemetry.yaw);
  return local_intent.value_or(zeroIntent());
}

}  // namespace banyu_vehicle_execution
