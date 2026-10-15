#include "banyu_mission/por_mission_config.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace banyu_mission
{

PorMissionConfig PorMissionConfig::load(rclcpp::Node & node)
{
  PorMissionConfig config;
  const auto parameter = [&node](const std::string & name, double fallback) {
      return node.declare_parameter<double>("mission." + name, fallback);
    };

  config.gate_depth_m = parameter("gate_depth_m", config.gate_depth_m);
  config.gate_depth_tolerance_m =
    parameter("gate_depth_tolerance_m", config.gate_depth_tolerance_m);
  config.surface_watchdog_timeout_s = parameter(
    "surface_watchdog_timeout_s", config.surface_watchdog_timeout_s);
  config.acquire_gate_timeout_s =
    parameter("acquire_gate_timeout_s", config.acquire_gate_timeout_s);
  config.gate_align_deadband_x = parameter("gate_align_deadband_x", config.gate_align_deadband_x);
  config.gate_align_deadband_y = parameter("gate_align_deadband_y", config.gate_align_deadband_y);
  config.gate_align_gain_x = parameter("gate_align_gain_x", config.gate_align_gain_x);
  config.gate_align_gain_y = parameter("gate_align_gain_y", config.gate_align_gain_y);
  config.gate_align_max_speed_y =
    parameter("gate_align_max_speed_y", config.gate_align_max_speed_y);
  config.gate_align_max_speed_z =
    parameter("gate_align_max_speed_z", config.gate_align_max_speed_z);
  config.gate_align_hold_time_s =
    parameter("gate_align_hold_time_s", config.gate_align_hold_time_s);
  config.gate_align_timeout_s = parameter("gate_align_timeout_s", config.gate_align_timeout_s);
  config.gate_pass_speed_mps = parameter("gate_pass_speed_mps", config.gate_pass_speed_mps);
  config.gate_pass_target_distance_m = parameter(
    "gate_pass_target_distance_m", config.gate_pass_target_distance_m);
  config.gate_pass_minimum_distance_m = parameter(
    "gate_pass_minimum_distance_m", config.gate_pass_minimum_distance_m);
  config.gate_pass_blind_duration_s = parameter(
    "gate_pass_blind_duration_s", config.gate_pass_blind_duration_s);
  config.gate_pass_timeout_s = parameter("gate_pass_timeout_s", config.gate_pass_timeout_s);
  config.marker_search_speed_mps = parameter(
    "marker_search_speed_mps",
    config.marker_search_speed_mps);
  config.marker_search_timeout_s = parameter(
    "marker_search_timeout_s",
    config.marker_search_timeout_s);
  config.marker_align_deadband_x = parameter(
    "marker_align_deadband_x",
    config.marker_align_deadband_x);
  config.marker_align_deadband_y = parameter(
    "marker_align_deadband_y",
    config.marker_align_deadband_y);
  config.marker_align_gain_x = parameter("marker_align_gain_x", config.marker_align_gain_x);
  config.marker_align_gain_y = parameter("marker_align_gain_y", config.marker_align_gain_y);
  config.marker_align_max_speed_y = parameter(
    "marker_align_max_speed_y", config.marker_align_max_speed_y);
  config.marker_align_max_speed_z = parameter(
    "marker_align_max_speed_z", config.marker_align_max_speed_z);
  config.marker_align_minimum_distance_m = parameter(
    "marker_align_minimum_distance_m", config.marker_align_minimum_distance_m);
  config.marker_align_standoff_min_m = parameter(
    "marker_align_standoff_min_m", config.marker_align_standoff_min_m);
  config.marker_align_standoff_max_m = parameter(
    "marker_align_standoff_max_m", config.marker_align_standoff_max_m);
  config.marker_align_hold_time_s = parameter(
    "marker_align_hold_time_s",
    config.marker_align_hold_time_s);
  config.marker_align_timeout_s =
    parameter("marker_align_timeout_s", config.marker_align_timeout_s);
  config.orbit_speed_mps = parameter("orbit_speed_mps", config.orbit_speed_mps);
  config.orbit_radius_m = parameter("orbit_radius_m", config.orbit_radius_m);
  config.orbit_range_gain = parameter("orbit_range_gain", config.orbit_range_gain);
  config.orbit_max_range_speed_mps = parameter(
    "orbit_max_range_speed_mps", config.orbit_max_range_speed_mps);
  config.orbit_yaw_gain = parameter("orbit_yaw_gain", config.orbit_yaw_gain);
  config.orbit_max_yaw_rate_rad_s = parameter(
    "orbit_max_yaw_rate_rad_s", config.orbit_max_yaw_rate_rad_s);
  config.orbit_vertical_gain = parameter("orbit_vertical_gain", config.orbit_vertical_gain);
  config.orbit_max_vertical_speed_mps = parameter(
    "orbit_max_vertical_speed_mps", config.orbit_max_vertical_speed_mps);
  config.orbit_angle_rad = parameter("orbit_angle_rad", config.orbit_angle_rad);
  config.orbit_timeout_s = parameter("orbit_timeout_s", config.orbit_timeout_s);
  config.turn_yaw_rate_rad_s = parameter("turn_yaw_rate_rad_s", config.turn_yaw_rate_rad_s);
  config.turn_capture_gain = parameter("turn_capture_gain", config.turn_capture_gain);
  config.turn_capture_deadband = parameter("turn_capture_deadband", config.turn_capture_deadband);
  config.turn_max_angle_rad = parameter("turn_max_angle_rad", config.turn_max_angle_rad);
  config.turn_timeout_s = parameter("turn_timeout_s", config.turn_timeout_s);
  config.gate_search_speed_mps = parameter("gate_search_speed_mps", config.gate_search_speed_mps);
  config.gate_search_timeout_s = parameter("gate_search_timeout_s", config.gate_search_timeout_s);
  config.return_gate_pass_speed_mps = parameter(
    "return_gate_pass_speed_mps", config.return_gate_pass_speed_mps);
  config.return_gate_pass_target_distance_m = parameter(
    "return_gate_pass_target_distance_m", config.return_gate_pass_target_distance_m);
  config.return_gate_pass_minimum_distance_m = parameter(
    "return_gate_pass_minimum_distance_m", config.return_gate_pass_minimum_distance_m);
  config.return_gate_pass_blind_duration_s = parameter(
    "return_gate_pass_blind_duration_s", config.return_gate_pass_blind_duration_s);
  config.return_gate_pass_timeout_s = parameter(
    "return_gate_pass_timeout_s", config.return_gate_pass_timeout_s);
  return config;
}

void PorMissionConfig::validate() const
{
  const std::array values {
    gate_depth_m, gate_depth_tolerance_m, acquire_gate_timeout_s,
    gate_align_deadband_x, gate_align_deadband_y, gate_align_gain_x, gate_align_gain_y,
    gate_align_max_speed_y, gate_align_max_speed_z, gate_align_hold_time_s, gate_align_timeout_s,
    gate_pass_speed_mps, gate_pass_target_distance_m, gate_pass_minimum_distance_m,
    gate_pass_blind_duration_s, gate_pass_timeout_s,
    marker_search_speed_mps, marker_search_timeout_s, marker_align_deadband_x,
    marker_align_deadband_y,
    marker_align_gain_x, marker_align_gain_y, marker_align_max_speed_y, marker_align_max_speed_z,
    marker_align_minimum_distance_m, marker_align_standoff_min_m, marker_align_standoff_max_m,
    marker_align_hold_time_s, marker_align_timeout_s,
    orbit_speed_mps, orbit_radius_m,
    orbit_range_gain, orbit_max_range_speed_mps, orbit_yaw_gain, orbit_max_yaw_rate_rad_s,
    orbit_vertical_gain, orbit_max_vertical_speed_mps, orbit_angle_rad, orbit_timeout_s,
    turn_yaw_rate_rad_s, turn_capture_gain, turn_capture_deadband, turn_max_angle_rad,
    turn_timeout_s, gate_search_speed_mps, gate_search_timeout_s, return_gate_pass_speed_mps,
    return_gate_pass_target_distance_m, return_gate_pass_minimum_distance_m,
    return_gate_pass_blind_duration_s, return_gate_pass_timeout_s};
  for (const double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("POR mission parameters must be finite and positive");
    }
  }
  if (!std::isfinite(surface_watchdog_timeout_s) || surface_watchdog_timeout_s < 0.0) {
    throw std::invalid_argument("POR Surface watchdog timeout must be finite and nonnegative");
  }
  if (std::abs(marker_align_standoff_min_m - 2.2) > 1e-9 ||
    std::abs(marker_align_standoff_max_m - 2.5) > 1e-9 ||
    marker_align_standoff_min_m >= marker_align_standoff_max_m ||
    marker_align_minimum_distance_m > marker_align_standoff_min_m)
  {
    throw std::invalid_argument("POR AlignMarker standoff must be 2.2-2.5 m");
  }
}

void PorMissionConfig::apply(const BT::Blackboard::Ptr & blackboard) const
{
  blackboard->set("gate_depth_m", gate_depth_m);
  blackboard->set("gate_depth_tolerance_m", gate_depth_tolerance_m);
  blackboard->set("surface_watchdog_timeout_s", surface_watchdog_timeout_s);
  blackboard->set("acquire_gate_timeout_s", acquire_gate_timeout_s);
  blackboard->set("gate_align_deadband_x", gate_align_deadband_x);
  blackboard->set("gate_align_deadband_y", gate_align_deadband_y);
  blackboard->set("gate_align_gain_x", gate_align_gain_x);
  blackboard->set("gate_align_gain_y", gate_align_gain_y);
  blackboard->set("gate_align_max_speed_y", gate_align_max_speed_y);
  blackboard->set("gate_align_max_speed_z", gate_align_max_speed_z);
  blackboard->set("gate_align_hold_time_s", gate_align_hold_time_s);
  blackboard->set("gate_align_timeout_s", gate_align_timeout_s);
  blackboard->set("gate_pass_speed_mps", gate_pass_speed_mps);
  blackboard->set("gate_pass_target_distance_m", gate_pass_target_distance_m);
  blackboard->set("gate_pass_minimum_distance_m", gate_pass_minimum_distance_m);
  blackboard->set("gate_pass_blind_duration_s", gate_pass_blind_duration_s);
  blackboard->set("gate_pass_timeout_s", gate_pass_timeout_s);
  blackboard->set("marker_search_speed_mps", marker_search_speed_mps);
  blackboard->set("marker_search_timeout_s", marker_search_timeout_s);
  blackboard->set("marker_align_deadband_x", marker_align_deadband_x);
  blackboard->set("marker_align_deadband_y", marker_align_deadband_y);
  blackboard->set("marker_align_gain_x", marker_align_gain_x);
  blackboard->set("marker_align_gain_y", marker_align_gain_y);
  blackboard->set("marker_align_max_speed_y", marker_align_max_speed_y);
  blackboard->set("marker_align_max_speed_z", marker_align_max_speed_z);
  blackboard->set("marker_align_minimum_distance_m", marker_align_minimum_distance_m);
  blackboard->set("marker_align_standoff_min_m", marker_align_standoff_min_m);
  blackboard->set("marker_align_standoff_max_m", marker_align_standoff_max_m);
  blackboard->set("marker_align_hold_time_s", marker_align_hold_time_s);
  blackboard->set("marker_align_timeout_s", marker_align_timeout_s);
  blackboard->set("orbit_speed_mps", orbit_speed_mps);
  blackboard->set("orbit_radius_m", orbit_radius_m);
  blackboard->set("orbit_range_gain", orbit_range_gain);
  blackboard->set("orbit_max_range_speed_mps", orbit_max_range_speed_mps);
  blackboard->set("orbit_yaw_gain", orbit_yaw_gain);
  blackboard->set("orbit_max_yaw_rate_rad_s", orbit_max_yaw_rate_rad_s);
  blackboard->set("orbit_vertical_gain", orbit_vertical_gain);
  blackboard->set("orbit_max_vertical_speed_mps", orbit_max_vertical_speed_mps);
  blackboard->set("orbit_angle_rad", orbit_angle_rad);
  blackboard->set("orbit_timeout_s", orbit_timeout_s);
  blackboard->set("turn_yaw_rate_rad_s", turn_yaw_rate_rad_s);
  blackboard->set("turn_capture_gain", turn_capture_gain);
  blackboard->set("turn_capture_deadband", turn_capture_deadband);
  blackboard->set("turn_max_angle_rad", turn_max_angle_rad);
  blackboard->set("turn_timeout_s", turn_timeout_s);
  blackboard->set("gate_search_speed_mps", gate_search_speed_mps);
  blackboard->set("gate_search_timeout_s", gate_search_timeout_s);
  blackboard->set("return_gate_pass_speed_mps", return_gate_pass_speed_mps);
  blackboard->set("return_gate_pass_target_distance_m", return_gate_pass_target_distance_m);
  blackboard->set("return_gate_pass_minimum_distance_m", return_gate_pass_minimum_distance_m);
  blackboard->set("return_gate_pass_blind_duration_s", return_gate_pass_blind_duration_s);
  blackboard->set("return_gate_pass_timeout_s", return_gate_pass_timeout_s);
}

}  // namespace banyu_mission
