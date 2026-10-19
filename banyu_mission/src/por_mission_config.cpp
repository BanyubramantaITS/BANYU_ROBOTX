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

  const auto bool_param = [&node](const std::string & name, bool fallback) {
      return node.declare_parameter<bool>("mission." + name, fallback);
    };
  config.use_vision = bool_param("use_vision", config.use_vision);
  config.use_vision_return = bool_param("use_vision_return", config.use_vision_return);
  config.use_vision_assist = bool_param("use_vision_assist", config.use_vision_assist);
  config.assist_gate_align_max_s = parameter(
    "assist_gate_align_max_s", config.assist_gate_align_max_s);
  config.assist_gate_deadband = parameter("assist_gate_deadband", config.assist_gate_deadband);
  config.assist_marker_align_max_s = parameter(
    "assist_marker_align_max_s", config.assist_marker_align_max_s);
  config.assist_marker_standoff_min_m = parameter(
    "assist_marker_standoff_min_m", config.assist_marker_standoff_min_m);
  config.assist_marker_standoff_max_m = parameter(
    "assist_marker_standoff_max_m", config.assist_marker_standoff_max_m);
  config.assist_marker_deadband = parameter(
    "assist_marker_deadband", config.assist_marker_deadband);
  config.assist_align_hold_s = parameter("assist_align_hold_s", config.assist_align_hold_s);
  config.use_sensors = bool_param("use_sensors", config.use_sensors);
  config.use_baro_depth = bool_param("use_baro_depth", config.use_baro_depth);
  config.use_imu_yaw_cap = bool_param("use_imu_yaw_cap", config.use_imu_yaw_cap);
  config.depth_kp = parameter("depth_kp", config.depth_kp);
  config.max_depth_speed_mps = parameter("max_depth_speed_mps", config.max_depth_speed_mps);

  config.dive_duration_s = parameter("dive_duration_s", config.dive_duration_s);
  config.dive_speed_x_mps = parameter("dive_speed_x_mps", config.dive_speed_x_mps);
  config.dive_speed_y_mps = parameter("dive_speed_y_mps", config.dive_speed_y_mps);
  config.dive_speed_z_mps = parameter("dive_speed_z_mps", config.dive_speed_z_mps);
  config.dive_yaw_rate_rad_s = parameter("dive_yaw_rate_rad_s", config.dive_yaw_rate_rad_s);

  config.timed_pass_gate_duration_s = parameter(
    "timed_pass_gate_duration_s", config.timed_pass_gate_duration_s);
  config.timed_pass_gate_speed_x_mps = parameter(
    "timed_pass_gate_speed_x_mps", config.timed_pass_gate_speed_x_mps);
  config.timed_pass_gate_speed_y_mps = parameter(
    "timed_pass_gate_speed_y_mps", config.timed_pass_gate_speed_y_mps);
  config.timed_pass_gate_speed_z_mps = parameter(
    "timed_pass_gate_speed_z_mps", config.timed_pass_gate_speed_z_mps);
  config.timed_pass_gate_yaw_rate_rad_s = parameter(
    "timed_pass_gate_yaw_rate_rad_s", config.timed_pass_gate_yaw_rate_rad_s);

  config.approach_pole_duration_s = parameter(
    "approach_pole_duration_s", config.approach_pole_duration_s);
  config.approach_pole_speed_x_mps = parameter(
    "approach_pole_speed_x_mps", config.approach_pole_speed_x_mps);
  config.approach_pole_speed_y_mps = parameter(
    "approach_pole_speed_y_mps", config.approach_pole_speed_y_mps);
  config.approach_pole_speed_z_mps = parameter(
    "approach_pole_speed_z_mps", config.approach_pole_speed_z_mps);
  config.approach_pole_yaw_rate_rad_s = parameter(
    "approach_pole_yaw_rate_rad_s", config.approach_pole_yaw_rate_rad_s);

  config.circle_turn_1_duration_s = parameter(
    "circle_turn_1_duration_s", config.circle_turn_1_duration_s);
  config.circle_turn_1_speed_x_mps = parameter(
    "circle_turn_1_speed_x_mps", config.circle_turn_1_speed_x_mps);
  config.circle_turn_1_speed_y_mps = parameter(
    "circle_turn_1_speed_y_mps", config.circle_turn_1_speed_y_mps);
  config.circle_turn_1_speed_z_mps = parameter(
    "circle_turn_1_speed_z_mps", config.circle_turn_1_speed_z_mps);
  config.circle_turn_1_yaw_rate_rad_s = parameter(
    "circle_turn_1_yaw_rate_rad_s", config.circle_turn_1_yaw_rate_rad_s);
  config.circle_turn_1_target_angle_rad = parameter(
    "circle_turn_1_target_angle_rad", config.circle_turn_1_target_angle_rad);
  config.circle_forward_1_duration_s = parameter(
    "circle_forward_1_duration_s", config.circle_forward_1_duration_s);
  config.circle_forward_1_speed_x_mps = parameter(
    "circle_forward_1_speed_x_mps", config.circle_forward_1_speed_x_mps);
  config.circle_forward_1_speed_y_mps = parameter(
    "circle_forward_1_speed_y_mps", config.circle_forward_1_speed_y_mps);
  config.circle_forward_1_speed_z_mps = parameter(
    "circle_forward_1_speed_z_mps", config.circle_forward_1_speed_z_mps);
  config.circle_forward_1_yaw_rate_rad_s = parameter(
    "circle_forward_1_yaw_rate_rad_s", config.circle_forward_1_yaw_rate_rad_s);

  config.circle_turn_2_duration_s = parameter(
    "circle_turn_2_duration_s", config.circle_turn_2_duration_s);
  config.circle_turn_2_speed_x_mps = parameter(
    "circle_turn_2_speed_x_mps", config.circle_turn_2_speed_x_mps);
  config.circle_turn_2_speed_y_mps = parameter(
    "circle_turn_2_speed_y_mps", config.circle_turn_2_speed_y_mps);
  config.circle_turn_2_speed_z_mps = parameter(
    "circle_turn_2_speed_z_mps", config.circle_turn_2_speed_z_mps);
  config.circle_turn_2_yaw_rate_rad_s = parameter(
    "circle_turn_2_yaw_rate_rad_s", config.circle_turn_2_yaw_rate_rad_s);
  config.circle_turn_2_target_angle_rad = parameter(
    "circle_turn_2_target_angle_rad", config.circle_turn_2_target_angle_rad);
  config.circle_forward_2_duration_s = parameter(
    "circle_forward_2_duration_s", config.circle_forward_2_duration_s);
  config.circle_forward_2_speed_x_mps = parameter(
    "circle_forward_2_speed_x_mps", config.circle_forward_2_speed_x_mps);
  config.circle_forward_2_speed_y_mps = parameter(
    "circle_forward_2_speed_y_mps", config.circle_forward_2_speed_y_mps);
  config.circle_forward_2_speed_z_mps = parameter(
    "circle_forward_2_speed_z_mps", config.circle_forward_2_speed_z_mps);
  config.circle_forward_2_yaw_rate_rad_s = parameter(
    "circle_forward_2_yaw_rate_rad_s", config.circle_forward_2_yaw_rate_rad_s);

  config.circle_turn_3_duration_s = parameter(
    "circle_turn_3_duration_s", config.circle_turn_3_duration_s);
  config.circle_turn_3_speed_x_mps = parameter(
    "circle_turn_3_speed_x_mps", config.circle_turn_3_speed_x_mps);
  config.circle_turn_3_speed_y_mps = parameter(
    "circle_turn_3_speed_y_mps", config.circle_turn_3_speed_y_mps);
  config.circle_turn_3_speed_z_mps = parameter(
    "circle_turn_3_speed_z_mps", config.circle_turn_3_speed_z_mps);
  config.circle_turn_3_yaw_rate_rad_s = parameter(
    "circle_turn_3_yaw_rate_rad_s", config.circle_turn_3_yaw_rate_rad_s);
  config.circle_turn_3_target_angle_rad = parameter(
    "circle_turn_3_target_angle_rad", config.circle_turn_3_target_angle_rad);
  config.circle_forward_3_duration_s = parameter(
    "circle_forward_3_duration_s", config.circle_forward_3_duration_s);
  config.circle_forward_3_speed_x_mps = parameter(
    "circle_forward_3_speed_x_mps", config.circle_forward_3_speed_x_mps);
  config.circle_forward_3_speed_y_mps = parameter(
    "circle_forward_3_speed_y_mps", config.circle_forward_3_speed_y_mps);
  config.circle_forward_3_speed_z_mps = parameter(
    "circle_forward_3_speed_z_mps", config.circle_forward_3_speed_z_mps);
  config.circle_forward_3_yaw_rate_rad_s = parameter(
    "circle_forward_3_yaw_rate_rad_s", config.circle_forward_3_yaw_rate_rad_s);

  config.circle_turn_4_duration_s = parameter(
    "circle_turn_4_duration_s", config.circle_turn_4_duration_s);
  config.circle_turn_4_speed_x_mps = parameter(
    "circle_turn_4_speed_x_mps", config.circle_turn_4_speed_x_mps);
  config.circle_turn_4_speed_y_mps = parameter(
    "circle_turn_4_speed_y_mps", config.circle_turn_4_speed_y_mps);
  config.circle_turn_4_speed_z_mps = parameter(
    "circle_turn_4_speed_z_mps", config.circle_turn_4_speed_z_mps);
  config.circle_turn_4_yaw_rate_rad_s = parameter(
    "circle_turn_4_yaw_rate_rad_s", config.circle_turn_4_yaw_rate_rad_s);
  config.circle_turn_4_target_angle_rad = parameter(
    "circle_turn_4_target_angle_rad", config.circle_turn_4_target_angle_rad);
  config.circle_forward_4_duration_s = parameter(
    "circle_forward_4_duration_s", config.circle_forward_4_duration_s);
  config.circle_forward_4_speed_x_mps = parameter(
    "circle_forward_4_speed_x_mps", config.circle_forward_4_speed_x_mps);
  config.circle_forward_4_speed_y_mps = parameter(
    "circle_forward_4_speed_y_mps", config.circle_forward_4_speed_y_mps);
  config.circle_forward_4_speed_z_mps = parameter(
    "circle_forward_4_speed_z_mps", config.circle_forward_4_speed_z_mps);
  config.circle_forward_4_yaw_rate_rad_s = parameter(
    "circle_forward_4_yaw_rate_rad_s", config.circle_forward_4_yaw_rate_rad_s);

  config.return_turn_duration_s = parameter(
    "return_turn_duration_s", config.return_turn_duration_s);
  config.return_turn_speed_x_mps = parameter(
    "return_turn_speed_x_mps", config.return_turn_speed_x_mps);
  config.return_turn_speed_y_mps = parameter(
    "return_turn_speed_y_mps", config.return_turn_speed_y_mps);
  config.return_turn_speed_z_mps = parameter(
    "return_turn_speed_z_mps", config.return_turn_speed_z_mps);
  config.return_turn_yaw_rate_rad_s = parameter(
    "return_turn_yaw_rate_rad_s", config.return_turn_yaw_rate_rad_s);
  config.return_turn_target_angle_rad = parameter(
    "return_turn_target_angle_rad", config.return_turn_target_angle_rad);

  config.return_to_gate_duration_s = parameter(
    "return_to_gate_duration_s", config.return_to_gate_duration_s);
  config.return_to_gate_speed_x_mps = parameter(
    "return_to_gate_speed_x_mps", config.return_to_gate_speed_x_mps);
  config.return_to_gate_speed_y_mps = parameter(
    "return_to_gate_speed_y_mps", config.return_to_gate_speed_y_mps);
  config.return_to_gate_speed_z_mps = parameter(
    "return_to_gate_speed_z_mps", config.return_to_gate_speed_z_mps);
  config.return_to_gate_yaw_rate_rad_s = parameter(
    "return_to_gate_yaw_rate_rad_s", config.return_to_gate_yaw_rate_rad_s);

  config.timed_surface_duration_s = parameter(
    "timed_surface_duration_s", config.timed_surface_duration_s);
  config.timed_surface_speed_x_mps = parameter(
    "timed_surface_speed_x_mps", config.timed_surface_speed_x_mps);
  config.timed_surface_speed_y_mps = parameter(
    "timed_surface_speed_y_mps", config.timed_surface_speed_y_mps);
  config.timed_surface_speed_z_mps = parameter(
    "timed_surface_speed_z_mps", config.timed_surface_speed_z_mps);
  config.timed_surface_yaw_rate_rad_s = parameter(
    "timed_surface_yaw_rate_rad_s", config.timed_surface_yaw_rate_rad_s);
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
    return_gate_pass_blind_duration_s, return_gate_pass_timeout_s,
    depth_kp, max_depth_speed_mps, assist_gate_deadband, assist_marker_standoff_min_m,
    assist_marker_standoff_max_m, assist_marker_deadband, assist_align_hold_s};
  for (const double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("POR mission parameters must be finite and positive");
    }
  }
  if (assist_marker_standoff_max_m < assist_marker_standoff_min_m) {
    throw std::invalid_argument("assist_marker_standoff_max_m must be >= the min");
  }
  const std::array nonnegative_values {
    surface_watchdog_timeout_s, assist_gate_align_max_s, assist_marker_align_max_s,
    dive_duration_s, timed_pass_gate_duration_s, approach_pole_duration_s,
    circle_turn_1_duration_s, circle_turn_1_target_angle_rad, circle_forward_1_duration_s,
    circle_turn_2_duration_s, circle_turn_2_target_angle_rad, circle_forward_2_duration_s,
    circle_turn_3_duration_s, circle_turn_3_target_angle_rad, circle_forward_3_duration_s,
    circle_turn_4_duration_s, circle_turn_4_target_angle_rad, circle_forward_4_duration_s,
    return_turn_duration_s, return_turn_target_angle_rad, return_to_gate_duration_s,
    timed_surface_duration_s};
  for (const double value : nonnegative_values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("POR duration/angle parameters must be finite and nonnegative");
    }
  }
  const std::array finite_signed_values {
    dive_speed_x_mps, dive_speed_y_mps, dive_speed_z_mps, dive_yaw_rate_rad_s,
    timed_pass_gate_speed_x_mps, timed_pass_gate_speed_y_mps,
    timed_pass_gate_speed_z_mps, timed_pass_gate_yaw_rate_rad_s,
    approach_pole_speed_x_mps, approach_pole_speed_y_mps,
    approach_pole_speed_z_mps, approach_pole_yaw_rate_rad_s,
    circle_turn_1_speed_x_mps, circle_turn_1_speed_y_mps,
    circle_turn_1_speed_z_mps, circle_turn_1_yaw_rate_rad_s,
    circle_forward_1_speed_x_mps, circle_forward_1_speed_y_mps,
    circle_forward_1_speed_z_mps, circle_forward_1_yaw_rate_rad_s,
    circle_turn_2_speed_x_mps, circle_turn_2_speed_y_mps,
    circle_turn_2_speed_z_mps, circle_turn_2_yaw_rate_rad_s,
    circle_forward_2_speed_x_mps, circle_forward_2_speed_y_mps,
    circle_forward_2_speed_z_mps, circle_forward_2_yaw_rate_rad_s,
    circle_turn_3_speed_x_mps, circle_turn_3_speed_y_mps,
    circle_turn_3_speed_z_mps, circle_turn_3_yaw_rate_rad_s,
    circle_forward_3_speed_x_mps, circle_forward_3_speed_y_mps,
    circle_forward_3_speed_z_mps, circle_forward_3_yaw_rate_rad_s,
    circle_turn_4_speed_x_mps, circle_turn_4_speed_y_mps,
    circle_turn_4_speed_z_mps, circle_turn_4_yaw_rate_rad_s,
    circle_forward_4_speed_x_mps, circle_forward_4_speed_y_mps,
    circle_forward_4_speed_z_mps, circle_forward_4_yaw_rate_rad_s,
    return_turn_speed_x_mps, return_turn_speed_y_mps,
    return_turn_speed_z_mps, return_turn_yaw_rate_rad_s,
    return_to_gate_speed_x_mps, return_to_gate_speed_y_mps,
    return_to_gate_speed_z_mps, return_to_gate_yaw_rate_rad_s,
    timed_surface_speed_x_mps, timed_surface_speed_y_mps,
    timed_surface_speed_z_mps, timed_surface_yaw_rate_rad_s};
  for (const double value : finite_signed_values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("POR timed speed/yaw-rate parameters must be finite");
    }
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

  blackboard->set("use_vision", use_vision);
  blackboard->set("use_vision_return", use_vision_return);
  blackboard->set("use_vision_assist", use_vision_assist);
  // BT Timeout decorators take whole milliseconds.
  blackboard->set(
    "assist_gate_align_max_ms", static_cast<unsigned>(std::lround(assist_gate_align_max_s * 1000.0)));
  blackboard->set(
    "assist_marker_align_max_ms",
    static_cast<unsigned>(std::lround(assist_marker_align_max_s * 1000.0)));
  blackboard->set("assist_gate_align_max_s", assist_gate_align_max_s);
  blackboard->set("assist_marker_align_max_s", assist_marker_align_max_s);
  blackboard->set("assist_gate_deadband", assist_gate_deadband);
  blackboard->set("assist_marker_standoff_min_m", assist_marker_standoff_min_m);
  blackboard->set("assist_marker_standoff_max_m", assist_marker_standoff_max_m);
  blackboard->set("assist_marker_deadband", assist_marker_deadband);
  blackboard->set("assist_align_hold_s", assist_align_hold_s);
  blackboard->set("use_sensors", use_sensors);
  blackboard->set("use_baro_depth", use_sensors && use_baro_depth);
  blackboard->set("use_imu_yaw_cap", use_sensors && use_imu_yaw_cap);
  blackboard->set("depth_kp", depth_kp);
  blackboard->set("max_depth_speed_mps", max_depth_speed_mps);

  blackboard->set("dive_duration_s", dive_duration_s);
  blackboard->set("dive_speed_x_mps", dive_speed_x_mps);
  blackboard->set("dive_speed_y_mps", dive_speed_y_mps);
  blackboard->set("dive_speed_z_mps", dive_speed_z_mps);
  blackboard->set("dive_yaw_rate_rad_s", dive_yaw_rate_rad_s);

  blackboard->set("timed_pass_gate_duration_s", timed_pass_gate_duration_s);
  blackboard->set("timed_pass_gate_speed_x_mps", timed_pass_gate_speed_x_mps);
  blackboard->set("timed_pass_gate_speed_y_mps", timed_pass_gate_speed_y_mps);
  blackboard->set("timed_pass_gate_speed_z_mps", timed_pass_gate_speed_z_mps);
  blackboard->set("timed_pass_gate_yaw_rate_rad_s", timed_pass_gate_yaw_rate_rad_s);

  blackboard->set("approach_pole_duration_s", approach_pole_duration_s);
  blackboard->set("approach_pole_speed_x_mps", approach_pole_speed_x_mps);
  blackboard->set("approach_pole_speed_y_mps", approach_pole_speed_y_mps);
  blackboard->set("approach_pole_speed_z_mps", approach_pole_speed_z_mps);
  blackboard->set("approach_pole_yaw_rate_rad_s", approach_pole_yaw_rate_rad_s);

  blackboard->set("circle_turn_1_duration_s", circle_turn_1_duration_s);
  blackboard->set("circle_turn_1_speed_x_mps", circle_turn_1_speed_x_mps);
  blackboard->set("circle_turn_1_speed_y_mps", circle_turn_1_speed_y_mps);
  blackboard->set("circle_turn_1_speed_z_mps", circle_turn_1_speed_z_mps);
  blackboard->set("circle_turn_1_yaw_rate_rad_s", circle_turn_1_yaw_rate_rad_s);
  blackboard->set("circle_turn_1_target_angle_rad", circle_turn_1_target_angle_rad);
  blackboard->set("circle_forward_1_duration_s", circle_forward_1_duration_s);
  blackboard->set("circle_forward_1_speed_x_mps", circle_forward_1_speed_x_mps);
  blackboard->set("circle_forward_1_speed_y_mps", circle_forward_1_speed_y_mps);
  blackboard->set("circle_forward_1_speed_z_mps", circle_forward_1_speed_z_mps);
  blackboard->set("circle_forward_1_yaw_rate_rad_s", circle_forward_1_yaw_rate_rad_s);

  blackboard->set("circle_turn_2_duration_s", circle_turn_2_duration_s);
  blackboard->set("circle_turn_2_speed_x_mps", circle_turn_2_speed_x_mps);
  blackboard->set("circle_turn_2_speed_y_mps", circle_turn_2_speed_y_mps);
  blackboard->set("circle_turn_2_speed_z_mps", circle_turn_2_speed_z_mps);
  blackboard->set("circle_turn_2_yaw_rate_rad_s", circle_turn_2_yaw_rate_rad_s);
  blackboard->set("circle_turn_2_target_angle_rad", circle_turn_2_target_angle_rad);
  blackboard->set("circle_forward_2_duration_s", circle_forward_2_duration_s);
  blackboard->set("circle_forward_2_speed_x_mps", circle_forward_2_speed_x_mps);
  blackboard->set("circle_forward_2_speed_y_mps", circle_forward_2_speed_y_mps);
  blackboard->set("circle_forward_2_speed_z_mps", circle_forward_2_speed_z_mps);
  blackboard->set("circle_forward_2_yaw_rate_rad_s", circle_forward_2_yaw_rate_rad_s);

  blackboard->set("circle_turn_3_duration_s", circle_turn_3_duration_s);
  blackboard->set("circle_turn_3_speed_x_mps", circle_turn_3_speed_x_mps);
  blackboard->set("circle_turn_3_speed_y_mps", circle_turn_3_speed_y_mps);
  blackboard->set("circle_turn_3_speed_z_mps", circle_turn_3_speed_z_mps);
  blackboard->set("circle_turn_3_yaw_rate_rad_s", circle_turn_3_yaw_rate_rad_s);
  blackboard->set("circle_turn_3_target_angle_rad", circle_turn_3_target_angle_rad);
  blackboard->set("circle_forward_3_duration_s", circle_forward_3_duration_s);
  blackboard->set("circle_forward_3_speed_x_mps", circle_forward_3_speed_x_mps);
  blackboard->set("circle_forward_3_speed_y_mps", circle_forward_3_speed_y_mps);
  blackboard->set("circle_forward_3_speed_z_mps", circle_forward_3_speed_z_mps);
  blackboard->set("circle_forward_3_yaw_rate_rad_s", circle_forward_3_yaw_rate_rad_s);

  blackboard->set("circle_turn_4_duration_s", circle_turn_4_duration_s);
  blackboard->set("circle_turn_4_speed_x_mps", circle_turn_4_speed_x_mps);
  blackboard->set("circle_turn_4_speed_y_mps", circle_turn_4_speed_y_mps);
  blackboard->set("circle_turn_4_speed_z_mps", circle_turn_4_speed_z_mps);
  blackboard->set("circle_turn_4_yaw_rate_rad_s", circle_turn_4_yaw_rate_rad_s);
  blackboard->set("circle_turn_4_target_angle_rad", circle_turn_4_target_angle_rad);
  blackboard->set("circle_forward_4_duration_s", circle_forward_4_duration_s);
  blackboard->set("circle_forward_4_speed_x_mps", circle_forward_4_speed_x_mps);
  blackboard->set("circle_forward_4_speed_y_mps", circle_forward_4_speed_y_mps);
  blackboard->set("circle_forward_4_speed_z_mps", circle_forward_4_speed_z_mps);
  blackboard->set("circle_forward_4_yaw_rate_rad_s", circle_forward_4_yaw_rate_rad_s);

  blackboard->set("return_turn_duration_s", return_turn_duration_s);
  blackboard->set("return_turn_speed_x_mps", return_turn_speed_x_mps);
  blackboard->set("return_turn_speed_y_mps", return_turn_speed_y_mps);
  blackboard->set("return_turn_speed_z_mps", return_turn_speed_z_mps);
  blackboard->set("return_turn_yaw_rate_rad_s", return_turn_yaw_rate_rad_s);
  blackboard->set("return_turn_target_angle_rad", return_turn_target_angle_rad);

  blackboard->set("return_to_gate_duration_s", return_to_gate_duration_s);
  blackboard->set("return_to_gate_speed_x_mps", return_to_gate_speed_x_mps);
  blackboard->set("return_to_gate_speed_y_mps", return_to_gate_speed_y_mps);
  blackboard->set("return_to_gate_speed_z_mps", return_to_gate_speed_z_mps);
  blackboard->set("return_to_gate_yaw_rate_rad_s", return_to_gate_yaw_rate_rad_s);

  blackboard->set("timed_surface_duration_s", timed_surface_duration_s);
  blackboard->set("timed_surface_speed_x_mps", timed_surface_speed_x_mps);
  blackboard->set("timed_surface_speed_y_mps", timed_surface_speed_y_mps);
  blackboard->set("timed_surface_speed_z_mps", timed_surface_speed_z_mps);
  blackboard->set("timed_surface_yaw_rate_rad_s", timed_surface_yaw_rate_rad_s);
}

}  // namespace banyu_mission
