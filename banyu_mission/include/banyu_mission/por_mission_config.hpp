#ifndef BANYU_MISSION__POR_MISSION_CONFIG_HPP_
#define BANYU_MISSION__POR_MISSION_CONFIG_HPP_

#include <memory>

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/rclcpp.hpp"

namespace banyu_mission
{

struct PorMissionConfig
{
  double gate_depth_m {2.5};
  double gate_depth_tolerance_m {0.3};
  // Continuous missing-prerequisite watchdog for POR Surface; not a phase duration.
  double surface_watchdog_timeout_s {5.0};
  double acquire_gate_timeout_s {20.0};
  double gate_align_deadband_x {0.08};
  double gate_align_deadband_y {0.08};
  double gate_align_gain_x {0.8};
  double gate_align_gain_y {0.8};
  double gate_align_max_speed_y {0.4};
  double gate_align_max_speed_z {0.3};
  double gate_align_hold_time_s {1.0};
  double gate_align_timeout_s {20.0};
  double gate_pass_speed_mps {0.8};
  double gate_pass_target_distance_m {4.0};
  double gate_pass_minimum_distance_m {0.8};
  double gate_pass_blind_duration_s {6.0};
  double gate_pass_timeout_s {20.0};
  double marker_search_speed_mps {0.6};
  double marker_search_timeout_s {45.0};
  double marker_align_deadband_x {0.08};
  double marker_align_deadband_y {0.08};
  double marker_align_gain_x {0.8};
  double marker_align_gain_y {0.8};
  double marker_align_max_speed_y {0.4};
  double marker_align_max_speed_z {0.3};
  double marker_align_minimum_distance_m {1.0};
  double marker_align_standoff_min_m {2.2};
  double marker_align_standoff_max_m {2.5};
  double marker_align_hold_time_s {1.0};
  double marker_align_timeout_s {20.0};
  double orbit_speed_mps {0.5};
  double orbit_radius_m {3.0};
  double orbit_range_gain {0.8};
  double orbit_max_range_speed_mps {0.6};
  double orbit_yaw_gain {0.8};
  double orbit_max_yaw_rate_rad_s {0.6};
  double orbit_vertical_gain {0.5};
  double orbit_max_vertical_speed_mps {0.3};
  double orbit_angle_rad {6.283185307};
  double orbit_timeout_s {60.0};
  double turn_yaw_rate_rad_s {0.6};
  double turn_capture_gain {0.8};
  double turn_capture_deadband {0.08};
  double turn_max_angle_rad {3.141592653};
  double turn_timeout_s {30.0};
  double gate_search_speed_mps {0.6};
  double gate_search_timeout_s {30.0};
  double return_gate_pass_speed_mps {0.8};
  double return_gate_pass_target_distance_m {10.5};
  double return_gate_pass_minimum_distance_m {0.8};
  double return_gate_pass_blind_duration_s {14.0};
  double return_gate_pass_timeout_s {35.0};

  static PorMissionConfig load(rclcpp::Node & node);
  void validate() const;
  void apply(const BT::Blackboard::Ptr & blackboard) const;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__POR_MISSION_CONFIG_HPP_
