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
  double gate_align_deadband_x {0.03};
  double gate_align_deadband_y {0.03};
  double gate_align_gain_x {1.2};
  double gate_align_gain_y {1.0};
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
  double marker_align_deadband_x {0.03};
  double marker_align_deadband_y {0.03};
  double marker_align_gain_x {1.2};
  double marker_align_gain_y {1.0};
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
  double orbit_yaw_gain {1.5};
  double orbit_max_yaw_rate_rad_s {0.6};
  double orbit_vertical_gain {1.0};
  double orbit_max_vertical_speed_mps {0.4};
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

  // Timed POR strategy configuration (selected when use_vision == false)
  bool use_vision {false};
  // Hybrid: timed outbound + marker loop, then camera-guided turn and gate return
  bool use_vision_return {false};
  bool use_sensors {true};
  bool use_baro_depth {true};
  bool use_imu_yaw_cap {true};
  double depth_kp {0.7};
  double max_depth_speed_mps {0.4};

  double dive_duration_s {5.0};
  double dive_speed_x_mps {0.0};
  double dive_speed_y_mps {0.0};
  double dive_speed_z_mps {-0.4};
  double dive_yaw_rate_rad_s {0.0};

  double timed_pass_gate_duration_s {6.0};
  double timed_pass_gate_speed_x_mps {0.6};
  double timed_pass_gate_speed_y_mps {0.0};
  double timed_pass_gate_speed_z_mps {0.0};
  double timed_pass_gate_yaw_rate_rad_s {0.0};

  double approach_pole_duration_s {8.0};
  double approach_pole_speed_x_mps {0.6};
  double approach_pole_speed_y_mps {0.0};
  double approach_pole_speed_z_mps {0.0};
  double approach_pole_yaw_rate_rad_s {0.0};

  double circle_turn_1_duration_s {2.62};
  double circle_turn_1_speed_x_mps {0.0};
  double circle_turn_1_speed_y_mps {0.0};
  double circle_turn_1_speed_z_mps {0.0};
  double circle_turn_1_yaw_rate_rad_s {-0.6};
  double circle_turn_1_target_angle_rad {1.570796327};
  double circle_forward_1_duration_s {3.0};
  double circle_forward_1_speed_x_mps {0.5};
  double circle_forward_1_speed_y_mps {0.0};
  double circle_forward_1_speed_z_mps {0.0};
  double circle_forward_1_yaw_rate_rad_s {0.0};

  double circle_turn_2_duration_s {2.62};
  double circle_turn_2_speed_x_mps {0.0};
  double circle_turn_2_speed_y_mps {0.0};
  double circle_turn_2_speed_z_mps {0.0};
  double circle_turn_2_yaw_rate_rad_s {0.6};
  double circle_turn_2_target_angle_rad {1.570796327};
  double circle_forward_2_duration_s {5.0};
  double circle_forward_2_speed_x_mps {0.5};
  double circle_forward_2_speed_y_mps {0.0};
  double circle_forward_2_speed_z_mps {0.0};
  double circle_forward_2_yaw_rate_rad_s {0.0};

  double circle_turn_3_duration_s {2.62};
  double circle_turn_3_speed_x_mps {0.0};
  double circle_turn_3_speed_y_mps {0.0};
  double circle_turn_3_speed_z_mps {0.0};
  double circle_turn_3_yaw_rate_rad_s {0.6};
  double circle_turn_3_target_angle_rad {1.570796327};
  double circle_forward_3_duration_s {5.0};
  double circle_forward_3_speed_x_mps {0.5};
  double circle_forward_3_speed_y_mps {0.0};
  double circle_forward_3_speed_z_mps {0.0};
  double circle_forward_3_yaw_rate_rad_s {0.0};

  double circle_turn_4_duration_s {2.62};
  double circle_turn_4_speed_x_mps {0.0};
  double circle_turn_4_speed_y_mps {0.0};
  double circle_turn_4_speed_z_mps {0.0};
  double circle_turn_4_yaw_rate_rad_s {0.6};
  double circle_turn_4_target_angle_rad {1.570796327};
  double circle_forward_4_duration_s {5.0};
  double circle_forward_4_speed_x_mps {0.5};
  double circle_forward_4_speed_y_mps {0.0};
  double circle_forward_4_speed_z_mps {0.0};
  double circle_forward_4_yaw_rate_rad_s {0.0};

  double return_turn_duration_s {2.62};
  double return_turn_speed_x_mps {0.0};
  double return_turn_speed_y_mps {0.0};
  double return_turn_speed_z_mps {0.0};
  double return_turn_yaw_rate_rad_s {0.6};
  double return_turn_target_angle_rad {1.570796327};

  double return_to_gate_duration_s {14.0};
  double return_to_gate_speed_x_mps {0.6};
  double return_to_gate_speed_y_mps {0.0};
  double return_to_gate_speed_z_mps {0.0};
  double return_to_gate_yaw_rate_rad_s {0.0};

  double timed_surface_duration_s {6.0};
  double timed_surface_speed_x_mps {0.0};
  double timed_surface_speed_y_mps {0.0};
  double timed_surface_speed_z_mps {0.4};
  double timed_surface_yaw_rate_rad_s {0.0};

  static PorMissionConfig load(rclcpp::Node & node);
  void validate() const;
  void apply(const BT::Blackboard::Ptr & blackboard) const;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__POR_MISSION_CONFIG_HPP_
