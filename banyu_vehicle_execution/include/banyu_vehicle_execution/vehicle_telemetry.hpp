#ifndef BANYU_VEHICLE_EXECUTION__VEHICLE_TELEMETRY_HPP_
#define BANYU_VEHICLE_EXECUTION__VEHICLE_TELEMETRY_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace banyu_vehicle_execution
{

struct VehicleTelemetry
{
  rclcpp::Time current_time{};

  // Local ENU pose
  bool pose_valid{false};
  rclcpp::Time pose_stamp{};
  double pos_x{0.0};
  double pos_y{0.0};
  double pos_z{0.0};
  double yaw{0.0};

  // GPS NavSatFix
  bool gps_valid{false};
  rclcpp::Time gps_stamp{};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};

  // Depth (positive downward, 0 = surface)
  bool depth_valid{false};
  rclcpp::Time depth_stamp{};
  double depth_m{0.0};

  // Vehicle status
  bool status_valid{false};
  rclcpp::Time status_stamp{};
  bool fcu_connected{false};
  bool fcu_status_fresh{false};
  bool rc_fresh{false};
  bool operator_takeover{false};
  std::string mode{};
  std::string authority{};

  // Pipeline observation
  bool pipeline_obs_valid{false};
  rclcpp::Time pipeline_obs_stamp{};
  uint8_t pipeline_status{0};
  double pipeline_offset_x{0.0};
  double pipeline_offset_y{0.0};
  double pipeline_heading_error_rad{0.0};
  double pipeline_confidence{0.0};

  // Ring observation
  bool ring_obs_valid{false};
  rclcpp::Time ring_obs_stamp{};
  bool ring_visible{false};
  uint8_t ring_color{0};
  double ring_offset_x{0.0};
  double ring_offset_y{0.0};
  double ring_confidence{0.0};

  // Guidance velocity intent (for FollowPipeline)
  bool guidance_intent_valid{false};
  rclcpp::Time guidance_intent_stamp{};
  uint8_t guidance_status{0};
  double guidance_surge_mps{0.0};
  double guidance_sway_mps{0.0};
  double guidance_yaw_rate_rad_s{0.0};
  bool guidance_yaw_control_active{false};
};

inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

inline double getYawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline void latLonToEnuOffsets(
  double cur_lat, double cur_lon,
  double target_lat, double target_lon,
  double & dx_east, double & dy_north)
{
  constexpr double kEarthRadius = 6378137.0;
  constexpr double kDegToRad = M_PI / 180.0;
  const double dlat = (target_lat - cur_lat) * kDegToRad;
  const double dlon = (target_lon - cur_lon) * kDegToRad;
  const double mean_lat = (target_lat + cur_lat) * 0.5 * kDegToRad;
  dx_east = dlon * std::cos(mean_lat) * kEarthRadius;
  dy_north = dlat * kEarthRadius;
}

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__VEHICLE_TELEMETRY_HPP_
