#ifndef BANYU_VEHICLE_EXECUTION__VEHICLE_TELEMETRY_HPP_
#define BANYU_VEHICLE_EXECUTION__VEHICLE_TELEMETRY_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "banyu_msgs/msg/detection.hpp"

namespace banyu_vehicle_execution
{

struct WaterDepthSample
{
  double depth_m{0.0};
  rclcpp::Time stamp{};
  std::chrono::steady_clock::time_point receipt_time{};
  bool valid{false};
  bool calibrated{false};
  uint32_t run_id{0};
  uint64_t epoch{0};
  std::string datum;
  std::string source_id;
  std::string calibration_id;
};

struct WaterDepthTelemetry
{
  bool valid{false};
  bool calibrated{false};
  double depth_m{0.0};
  rclcpp::Time stamp{};
  uint32_t run_id{0};
  uint64_t epoch{0};
  std::string datum;
  std::string source_id;
  std::string calibration_id;
  bool receipt_valid{false};
  std::chrono::steady_clock::time_point receipt_time{};
  uint64_t continuity_generation{0};
  std::vector<WaterDepthSample> samples_since_last_tick{};
};

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
  uint32_t current_run_id{0};
  uint64_t current_run_epoch{0};
  bool depth_receipt_valid{false};
  std::chrono::steady_clock::time_point depth_receipt_time{};
  uint64_t depth_continuity_generation{0};
  std::vector<double> depth_samples_since_last_tick{};
  WaterDepthTelemetry water_depth;

  // Vehicle status
  bool status_valid{false};
  rclcpp::Time status_stamp{};
  bool fcu_connected{false};
  bool armed{false};
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

  // Best POR detection. The stamp is the receipt/header stamp used by the
  // execution watchdog; controllers must not command from stale imagery.
  bool detection_valid{false};
  rclcpp::Time detection_stamp{};
  banyu_msgs::msg::Detection detection{};
};

inline bool detectionIsFresh(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double timeout_s,
  int64_t object_id, banyu_msgs::msg::Detection * result = nullptr)
{
  if (!telemetry.detection_valid || telemetry.detection.object_id != object_id ||
    telemetry.detection_stamp.nanoseconds() == 0 || !std::isfinite(timeout_s) || timeout_s < 0.0)
  {
    return false;
  }
  const double age = (now - telemetry.detection_stamp).seconds();
  if (age < 0.0 || age > timeout_s) {
    return false;
  }
  if (result != nullptr) {
    *result = telemetry.detection;
  }
  return true;
}

inline double detectionAge(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now)
{
  if (!telemetry.detection_valid || telemetry.detection_stamp.nanoseconds() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return (now - telemetry.detection_stamp).seconds();
}

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
