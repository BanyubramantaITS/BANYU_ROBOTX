#ifndef BANYU_VEHICLE_EXECUTION__VEHICLE_TELEMETRY_HPP_
#define BANYU_VEHICLE_EXECUTION__VEHICLE_TELEMETRY_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "banyu_msgs/msg/detection.hpp"
#include "banyu_vehicle_execution/task4_zones.hpp"

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

  // IMU attitude (ENU yaw). Heading source for camera-guided POR actions when
  // the local pose is unavailable (no DVL/position estimate on the vehicle).
  bool imu_valid{false};
  rclcpp::Time imu_stamp{};
  double imu_yaw{0.0};

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
  uint8_t operational_mode{0};
  bool hardware_kill_active{false};

  // Pipeline observation
  bool pipeline_obs_valid{false};
  rclcpp::Time pipeline_obs_stamp{};
  uint8_t pipeline_status{0};
  double pipeline_offset_x{0.0};
  double pipeline_offset_y{0.0};
  double pipeline_heading_error_rad{0.0};
  double pipeline_confidence{0.0};
  double pipeline_width{std::numeric_limits<double>::quiet_NaN()};

  // Depth-free mode: no depth sensor. heave_odom_m integrates the commanded heave (down positive) so actions can make
  // relative vertical moves; camera_range_m is the range to the pipe from its apparent width.
  bool depth_free{false};
  double heave_odom_m{0.0};
  bool camera_range_valid{false};
  double camera_range_m{0.0};

  // Ring observation
  bool ring_obs_valid{false};
  rclcpp::Time ring_obs_stamp{};
  bool ring_visible{false};
  uint8_t ring_color{0};
  double ring_offset_x{0.0};
  double ring_offset_y{0.0};
  double ring_confidence{0.0};
  double ring_size{0.0};

  // Light seen with no pipe in view (detector light_cue topic): a steering cue only, never a ring record.
  bool light_cue_valid{false};
  rclcpp::Time light_cue_stamp{};
  bool light_cue_visible{false};
  uint8_t light_cue_color{0};
  double light_cue_offset_x{0.0};
  double light_cue_offset_y{0.0};
  double light_cue_confidence{0.0};

  // Task 4 keep-out zones (RoboCommand), run-scoped; the moving object is predicted from its receipt time.
  ZoneField zones;
  rclcpp::Time zones_moving_stamp{};
  std::vector<double> keep_out_latitude_deg{};
  std::vector<double> keep_out_longitude_deg{};
  double keep_out_radius_m{0.0};

  // Zones as capsules around the last known position (the last GPS fix when under water).
  std::vector<Capsule> zonesAroundVehicle(const rclcpp::Time & now) const
  {
    const double age = zones_moving_stamp.nanoseconds() != 0 ? (now - zones_moving_stamp).seconds() : 0.0;
    return zonesAround(zones, latitude_deg, longitude_deg, std::max(0.0, age));
  }

  // Fixed surface obstacles per source (e.g. the UAV's Task 1 buoy field); NavigateSurfaceGps detours round them.
  struct SurfaceObstacle
  {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double radius_m{0.0};
  };
  std::map<std::string, std::vector<SurfaceObstacle>> surface_obstacles{};

  std::vector<Capsule> surfaceObstaclesAroundVehicle() const
  {
    std::vector<Capsule> discs;
    for (const auto & [source, list] : surface_obstacles) {
      for (const auto & o : list) {
        const Vec2 c = enuOffset(latitude_deg, longitude_deg, o.latitude_deg, o.longitude_deg);
        discs.push_back(Capsule{c, c, o.radius_m, false});
      }
    }
    return discs;
  }

  // Known buoys as planar ENU offsets from the vehicle.
  std::vector<Vec2> buoysAroundVehicle() const
  {
    std::vector<Vec2> buoys;
    const std::size_t count = std::min(keep_out_latitude_deg.size(), keep_out_longitude_deg.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (!std::isfinite(keep_out_latitude_deg[i]) || !std::isfinite(keep_out_longitude_deg[i])) {
        continue;
      }
      buoys.push_back(enuOffset(latitude_deg, longitude_deg, keep_out_latitude_deg[i], keep_out_longitude_deg[i]));
    }
    return buoys;
  }

  // Guidance velocity intent (for FollowPipeline)
  bool guidance_intent_valid{false};
  rclcpp::Time guidance_intent_stamp{};
  uint8_t guidance_status{0};
  double guidance_surge_mps{0.0};
  double guidance_sway_mps{0.0};
  double guidance_yaw_rate_rad_s{0.0};
  bool guidance_yaw_control_active{false};

  // Pinger observation
  bool pinger_obs_valid{false};
  rclcpp::Time pinger_obs_stamp{};
  double pinger_forward{0.0};
  double pinger_left{0.0};
  double pinger_bearing_rad{0.0};

  // Best POR detection. The stamp is the receipt/header stamp used by the
  // execution watchdog; controllers must not command from stale imagery.
  bool detection_valid{false};
  rclcpp::Time detection_stamp{};
  banyu_msgs::msg::Detection detection{};

  // Latest detection per POR object id (0 = gate, 1 = marker), so both targets
  // stay available when the detector publishes them in the same frame.
  static constexpr int kDetectionSlots = 2;
  bool detection_slot_valid[kDetectionSlots]{false, false};
  rclcpp::Time detection_slot_stamp[kDetectionSlots]{};
  banyu_msgs::msg::Detection detection_slot[kDetectionSlots]{};
};

inline bool detectionIsFresh(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double timeout_s,
  int64_t object_id = -1, banyu_msgs::msg::Detection * result = nullptr)
{
  if (object_id >= 0 && object_id < VehicleTelemetry::kDetectionSlots &&
    telemetry.detection_slot_valid[object_id])
  {
    const auto & stamp = telemetry.detection_slot_stamp[object_id];
    if (stamp.nanoseconds() == 0 || !std::isfinite(timeout_s) || timeout_s < 0.0) {
      return false;
    }
    const double slot_age = (now - stamp).seconds();
    if (slot_age < 0.0 || slot_age > timeout_s) {
      return false;
    }
    if (result != nullptr) {
      *result = telemetry.detection_slot[object_id];
    }
    return true;
  }
  if (object_id < 0) {
    for (int i = 0; i < VehicleTelemetry::kDetectionSlots; ++i) {
      if (telemetry.detection_slot_valid[i]) {
        const auto & stamp = telemetry.detection_slot_stamp[i];
        if (stamp.nanoseconds() != 0 && std::isfinite(timeout_s) && timeout_s >= 0.0) {
          const double slot_age = (now - stamp).seconds();
          if (slot_age >= 0.0 && slot_age <= timeout_s) {
            if (result != nullptr) {
              *result = telemetry.detection_slot[i];
            }
            return true;
          }
        }
      }
    }
  }
  if (!telemetry.detection_valid ||
    (object_id >= 0 && telemetry.detection.object_id != object_id) ||
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

inline double detectionAge(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, int64_t object_id)
{
  if (object_id >= 0 && object_id < VehicleTelemetry::kDetectionSlots &&
    telemetry.detection_slot_valid[object_id] &&
    telemetry.detection_slot_stamp[object_id].nanoseconds() != 0)
  {
    return (now - telemetry.detection_slot_stamp[object_id]).seconds();
  }
  return detectionAge(telemetry, now);
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
