#ifndef BANYU_VEHICLE_EXECUTION__CALIBRATED_DEPTH_NODE_HPP_
#define BANYU_VEHICLE_EXECUTION__CALIBRATED_DEPTH_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace banyu_vehicle_execution
{

class CalibratedDepthNode : public rclcpp::Node
{
public:
  explicit CalibratedDepthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CalibratedDepthNode() = default;

  bool isCalibrated() const;
  double surfaceDatumOffset() const;
  uint32_t runId() const;
  uint64_t epoch() const;
  void calibrateDatum(double surface_rel_alt);

private:
  void handleRelAlt(const std_msgs::msg::Float64::SharedPtr msg);
  void handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg);

  mutable std::mutex mutex_;
  std::string source_id_;
  std::string datum_;
  std::string calibration_id_;
  double surface_datum_offset_{0.0};
  bool auto_calibrate_at_start_{true};
  bool calibrated_{false};
  // Datum captured at the surface before the current run went active.
  bool pre_start_calibration_{false};
  double max_rel_alt_age_s_{1.0};

  uint32_t run_id_{0};
  uint64_t epoch_{0};
  uint8_t current_phase_{banyu_msgs::msg::RunLifecycle::IDLE};
  double last_rel_alt_{0.0};
  rclcpp::Time last_rel_alt_time_{0, 0, RCL_ROS_TIME};
  bool rel_alt_received_{false};
  bool been_deep_{false};   // since the datum was dropped, the vehicle has been below the surface band

  bool isNearSurface(double alt) const;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;
  rclcpp::Subscription<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_sub_;
  rclcpp::Publisher<banyu_msgs::msg::WaterDepth>::SharedPtr depth_pub_;
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__CALIBRATED_DEPTH_NODE_HPP_
