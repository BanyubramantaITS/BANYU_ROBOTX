#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SURFACE_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SURFACE_SERVER_HPP_

#include <chrono>
#include <string>

#include "banyu_msgs/action/surface.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class SurfaceServer final
  : public TypedActionServer<banyu_msgs::action::Surface>
{
public:
  SurfaceServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator,
    double depth_kp = 0.5,
    double max_depth_rate = 0.5,
    bool require_calibrated_depth = false,
    double hold_time_s = 1.0,
    double depth_timeout_s = 0.5,
    double max_surface_depth_m = 0.2);

  // With no depth (no depth sensor, or it failed and the depth comes from the GPS surface proof): rise at this rate until
  // the surface is proven, for at most timeout_s. Rate 0 keeps the old behaviour (wait, then the watchdog aborts).
  void setBlindRise(double rate_mps, double timeout_s);

  // Goals with hold_for_zones wait at depth, for at most timeout_s in all, while the last known position is inside a
  // Task 4 zone; after that they ascend anyway (the report must not be lost).
  void setZoneHold(double timeout_s) {zone_hold_timeout_s_ = timeout_s;}

  void onGoalStarted() override;
  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  double depth_kp_{0.5};
  double max_depth_rate_{0.5};
  bool require_calibrated_depth_{false};
  double hold_time_s_{1.0};
  double depth_timeout_s_{0.5};
  double max_surface_depth_m_{0.2};
  std::chrono::steady_clock::time_point last_valid_depth_time_{};
  std::chrono::steady_clock::time_point hold_start_time_{};
  bool hold_active_{false};
  uint64_t hold_depth_generation_{0};
  int64_t hold_first_source_stamp_ns_{0};
  double blind_rise_rate_{0.0};
  double blind_timeout_s_{0.0};
  double blind_s_{0.0};
  double zone_hold_timeout_s_{60.0};
  double zone_hold_s_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SURFACE_SERVER_HPP_
