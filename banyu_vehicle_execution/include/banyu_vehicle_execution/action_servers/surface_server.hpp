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
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SURFACE_SERVER_HPP_
