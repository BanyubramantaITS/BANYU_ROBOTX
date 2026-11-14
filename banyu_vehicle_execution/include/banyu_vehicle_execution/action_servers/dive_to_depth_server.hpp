#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__DIVE_TO_DEPTH_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__DIVE_TO_DEPTH_SERVER_HPP_

#include <string>

#include "banyu_msgs/action/dive_to_depth.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class DiveToDepthServer final
  : public TypedActionServer<banyu_msgs::action::DiveToDepth>
{
public:
  DiveToDepthServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator,
    double depth_kp = 0.5,
    double max_depth_rate = 0.5,
    double yaw_kp = 1.0,
    double max_yaw_rate = 0.5);

  // No depth sensor: sink at `rate_mps` holding the heading until the camera has the pipe (with a range from its width)
  // or a lit light for `lock_s`; fail after `max_time_s` (the dive's depth limit at that rate).
  // surge_mps: creep forward along the target heading while sinking (the dive point, the start buoy, is beyond the pipe's end).
  void setCameraDescent(double rate_mps, double lock_s, double max_time_s, double surge_mps = 0.0);
  // With a depth sensor (sensors:=baro): the camera descent also ends the dive, and the measured depth is a floor:
  // past `max_depth_m` with nothing in view the dive fails.
  void setCameraDescentWithDepth(double max_depth_m);

  void onGoalStarted() override;
  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  MotionIntent cameraDescentTick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now, double dt, const Goal & goal);

  double depth_kp_{0.5};
  double max_depth_rate_{0.5};
  double yaw_kp_{1.0};
  double max_yaw_rate_{0.5};
  double camera_rate_mps_{0.15};
  double camera_lock_s_{1.0};
  double camera_max_time_s_{100.0};
  double camera_surge_mps_{0.0};
  bool camera_with_depth_{false};
  double camera_max_depth_m_{15.0};
  double elapsed_s_{0.0};
  double locked_s_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__DIVE_TO_DEPTH_SERVER_HPP_
