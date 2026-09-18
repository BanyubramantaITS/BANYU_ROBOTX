#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SURFACE_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SURFACE_SERVER_HPP_

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
    double max_depth_rate = 0.5);

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  double depth_kp_{0.5};
  double max_depth_rate_{0.5};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SURFACE_SERVER_HPP_
