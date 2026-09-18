#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__CENTER_ON_RING_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__CENTER_ON_RING_SERVER_HPP_

#include <string>

#include "banyu_msgs/action/center_on_ring.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class CenterOnRingServer final
  : public TypedActionServer<banyu_msgs::action::CenterOnRing>
{
public:
  CenterOnRingServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator,
    double ring_kp_sway = 0.8,
    double ring_kp_z = 0.5);

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  double ring_kp_sway_{0.8};
  double ring_kp_z_{0.5};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__CENTER_ON_RING_SERVER_HPP_
