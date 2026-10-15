#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ALIGN_HEADING_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ALIGN_HEADING_SERVER_HPP_

#include <string>

#include "banyu_msgs/action/align_heading.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class AlignHeadingServer final
  : public TypedActionServer<banyu_msgs::action::AlignHeading>
{
public:
  AlignHeadingServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator,
    double yaw_kp = 1.0,
    double max_yaw_rate = 0.5);

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  double yaw_kp_{1.0};
  double max_yaw_rate_{0.5};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ALIGN_HEADING_SERVER_HPP_
