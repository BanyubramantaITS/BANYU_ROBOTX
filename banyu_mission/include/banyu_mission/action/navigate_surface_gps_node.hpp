#ifndef BANYU_MISSION__ACTION__NAVIGATE_SURFACE_GPS_NODE_HPP_
#define BANYU_MISSION__ACTION__NAVIGATE_SURFACE_GPS_NODE_HPP_

#include <string>

#include "banyu_msgs/action/navigate_surface_gps.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class NavigateSurfaceGpsNode final
  : public ActionClientNode<banyu_msgs::action::NavigateSurfaceGps>
{
public:
  static BT::PortsList providedPorts();

  NavigateSurfaceGpsNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__NAVIGATE_SURFACE_GPS_NODE_HPP_
