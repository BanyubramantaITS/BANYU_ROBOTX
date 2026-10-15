#include "banyu_mission/action/navigate_surface_gps_node.hpp"

#include <utility>

namespace banyu_mission
{

BT::PortsList NavigateSurfaceGpsNode::providedPorts()
{
  return {
    BT::InputPort<double>("target_latitude"), BT::InputPort<double>("target_longitude"),
    BT::InputPort<double>("start_latitude"), BT::InputPort<double>("start_longitude"),
    BT::InputPort<double>("tolerance")};
}

NavigateSurfaceGpsNode::NavigateSurfaceGpsNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

NavigateSurfaceGpsNode::Goal NavigateSurfaceGpsNode::makeGoal()
{
  Goal goal;
  goal.target_latitude_deg = input<double>("target_latitude");
  goal.target_longitude_deg = input<double>("target_longitude");
  goal.start_latitude_deg = input<double>("start_latitude");
  goal.start_longitude_deg = input<double>("start_longitude");
  goal.tolerance_m = input<double>("tolerance");
  return goal;
}

}  // namespace banyu_mission
