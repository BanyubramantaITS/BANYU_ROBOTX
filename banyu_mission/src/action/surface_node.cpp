#include "banyu_mission/action/surface_node.hpp"

#include <utility>

namespace banyu_mission
{

BT::PortsList SurfaceNode::providedPorts()
{
  return {BT::InputPort<double>("target_depth"), BT::InputPort<double>("depth_tolerance")};
}

SurfaceNode::SurfaceNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

SurfaceNode::Goal SurfaceNode::makeGoal()
{
  Goal goal;
  goal.target_depth_m = input<double>("target_depth");
  goal.depth_tolerance_m = input<double>("depth_tolerance");
  return goal;
}

}  // namespace banyu_mission
