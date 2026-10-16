#include "banyu_mission/action/align_heading_node.hpp"

#include <utility>

namespace banyu_mission
{

BT::PortsList AlignHeadingNode::providedPorts()
{
  return {BT::InputPort<double>("target_heading"), BT::InputPort<double>("tolerance")};
}

AlignHeadingNode::AlignHeadingNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

AlignHeadingNode::Goal AlignHeadingNode::makeGoal()
{
  Goal goal;
  goal.target_heading_rad = input<double>("target_heading");
  goal.tolerance_rad = input<double>("tolerance");
  return goal;
}

}  // namespace banyu_mission
