#include "banyu_mission/action/center_on_ring_node.hpp"

#include <utility>

namespace banyu_mission
{

BT::PortsList CenterOnRingNode::providedPorts()
{
  return {BT::InputPort<std::string>("ring_color"), BT::InputPort<double>("tolerance")};
}

CenterOnRingNode::CenterOnRingNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

CenterOnRingNode::Goal CenterOnRingNode::makeGoal()
{
  Goal goal;
  goal.ring_color = input<std::string>("ring_color");
  goal.tolerance = input<double>("tolerance");
  return goal;
}

}  // namespace banyu_mission
