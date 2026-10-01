#include "banyu_mission/action/dive_to_depth_node.hpp"

#include <utility>

namespace banyu_mission
{

BT::PortsList DiveToDepthNode::providedPorts()
{
  return {
    BT::InputPort<double>("target_depth"), BT::InputPort<double>("target_heading"),
    BT::InputPort<double>("depth_tolerance"), BT::InputPort<double>("heading_tolerance")};
}

DiveToDepthNode::DiveToDepthNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

DiveToDepthNode::Goal DiveToDepthNode::makeGoal()
{
  Goal goal;
  goal.target_depth_m = input<double>("target_depth");
  goal.target_heading_rad = input<double>("target_heading");
  goal.depth_tolerance_m = input<double>("depth_tolerance");
  goal.heading_tolerance_rad = input<double>("heading_tolerance");
  return goal;
}

}  // namespace banyu_mission
