#include "banyu_mission/action/hold_probe_on_ring_node.hpp"

#include <utility>

namespace banyu_mission
{

BT::PortsList HoldProbeOnRingNode::providedPorts()
{
  return {BT::InputPort<double>("hold_duration"), BT::InputPort<double>("alignment_tolerance")};
}

HoldProbeOnRingNode::HoldProbeOnRingNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

HoldProbeOnRingNode::Goal HoldProbeOnRingNode::makeGoal()
{
  Goal goal;
  goal.hold_duration_s = input<double>("hold_duration");
  goal.alignment_tolerance = input<double>("alignment_tolerance");
  return goal;
}

}  // namespace banyu_mission
