#include "banyu_mission/action/hold_probe_on_ring_node.hpp"

#include <utility>

namespace banyu_mission
{

BT::PortsList HoldProbeOnRingNode::providedPorts()
{
  return {BT::InputPort<double>("hold_duration"), BT::InputPort<double>("alignment_tolerance"),
    BT::InputPort<double>("descent_rate", 0.0, "m/s; 0 holds the depth, above 0 sinks onto the ring"),
    BT::InputPort<double>("contact_ring_size", 0.0, "ring size (image width fraction) at contact"),
    BT::InputPort<double>("max_descent", 0.0, "m, cap on the sink"),
    BT::InputPort<bool>("climb_back", true, "climb back to the starting depth after the repair")};
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
  goal.descent_rate_mps = input<double>("descent_rate");
  goal.contact_ring_size = input<double>("contact_ring_size");
  goal.max_descent_m = input<double>("max_descent");
  goal.climb_back = input<bool>("climb_back");
  return goal;
}

}  // namespace banyu_mission
