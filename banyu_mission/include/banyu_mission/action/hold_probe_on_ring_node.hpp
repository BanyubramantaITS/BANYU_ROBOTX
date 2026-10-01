#ifndef BANYU_MISSION__ACTION__HOLD_PROBE_ON_RING_NODE_HPP_
#define BANYU_MISSION__ACTION__HOLD_PROBE_ON_RING_NODE_HPP_

#include <string>

#include "banyu_msgs/action/hold_probe_on_ring.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class HoldProbeOnRingNode final : public ActionClientNode<banyu_msgs::action::HoldProbeOnRing>
{
public:
  static BT::PortsList providedPorts();

  HoldProbeOnRingNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__HOLD_PROBE_ON_RING_NODE_HPP_
