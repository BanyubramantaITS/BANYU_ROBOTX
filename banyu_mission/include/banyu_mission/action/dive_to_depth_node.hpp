#ifndef BANYU_MISSION__ACTION__DIVE_TO_DEPTH_NODE_HPP_
#define BANYU_MISSION__ACTION__DIVE_TO_DEPTH_NODE_HPP_

#include <string>

#include "banyu_msgs/action/dive_to_depth.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class DiveToDepthNode final : public ActionClientNode<banyu_msgs::action::DiveToDepth>
{
public:
  static BT::PortsList providedPorts();

  DiveToDepthNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__DIVE_TO_DEPTH_NODE_HPP_
