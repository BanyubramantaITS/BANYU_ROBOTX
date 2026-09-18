#ifndef BANYU_MISSION__ACTION__ALIGN_HEADING_NODE_HPP_
#define BANYU_MISSION__ACTION__ALIGN_HEADING_NODE_HPP_

#include <string>

#include "banyu_msgs/action/align_heading.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class AlignHeadingNode final : public ActionClientNode<banyu_msgs::action::AlignHeading>
{
public:
  static BT::PortsList providedPorts();

  AlignHeadingNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__ALIGN_HEADING_NODE_HPP_
