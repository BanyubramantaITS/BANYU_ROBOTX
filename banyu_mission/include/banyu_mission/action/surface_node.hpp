#ifndef BANYU_MISSION__ACTION__SURFACE_NODE_HPP_
#define BANYU_MISSION__ACTION__SURFACE_NODE_HPP_

#include <string>

#include "banyu_msgs/action/surface.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class SurfaceNode final : public ActionClientNode<banyu_msgs::action::Surface>
{
public:
  static BT::PortsList providedPorts();

  SurfaceNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__SURFACE_NODE_HPP_
