#ifndef BANYU_MISSION__ACTION__POR_ACTION_NODES_HPP_
#define BANYU_MISSION__ACTION__POR_ACTION_NODES_HPP_

#include <string>

#include "banyu_msgs/action/acquire_gate.hpp"
#include "banyu_msgs/action/align_gate.hpp"
#include "banyu_msgs/action/align_marker.hpp"
#include "banyu_msgs/action/orbit_marker.hpp"
#include "banyu_msgs/action/pass_gate.hpp"
#include "banyu_msgs/action/search_gate.hpp"
#include "banyu_msgs/action/search_marker.hpp"
#include "banyu_msgs/action/turn_around.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

#define BANYU_DECLARE_POR_ACTION_NODE(ClassName, ActionType) \
  class ClassName final : public ActionClientNode<ActionType> \
  { \
public: \
    using Base = ActionClientNode<ActionType>; \
    using Goal = typename Base::Goal; \
    static BT::PortsList providedPorts(); \
    ClassName( \
      const std::string &, const BT::NodeConfig &, rclcpp::Node::SharedPtr, \
      const std::string &); \
protected: \
    Goal makeGoal() override; \
  }

BANYU_DECLARE_POR_ACTION_NODE(AcquireGateNode, banyu_msgs::action::AcquireGate);
BANYU_DECLARE_POR_ACTION_NODE(SearchMarkerNode, banyu_msgs::action::SearchMarker);
BANYU_DECLARE_POR_ACTION_NODE(SearchGateNode, banyu_msgs::action::SearchGate);
BANYU_DECLARE_POR_ACTION_NODE(AlignGateNode, banyu_msgs::action::AlignGate);
BANYU_DECLARE_POR_ACTION_NODE(AlignMarkerNode, banyu_msgs::action::AlignMarker);
BANYU_DECLARE_POR_ACTION_NODE(PassGateNode, banyu_msgs::action::PassGate);
BANYU_DECLARE_POR_ACTION_NODE(OrbitMarkerNode, banyu_msgs::action::OrbitMarker);
BANYU_DECLARE_POR_ACTION_NODE(TurnAroundNode, banyu_msgs::action::TurnAround);

#undef BANYU_DECLARE_POR_ACTION_NODE

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__POR_ACTION_NODES_HPP_
