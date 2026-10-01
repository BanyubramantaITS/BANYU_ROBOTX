#include "banyu_mission/action/follow_pipeline_node.hpp"

#include <utility>

namespace banyu_mission
{

bool FollowPipelineNode::semanticResultSuccess(const Result & result, bool expected_enable)
{
  return result.success && result.enabled == expected_enable;
}

BT::PortsList FollowPipelineNode::providedPorts()
{
  return {BT::InputPort<bool>("enable")};
}

FollowPipelineNode::FollowPipelineNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

FollowPipelineNode::Goal FollowPipelineNode::makeGoal()
{
  Goal goal;
  goal.enable = input<bool>("enable");
  return goal;
}

bool FollowPipelineNode::validateResult(const Goal & goal, const Result & result) const
{
  return semanticResultSuccess(result, goal.enable);
}

}  // namespace banyu_mission
