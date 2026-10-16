#include "banyu_mission/action/search_pipeline_node.hpp"

#include <utility>

namespace banyu_mission
{

bool SearchPipelineNode::semanticResultSuccess(const Result & result)
{
  return result.success && result.found;
}

BT::PortsList SearchPipelineNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("camera_name"), BT::InputPort<bool>("use_heading_prior"),
    BT::InputPort<double>("heading_prior"), BT::InputPort<double>("search_span"),
    BT::InputPort<double>("sweep_rate")};
}

SearchPipelineNode::SearchPipelineNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

SearchPipelineNode::Goal SearchPipelineNode::makeGoal()
{
  Goal goal;
  goal.camera_name = input<std::string>("camera_name");
  goal.use_heading_prior = input<bool>("use_heading_prior");
  goal.heading_prior_rad = input<double>("heading_prior");
  goal.search_span_rad = input<double>("search_span");
  goal.sweep_rate_rad_s = input<double>("sweep_rate");
  return goal;
}

bool SearchPipelineNode::validateResult(const Goal &, const Result & result) const
{
  return semanticResultSuccess(result);
}

}  // namespace banyu_mission
