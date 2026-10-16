#include "banyu_mission/action/acquire_pipeline_node.hpp"

#include <utility>

namespace banyu_mission
{

bool AcquirePipelineNode::semanticResultSuccess(const Result & result)
{
  return result.success && result.acquired;
}

BT::PortsList AcquirePipelineNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("camera_name"), BT::InputPort<bool>("use_heading_prior"),
    BT::InputPort<double>("heading_prior"), BT::InputPort<double>("heading_tolerance")};
}

AcquirePipelineNode::AcquirePipelineNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

AcquirePipelineNode::Goal AcquirePipelineNode::makeGoal()
{
  Goal goal;
  goal.camera_name = input<std::string>("camera_name");
  goal.use_heading_prior = input<bool>("use_heading_prior");
  goal.heading_prior_rad = input<double>("heading_prior");
  goal.heading_prior_tolerance_rad = input<double>("heading_tolerance");
  return goal;
}

bool AcquirePipelineNode::validateResult(const Goal &, const Result & result) const
{
  return semanticResultSuccess(result);
}

}  // namespace banyu_mission
