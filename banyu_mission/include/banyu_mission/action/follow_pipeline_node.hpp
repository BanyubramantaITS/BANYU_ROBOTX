#ifndef BANYU_MISSION__ACTION__FOLLOW_PIPELINE_NODE_HPP_
#define BANYU_MISSION__ACTION__FOLLOW_PIPELINE_NODE_HPP_

#include <string>

#include "banyu_msgs/action/follow_pipeline.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class FollowPipelineNode final : public ActionClientNode<banyu_msgs::action::FollowPipeline>
{
public:
  using Base = ActionClientNode<banyu_msgs::action::FollowPipeline>;
  using Result = Base::Result;

  static bool semanticResultSuccess(const Result & result, bool expected_enable);
  static BT::PortsList providedPorts();

  FollowPipelineNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
  bool validateResult(const Goal & goal, const Result & result) const override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__FOLLOW_PIPELINE_NODE_HPP_
