#ifndef BANYU_MISSION__ACTION__SEARCH_PIPELINE_NODE_HPP_
#define BANYU_MISSION__ACTION__SEARCH_PIPELINE_NODE_HPP_

#include <string>

#include "banyu_msgs/action/search_pipeline.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class SearchPipelineNode final : public ActionClientNode<banyu_msgs::action::SearchPipeline>
{
public:
  using Base = ActionClientNode<banyu_msgs::action::SearchPipeline>;
  using Result = Base::Result;

  static bool semanticResultSuccess(const Result & result);
  static BT::PortsList providedPorts();

  SearchPipelineNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
  bool validateResult(const Goal &, const Result & result) const override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__SEARCH_PIPELINE_NODE_HPP_
