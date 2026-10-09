#ifndef BANYU_MISSION__ACTION__PUBLISH_PIPELINE_SURVEY_NODE_HPP_
#define BANYU_MISSION__ACTION__PUBLISH_PIPELINE_SURVEY_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "banyu_msgs/srv/trigger_pipeline_survey.hpp"

namespace banyu_mission
{

class PublishPipelineSurveyNode : public BT::StatefulActionNode
{
public:
  PublishPipelineSurveyNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    std::string service_name = "/banyu/communication/trigger_pipeline_survey");

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<banyu_msgs::srv::TriggerPipelineSurvey>::SharedPtr client_;
  std::mutex mutex_;
  uint64_t generation_{0};
  bool done_{false};
  bool success_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__PUBLISH_PIPELINE_SURVEY_NODE_HPP_
