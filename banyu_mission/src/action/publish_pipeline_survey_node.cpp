#include "banyu_mission/action/publish_pipeline_survey_node.hpp"

#include <utility>

namespace banyu_mission
{

PublishPipelineSurveyNode::PublishPipelineSurveyNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, std::string service_name)
: BT::StatefulActionNode(name, config), node_(std::move(node)),
  client_(node_->create_client<banyu_msgs::srv::TriggerPipelineSurvey>(service_name))
{
}

BT::NodeStatus PublishPipelineSurveyNode::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  if (!client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_WARN(node_->get_logger(), "Pipeline survey trigger service unavailable");
    return BT::NodeStatus::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation_++;
    done_ = false;
    success_ = false;
    const auto generation = generation_;
    auto request = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
    client_->async_send_request(
      request, [this, generation](
        rclcpp::Client<banyu_msgs::srv::TriggerPipelineSurvey>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> callback_lock(mutex_);
        if (generation != generation_) {
          return;
        }
        done_ = true;
        success_ = response->success;
      });
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishPipelineSurveyNode::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!done_) {
    if (std::chrono::steady_clock::now() - start_time_ > std::chrono::seconds(5)) {
      RCLCPP_WARN(node_->get_logger(), "Pipeline survey trigger timed out after 5 seconds");
      generation_++;
      client_->prune_pending_requests();
      done_ = true;
      success_ = false;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void PublishPipelineSurveyNode::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_++;
  client_->prune_pending_requests();
}

}  // namespace banyu_mission
