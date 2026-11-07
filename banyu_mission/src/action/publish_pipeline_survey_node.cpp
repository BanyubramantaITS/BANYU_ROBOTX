#include "banyu_mission/action/publish_pipeline_survey_node.hpp"

#include <cmath>
#include <utility>

namespace banyu_mission
{

PublishPipelineSurveyNode::PublishPipelineSurveyNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, std::shared_ptr<MissionState> state, std::string service_name)
: BT::StatefulActionNode(name, config), node_(std::move(node)), state_(std::move(state)),
  client_(node_->create_client<banyu_msgs::srv::TriggerPipelineSurvey>(service_name))
{
}

BT::NodeStatus PublishPipelineSurveyNode::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    start_time_ = std::chrono::steady_clock::now();
    retry_at_ = start_time_;
    const auto context = state_->snapshot();
    run_id_ = context.run_id;
    epoch_ = context.epoch;
    generation_++;
    done_ = false;
    success_ = false;
    request_sent_ = false;
    retryable_ = false;
    best_effort_ = getInput<bool>("best_effort").value_or(false);
    const double timeout_s = getInput<double>("timeout_s").value_or(5.0);
    timeout_ = std::chrono::duration<double>(std::isfinite(timeout_s) && timeout_s > 0.0 ? timeout_s : 5.0);
  }
  return onRunning();
}

BT::NodeStatus PublishPipelineSurveyNode::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto context = state_->snapshot();
  if (run_id_ == 0 || epoch_ == 0 || context.run_id != run_id_ || context.epoch != epoch_ ||
    state_->lifecycle() != MissionState::Lifecycle::Active)
  {return BT::NodeStatus::FAILURE;}
  const auto current_time = std::chrono::steady_clock::now();
  if (current_time - start_time_ > timeout_) {
    generation_++;
    client_->prune_pending_requests();
    RCLCPP_WARN(node_->get_logger(), "Pipeline survey local acceptance timed out");
    return BT::NodeStatus::FAILURE;
  }
  if (done_) {
    if (success_) {return BT::NodeStatus::SUCCESS;}
    if (!retryable_) {return BT::NodeStatus::FAILURE;}
    done_ = false;
    request_sent_ = false;
    retry_at_ = current_time + std::chrono::milliseconds(100);
  }
  if (!request_sent_ && current_time >= retry_at_ && client_->service_is_ready()) {
    auto request = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
    request->run_id = run_id_;
    request->epoch = epoch_;
    request->report_id = "task2-survey-" + std::to_string(run_id_) + "-" + std::to_string(epoch_) +
      (best_effort_ ? ".best_effort" : "");
    const auto generation = generation_;
    const auto report_id = request->report_id;
    request_sent_ = true;
    client_->async_send_request(request, [this, generation, report_id](
        rclcpp::Client<banyu_msgs::srv::TriggerPipelineSurvey>::SharedFuture future) {
        std::lock_guard<std::mutex> callback_lock(mutex_);
        if (generation != generation_) {return;}
        done_ = true;
        try {
          const auto response = future.get();
          success_ = response->success && response->report_id == report_id;
          using Response = banyu_msgs::srv::TriggerPipelineSurvey::Response;
          retryable_ = response->result_code == Response::NOT_SURFACED ||
          response->result_code == Response::SURFACE_TELEMETRY_STALE ||
          response->result_code == Response::SURVEY_INVALID ||
          response->result_code == Response::RUN_NOT_ACTIVE ||
          response->result_code == Response::AUTHORIZATION_REJECTED;
        } catch (const std::exception & error) {
          success_ = false;
          retryable_ = false;
          RCLCPP_WARN(node_->get_logger(), "Report service failed: %s", error.what());
        }
      });
  }
  return BT::NodeStatus::RUNNING;
}

void PublishPipelineSurveyNode::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_++;
  client_->prune_pending_requests();
}

}  // namespace banyu_mission
