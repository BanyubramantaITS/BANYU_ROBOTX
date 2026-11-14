#include "banyu_mission/action/decode_resource_code_node.hpp"

#include <chrono>
#include <utility>

namespace banyu_mission
{

DecodeResourceCodeNode::DecodeResourceCodeNode(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node,
  std::shared_ptr<MissionState> state, std::string window_service, std::string result_topic)
: BT::StatefulActionNode(name, config), node_(std::move(node)), state_(std::move(state)),
  client_(node_->create_client<std_srvs::srv::SetBool>(window_service)),
  started_(0, 0, node_->get_clock()->get_clock_type())
{
  sub_ = node_->create_subscription<banyu_msgs::msg::ResourceCode>(
    result_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](const banyu_msgs::msg::ResourceCode::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      // The topic is latched: ignore results from an earlier window or run.
      if (!open_ || msg->run_id != run_id_ || msg->epoch != epoch_ ||
      rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type()) < started_)
      {
        return;
      }
      complete_ = msg->complete;
    });
}

BT::NodeStatus DecodeResourceCodeNode::onStart()
{
  double timeout = 40.0;
  getInput("timeout_s", timeout);
  const auto context = state_->snapshot();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_s_ = timeout;
    run_id_ = context.run_id;
    epoch_ = context.epoch;
    started_ = node_->now();
    complete_ = false;
    open_ = true;
  }
  if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
    RCLCPP_WARN(node_->get_logger(), "decode window service unavailable; skipping the flash code");
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  client_->async_send_request(request);
  RCLCPP_INFO(node_->get_logger(), "decoding the flash code (timeout %.0f s)", timeout);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DecodeResourceCodeNode::onRunning()
{
  bool done = false;
  bool timed_out = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    done = complete_;
    timed_out = (node_->now() - started_).seconds() >= timeout_s_;
  }
  if (!done && !timed_out) {
    return BT::NodeStatus::RUNNING;
  }
  RCLCPP_INFO(
    node_->get_logger(), done ? "flash code read" : "flash code not read before the timeout");
  closeWindow();
  return BT::NodeStatus::SUCCESS;
}

void DecodeResourceCodeNode::onHalted()
{
  closeWindow();
}

void DecodeResourceCodeNode::closeWindow()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) {return;}
    open_ = false;
  }
  if (client_->wait_for_service(std::chrono::milliseconds(0))) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    client_->async_send_request(request);
  }
}

}  // namespace banyu_mission
