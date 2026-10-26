#include "banyu_mission/action/set_fcu_mode_node.hpp"

#include <utility>

namespace banyu_mission
{

SetFcuModeNode::SetFcuModeNode(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node,
  std::shared_ptr<MissionState> state, std::string set_mode_service)
: BT::StatefulActionNode(name, config), node_(std::move(node)), state_(std::move(state)),
  client_(node_->create_client<mavros_msgs::srv::SetMode>(set_mode_service))
{
}

BT::NodeStatus SetFcuModeNode::onStart()
{
  mode_ = "ALT_HOLD";
  getInput("mode", mode_);
  getInput("timeout_s", timeout_s_);
  started_ = std::chrono::steady_clock::now();
  if (state_->snapshot().mode == mode_) {return BT::NodeStatus::SUCCESS;}
  sendRequest();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetFcuModeNode::onRunning()
{
  const auto now = std::chrono::steady_clock::now();
  if (state_->snapshot().mode == mode_) {
    RCLCPP_INFO(node_->get_logger(), "flight mode is %s", mode_.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  if (std::chrono::duration<double>(now - started_).count() > timeout_s_) {
    RCLCPP_ERROR(
      node_->get_logger(), "flight mode %s not reached in %.1f s (still %s)", mode_.c_str(),
      timeout_s_, state_->snapshot().mode.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (std::chrono::duration<double>(now - last_request_).count() > 1.0) {sendRequest();}
  return BT::NodeStatus::RUNNING;
}

void SetFcuModeNode::onHalted() {}

void SetFcuModeNode::sendRequest()
{
  last_request_ = std::chrono::steady_clock::now();
  if (!client_->wait_for_service(std::chrono::milliseconds(0))) {return;}
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode_;
  client_->async_send_request(request);
}

}  // namespace banyu_mission
