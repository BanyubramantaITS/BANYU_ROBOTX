#include "banyu_mission/action/acknowledge_red_ring_node.hpp"

#include <utility>

namespace banyu_mission
{

AcknowledgeRedRingNode::AcknowledgeRedRingNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, std::shared_ptr<MissionState> state,
  std::string service_name)
: BT::StatefulActionNode(name, config), node_(std::move(node)), state_(std::move(state)),
  client_(node_->create_client<banyu_msgs::srv::AcknowledgeRingRepair>(service_name))
{
}

BT::NodeStatus AcknowledgeRedRingNode::onStart()
{
  const uint32_t index = state_->pendingRedIndex();
  if (!state_->redPending()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
    RCLCPP_WARN(node_->get_logger(), "ring repair acknowledgement service unavailable");
    return BT::NodeStatus::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation_++;
    request_index_ = index;
    done_ = false;
    success_ = false;
    const auto generation = generation_;
    auto request = std::make_shared<banyu_msgs::srv::AcknowledgeRingRepair::Request>();
    request->ring_index = index;
    const auto context = state_->snapshot();
    request->run_id = context.run_id;
    request->epoch = context.epoch;
    client_->async_send_request(
      request, [this, generation, run_id = context.run_id, epoch = context.epoch](
        rclcpp::Client<banyu_msgs::srv::AcknowledgeRingRepair>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> callback_lock(mutex_);
        const auto current = state_->snapshot();
        if (generation != generation_ || current.run_id != run_id || current.epoch != epoch) {
          return;
        }
        done_ = true;
        success_ = response->success && response->accepted_ring_index == request_index_;
        if (success_) {
          state_->acknowledgeRed(request_index_);
        }
      });
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AcknowledgeRedRingNode::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!done_) {
    return BT::NodeStatus::RUNNING;
  }
  return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void AcknowledgeRedRingNode::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_++;
}

}  // namespace banyu_mission
