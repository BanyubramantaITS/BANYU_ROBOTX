#ifndef BANYU_MISSION__ACTION__ACKNOWLEDGE_RED_RING_NODE_HPP_
#define BANYU_MISSION__ACTION__ACKNOWLEDGE_RED_RING_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "banyu_msgs/srv/acknowledge_ring_repair.hpp"
#include "banyu_mission/mission_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace banyu_mission
{

class AcknowledgeRedRingNode final : public BT::StatefulActionNode
{
public:
  AcknowledgeRedRingNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, std::shared_ptr<MissionState> state,
    std::string service_name);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionState> state_;
  rclcpp::Client<banyu_msgs::srv::AcknowledgeRingRepair>::SharedPtr client_;
  std::mutex mutex_;
  uint64_t generation_{0};
  uint32_t request_index_{0};
  bool done_{false};
  bool success_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__ACKNOWLEDGE_RED_RING_NODE_HPP_
