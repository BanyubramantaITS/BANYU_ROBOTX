// Base for the Task 2 one-shot BT nodes.
#ifndef BANYU_MISSION_TASK2_SYNC_NODES_BASE_HPP_
#define BANYU_MISSION_TASK2_SYNC_NODES_BASE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission::task2 {

class Task2SyncAction : public BT::SyncActionNode {
 public:
  Task2SyncAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node,
                  std::shared_ptr<MissionContext> ctx)
      : BT::SyncActionNode(name, config), node_(std::move(node)), ctx_(std::move(ctx)) {}

 protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_SYNC_NODES_BASE_HPP_
