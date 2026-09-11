#ifndef BANYU_MISSION_BT_NODES_VISUAL_ALIGN_HPP_
#define BANYU_MISSION_BT_NODES_VISUAL_ALIGN_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class VisualAlign : public BT::StatefulActionNode {
 public:
  VisualAlign(const std::string& name, const BT::NodeConfig& config,
              const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx,
              int object_id, const std::string& prefix);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts() { return {}; }

 protected:
  double param(const std::string& name) const;

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  int object_id_;
  std::string prefix_;
  rclcpp::Time start_time_;
  rclcpp::Time aligned_since_;
  bool alignment_started_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_VISUAL_ALIGN_HPP_
