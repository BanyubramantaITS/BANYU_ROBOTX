#ifndef BANYU_MISSION_BT_NODES_TASK2_REPORT_HPP_
#define BANYU_MISSION_BT_NODES_TASK2_REPORT_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

bool claimTask2Terminal(const std::shared_ptr<MissionContext>& ctx, uint8_t terminal_status);
bool cancelTask2Terminal(const std::shared_ptr<MissionContext>& ctx);

bool publishTask2Report(const std::shared_ptr<MissionContext>& ctx,
                        const rclcpp::Time& stamp, uint8_t terminal_status,
                        const std::string& detail, double source_timeout_s,
                        double future_tolerance_s, bool terminal_claimed = false);

class Task2ReportNode : public BT::StatefulActionNode {
 public:
  Task2ReportNode(const std::string& name, const BT::NodeConfig& config,
                  const rclcpp::Node::SharedPtr& node,
                  std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_TASK2_REPORT_HPP_
