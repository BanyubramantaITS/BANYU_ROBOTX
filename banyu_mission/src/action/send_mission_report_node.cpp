#include "banyu_mission/action/send_mission_report_node.hpp"

#include <utility>

namespace banyu_mission
{

bool SendMissionReportNode::semanticResultSuccess(const Result & result, uint32_t expected_crc)
{
  return result.success && result.acknowledged && result.acknowledged_crc32 == expected_crc;
}

BT::PortsList SendMissionReportNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("mission_id"),
    BT::InputPort<std::vector<uint8_t>>("ring_order"),
    BT::InputPort<uint32_t>("crc32"),
    BT::InputPort<uint8_t>("max_retries")};
}

SendMissionReportNode::SendMissionReportNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

SendMissionReportNode::Goal SendMissionReportNode::makeGoal()
{
  Goal goal;
  goal.mission_id = input<std::string>("mission_id");
  goal.ring_order = input<std::vector<uint8_t>>("ring_order");
  goal.crc32 = input<uint32_t>("crc32");
  goal.max_retries = input<uint8_t>("max_retries");
  const auto stamp = node_->now();
  goal.report_time.sec = static_cast<int32_t>(stamp.nanoseconds() / 1000000000LL);
  goal.report_time.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
  return goal;
}

bool SendMissionReportNode::validateResult(const Goal & goal, const Result & result) const
{
  return semanticResultSuccess(result, goal.crc32);
}

}  // namespace banyu_mission
