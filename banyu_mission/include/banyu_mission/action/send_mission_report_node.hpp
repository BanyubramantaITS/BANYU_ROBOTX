#ifndef BANYU_MISSION__ACTION__SEND_MISSION_REPORT_NODE_HPP_
#define BANYU_MISSION__ACTION__SEND_MISSION_REPORT_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "banyu_msgs/action/send_mission_report.hpp"
#include "banyu_mission/action/action_client_node.hpp"

namespace banyu_mission
{

class SendMissionReportNode final
  : public ActionClientNode<banyu_msgs::action::SendMissionReport>
{
public:
  using Base = ActionClientNode<banyu_msgs::action::SendMissionReport>;
  using Result = Base::Result;

  static bool semanticResultSuccess(const Result & result, uint32_t expected_crc);
  static BT::PortsList providedPorts();

  SendMissionReportNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action);

protected:
  Goal makeGoal() override;
  bool validateResult(const Goal & goal, const Result & result) const override;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__SEND_MISSION_REPORT_NODE_HPP_
