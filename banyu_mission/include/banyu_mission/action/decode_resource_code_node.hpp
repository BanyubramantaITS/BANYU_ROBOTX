#ifndef BANYU_MISSION__ACTION__DECODE_RESOURCE_CODE_NODE_HPP_
#define BANYU_MISSION__ACTION__DECODE_RESOURCE_CODE_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "banyu_msgs/msg/resource_code.hpp"
#include "banyu_mission/mission_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace banyu_mission
{

// Opens the decoder window, then waits until the post-repair flash code has been read or the timeout
// passes. It always ends in SUCCESS: a code that could not be read must not stop the survey report
// (the Core-level points do not depend on it). Run it in parallel with a station-keeping hold.
class DecodeResourceCodeNode final : public BT::StatefulActionNode
{
public:
  DecodeResourceCodeNode(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node,
    std::shared_ptr<MissionState> state, std::string window_service, std::string result_topic);

  static BT::PortsList providedPorts() {return {BT::InputPort<double>("timeout_s", 40.0, "")};}

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void closeWindow();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionState> state_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Subscription<banyu_msgs::msg::ResourceCode>::SharedPtr sub_;
  std::mutex mutex_;
  rclcpp::Time started_;
  double timeout_s_{40.0};
  bool open_{false};
  bool complete_{false};
  uint32_t run_id_{0};
  uint64_t epoch_{0};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__DECODE_RESOURCE_CODE_NODE_HPP_
