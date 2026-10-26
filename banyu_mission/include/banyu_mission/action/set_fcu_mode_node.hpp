#ifndef BANYU_MISSION__ACTION__SET_FCU_MODE_NODE_HPP_
#define BANYU_MISSION__ACTION__SET_FCU_MODE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "banyu_mission/mission_state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace banyu_mission
{

// Switches the flight mode and waits until the vehicle reports it. Used to leave GUIDED (needs a position
// estimate, which the GPS gives only on the surface) for ALT_HOLD before a dive: see
// docs/task2_mode_experiments.md. Fails after the timeout so a refused switch cannot be missed.
class SetFcuModeNode final : public BT::StatefulActionNode
{
public:
  SetFcuModeNode(
    const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node,
    std::shared_ptr<MissionState> state, std::string set_mode_service);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mode", "ALT_HOLD", "FCU custom mode to select"),
      BT::InputPort<double>("timeout_s", 5.0, "give up after this long")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void sendRequest();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionState> state_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_;
  std::string mode_;
  double timeout_s_{5.0};
  std::chrono::steady_clock::time_point started_;
  std::chrono::steady_clock::time_point last_request_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__SET_FCU_MODE_NODE_HPP_
