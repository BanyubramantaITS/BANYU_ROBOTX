#ifndef BANYU_MISSION__ACTION__ARM_AND_SET_GUIDED_HPP_
#define BANYU_MISSION__ACTION__ARM_AND_SET_GUIDED_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "banyu_mission/mission_state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace banyu_mission
{

class ArmAndSetGuided final : public BT::StatefulActionNode
{
public:
  ArmAndSetGuided(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, std::shared_ptr<MissionState> state,
    std::string set_mode_service = "/mavros/set_mode",
    std::string arming_service = "/mavros/cmd/arming");

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Step
  {
    WaitingForReadiness,
    WaitingForServices,
    WaitingForSetMode,
    WaitingForArming,
    Done
  };

  void sendSetModeRequest();
  void sendArmingRequest();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionState> state_;
  std::string set_mode_service_;
  std::string arming_service_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

  std::mutex mutex_;
  uint64_t generation_{0};
  Step step_{Step::WaitingForReadiness};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double timeout_s_{30.0};
  bool set_mode_done_{false};
  bool set_mode_success_{false};
  bool arming_done_{false};
  bool arming_success_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__ARM_AND_SET_GUIDED_HPP_
