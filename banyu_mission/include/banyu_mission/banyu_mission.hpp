#ifndef BANYU_MISSION__BANYU_MISSION_HPP_
#define BANYU_MISSION__BANYU_MISSION_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <functional>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "banyu_mission/mission_state.hpp"
#include "banyu_mission/por_mission_config.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace banyu_mission
{

void registerMissionNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MissionState> & state,
  std::function<void()> on_abort = nullptr);

class BanyuMission : public rclcpp::Node
{
public:
  explicit BanyuMission(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BanyuMission() override;
  void initialize();
  MissionState::Lifecycle lifecycle() const;

private:
  void tickLoop(int tick_period_ms);
  std::string xmlPath();
  void configurePorBlackboard(const BT::Blackboard::Ptr & blackboard);
  void setFlightMode(const std::string & mode);
  void fallbackToSurface();

  std::shared_ptr<MissionState> state_;
  std::shared_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::StatusChangeLogger> tree_logger_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
  std::thread tick_thread_;
  std::atomic_bool initialized_{false};
  std::atomic_bool stop_requested_{false};
  std::mutex tree_mutex_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_service_;
  std::atomic_bool mission_config_valid_{false};
  std::string fallback_mode_{"SURFACE"};
  std::string set_mode_service_{"/mavros/set_mode"};
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Time last_fallback_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__BANYU_MISSION_HPP_
