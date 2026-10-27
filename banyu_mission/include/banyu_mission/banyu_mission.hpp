#ifndef BANYU_MISSION__BANYU_MISSION_HPP_
#define BANYU_MISSION__BANYU_MISSION_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include <optional>
#include <vector>

#include <functional>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "banyu_mission/mission_state.hpp"
#include "banyu_mission/por_mission_config.hpp"
#include "banyu_mission/run_coordinator.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/execution_readiness.hpp"
#include "banyu_msgs/msg/task2_buoy_target.hpp"
#include "banyu_msgs/msg/task2_survey_progress.hpp"
#include "banyu_msgs/srv/reset_task2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rx_msgs/msg/command.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace banyu_mission
{

void registerMissionNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MissionState> & state,
  std::function<void()> on_abort = nullptr,
  std::function<BT::NodeStatus()> on_survey_complete = nullptr,
  std::function<BT::NodeStatus()> on_surface_success = nullptr,
  std::function<BT::NodeStatus()> on_lock_buoy_target = nullptr);

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
  void disarmVehicle();
  void commandCallback(const rx_msgs::msg::Command::SharedPtr command);
  void publishLifecycle(
    uint8_t event = 0, const std::string & reason = "",
    uint32_t requested_run_id = 0);
  void tickTask2Preparation();
  void tickRunInitialization();
  bool autonomousReady() const;
  std::string autonomyNotReadyReason() const;
  bool executionHolding() const;
  bool surveyEvidenceReady() const;
  bool operatorHasControl() const;
  void terminateRun(bool failure, const std::string & reason, bool recover);
  void applyBuoyTarget();

  struct ResetConsumer
  {
    rclcpp::Client<banyu_msgs::srv::ResetTask2>::SharedPtr client;
    rclcpp::Client<banyu_msgs::srv::ResetTask2>::SharedFuture future;
    bool acknowledged{false};
  };

  std::shared_ptr<MissionState> state_;
  std::shared_ptr<BT::Tree> tree_;
  std::shared_ptr<BT::Tree> preparation_tree_;
  std::unique_ptr<RunCoordinator> run_;
  std::string mission_mode_;
  std::string authority_instance_;
  uint64_t lifecycle_revision_{0};
  rclcpp::Time accepted_run_at_{0, 0, RCL_ROS_TIME};
  std::recursive_mutex control_mutex_;
  std::vector<ResetConsumer> reset_consumers_;
  std::map<uint32_t, uint32_t> start_commands_;
  uint32_t last_stop_sequence_{0};
  std::chrono::steady_clock::time_point preparation_started_{};
  std::chrono::steady_clock::time_point initialization_started_{};
  std::chrono::steady_clock::time_point lifecycle_published_at_{};
  std::chrono::steady_clock::time_point pose_received_at_{};
  bool pose_valid_{false};
  banyu_msgs::msg::ExecutionReadiness execution_readiness_;
  std::chrono::steady_clock::time_point execution_received_at_{};
  double execution_readiness_timeout_s_{1.0};
  bool preparation_hold_seen_{false};
  banyu_msgs::msg::Task2SurveyProgress survey_progress_;
  std::chrono::steady_clock::time_point survey_progress_received_at_{};
  uint8_t task2_tier_{2};
  double preparation_timeout_s_{30.0};
  double reset_timeout_s_{3.0};
  rclcpp::Publisher<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_pub_;
  rclcpp::Subscription<rx_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<banyu_msgs::msg::ExecutionReadiness>::SharedPtr execution_readiness_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Task2SurveyProgress>::SharedPtr survey_progress_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Task2BuoyTarget>::SharedPtr buoy_target_sub_;
  // USV active buoy target; it replaces the configured target until LockBuoyTarget runs.
  std::optional<banyu_msgs::msg::Task2BuoyTarget> buoy_target_;
  bool buoy_target_locked_{false};
  bool explicit_heading_{false};
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preparation_service_;
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
  bool por_use_vision_{false};
  bool por_use_vision_return_{false};
  bool por_use_vision_assist_{false};
  bool por_use_replay_{false};
  std::string fallback_mode_{"SURFACE"};
  std::string set_mode_service_{"/mavros/set_mode"};
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  std::string arming_service_{"/mavros/cmd/arming"};
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Time last_fallback_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__BANYU_MISSION_HPP_
