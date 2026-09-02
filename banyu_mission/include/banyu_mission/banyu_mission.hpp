#ifndef BANYU_MISSION_BANYU_MISSION_HPP_
#define BANYU_MISSION_BANYU_MISSION_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <banyu_msgs/msg/detection.hpp>
#include <banyu_msgs/msg/hydrophone_heading.hpp>
#include <banyu_msgs/msg/pipeline_ring_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class BanyuMission : public rclcpp::Node {
 public:
  explicit BanyuMission(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BanyuMission() override;

  // Two-phase init: shared_from_this() is only valid once the node is owned
  // by a shared_ptr, so the BehaviorTree setup runs here, not in the ctor.
  void initialize();

 private:
  void declareParameters();
  void setupBehaviorTree();
  bool startMission(std::string& failure_message);
  void abortMission();
  void beginHandoff();
  void onHandoffTimer();

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onState(const mavros_msgs::msg::State::SharedPtr msg);
  void onDetection(const banyu_msgs::msg::Detection::SharedPtr msg);
  void onHydrophoneHeading(const banyu_msgs::msg::HydrophoneHeading::SharedPtr msg);
  void onPipelineRingState(const banyu_msgs::msg::PipelineRingState::SharedPtr msg);
  void onActiveBuoyPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPipelineRoute(const nav_msgs::msg::Path::SharedPtr msg);
  void onDvlLive(const std_msgs::msg::Bool::SharedPtr msg);

  void armAndSetGuided();

  // ROS plumbing.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Detection>::SharedPtr detection_sub_;
  rclcpp::Subscription<banyu_msgs::msg::HydrophoneHeading>::SharedPtr hydrophone_heading_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineRingState>::SharedPtr pipeline_ring_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr active_buoy_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pipeline_route_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dvl_live_sub_;
  rclcpp::TimerBase::SharedPtr detection_watchdog_timer_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robocommand_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr abort_service_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr handoff_timer_;

  // BehaviorTree.CPP plumbing.
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
  std::shared_ptr<MissionContext> ctx_;
  std::string mission_mode_;
  double handoff_request_timeout_s_{1.0};
  double detection_timeout_s_{0.3};
  double pose_state_timeout_s_{0.5};
  double task2_dvl_timeout_s_{0.5};
  double hydrophone_heading_timeout_s_{0.5};
  double pipeline_ring_state_timeout_s_{0.5};
  double active_buoy_pose_timeout_s_{0.5};
  double pipeline_route_timeout_s_{0.5};

  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> handoff_active_{false};
  std::atomic<bool> handoff_request_in_flight_{false};
  std::mutex lifecycle_mutex_;
  std::mutex handoff_mutex_;
  std::chrono::steady_clock::time_point handoff_request_deadline_{};
  uint64_t handoff_request_generation_{0};

  std::thread tick_thread_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BANYU_MISSION_HPP_
