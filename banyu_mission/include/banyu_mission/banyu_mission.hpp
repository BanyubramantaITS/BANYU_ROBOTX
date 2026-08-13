#ifndef BANYU_MISSION_BANYU_MISSION_HPP_
#define BANYU_MISSION_BANYU_MISSION_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <banyu_msgs/msg/detection.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
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
  void startMission();
  void abortMission();

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onState(const mavros_msgs::msg::State::SharedPtr msg);
  void onDetection(const banyu_msgs::msg::Detection::SharedPtr msg);

  void armAndSetGuided();
  void setAbortMode();

  // ROS plumbing.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Detection>::SharedPtr detection_sub_;
  rclcpp::TimerBase::SharedPtr detection_watchdog_timer_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr abort_service_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  // BehaviorTree.CPP plumbing.
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
  std::shared_ptr<MissionContext> ctx_;
  double detection_timeout_s_{0.3};

  std::thread tick_thread_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BANYU_MISSION_HPP_
