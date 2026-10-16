#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__POR_ACTION_SERVERS_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__POR_ACTION_SERVERS_HPP_

#include <limits>
#include <string>

#include "banyu_msgs/action/acquire_gate.hpp"
#include "banyu_msgs/action/align_gate.hpp"
#include "banyu_msgs/action/align_marker.hpp"
#include "banyu_msgs/action/orbit_marker.hpp"
#include "banyu_msgs/action/pass_gate.hpp"
#include "banyu_msgs/action/search_gate.hpp"
#include "banyu_msgs/action/search_marker.hpp"
#include "banyu_msgs/action/turn_around.hpp"
#include "banyu_vehicle_execution/requirement_watchdog.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"
#include "banyu_vehicle_execution/por_visual_helpers.hpp"

namespace banyu_vehicle_execution
{

constexpr double kPorSafeOrbitRadiusM = 2.2;

class AcquireGateServer final : public TypedActionServer<banyu_msgs::action::AcquireGate>
{
public:
  AcquireGateServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
};

class SearchMarkerServer final : public TypedActionServer<banyu_msgs::action::SearchMarker>
{
public:
  SearchMarkerServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
};

class SearchGateServer final : public TypedActionServer<banyu_msgs::action::SearchGate>
{
public:
  SearchGateServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
};

class AlignGateServer final : public TypedActionServer<banyu_msgs::action::AlignGate>
{
public:
  AlignGateServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
  rclcpp::Time aligned_since_{0, 0, RCL_ROS_TIME};
};

class AlignMarkerServer final : public TypedActionServer<banyu_msgs::action::AlignMarker>
{
public:
  AlignMarkerServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
  rclcpp::Time aligned_since_{0, 0, RCL_ROS_TIME};
};

class PassGateServer final : public TypedActionServer<banyu_msgs::action::PassGate>
{
public:
  PassGateServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
  double min_distance_{std::numeric_limits<double>::infinity()};
  rclcpp::Time blind_start_time_{0, 0, RCL_ROS_TIME};
  bool blind_mode_{false};
  bool gate_seen_{false};
  bool start_pose_set_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_z_{0.0};
};

class OrbitMarkerServer final : public TypedActionServer<banyu_msgs::action::OrbitMarker>
{
public:
  OrbitMarkerServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
  double last_yaw_{0.0};
  double accumulated_yaw_{0.0};
  bool yaw_initialized_{false};
  double last_distance_{0.0};
};

class TurnAroundServer final : public TypedActionServer<banyu_msgs::action::TurnAround>
{
public:
  TurnAroundServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RequirementWatchdog watchdog_;
  double last_yaw_{0.0};
  double accumulated_yaw_{0.0};
  bool yaw_initialized_{false};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__POR_ACTION_SERVERS_HPP_
