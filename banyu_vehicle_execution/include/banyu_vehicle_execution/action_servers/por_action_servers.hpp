#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__POR_ACTION_SERVERS_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__POR_ACTION_SERVERS_HPP_

#include <limits>
#include <string>

#include "banyu_msgs/action/acquire_gate.hpp"
#include "banyu_msgs/action/align_gate.hpp"
#include "banyu_msgs/action/align_marker.hpp"
#include "banyu_msgs/action/orbit_marker.hpp"
#include "banyu_msgs/action/pass_gate.hpp"
#include "banyu_msgs/action/replay_track.hpp"
#include "banyu_msgs/action/search_gate.hpp"
#include "banyu_msgs/action/search_marker.hpp"
#include "banyu_msgs/action/timed_move.hpp"
#include "banyu_msgs/action/turn_around.hpp"
#include "banyu_vehicle_execution/command_encoder.hpp"
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

class TimedMoveServer final : public TypedActionServer<banyu_msgs::action::TimedMove>
{
public:
  TimedMoveServer(rclcpp::Node *, const std::string &, ActionServerCoordinator *);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double last_yaw_{0.0};
  double accumulated_yaw_{0.0};
  double dead_reckon_yaw_{0.0};
  bool yaw_initialized_{false};
  // Straight legs hold the heading captured at the start to counter thruster-asymmetry yaw drift.
  bool heading_hold_enabled_{false};
  double heading_hold_kp_{1.0};
  double heading_hold_max_rate_rad_s_{0.3};
  double heading_hold_deadband_rad_{0.02};
  // Hard cap on |vx| and |vy| for timed legs; 0 disables.
  double max_horizontal_speed_mps_{0.0};
  double heading_ref_{0.0};
  rclcpp::Time depth_reached_since_{0, 0, RCL_ROS_TIME};
};

// Replays a teach track: recorded sticks open loop (converted back through the RC override
// gains so the same PWM reaches the Pixhawk), recorded depth and heading closed loop.
class ReplayTrackServer final : public TypedActionServer<banyu_msgs::action::ReplayTrack>
{
public:
  ReplayTrackServer(
    rclcpp::Node *, const std::string &, ActionServerCoordinator *, const RcOverrideConfig &);
  bool validateGoal(const Goal &, std::string &) override;
  void onGoalStarted() override;
  MotionIntent tick(const VehicleTelemetry &, const rclcpp::Time &, double) override;

private:
  RcOverrideConfig rc_config_;
  double heading_kp_{1.0};
  double max_heading_correction_rad_s_{0.3};
  double heading_deadband_rad_{0.02};
  // Pause the track clock while |depth error| > depth_pause_m, until it is < depth_resume_m,
  // for at most max_pause_s in total (depth_pause_m 0 disables).
  double depth_pause_m_{0.4};
  double depth_resume_m_{0.2};
  double max_pause_s_{30.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  double track_time_s_{0.0};
  double paused_s_{0.0};
  bool paused_{false};
  // Current heading minus the recorded relative heading, captured on the first fresh heading
  double heading_offset_{std::numeric_limits<double>::quiet_NaN()};
  std::size_t cursor_{0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__POR_ACTION_SERVERS_HPP_
