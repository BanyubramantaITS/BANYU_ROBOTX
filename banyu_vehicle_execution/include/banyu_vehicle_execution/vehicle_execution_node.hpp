#ifndef BANYU_VEHICLE_EXECUTION__VEHICLE_EXECUTION_NODE_HPP_
#define BANYU_VEHICLE_EXECUTION__VEHICLE_EXECUTION_NODE_HPP_

#include <optional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/execution_readiness.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "banyu_msgs/msg/vehicle_status.hpp"
#include "banyu_msgs/msg/pipeline_observation.hpp"
#include "banyu_msgs/msg/pipeline_ring_observation.hpp"
#include "banyu_msgs/msg/detection.hpp"
#include "banyu_guidance/msg/velocity_intent.hpp"

#include "banyu_msgs/msg/pinger_observation.hpp"
#include "banyu_vehicle_execution/action_servers.hpp"
#include "banyu_vehicle_execution/camera_heave.hpp"
#include "banyu_vehicle_execution/command_encoder.hpp"
#include "banyu_vehicle_execution/goal_arbiter.hpp"
#include "banyu_vehicle_execution/motion_intent.hpp"
#include "banyu_vehicle_execution/vehicle_telemetry.hpp"

namespace banyu_vehicle_execution
{

class SystemSteadyClock : public SteadyTimeSource
{
public:
  SteadyTime now() const override
  {
    return std::chrono::steady_clock::now();
  }
};

class VehicleExecutionNode : public rclcpp::Node, public ActionServerCoordinator
{
public:
  explicit VehicleExecutionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VehicleExecutionNode() override = default;

  // ActionServerCoordinator implementation
  GoalRequest requestGoal(const GoalIdentity & identity) override;
  bool cancelGoal(const GoalHandle & handle) override;
  void completeGoal(const GoalHandle & handle) override;
  void faultGoal(const GoalHandle & handle) override;
  void notifyDisplaced(
    const GoalIdentity & displaced_identity,
    const std::string & new_owner) override;

  // Accessors for testing and inspection
  const GoalArbiter & arbiter() const noexcept {return arbiter_;}
  GoalArbiter & arbiter() noexcept {return arbiter_;}
  const VehicleTelemetry & telemetry() const noexcept {return telemetry_;}

private:
  void initParameters();
  void initSubscribers();
  void initPublishers();
  void initActionServers();

  void timerCallback();

  void handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handleGps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void handleRelAlt(const std_msgs::msg::Float64::SharedPtr msg);
  void handleWaterDepth(const banyu_msgs::msg::WaterDepth::SharedPtr msg);
  void handleStatus(const banyu_msgs::msg::VehicleStatus::SharedPtr msg);
  void handlePipelineObs(const banyu_msgs::msg::PipelineObservation::SharedPtr msg);
  void handleRingObs(const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg);
  void handleDetection(const banyu_msgs::msg::Detection::SharedPtr msg);
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void handleGuidanceIntent(const banyu_guidance::msg::VelocityIntent::SharedPtr msg);
  void handlePingerObs(const banyu_msgs::msg::PingerObservation::SharedPtr msg);
  void handleRunLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg);
  bool lifecycleFresh() const;
  bool runMotionAuthorized() const;
  bool readinessHoldAuthorized() const;
  bool hardwareStatusReady() const;
  void revokeGoals(const std::string & reason);
  MotionIntent readinessHoldIntent();
  void publishReadiness(bool holding, const std::string & reason);

  rclcpp::Time resolveStamp(const builtin_interfaces::msg::Time & stamp_msg);

  SystemSteadyClock steady_clock_;
  uint32_t required_zero_cycles_{2};
  GoalArbiter arbiter_;
  IntentLimits limits_{kHardMaxLinearComponent, kHardMaxYawRate};

  bool enable_mavros_publish_{false};
  bool require_guided_{true};
  // Flight mode the vehicle must be in while autonomous ("GUIDED" for setpoint
  // output; "ALT_HOLD"/"STABILIZE"/"MANUAL" for rc_override, no position needed).
  std::string autonomy_mode_{"GUIDED"};
  // Second mode of a two-mode list ("GUIDED,ALT_HOLD"): the mode used under water, driven by RC override.
  std::string underwater_mode_;
  bool modeAllowed(const std::string & mode) const;
  bool rcModeActive() const;
  bool setpointModeActive() const;
  // "setpoint" (/mavros/setpoint_raw/local, GUIDED), "rc_override" (/mavros/rc/override) or "auto":
  // setpoint in autonomy_mode (surface, GPS) and rc_override in underwater_mode (no position needed).
  std::string output_mode_{"setpoint"};
  RcOverrideConfig rc_override_config_{};
  bool rc_override_neutral_sent_{false};
  // The sticks belong to the mission only while a goal runs and for idle_release_s after it;
  // otherwise they are released so a pilot (QGC joystick, TX16S) is never fought.
  double rc_override_idle_release_s_{2.0};
  std::optional<SteadyTime> last_goal_time_;
  void publishRcOverride(const MotionIntent & intent, bool output_allowed, bool own_sticks);
  bool status_ever_received_{false};
  bool require_run_authorization_{false};
  double run_lifecycle_timeout_s_{1.0};
  double pose_timeout_s_{0.5};
  banyu_msgs::msg::RunLifecycle run_lifecycle_;
  bool lifecycle_received_{false};
  bool authority_handshake_{false};
  std::unordered_set<std::string> retired_authorities_;
  SteadyTime lifecycle_receipt_{};
  SteadyTime status_receipt_{};
  SteadyTime pose_receipt_{};
  bool hold_pose_captured_{false};
  double hold_x_{0.0};
  double hold_y_{0.0};
  double hold_z_{0.0};
  double hold_kp_{0.5};
  uint32_t terminal_zero_cycles_remaining_{0};
  uint64_t revoked_epoch_{0};
  bool require_calibrated_surface_depth_{false};
  double surface_depth_timeout_s_{0.5};
  // No depth sensor: rel_alt and the EKF height are never used as a depth.
  bool use_pressure_depth_{true};
  // Heave from the pipe's apparent width for the actions that hold the vehicle over the pipe.
  bool camera_heave_enabled_{false};
  CameraHeave camera_heave_;
  std::unordered_set<std::string> camera_heave_servers_;
  std::vector<std::string> surface_depth_sources_;
  int64_t run_context_start_ns_{0};
  int64_t last_water_depth_stamp_ns_{0};

  VehicleTelemetry telemetry_{};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};

  std::vector<std::shared_ptr<ActionServerBase>> servers_;
  std::unordered_map<std::string, std::shared_ptr<ActionServerBase>> server_map_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;
  rclcpp::Subscription<banyu_msgs::msg::WaterDepth>::SharedPtr water_depth_sub_;
  rclcpp::Subscription<banyu_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineObservation>::SharedPtr pipeline_obs_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineRingObservation>::SharedPtr ring_obs_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Detection>::SharedPtr detection_sub_;
  rclcpp::Subscription<banyu_guidance::msg::VelocityIntent>::SharedPtr guidance_intent_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PingerObservation>::SharedPtr pinger_obs_sub_;
  bool use_hydrophone_{false};
  rclcpp::Subscription<banyu_msgs::msg::RunLifecycle>::SharedPtr run_lifecycle_sub_;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr preview_target_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr mavros_target_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  rclcpp::Publisher<banyu_msgs::msg::ExecutionReadiness>::SharedPtr readiness_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__VEHICLE_EXECUTION_NODE_HPP_
