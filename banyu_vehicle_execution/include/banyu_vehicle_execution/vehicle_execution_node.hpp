#ifndef BANYU_VEHICLE_EXECUTION__VEHICLE_EXECUTION_NODE_HPP_
#define BANYU_VEHICLE_EXECUTION__VEHICLE_EXECUTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "banyu_msgs/msg/action_lease.hpp"
#include "banyu_msgs/msg/vehicle_status.hpp"
#include "banyu_msgs/msg/pipeline_observation.hpp"
#include "banyu_msgs/msg/pipeline_ring_observation.hpp"
#include "banyu_guidance/msg/velocity_intent.hpp"

#include "banyu_vehicle_execution/action_servers.hpp"
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
  void handleStatus(const banyu_msgs::msg::VehicleStatus::SharedPtr msg);
  void handlePipelineObs(const banyu_msgs::msg::PipelineObservation::SharedPtr msg);
  void handleRingObs(const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg);
  void handleGuidanceIntent(const banyu_guidance::msg::VelocityIntent::SharedPtr msg);
  void handleLease(const banyu_msgs::msg::ActionLease::SharedPtr msg);

  rclcpp::Time resolveStamp(const builtin_interfaces::msg::Time & stamp_msg);

  SystemSteadyClock steady_clock_;
  uint32_t required_zero_cycles_{2};
  GoalArbiter arbiter_;
  IntentLimits limits_{kHardMaxLinearComponent, kHardMaxYawRate};

  bool require_lease_{true};
  double lease_timeout_s_{1.5};
  bool enable_mavros_publish_{false};
  bool require_guided_{true};
  bool status_ever_received_{false};

  VehicleTelemetry telemetry_{};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};

  std::vector<std::shared_ptr<ActionServerBase>> servers_;
  std::unordered_map<std::string, std::shared_ptr<ActionServerBase>> server_map_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;
  rclcpp::Subscription<banyu_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineObservation>::SharedPtr pipeline_obs_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineRingObservation>::SharedPtr ring_obs_sub_;
  rclcpp::Subscription<banyu_guidance::msg::VelocityIntent>::SharedPtr guidance_intent_sub_;
  rclcpp::Subscription<banyu_msgs::msg::ActionLease>::SharedPtr lease_sub_;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr preview_target_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr mavros_target_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__VEHICLE_EXECUTION_NODE_HPP_
