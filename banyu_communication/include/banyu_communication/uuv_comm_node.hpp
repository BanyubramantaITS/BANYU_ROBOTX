#ifndef BANYU_COMMUNICATION_UUV_COMM_NODE_HPP_
#define BANYU_COMMUNICATION_UUV_COMM_NODE_HPP_

#include <cstdint>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include <memory>

#include <banyu_msgs/msg/pipeline_ring_state.hpp>
#include <banyu_msgs/msg/run_lifecycle.hpp>
#include <banyu_msgs/msg/task2_survey_evidence.hpp>
#include <banyu_msgs/msg/task2_survey_progress.hpp>
#include <banyu_msgs/msg/water_depth.hpp>
#include <banyu_msgs/msg/vehicle_status.hpp>
#include <banyu_msgs/srv/trigger_pipeline_survey.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rx_msgs/msg/command.hpp>
#include <rx_msgs/msg/course.hpp>
#include <rx_msgs/msg/heartbeat.hpp>
#include <rx_msgs/msg/incident_ack.hpp>
#include <rx_msgs/msg/mission_status.hpp>
#include <rx_msgs/msg/pipeline_survey.hpp>
#include <rx_msgs/msg/readiness_report.hpp>
#include <rx_msgs/msg/resource_delivery.hpp>
#include <rx_msgs/msg/rx_task.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "banyu_communication/survey_reporting.hpp"
#include "banyu_communication/task4_zones.hpp"
#include <banyu_msgs/msg/task4_zones.hpp>

class UuvCommNodeTest;

namespace banyu_communication
{

class UuvCommNode : public rclcpp::Node
{
public:
  friend class ::UuvCommNodeTest;
  explicit UuvCommNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UuvCommNode() override;

private:
  using TriggerPipelineSurvey = banyu_msgs::srv::TriggerPipelineSurvey;

  void publishHeartbeat();
  void publishMissionStatus(
    const std::string & message, std::optional<uint32_t> expected_run_id = std::nullopt);
  void commandCallback(const rx_msgs::msg::Command::SharedPtr msg);
  void courseCallback(const rx_msgs::msg::Course::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void vehicleStatusCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg);
  void lifecycleCallback(const banyu_msgs::msg::RunLifecycle::SharedPtr msg);
  void evidenceCallback(const banyu_msgs::msg::Task2SurveyEvidence::SharedPtr msg);
  void depthCallback(const banyu_msgs::msg::WaterDepth::SharedPtr msg);
  void publishSurveyProgress();
  void publishZones();
  void surveyCallback(
    std::shared_ptr<rclcpp::Service<TriggerPipelineSurvey>> service,
    std::shared_ptr<rmw_request_id_t> header,
    std::shared_ptr<TriggerPipelineSurvey::Request> request);
  uint8_t robotStateLocked() const;
  bool lifecycleFreshLocked() const;

  std::mutex mutex_;
  double heartbeat_rate_hz_;
  double lifecycle_timeout_s_;
  rx_msgs::msg::LatLng anchor_;
  double anchor_altitude_hae_m_;
  sensor_msgs::msg::NavSatFix gps_;
  rclcpp::Time last_gps_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped pose_;
  banyu_msgs::msg::VehicleStatus vehicle_status_;
  bool gps_received_{false};
  bool vehicle_status_received_{false};
  bool lifecycle_received_{false};
  // Vehicle status is published on the ROS clock (sim time in the sim): its freshness is judged on it.
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  // Calibrated water depth (Bar30, or the GPS surface proof) for the heartbeat; the EKF pose has no depth under water.
  // Last valid GPS fix: the heartbeat position while the fix is stale (always under water); the anchor only before any fix.
  bool has_last_fix_{false};
  rx_msgs::msg::LatLng last_fix_;
  double last_fix_altitude_hae_m_{0.0};
  double water_depth_m_{0.0};
  bool water_depth_valid_{false};
  rclcpp::Time last_water_depth_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_lifecycle_received_;
  banyu_msgs::msg::RunLifecycle lifecycle_;
  std::set<std::string> retired_authorities_;
  double speed_mps_{0.0};
  uint32_t run_id_{0};
  uint8_t current_task_{rx_msgs::msg::RxTask::TASK_NONE};
  rx_msgs::msg::Course course_;
  uint32_t pinger_freq_hz_{0};
  std::unique_ptr<SurveyLedger> survey_ledger_;
  std::unique_ptr<SurfaceGuard> surface_guard_;
  // Keyed by epoch: RoboCommand may reuse a run_id after a run ID reset.
  std::set<uint64_t> submitted_epochs_;
  uint64_t evidence_revision_{0};
  // Task 4 keep-out zones (RoboCommand), published for the surface avoidance; cleared at prepare_autonomy.
  std::unique_ptr<Task4ZoneTracker> zones_;
  rclcpp::Publisher<banyu_msgs::msg::Task4Zones>::SharedPtr zones_pub_;

  rclcpp::Publisher<rx_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<rx_msgs::msg::MissionStatus>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<rx_msgs::msg::IncidentAck>::SharedPtr incident_ack_pub_;
  rclcpp::Publisher<rx_msgs::msg::ReadinessReport>::SharedPtr readiness_pub_;
  rclcpp::Publisher<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_pub_;
  rclcpp::Publisher<rx_msgs::msg::ResourceDelivery>::SharedPtr delivery_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2SurveyProgress>::SharedPtr survey_progress_pub_;
  rclcpp::Subscription<rx_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<rx_msgs::msg::Course>::SharedPtr course_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<banyu_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Task2SurveyEvidence>::SharedPtr evidence_sub_;
  rclcpp::Subscription<banyu_msgs::msg::WaterDepth>::SharedPtr water_depth_sub_;
  rclcpp::Service<TriggerPipelineSurvey>::SharedPtr survey_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION_UUV_COMM_NODE_HPP_
