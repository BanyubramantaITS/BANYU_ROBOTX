#ifndef BANYU_COMMUNICATION_UUV_COMM_NODE_HPP_
#define BANYU_COMMUNICATION_UUV_COMM_NODE_HPP_

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include <banyu_msgs/msg/pipeline_ring_state.hpp>
#include <banyu_msgs/msg/vehicle_status.hpp>
#include <banyu_msgs/srv/trigger_pipeline_survey.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rx_msgs/msg/command.hpp>
#include <rx_msgs/msg/course.hpp>
#include <rx_msgs/msg/heartbeat.hpp>
#include <rx_msgs/msg/incident_ack.hpp>
#include <rx_msgs/msg/mission_status.hpp>
#include <rx_msgs/msg/pipeline_survey.hpp>
#include <rx_msgs/msg/readiness_report.hpp>
#include <rx_msgs/msg/rx_task.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace banyu_communication
{

class UuvCommNode : public rclcpp::Node
{
public:
  explicit UuvCommNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using TriggerPipelineSurvey = banyu_msgs::srv::TriggerPipelineSurvey;
  static constexpr uint32_t kMaxRingIndex = 20;
  static constexpr size_t kDefaultPipelineSegments = 4;

  void publishHeartbeat();
  void publishMissionStatus(
    const std::string & message, std::optional<uint32_t> expected_run_id = std::nullopt);
  void commandCallback(const rx_msgs::msg::Command::SharedPtr msg);
  void courseCallback(const rx_msgs::msg::Course::SharedPtr msg);
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void vehicleStatusCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg);
  void ringStateCallback(const banyu_msgs::msg::PipelineRingState::SharedPtr msg);
  void surveyCallback(
    const std::shared_ptr<TriggerPipelineSurvey::Request> request,
    std::shared_ptr<TriggerPipelineSurvey::Response> response);
  uint8_t robotStateLocked() const;

  std::mutex mutex_;
  double heartbeat_rate_hz_;
  rx_msgs::msg::LatLng anchor_;
  double anchor_altitude_hae_m_;
  rx_msgs::msg::LatLng active_buoy_position_;
  mavros_msgs::msg::State state_;
  sensor_msgs::msg::NavSatFix gps_;
  rclcpp::Time last_gps_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped pose_;
  banyu_msgs::msg::VehicleStatus vehicle_status_;
  bool gps_received_{false};
  bool vehicle_status_received_{false};
  double speed_mps_{0.0};
  uint32_t run_id_{0};
  uint64_t mission_start_generation_{0};
  uint8_t current_task_{rx_msgs::msg::RxTask::TASK_NONE};
  rx_msgs::msg::Course course_;
  uint32_t pinger_freq_hz_{0};
  std::map<uint32_t, uint8_t> segments_by_index_;

  rclcpp::Publisher<rx_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<rx_msgs::msg::MissionStatus>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<rx_msgs::msg::IncidentAck>::SharedPtr incident_ack_pub_;
  rclcpp::Publisher<rx_msgs::msg::ReadinessReport>::SharedPtr readiness_pub_;
  rclcpp::Publisher<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_pub_;
  rclcpp::Subscription<rx_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<rx_msgs::msg::Course>::SharedPtr course_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<banyu_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineRingState>::SharedPtr ring_state_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mission_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mission_abort_client_;
  rclcpp::Service<TriggerPipelineSurvey>::SharedPtr survey_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION_UUV_COMM_NODE_HPP_
