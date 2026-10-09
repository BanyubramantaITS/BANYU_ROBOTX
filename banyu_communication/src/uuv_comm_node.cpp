#include "banyu_communication/uuv_comm_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rx_msgs/msg/pipeline_segment_status.hpp>
#include <rx_msgs/msg/robot_state.hpp>
#include <rx_msgs/msg/vehicle_type.hpp>

namespace banyu_communication
{
namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

bool validPosition(double latitude, double longitude)
{
  return std::isfinite(latitude) && std::isfinite(longitude) &&
         latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0;
}

// CMD_RUN_STOP is available only in some rx_msgs schemas.
template<typename CommandMessage>
auto isRunStop(uint8_t command_type, int) -> decltype(CommandMessage::CMD_RUN_STOP, bool())
{
  return command_type == CommandMessage::CMD_RUN_STOP;
}

template<typename CommandMessage>
bool isRunStop(uint8_t, long)
{
  return false;
}
}  // namespace

UuvCommNode::UuvCommNode(const rclcpp::NodeOptions & options)
: Node("uuv_comm_node", options)
{
  heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 2.0);
  anchor_.latitude = declare_parameter<double>("anchor_latitude", 0.0);
  anchor_.longitude = declare_parameter<double>("anchor_longitude", 0.0);
  anchor_altitude_hae_m_ = declare_parameter<double>("anchor_altitude_hae_m", 0.0);
  active_buoy_position_.latitude = declare_parameter<double>("active_buoy_latitude", 0.0);
  active_buoy_position_.longitude = declare_parameter<double>("active_buoy_longitude", 0.0);
  if (!std::isfinite(heartbeat_rate_hz_) || heartbeat_rate_hz_ <= 0.0) {
    throw std::invalid_argument("heartbeat_rate_hz must be finite and positive");
  }
  const std::chrono::duration<double> period(1.0 / heartbeat_rate_hz_);
  if (!std::isfinite(period.count()) || period < std::chrono::nanoseconds(1) ||
    period >= std::chrono::nanoseconds::max())
  {
    throw std::invalid_argument("heartbeat_rate_hz is outside the timer's supported range");
  }
  const auto heartbeat_period = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  if (!validPosition(anchor_.latitude, anchor_.longitude) ||
    !validPosition(active_buoy_position_.latitude, active_buoy_position_.longitude) ||
    !std::isfinite(anchor_altitude_hae_m_))
  {
    throw std::invalid_argument("Anchor and active buoy coordinates must be valid WGS-84 values");
  }

  heartbeat_pub_ = create_publisher<rx_msgs::msg::Heartbeat>(
    "/system/vehicle/uuv/heartbeat", 10);
  mission_status_pub_ = create_publisher<rx_msgs::msg::MissionStatus>(
    "/system/mission/status", 10);
  incident_ack_pub_ = create_publisher<rx_msgs::msg::IncidentAck>(
    "/system/vehicle/uuv/task4/incident_ack", 10);
  readiness_pub_ = create_publisher<rx_msgs::msg::ReadinessReport>(
    "/system/vehicle/uuv/task4/readiness", 10);
  survey_pub_ = create_publisher<rx_msgs::msg::PipelineSurvey>(
    "/system/vehicle/uuv/task2/pipeline_survey", rclcpp::QoS(10).transient_local());

  using std::placeholders::_1;
  using std::placeholders::_2;
  command_sub_ = create_subscription<rx_msgs::msg::Command>(
    "/system/mission/command", 10, std::bind(&UuvCommNode::commandCallback, this, _1));
  course_sub_ = create_subscription<rx_msgs::msg::Course>(
    "/system/mission/course", 10, std::bind(&UuvCommNode::courseCallback, this, _1));
  state_sub_ = create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10, std::bind(&UuvCommNode::stateCallback, this, _1));
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", rclcpp::SensorDataQoS(),
    std::bind(&UuvCommNode::gpsCallback, this, _1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
    std::bind(&UuvCommNode::poseCallback, this, _1));
  velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
    std::bind(&UuvCommNode::velocityCallback, this, _1));
  vehicle_status_sub_ = create_subscription<banyu_msgs::msg::VehicleStatus>(
    "/banyu/vehicle_status", 10, std::bind(&UuvCommNode::vehicleStatusCallback, this, _1));
  ring_state_sub_ = create_subscription<banyu_msgs::msg::PipelineRingState>(
    "/banyu/pipeline/ring_state", 10,
    std::bind(&UuvCommNode::ringStateCallback, this, _1));
  mission_start_client_ = create_client<std_srvs::srv::Trigger>("/banyu/mission/start");
  mission_abort_client_ = create_client<std_srvs::srv::Trigger>("/banyu/mission/abort");
  survey_service_ = create_service<TriggerPipelineSurvey>(
    "/banyu/communication/trigger_pipeline_survey",
    std::bind(&UuvCommNode::surveyCallback, this, _1, _2));
  heartbeat_timer_ = create_wall_timer(
    heartbeat_period, std::bind(&UuvCommNode::publishHeartbeat, this));
}

uint8_t UuvCommNode::robotStateLocked() const
{
  using rx_msgs::msg::RobotState;
  // MAV_STATE values 5 through 8 indicate critical, emergency, poweroff or termination.
  if (!state_.connected || !state_.armed || state_.system_status >= 5 ||
    (vehicle_status_received_ &&
    (!vehicle_status_.fcu_connected || !vehicle_status_.fcu_status_fresh ||
    vehicle_status_.authority == "FAILSAFE")))
  {
    return RobotState::STATE_KILLED;
  }
  if (vehicle_status_received_ && vehicle_status_.operator_takeover) {
    return RobotState::STATE_MANUAL;
  }
  if (state_.guided || state_.mode == "GUIDED" || state_.mode == "GUIDED_NOGPS" ||
    state_.mode == "AUTO")
  {
    return RobotState::STATE_AUTO;
  }
  return RobotState::STATE_MANUAL;
}

void UuvCommNode::publishHeartbeat()
{
  const auto heartbeat_time = now();
  rx_msgs::msg::Heartbeat heartbeat;
  heartbeat.header.stamp = heartbeat_time;
  heartbeat.header.frame_id = "uuv";
  heartbeat.vehicle_type = rx_msgs::msg::VehicleType::TYPE_UUV;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    heartbeat.state = robotStateLocked();
    heartbeat.current_task = current_task_;
    const bool gps_valid = gps_received_ &&
      (heartbeat_time - last_gps_time_).seconds() <= 3.0 &&
      gps_.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
      validPosition(gps_.latitude, gps_.longitude);
    heartbeat.position = anchor_;
    heartbeat.altitude_hae_m = static_cast<float>(anchor_altitude_hae_m_);
    if (gps_valid) {
      heartbeat.position.latitude = gps_.latitude;
      heartbeat.position.longitude = gps_.longitude;
      if (std::isfinite(gps_.altitude)) {
        heartbeat.altitude_hae_m = static_cast<float>(gps_.altitude);
      }
    }
    heartbeat.spd_mps = (heartbeat_time - last_velocity_time_).seconds() > 3.0 ?
      0.0F : static_cast<float>(speed_mps_);
    const bool pose_fresh = (heartbeat_time - last_pose_time_).seconds() <= 3.0;
    if (pose_fresh && std::isfinite(pose_.pose.position.z)) {
      heartbeat.depth_m = static_cast<float>(std::max(0.0, -pose_.pose.position.z));
    }
    const auto & q = pose_.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (pose_fresh && std::isfinite(norm) && norm > 0.0) {
      const double x = q.x / norm;
      const double y = q.y / norm;
      const double z = q.z / norm;
      const double w = q.w / norm;
      const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
      const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
      const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
      heartbeat.roll_deg = static_cast<float>(roll * kRadToDeg);
      heartbeat.pitch_deg = static_cast<float>(pitch * kRadToDeg);
      const double yaw_enu_deg = yaw * kRadToDeg;
      heartbeat.heading_deg = static_cast<float>(std::fmod(90.0 - yaw_enu_deg + 360.0, 360.0));
      if (heartbeat.heading_deg >= 360.0F) {
        heartbeat.heading_deg = 0.0F;
      }
    }
  }
  heartbeat_pub_->publish(heartbeat);
}

void UuvCommNode::publishMissionStatus(
  const std::string & message, std::optional<uint32_t> expected_run_id)
{
  rx_msgs::msg::MissionStatus status;
  status.header.stamp = now();
  status.header.frame_id = "uuv";
  status.vehicle_id = "uuv";
  status.message = message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (expected_run_id && run_id_ != *expected_run_id) {
      return;
    }
    status.run_id = run_id_;
    status.state = robotStateLocked();
    status.current_task = current_task_;
  }
  mission_status_pub_->publish(status);
}

void UuvCommNode::commandCallback(const rx_msgs::msg::Command::SharedPtr msg)
{
  using rx_msgs::msg::Command;
  using rx_msgs::msg::VehicleType;
  if (msg->target_vehicle != VehicleType::TYPE_UUV &&
    msg->target_vehicle != VehicleType::TYPE_UNKNOWN)
  {
    return;
  }
  if (msg->command_type == Command::CMD_RUN_START) {
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      run_id_ = msg->run_id;
      generation = ++mission_start_generation_;
      current_task_ = rx_msgs::msg::RxTask::TASK_NONE;
      segments_by_index_.clear();
    }
    if (!mission_start_client_->service_is_ready()) {
      publishMissionStatus("Mission start service unavailable", msg->run_id);
      RCLCPP_WARN(get_logger(), "Mission start service unavailable for run %u", msg->run_id);
      return;
    }
    publishMissionStatus("Mission start requested", msg->run_id);
    mission_start_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, run_id = msg->run_id, generation](
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation != mission_start_generation_) {
            return;
          }
          current_task_ = response->success ? rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR :
          rx_msgs::msg::RxTask::TASK_NONE;
        }
        publishMissionStatus(
          std::string(response->success ? "Mission started: " : "Mission start rejected: ") +
          response->message, run_id);
      });
  } else if (msg->command_type == Command::CMD_MISSION_DONE ||
    isRunStop<Command>(msg->command_type, 0))
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++mission_start_generation_;
      current_task_ = rx_msgs::msg::RxTask::TASK_NONE;
    }
    if (mission_abort_client_->service_is_ready()) {
      mission_abort_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    } else {
      RCLCPP_WARN(get_logger(), "Mission abort service unavailable");
    }
    publishMissionStatus("Mission stopped");
  } else if (msg->command_type == Command::CMD_TASK4_ASSISTANCE ||
    msg->command_type == Command::CMD_TASK4_KEEP_OUT_ZONE ||
    msg->command_type == Command::CMD_TASK4_ALL_CLEAR)
  {
    rx_msgs::msg::IncidentAck ack;
    ack.header.stamp = now();
    ack.header.frame_id = "uuv";
    ack.command_seq = msg->command_seq;
    incident_ack_pub_->publish(ack);
    if (msg->command_type == Command::CMD_TASK4_ASSISTANCE) {
      rx_msgs::msg::ReadinessReport readiness;
      readiness.header = ack.header;
      readiness.command_seq = msg->command_seq;
      readiness_pub_->publish(readiness);
    }
  }
}

void UuvCommNode::courseCallback(const rx_msgs::msg::Course::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  course_ = *msg;
  pinger_freq_hz_ = msg->pinger_freq_hz;
}

void UuvCommNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = *msg;
}

void UuvCommNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  gps_ = *msg;
  last_gps_time_ = now();
  gps_received_ = true;
}

void UuvCommNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = *msg;
  last_pose_time_ = now();
}

void UuvCommNode::velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const auto & velocity = msg->twist.linear;
  const double speed = std::hypot(velocity.x, velocity.y, velocity.z);
  std::lock_guard<std::mutex> lock(mutex_);
  speed_mps_ = std::isfinite(speed) ? speed : 0.0;
  last_velocity_time_ = now();
}

void UuvCommNode::vehicleStatusCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  vehicle_status_ = *msg;
  vehicle_status_received_ = true;
}

void UuvCommNode::ringStateCallback(const banyu_msgs::msg::PipelineRingState::SharedPtr msg)
{
  using banyu_msgs::msg::PipelineRingState;
  using rx_msgs::msg::PipelineSegmentStatus;
  if (msg->index >= kMaxRingIndex) {
    RCLCPP_WARN(get_logger(), "Ignoring out-of-range pipeline ring index %u", msg->index);
    return;
  }
  if (msg->color != PipelineRingState::COLOR_RED &&
    msg->color != PipelineRingState::COLOR_GREEN && msg->color != PipelineRingState::COLOR_BLUE)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  segments_by_index_[msg->index] = msg->color == PipelineRingState::COLOR_RED ?
    PipelineSegmentStatus::PIPELINE_SEGMENT_DAMAGED :
    PipelineSegmentStatus::PIPELINE_SEGMENT_INTACT;
}

void UuvCommNode::surveyCallback(
  const std::shared_ptr<TriggerPipelineSurvey::Request>,
  std::shared_ptr<TriggerPipelineSurvey::Response> response)
{
  rx_msgs::msg::PipelineSurvey survey;
  survey.header.stamp = now();
  survey.header.frame_id = "uuv";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    survey.active_buoy_position = active_buoy_position_;
    std::vector<uint8_t> segments(
      kDefaultPipelineSegments, rx_msgs::msg::PipelineSegmentStatus::PIPELINE_SEGMENT_UNKNOWN);
    if (segments_by_index_.empty()) {
      segments = {1, 1, 1, 2};
    } else {
      for (const auto & ring : segments_by_index_) {
        if (ring.first < kDefaultPipelineSegments) {
          segments[ring.first] = ring.second;
        } else if (ring.first < kMaxRingIndex) {
          if (ring.first >= segments.size()) {
            segments.resize(
              static_cast<size_t>(ring.first) + 1,
              rx_msgs::msg::PipelineSegmentStatus::PIPELINE_SEGMENT_UNKNOWN);
          }
          segments[ring.first] = ring.second;
        }
      }
    }
    survey.segments = segments;
  }
  survey_pub_->publish(survey);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++mission_start_generation_;
    current_task_ = rx_msgs::msg::RxTask::TASK_NONE;
  }
  publishMissionStatus("Pipeline survey published");
  response->success = true;
  response->message = "Pipeline survey published";
}

}  // namespace banyu_communication

#ifndef BANYU_COMMUNICATION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_communication::UuvCommNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
