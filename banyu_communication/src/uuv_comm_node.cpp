#include "banyu_communication/uuv_comm_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cstdlib>
#include <filesystem>

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

}  // namespace

UuvCommNode::UuvCommNode(const rclcpp::NodeOptions & options)
: Node("uuv_comm_node", options)
{
  heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 2.0);
  lifecycle_timeout_s_ = declare_parameter<double>("timing.lifecycle_timeout_s", 2.0);
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");
  anchor_.latitude = declare_parameter<double>("anchor_latitude", 0.0);
  anchor_.longitude = declare_parameter<double>("anchor_longitude", 0.0);
  anchor_altitude_hae_m_ = declare_parameter<double>("anchor_altitude_hae_m", 0.0);
  if (!std::isfinite(heartbeat_rate_hz_) || heartbeat_rate_hz_ <= 0.0) {
    throw std::invalid_argument("heartbeat_rate_hz must be finite and positive");
  }
  if (!std::isfinite(lifecycle_timeout_s_) || lifecycle_timeout_s_ <= 0.0 ||
    lifecycle_topic.empty())
  {
    throw std::invalid_argument("Lifecycle topic and positive finite timeout are required");
  }
  const std::chrono::duration<double> period(1.0 / heartbeat_rate_hz_);
  if (!std::isfinite(period.count()) || period < std::chrono::nanoseconds(1) ||
    period >= std::chrono::nanoseconds::max())
  {
    throw std::invalid_argument("heartbeat_rate_hz is outside the timer's supported range");
  }
  const auto heartbeat_period = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  if (!validPosition(anchor_.latitude, anchor_.longitude) ||
    !std::isfinite(anchor_altitude_hae_m_))
  {
    throw std::invalid_argument("Anchor coordinates must be valid WGS-84 values");
  }

  heartbeat_pub_ = create_publisher<rx_msgs::msg::Heartbeat>(
    "/system/vehicle/uuv/heartbeat", 10);
  mission_status_pub_ = create_publisher<rx_msgs::msg::MissionStatus>(
    "/system/mission/status", 10);
  incident_ack_pub_ = create_publisher<rx_msgs::msg::IncidentAck>(
    "/system/vehicle/uuv/task4/incident_ack", 10);
  readiness_pub_ = create_publisher<rx_msgs::msg::ReadinessReport>(
    "/system/vehicle/uuv/task4/readiness", 10);
  SurveyLedger::Config ledger_config;
  ledger_config.evidence_sources = declare_parameter<std::vector<std::string>>(
    "survey.evidence_sources", {"pipeline_topology"});
  ledger_config.buoy_sources = declare_parameter<std::vector<std::string>>(
    "survey.buoy_sources",
    {"usv_pipeline_buoys", "uuv_buoy_localizer", "gcs_buoy_observer"});
  ledger_config.minimum_confidence = declare_parameter<double>("survey.minimum_confidence", 0.2);
  ledger_config.maximum_buoy_uncertainty_m = declare_parameter<double>(
    "survey.maximum_buoy_uncertainty_m", 5.0);
  ledger_config.ingress_max_age_s = declare_parameter<double>("survey.ingress_max_age_s", 2.0);
  ledger_config.minimum_progress_separation_m = declare_parameter<double>(
    "survey.minimum_progress_separation_m", 0.0);
  const auto tier = declare_parameter<int>("survey.task2_tier", 2);
  if (tier < 2 || tier > 4) {throw std::invalid_argument("Invalid declared Task 2 tier");}
  ledger_config.tier = static_cast<uint8_t>(tier);
  survey_ledger_ = std::make_unique<SurveyLedger>(ledger_config);
  SurfaceGuard::Config guard_config;
  guard_config.hold_s = declare_parameter<double>("survey.surface_hold_s", 1.0);
  guard_config.tolerance_m = declare_parameter<double>("survey.surface_tolerance_m", 0.2);
  guard_config.ttl_s = declare_parameter<double>("survey.depth_timeout_s", 0.5);
  guard_config.sources = declare_parameter<std::vector<std::string>>(
    "survey.depth_sources", {"pressure_depth"});
  surface_guard_ = std::make_unique<SurfaceGuard>(guard_config);
  const auto survey_topic = declare_parameter<std::string>(
    "topics.pipeline_survey", "/system/vehicle/uuv/task2/pipeline_survey");
  const auto delivery_topic = declare_parameter<std::string>(
    "topics.delivery", "/system/vehicle/uuv/task2/delivery");
  survey_pub_ = create_publisher<rx_msgs::msg::PipelineSurvey>(
    survey_topic, rclcpp::QoS(10).reliable().transient_local());
  delivery_pub_ = create_publisher<rx_msgs::msg::ResourceDelivery>(
    delivery_topic, rclcpp::QoS(10).reliable().transient_local());
  survey_progress_pub_ = create_publisher<banyu_msgs::msg::Task2SurveyProgress>(
    declare_parameter<std::string>("topics.survey_progress",
      "/banyu/communication/survey_progress"),
    rclcpp::QoS(1).reliable().transient_local());

  using std::placeholders::_1;
  command_sub_ = create_subscription<rx_msgs::msg::Command>(
    "/system/mission/command", 10, std::bind(&UuvCommNode::commandCallback, this, _1));
  course_sub_ = create_subscription<rx_msgs::msg::Course>(
    "/system/mission/course", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&UuvCommNode::courseCallback, this, _1));
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
  lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&UuvCommNode::lifecycleCallback, this, _1));
  evidence_sub_ = create_subscription<banyu_msgs::msg::Task2SurveyEvidence>(
    declare_parameter<std::string>("topics.survey_evidence", "/banyu/pipeline/survey_evidence"),
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&UuvCommNode::evidenceCallback, this, _1));
  water_depth_sub_ = create_subscription<banyu_msgs::msg::WaterDepth>(
    declare_parameter<std::string>("topics.water_depth", "/banyu/vehicle/water_depth"),
    rclcpp::QoS(5).reliable(), std::bind(&UuvCommNode::depthCallback, this, _1));
  survey_service_ = create_service<TriggerPipelineSurvey>(
    "/banyu/communication/trigger_pipeline_survey",
    [this](std::shared_ptr<rclcpp::Service<TriggerPipelineSurvey>> service,
    std::shared_ptr<rmw_request_id_t> header,
    std::shared_ptr<TriggerPipelineSurvey::Request> request) {
      surveyCallback(std::move(service), std::move(header), std::move(request));
    });
  heartbeat_timer_ = create_wall_timer(
    heartbeat_period, std::bind(&UuvCommNode::publishHeartbeat, this));
}

UuvCommNode::~UuvCommNode()
{
  if (heartbeat_timer_) {heartbeat_timer_->cancel();}
}

uint8_t UuvCommNode::robotStateLocked() const
{
  using rx_msgs::msg::RobotState;
  const auto current_time = std::chrono::steady_clock::now();
  const bool telemetry_fresh = state_received_ && vehicle_status_received_ &&
    std::chrono::duration<double>(current_time - last_state_received_).count() <=
    lifecycle_timeout_s_ &&
    std::chrono::duration<double>(current_time - last_status_received_).count() <=
    lifecycle_timeout_s_;
  // MAV_STATE 6-8 (emergency, poweroff, termination) mean the vehicle cannot operate. CRITICAL (5)
  // is a failsafe the vehicle can still navigate in; ArduSub reports it without a pilot joystick.
  if (!telemetry_fresh || !state_.connected || !state_.armed || state_.system_status >= 6 ||
    !vehicle_status_.armed || !vehicle_status_.fcu_connected ||
    !vehicle_status_.fcu_status_fresh || vehicle_status_.authority == "FAILSAFE")
  {
    return RobotState::STATE_KILLED;
  }
  if (vehicle_status_.operator_takeover) {
    return RobotState::STATE_MANUAL;
  }
  if (!vehicle_status_.rc_fresh) {
    return RobotState::STATE_KILLED;
  }
  if (state_.mode == "GUIDED" && vehicle_status_.mode == "GUIDED" &&
    lifecycleFreshLocked() && lifecycle_.autonomous_ready)
  {
    return RobotState::STATE_AUTO;
  }
  return RobotState::STATE_MANUAL;
}

bool UuvCommNode::lifecycleFreshLocked() const
{
  return lifecycle_received_ &&
         std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_lifecycle_received_).count() <= lifecycle_timeout_s_;
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
    heartbeat.current_task =
      lifecycleFreshLocked() ? current_task_ : rx_msgs::msg::RxTask::TASK_NONE;
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
  publishSurveyProgress();
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
    status.current_task = lifecycleFreshLocked() ? current_task_ : rx_msgs::msg::RxTask::TASK_NONE;
  }
  mission_status_pub_->publish(status);
}

void UuvCommNode::commandCallback(const rx_msgs::msg::Command::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  using rx_msgs::msg::Command;
  using rx_msgs::msg::VehicleType;
  if (msg->target_vehicle != VehicleType::TYPE_UUV &&
    msg->target_vehicle != VehicleType::TYPE_UNKNOWN)
  {
    return;
  }
  if (msg->command_type == Command::CMD_TASK4_ASSISTANCE ||
    msg->command_type == Command::CMD_TASK4_KEEP_OUT_ZONE ||
    msg->command_type == Command::CMD_TASK4_ALL_CLEAR)
  {
    rx_msgs::msg::IncidentAck ack;
    ack.header.stamp = now();
    ack.header.frame_id = "uuv";
    ack.command_seq = msg->command_seq;
    incident_ack_pub_->publish(ack);
    if (msg->command_type == Command::CMD_TASK4_ASSISTANCE) {
      publishMissionStatus("assistance execution unsupported/not ready");
    }
  }
}

void UuvCommNode::courseCallback(const rx_msgs::msg::Course::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  course_ = *msg;
  pinger_freq_hz_ = msg->pinger_freq_hz;
  survey_ledger_->setCourse(msg->course_id, msg->pinger_freq_hz);
}

void UuvCommNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = *msg;
  state_received_ = true;
  last_state_received_ = std::chrono::steady_clock::now();
}

void UuvCommNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  gps_ = *msg;
  last_gps_time_ = now();
  gps_received_ = true;
}

void UuvCommNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = *msg;
  last_pose_time_ = now();
}

void UuvCommNode::velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const auto & velocity = msg->twist.linear;
  const double speed = std::hypot(velocity.x, velocity.y, velocity.z);
  std::lock_guard<std::mutex> lock(mutex_);
  speed_mps_ = std::isfinite(speed) ? speed : 0.0;
  last_velocity_time_ = now();
}

void UuvCommNode::vehicleStatusCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  vehicle_status_ = *msg;
  vehicle_status_received_ = true;
  last_status_received_ = std::chrono::steady_clock::now();
}

void UuvCommNode::lifecycleCallback(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!msg) {
    return;
  }
  if ((msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ||
    msg->header.stamp.nanosec >= 1000000000U) {return;}
  const double source_age = (now() - rclcpp::Time(msg->header.stamp,
    get_clock()->get_clock_type())).seconds();
  if (source_age < 0.0 || source_age > lifecycle_timeout_s_) {return;}
  if (msg->authority_instance.empty() || msg->revision == 0 ||
    msg->phase > Lifecycle::INTERRUPTED ||
    (msg->phase >= Lifecycle::INITIALIZING && (msg->run_id == 0 || msg->epoch == 0)))
  {
    return;
  }
  bool notify = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_authorities_.count(msg->authority_instance) != 0) {
      return;
    }
    if (lifecycle_received_) {
      if (msg->epoch < lifecycle_.epoch ||
        (msg->epoch == lifecycle_.epoch && lifecycle_.run_id != 0 &&
        msg->run_id != lifecycle_.run_id) ||
        (msg->authority_instance == lifecycle_.authority_instance &&
        msg->revision <= lifecycle_.revision))
      {
        return;
      }
      if (msg->authority_instance != lifecycle_.authority_instance) {
        const bool live = msg->phase >= Lifecycle::INITIALIZING &&
          msg->phase <= Lifecycle::SURFACED;
        if (msg->epoch == lifecycle_.epoch && live && lifecycleFreshLocked()) {
          return;
        }
        retired_authorities_.insert(lifecycle_.authority_instance);
      }
    }
    const bool new_accepted_epoch = msg->epoch != 0 && msg->phase >= Lifecycle::INITIALIZING &&
      (!lifecycle_received_ || msg->epoch > lifecycle_.epoch);
    if (new_accepted_epoch) {
      const int64_t accepted_ns = static_cast<int64_t>(msg->accepted_at.sec) * 1000000000LL +
        msg->accepted_at.nanosec;
      survey_ledger_->beginRun(msg->run_id, msg->epoch, accepted_ns);
      surface_guard_->beginRun(msg->run_id, msg->epoch);
      evidence_revision_ = 0;
    }
    notify = !lifecycle_received_ || msg->event != Lifecycle::SNAPSHOT ||
      msg->run_id != lifecycle_.run_id || msg->phase != lifecycle_.phase ||
      msg->reason != lifecycle_.reason || msg->current_task != lifecycle_.current_task;
    lifecycle_ = *msg;
    lifecycle_received_ = true;
    last_lifecycle_received_ = std::chrono::steady_clock::now();
    run_id_ = msg->run_id;
    current_task_ = msg->phase >= Lifecycle::ACTIVE && msg->phase <= Lifecycle::SURFACED ?
      msg->current_task : rx_msgs::msg::RxTask::TASK_NONE;
  }
  if (notify) {
    publishMissionStatus(msg->reason.empty() ? "Mission lifecycle updated" : msg->reason,
        msg->run_id);
  }
}

void UuvCommNode::evidenceCallback(const banyu_msgs::msg::Task2SurveyEvidence::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!lifecycleFreshLocked() || lifecycle_.phase < Lifecycle::ACTIVE ||
    lifecycle_.phase > Lifecycle::SURFACED || msg->run_id != run_id_ ||
    msg->epoch != lifecycle_.epoch) {return;}
  const auto diagnostic = survey_ledger_->ingest(*msg, now().nanoseconds());
  if (diagnostic.empty()) {evidence_revision_ = msg->revision;} else {
    RCLCPP_WARN(get_logger(), "Survey evidence rejected: %s", diagnostic.c_str());
  }
  lock.unlock();
  publishSurveyProgress();
}

void UuvCommNode::depthCallback(const banyu_msgs::msg::WaterDepth::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!lifecycleFreshLocked() || (lifecycle_.phase != Lifecycle::SURVEY_COMPLETE &&
    lifecycle_.phase != Lifecycle::SURFACED)) {return;}
  const int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  surface_guard_->observe(*msg, now().nanoseconds(), steady_ns);
}

void UuvCommNode::publishSurveyProgress()
{
  std::lock_guard<std::mutex> lock(mutex_);
  banyu_msgs::msg::Task2SurveyProgress progress;
  progress.header.stamp = now();
  progress.authority_instance = lifecycle_.authority_instance;
  progress.run_id = run_id_;
  progress.epoch = lifecycle_.epoch;
  progress.evidence_revision = evidence_revision_;
  progress.survey_complete = lifecycleFreshLocked() && survey_ledger_->complete();
  progress.reason = survey_ledger_->reason();
  survey_progress_pub_->publish(progress);
}

void UuvCommNode::surveyCallback(
  std::shared_ptr<rclcpp::Service<TriggerPipelineSurvey>> service,
  std::shared_ptr<rmw_request_id_t> header,
  std::shared_ptr<TriggerPipelineSurvey::Request> request)
{
  if (!service || !header || !request) {
    return;
  }
  using Response = TriggerPipelineSurvey::Response;
  std::lock_guard<std::mutex> lock(mutex_);
  auto reply = [&](uint8_t code, const std::string & message, bool success = false) {
      Response response;
      response.success = success;
      response.result_code = code;
      response.message = message;
      response.report_id = request->report_id;
      service->send_response(*header, response);
    };

  if (request->run_id == 0 || request->epoch == 0 || request->run_id != run_id_ ||
    request->epoch != lifecycle_.epoch)
  {
    reply(Response::WRONG_RUN, "WRONG_RUN: accepted run/epoch required");
    return;
  }
  if (submitted_epochs_.count(request->epoch) > 0) {
    reply(Response::ALREADY_ACCEPTED, "Previously submitted report for this run", true);
    return;
  }
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!lifecycleFreshLocked() || !lifecycle_.autonomous_ready ||
    lifecycle_.phase < Lifecycle::ACTIVE || lifecycle_.phase > Lifecycle::SURFACED)
  {
    reply(Response::RUN_NOT_ACTIVE, "RUN_NOT_ACTIVE: fresh eligible mission required");
    return;
  }
  // ".best_effort": the mission gave up on a full survey (pipe not found, traverse lost); report what was seen.
  const std::string kBestEffort = ".best_effort";
  const bool best_effort = request->report_id.size() > kBestEffort.size() &&
    request->report_id.compare(request->report_id.size() - kBestEffort.size(), kBestEffort.size(), kBestEffort) == 0;
  if (!survey_ledger_->complete() && !(best_effort && survey_ledger_->bestEffortAvailable())) {
    reply(Response::SURVEY_INVALID, survey_ledger_->reason());
    return;
  }
  if (lifecycle_.phase != Lifecycle::SURFACED) {
    reply(Response::NOT_SURFACED, "NOT_SURFACED: current Surface action has not succeeded");
    return;
  }
  const int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (!surface_guard_->ready(now().nanoseconds(), steady_ns)) {
    reply(Response::SURFACE_TELEMETRY_STALE, surface_guard_->reason());
    return;
  }

  SurveyLedger::Report report;
  try {
    report = best_effort ? survey_ledger_->buildBestEffort(request->report_id) :
      survey_ledger_->build(request->report_id);
    if (best_effort && !survey_ledger_->complete()) {
      RCLCPP_WARN(get_logger(), "Best-effort Core survey (incomplete: %s)", survey_ledger_->reason().c_str());
    }
  } catch (const std::exception & error) {
    reply(Response::SURVEY_INVALID, error.what());
    return;
  }

  auto survey = report.toPipelineSurvey();
  survey.header.stamp = now();
  survey.header.frame_id = "uuv";
  survey_pub_->publish(survey);

  if (report.tier > 2) {
    auto delivery = report.toResourceDelivery();
    delivery.header = survey.header;
    delivery_pub_->publish(delivery);
  }

  submitted_epochs_.insert(request->epoch);
  reply(Response::ACCEPTED, "Report published to GCS", true);
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
