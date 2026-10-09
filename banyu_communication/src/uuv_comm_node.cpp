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
#include "banyu_msgs/task2_submission.hpp"

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
    "survey.buoy_sources", {"uuv_buoy_localizer", "gcs_buoy_observer"});
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
  const char * user_directory = std::getenv("HOME");
  auto outbox_path = declare_parameter<std::string>("survey.outbox_path",
    std::string(user_directory ? user_directory : "/tmp") + "/.ros/banyu/uuv_reports.outbox");
  if (user_directory && outbox_path.rfind("~/", 0) == 0) {
    outbox_path = std::string(user_directory) + outbox_path.substr(1);
  }
  if (outbox_path.empty()) {throw std::invalid_argument("A durable report outbox is required");}
  std::error_code directory_error;
  std::filesystem::create_directories(std::filesystem::path(outbox_path).parent_path(),
      directory_error);
  if (directory_error) {
    RCLCPP_ERROR(get_logger(), "Cannot provision report outbox: %s",
        directory_error.message().c_str());
  }
  survey_outbox_ = std::make_unique<SurveyOutbox>(outbox_path);
  publish_legacy_copies_ = declare_parameter<bool>("survey.publish_legacy_copies", false);
  survey_pub_ = create_publisher<rx_msgs::msg::PipelineSurvey>(
    "/system/vehicle/uuv/task2/pipeline_survey", rclcpp::QoS(1).reliable().transient_local());
  delivery_pub_ = create_publisher<rx_msgs::msg::ResourceDelivery>(
    "/system/vehicle/uuv/task2/delivery", rclcpp::QoS(1).reliable().transient_local());
  submission_pub_ = create_publisher<banyu_msgs::msg::Task2Submission>(
    declare_parameter<std::string>("topics.task2_submission",
      "/system/vehicle/uuv/task2/submission"),
    rclcpp::QoS(32).reliable().transient_local());
  survey_progress_pub_ = create_publisher<banyu_msgs::msg::Task2SurveyProgress>(
    declare_parameter<std::string>("topics.survey_progress",
      "/banyu/communication/survey_progress"),
    rclcpp::QoS(1).reliable().transient_local());
  seal_client_ = create_client<banyu_msgs::srv::SealTask2Report>(
    declare_parameter<std::string>("services.seal_task2_report",
      "/banyu/mission/seal_task2_report"));

  using std::placeholders::_1;
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
  if (seal_client_) {seal_client_->prune_pending_requests();}
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
  // MAV_STATE values 5 through 8 indicate critical, emergency, poweroff or termination.
  if (!telemetry_fresh || !state_.connected || !state_.armed || state_.system_status >= 5 ||
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
  survey_ledger_->setCourse(msg->course_id, msg->pinger_freq_hz);
}

void UuvCommNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = *msg;
  state_received_ = true;
  last_state_received_ = std::chrono::steady_clock::now();
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
  last_status_received_ = std::chrono::steady_clock::now();
}

void UuvCommNode::lifecycleCallback(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
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
      recovered_outbox_ = false;
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
    if (current_task_ == rx_msgs::msg::RxTask::TASK_UNKNOWN) {
      current_task_ = rx_msgs::msg::RxTask::TASK_NONE;
    }
  }
  if (notify) {
    publishMissionStatus(msg->reason.empty() ? "Mission lifecycle updated" : msg->reason,
        msg->run_id);
  }
}

void UuvCommNode::evidenceCallback(const banyu_msgs::msg::Task2SurveyEvidence::SharedPtr msg)
{
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
  if (!recovered_outbox_ && lifecycleFreshLocked()) {
    for (const auto & entry : survey_outbox_->entries()) {
      if (entry.authorized && entry.submission.run_id == run_id_ &&
        entry.submission.epoch == lifecycle_.epoch &&
        (lifecycle_.phase == banyu_msgs::msg::RunLifecycle::SURFACED ||
        lifecycle_.phase == banyu_msgs::msg::RunLifecycle::SUCCEEDED))
      {
        publishSubmission(entry.submission);
      }
    }
    recovered_outbox_ = true;
  }
  std::vector<std::shared_ptr<PendingSurvey>> expired;
  for (const auto & item : pending_surveys_) {
    if (std::chrono::steady_clock::now() - item.second->started > std::chrono::seconds(3)) {
      expired.push_back(item.second);
    }
  }
  for (const auto & pending : expired) {
    finishSurvey(pending, false, TriggerPipelineSurvey::Response::AUTHORIZATION_REJECTED,
      "MISSION_SEAL_TIMEOUT: staged report retained; forwarding unconfirmed");
  }
}

void UuvCommNode::surveyCallback(
  std::shared_ptr<rclcpp::Service<TriggerPipelineSurvey>> service,
  std::shared_ptr<rmw_request_id_t> header,
  std::shared_ptr<TriggerPipelineSurvey::Request> request)
{
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
  const auto existing = survey_outbox_->find(request->run_id, request->epoch,
    banyu_msgs::msg::Task2Submission::SURVEY);
  if (existing && existing->submission.report_id != request->report_id) {
    reply(Response::AUTHORIZATION_REJECTED, "REPORT_ID_CONFLICT: logical report already staged");
    return;
  }
  if (existing && existing->authorized) {
    const auto delivery = survey_outbox_->find(request->run_id, request->epoch,
      banyu_msgs::msg::Task2Submission::DELIVERY);
    if (existing->submission.tier == 2 || (delivery && delivery->authorized)) {
      reply(Response::ALREADY_ACCEPTED,
        "Previously accepted into durable local outbox; GCS forwarding unconfirmed", true);
      return;
    }
  }
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (request->run_id == 0 || request->epoch == 0 || request->run_id != run_id_ ||
    request->epoch != lifecycle_.epoch)
  {reply(Response::WRONG_RUN, "WRONG_RUN: accepted run/epoch required"); return;}
  if (!lifecycleFreshLocked() || !lifecycle_.autonomous_ready ||
    lifecycle_.phase < Lifecycle::ACTIVE || lifecycle_.phase > Lifecycle::SURFACED)
  {reply(Response::RUN_NOT_ACTIVE, "RUN_NOT_ACTIVE: fresh eligible mission required"); return;}
  if (!survey_ledger_->complete()) {
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
  const auto pending_it = pending_surveys_.find(request->report_id);
  if (pending_it != pending_surveys_.end()) {
    pending_it->second->waiters.push_back({std::move(service), std::move(header)});
    return;
  }
  if (!seal_client_->service_is_ready()) {
    reply(Response::AUTHORIZATION_REJECTED, "MISSION_SEAL_UNAVAILABLE");
    return;
  }
  auto pending = std::make_shared<PendingSurvey>();
  try {
    pending->survey = survey_ledger_->build(request->report_id);
  } catch (const std::exception & error) {
    reply(Response::SURVEY_INVALID, error.what());
    return;
  }
  if (!banyu_msgs::task2::validSubmission(pending->survey)) {
    reply(Response::SURVEY_INVALID, "INVALID_REPORT_ID_OR_PAYLOAD");
    return;
  }
  const auto staged = survey_outbox_->stage(pending->survey);
  if (!staged.success) {reply(Response::OUTBOX_UNAVAILABLE, staged.message); return;}
  if (pending->survey.tier > 2) {
    pending->delivery = pending->survey;
    pending->delivery->kind = banyu_msgs::msg::Task2Submission::DELIVERY;
    pending->delivery->report_id += "-delivery";
    const auto delivery_stage = survey_outbox_->stage(*pending->delivery);
    if (!delivery_stage.success) {
      reply(Response::OUTBOX_UNAVAILABLE, delivery_stage.message); return;
    }
  }
  pending->waiters.push_back({std::move(service), std::move(header)});
  pending->started = std::chrono::steady_clock::now();
  pending_surveys_[request->report_id] = pending;
  requestSeal(pending, false);
}

void UuvCommNode::requestSeal(const std::shared_ptr<PendingSurvey> & pending, bool delivery)
{
  auto request = std::make_shared<banyu_msgs::srv::SealTask2Report::Request>();
  request->submission = delivery ? *pending->delivery : pending->survey;
  const auto report_id = request->submission.report_id;
  const auto future = seal_client_->async_send_request(request,
      [this, pending, delivery, report_id](
        rclcpp::Client<banyu_msgs::srv::SealTask2Report>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending->completed) {return;}
        try {
          const auto response = future.get();
          if (!response->success || response->authorization_revision == 0) {
            finishSurvey(pending, false, TriggerPipelineSurvey::Response::AUTHORIZATION_REJECTED,
            "MISSION_SEAL_REJECTED: " + response->message);
            return;
          }
          using Lifecycle = banyu_msgs::msg::RunLifecycle;
          if (pending->survey.run_id != run_id_ || pending->survey.epoch != lifecycle_.epoch ||
          !lifecycleFreshLocked() || lifecycle_.phase != Lifecycle::SURFACED)
          {
            finishSurvey(pending, false, TriggerPipelineSurvey::Response::RUN_NOT_ACTIVE,
            "RUN_NOT_ACTIVE: mission state changed before authorization completed");
            return;
          }
          const auto result = survey_outbox_->authorize(report_id,
          response->authorization_revision);
          if (!result.success || !result.entry) {
            finishSurvey(pending, false, TriggerPipelineSurvey::Response::OUTBOX_UNAVAILABLE,
            result.message);
            return;
          }
          if (!result.duplicate) {publishSubmission(result.entry->submission);}
          if (!delivery && pending->delivery) {requestSeal(pending, true);} else {
            finishSurvey(pending, true, TriggerPipelineSurvey::Response::ACCEPTED,
            "Accepted into durable local outbox; GCS forwarding unconfirmed");
          }
        } catch (const std::exception & error) {
          finishSurvey(pending, false, TriggerPipelineSurvey::Response::AUTHORIZATION_REJECTED,
          "MISSION_SEAL_FAILED: " + std::string(error.what()));
        }
    });
  pending->seal_request_id = future.request_id;
}

void UuvCommNode::finishSurvey(
  const std::shared_ptr<PendingSurvey> & pending,
  bool success, uint8_t code, const std::string & message)
{
  if (pending->completed) {return;}
  pending->completed = true;
  if (pending->seal_request_id != 0) {
    seal_client_->remove_pending_request(pending->seal_request_id);
  }
  for (const auto & waiter : pending->waiters) {
    TriggerPipelineSurvey::Response response;
    response.success = success;
    response.result_code = code;
    response.message = message;
    response.report_id = pending->survey.report_id;
    waiter.service->send_response(*waiter.header, response);
  }
  pending_surveys_.erase(pending->survey.report_id);
}

void UuvCommNode::publishSubmission(const banyu_msgs::msg::Task2Submission & value)
{
  auto submission = value;
  submission.header.stamp = now();
  submission.header.frame_id = "uuv";
  submission_pub_->publish(submission);
  if (!publish_legacy_copies_) {return;}
  if (submission.kind == banyu_msgs::msg::Task2Submission::SURVEY) {
    rx_msgs::msg::PipelineSurvey survey;
    survey.header = submission.header;
    survey.active_buoy_position.latitude = submission.buoy_latitude;
    survey.active_buoy_position.longitude = submission.buoy_longitude;
    survey.segments.assign(submission.original_conditions.begin(),
        submission.original_conditions.end());
    survey_pub_->publish(survey);
  } else {
    rx_msgs::msg::ResourceDelivery delivery;
    delivery.header = submission.header;
    delivery.task = rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR;
    delivery.resource_color = submission.resource_color;
    delivery.delivery_circle_color = submission.delivery_circle_color;
    delivery_pub_->publish(delivery);
  }
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
