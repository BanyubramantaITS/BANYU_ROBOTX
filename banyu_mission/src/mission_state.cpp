#include "banyu_mission/mission_state.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace banyu_mission
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite(double value)
{
  return std::isfinite(value);
}
}  // namespace

MissionState::MissionState(rclcpp::Node * node, bool subscribe)
: node_(node), last_pipeline_(node->now()), last_state_(node->now())
{
  if (!subscribe) {
    return;
  }
  const auto vehicle_status_topic = node_->declare_parameter<std::string>(
    "topics.vehicle_status", "/banyu/vehicle_status");
  const auto tracker_state_topic = node_->declare_parameter<std::string>(
    "topics.tracker_state", "/banyu/pipeline/ring_state");
  const bool tracker_state_transient_local = node_->declare_parameter<bool>(
    "topics.tracker_state_transient_local", false);
  const auto pipeline_topic = node_->declare_parameter<std::string>(
    "topics.pipeline_observation", "/banyu/pipeline/observation");
  const auto ring_topic = node_->declare_parameter<std::string>(
    "topics.ring_observation", "/banyu/pipeline/ring_observation");
  const auto end_topic = node_->declare_parameter<std::string>(
    "topics.pipeline_end", "/banyu/pipeline/end");

  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::VehicleStatus>(
      vehicle_status_topic, rclcpp::QoS(5),
      std::bind(&MissionState::stateCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineObservation>(
      pipeline_topic, rclcpp::QoS(5),
      std::bind(&MissionState::pipelineCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineRingObservation>(
      ring_topic, rclcpp::QoS(5),
      std::bind(&MissionState::ringCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineRingState>(
      tracker_state_topic,
      tracker_state_transient_local ? rclcpp::QoS(5).reliable().transient_local() :
      rclcpp::QoS(5).reliable(),
      std::bind(&MissionState::ringStateCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<std_msgs::msg::Bool>(
      end_topic, rclcpp::QoS(5),
      std::bind(&MissionState::endCallback, this, std::placeholders::_1)));
}

void MissionState::configure(
  double gps_timeout_s, double pipeline_timeout_s, double state_timeout_s,
  int rc_interrupt_channel, int rc_interrupt_threshold,
  std::string autonomy_mode, std::string autonomy_authority)
{
  std::lock_guard<std::mutex> lock(mutex_);
  gps_timeout_s_ = finite(gps_timeout_s) && gps_timeout_s > 0.0 ? gps_timeout_s : 0.01;
  pipeline_timeout_s_ = finite(pipeline_timeout_s) &&
    pipeline_timeout_s > 0.0 ? pipeline_timeout_s : 0.01;
  state_timeout_s_ = finite(state_timeout_s) && state_timeout_s > 0.0 ? state_timeout_s : 0.01;
  rc_interrupt_channel_ = std::max(0, rc_interrupt_channel);
  rc_interrupt_threshold_ = rc_interrupt_threshold;
  autonomy_mode_ = std::move(autonomy_mode);
  autonomy_authority_ = std::move(autonomy_authority);
}

bool MissionState::validateMissionCoordinates(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude)
{
  return finite(start_latitude) && finite(start_longitude) && finite(end_latitude) &&
         finite(end_longitude) && start_latitude >= -90.0 && start_latitude <= 90.0 &&
         end_latitude >= -90.0 && end_latitude <= 90.0 && start_longitude >= -180.0 &&
         start_longitude <= 180.0 && end_longitude >= -180.0 && end_longitude <= 180.0 &&
         (start_latitude != end_latitude || start_longitude != end_longitude);
}

bool MissionState::deriveEnuInitialBearing(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude, double & bearing)
{
  if (!validateMissionCoordinates(start_latitude, start_longitude, end_latitude, end_longitude)) {
    return false;
  }
  const double lat0 = start_latitude * kPi / 180.0;
  const double lat1 = end_latitude * kPi / 180.0;
  double dlon = (end_longitude - start_longitude) * kPi / 180.0;
  while (dlon > kPi) {dlon -= 2.0 * kPi;}
  while (dlon < -kPi) {dlon += 2.0 * kPi;}
  const double east = dlon * std::cos((lat0 + lat1) * 0.5);
  const double north = (lat1 - lat0);
  bearing = std::atan2(north, east);
  if (bearing < 0.0) {bearing += 2.0 * kPi;}
  return finite(bearing);
}

bool MissionState::configureMissionCoordinates(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude)
{
  std::lock_guard<std::mutex> lock(mutex_);
  start_latitude_ = start_latitude;
  start_longitude_ = start_longitude;
  end_latitude_ = end_latitude;
  end_longitude_ = end_longitude;
  mission_coordinates_valid_ = deriveEnuInitialBearing(
    start_latitude, start_longitude, end_latitude, end_longitude, mission_heading_);
  return mission_coordinates_valid_;
}

bool MissionState::resetForStart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ != Lifecycle::Idle || !mission_coordinates_valid_) {
    return false;
  }
  mission_coordinates_valid_ = deriveEnuInitialBearing(
    start_latitude_, start_longitude_, end_latitude_, end_longitude_, mission_heading_);
  if (!mission_coordinates_valid_) {
    return false;
  }
  mission_started_ = true;
  lifecycle_ = Lifecycle::Active;
  abort_requested_ = false;
  pipeline_ended_ = false;
  pipeline_seen_ = false;
  red_pending_ = false;
  ring_visible_ = false;
  ring_color_ = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  return true;
}

void MissionState::requestAbort()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {
    abort_requested_ = true;
  }
}

bool MissionState::requestStart()
{
  return resetForStart();
}

void MissionState::markSucceeded()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {lifecycle_ = Lifecycle::Succeeded;}
}

void MissionState::markFailed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {lifecycle_ = Lifecycle::Failed;}
}

void MissionState::markInterrupted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {lifecycle_ = Lifecycle::Interrupted;}
}

MissionState::Lifecycle MissionState::lifecycle() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return lifecycle_;
}

double MissionState::missionHeading() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mission_heading_;
}

MissionState::Snapshot MissionState::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_->now();
  const bool state_fresh = fcu_status_fresh_ && (now - last_state_).seconds() <= state_timeout_s_;
  const bool pipeline_fresh = pipeline_seen_ &&
    (now - last_pipeline_).seconds() <= pipeline_timeout_s_;
  Snapshot value;
  value.fcu_connected = fcu_connected_;
  value.fcu_status_fresh = state_fresh;
  value.rc_fresh = rc_fresh_;
  value.operator_takeover = operator_takeover_;
  value.manual_interrupt = manual_interrupt_;
  value.abort_requested = abort_requested_;
  value.mission_started = mission_started_;
  value.gps_available = gps_valid_;
  value.pipeline_fresh = pipeline_fresh && pipeline_tracked_;
  value.pipeline_lost = !value.pipeline_fresh;
  value.pipeline_ended = pipeline_ended_;
  value.red_pending = red_pending_;
  value.red_record_index = red_record_index_;
  value.ring_visible = ring_visible_;
  value.ring_color = ring_color_;
  value.mode = mode_;
  value.authority = authority_;
  return value;
}

bool MissionState::autonomyInterrupted() const
{
  const auto value = snapshot();
  if (!value.mission_started || lifecycle() != Lifecycle::Active) {return false;}
  const bool ready = value.fcu_connected && value.fcu_status_fresh && value.rc_fresh &&
    !value.operator_takeover && value.mode == autonomy_mode_ &&
    (autonomy_authority_.empty() || value.authority == autonomy_authority_);
  return value.abort_requested || !ready;
}

bool MissionState::vehicleReady() const
{
  const auto value = snapshot();
  return value.fcu_connected && value.fcu_status_fresh && value.rc_fresh &&
         !value.operator_takeover && value.mode == autonomy_mode_ &&
         (autonomy_authority_.empty() || value.authority == autonomy_authority_);
}

bool MissionState::gpsAvailable() const {return snapshot().gps_available;}
bool MissionState::pipelineLost() const {return snapshot().pipeline_lost;}
bool MissionState::pipelineEnded() const {return snapshot().pipeline_ended;}
bool MissionState::redPending() const {return snapshot().red_pending;}
uint32_t MissionState::pendingRedIndex() const {return snapshot().red_record_index;}

bool MissionState::acknowledgeRed(uint32_t index)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!red_pending_ || red_record_index_ != index) {return false;}
  red_pending_ = false;
  return true;
}

void MissionState::stateCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fcu_connected_ = msg->fcu_connected;
  fcu_status_fresh_ = msg->fcu_status_fresh;
  rc_fresh_ = msg->rc_fresh;
  operator_takeover_ = msg->operator_takeover;
  mode_ = msg->mode;
  authority_ = msg->authority;
  gps_valid_ = msg->gps_valid && msg->gps_fresh;
  manual_interrupt_ = msg->operator_takeover;
  last_state_ = node_->now();
}

void MissionState::pipelineCallback(const banyu_msgs::msg::PipelineObservation::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_seen_ = true;
  pipeline_tracked_ = msg->status == banyu_msgs::msg::PipelineObservation::TRACKED &&
    finite(msg->offset_x) && finite(msg->offset_y) && finite(msg->heading_error_rad) &&
    finite(msg->confidence) && msg->confidence >= 0.0F;
  last_pipeline_ = node_->now();
}

void MissionState::ringCallback(const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ring_visible_ = msg->visible;
  ring_color_ = msg->visible ? msg->color : banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
}

void MissionState::ringStateCallback(const banyu_msgs::msg::PipelineRingState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ring_color_ = msg->color;
  if (msg->color == banyu_msgs::msg::PipelineRingState::COLOR_RED && finite(msg->confidence) &&
    msg->confidence > 0.0F)
  {
    red_pending_ = true;
    red_record_index_ = msg->index;
  }
}

void MissionState::endCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_ended_ = msg->data;
}

}  // namespace banyu_mission
