#include "banyu_mission/mission_state.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace banyu_mission
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite(double value) {return std::isfinite(value);}
}  // namespace

MissionState::MissionState(rclcpp::Node * node, bool subscribe)
: node_(node), last_pipeline_(node->now()), last_state_(node->now())
{
  if (!subscribe) {
    return;
  }
  const auto vehicle_status_topic = node_->declare_parameter<std::string>(
    "topics.vehicle_status",
    "/banyu/vehicle_status");
  const auto tracker_state_topic = node_->declare_parameter<std::string>(
    "topics.tracker_state",
    "/banyu/pipeline/ring_state");
  const bool tracker_state_transient_local = node_->declare_parameter<bool>(
    "topics.tracker_state_transient_local", false);
  const auto pipeline_topic = node_->declare_parameter<std::string>(
    "topics.pipeline_observation",
    "/banyu/pipeline/observation");
  const auto ring_topic = node_->declare_parameter<std::string>(
    "topics.ring_observation",
    "/banyu/pipeline/ring_observation");
  const auto end_topic = node_->declare_parameter<std::string>(
    "topics.pipeline_end",
    "/banyu/pipeline/end");
  const auto end_state_topic = node_->declare_parameter<std::string>(
    "topics.pipeline_end_state", "/banyu/pipeline/end_state");

  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::VehicleStatus>(
      vehicle_status_topic, rclcpp::QoS(5),
      std::bind(&MissionState::stateCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineObservation>(
      pipeline_topic, rclcpp::SensorDataQoS(),
      std::bind(&MissionState::pipelineCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineRingObservation>(
      ring_topic, rclcpp::SensorDataQoS(),
      std::bind(&MissionState::ringCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineRingState>(
      tracker_state_topic,
      tracker_state_transient_local ? rclcpp::QoS(5).reliable().transient_local() : rclcpp::QoS(5).
      reliable(),
      std::bind(&MissionState::ringStateCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<std_msgs::msg::Bool>(
      end_topic, rclcpp::QoS(5),
      std::bind(&MissionState::endCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineEndState>(
      end_state_topic, rclcpp::QoS(5).reliable().transient_local(),
      std::bind(&MissionState::endStateCallback, this, std::placeholders::_1)));
}

void MissionState::configure(
  double gps_timeout_s, double pipeline_timeout_s, double state_timeout_s, int rc_interrupt_channel,
  int rc_interrupt_threshold, std::string autonomy_mode, std::string autonomy_authority,
  bool require_gps)
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
  gps_required_ = require_gps;
}

bool MissionState::validateMissionCoordinates(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude)
{
  return finite(start_latitude) && finite(start_longitude) && finite(end_latitude) && finite(
    end_longitude) && start_latitude >= -90.0 &&
         start_latitude <= 90.0 && end_latitude >= -90.0 && end_latitude <= 90.0 &&
         start_longitude >= -180.0 && start_longitude <= 180.0 &&
         end_longitude >= -180.0 && end_longitude <= 180.0 &&
         (start_latitude != end_latitude || start_longitude != end_longitude);
}

bool MissionState::deriveEnuInitialBearing(
  double start_latitude, double start_longitude, double end_latitude, double end_longitude,
  double & bearing)
{
  if (!validateMissionCoordinates(start_latitude, start_longitude, end_latitude, end_longitude)) {
    return false;
  }
  const double lat0 = start_latitude * kPi / 180.0;
  const double lat1 = end_latitude * kPi / 180.0;
  double dlon = (end_longitude - start_longitude) * kPi / 180.0;
  while (dlon > kPi) {
    dlon -= 2.0 * kPi;
  }
  while (dlon < -kPi) {
    dlon += 2.0 * kPi;
  }
  const double east = dlon * std::cos((lat0 + lat1) * 0.5);
  const double north = (lat1 - lat0);
  bearing = std::atan2(north, east);
  if (bearing < 0.0) {
    bearing += 2.0 * kPi;
  }
  return finite(bearing);
}

bool MissionState::configureMissionCoordinates(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude, bool require_coordinates)
{
  std::lock_guard<std::mutex> lock(mutex_);
  start_latitude_ = start_latitude;
  start_longitude_ = start_longitude;
  end_latitude_ = end_latitude;
  end_longitude_ = end_longitude;
  mission_coordinates_required_ = require_coordinates;
  mission_coordinates_valid_ = deriveEnuInitialBearing(
    start_latitude, start_longitude,
    end_latitude, end_longitude,
    mission_heading_);
  return !mission_coordinates_required_ || mission_coordinates_valid_;
}

bool MissionState::resetForStart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if ((lifecycle_ != Lifecycle::Idle &&
    lifecycle_ != Lifecycle::Interrupted &&
    lifecycle_ != Lifecycle::Failed) ||
    (mission_coordinates_required_ && !mission_coordinates_valid_))
  {
    return false;
  }
  if (mission_coordinates_required_) {
    mission_coordinates_valid_ = deriveEnuInitialBearing(
      start_latitude_, start_longitude_,
      end_latitude_, end_longitude_,
      mission_heading_);
    if (!mission_coordinates_valid_) {
      return false;
    }
  } else {
    mission_heading_ = 0.0;
  }
  mission_started_ = true;
  autonomy_engaged_ = false;
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

bool MissionState::requestStart() {return resetForStart();}

void MissionState::requireRunContext(bool required)
{
  std::lock_guard<std::mutex> lock(mutex_);
  require_run_context_ = required;
}

void MissionState::prepareAutonomy()
{
  std::lock_guard<std::mutex> lock(mutex_);
  autonomy_engaged_ = false;
}

bool MissionState::acceptRun(uint32_t run_id, uint64_t epoch)
{
  if (run_id == 0 || epoch == 0) {return false;}
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lifecycle_ == Lifecycle::Active || epoch <= epoch_) {return false;}
    lifecycle_ = Lifecycle::Idle;
    run_id_ = run_id;
    epoch_ = epoch;
    run_started_at_ = node_->now();
  }
  const bool accepted = resetForStart();
  if (accepted) {markAutonomyEngaged();}
  return accepted;
}

void MissionState::markSucceeded()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {
    lifecycle_ = Lifecycle::Succeeded;
    mission_started_ = false;
  }
}

void MissionState::markFailed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {
    lifecycle_ = Lifecycle::Failed;
    mission_started_ = false;
  }
}

void MissionState::markInterrupted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {
    lifecycle_ = Lifecycle::Interrupted;
    mission_started_ = false;
  }
}

void MissionState::markAutonomyEngaged()
{
  std::lock_guard<std::mutex> lock(mutex_);
  autonomy_engaged_ = true;
}

bool MissionState::autonomyEngaged() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return autonomy_engaged_;
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

uint32_t MissionState::currentRunId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return run_id_;
}

uint64_t MissionState::currentRunEpoch() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return epoch_;
}

MissionState::Snapshot MissionState::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_->now();
  const bool state_fresh = fcu_status_fresh_ &&
    state_received_at_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() -
    state_received_at_).count() <= state_timeout_s_;
  const bool pipeline_fresh = pipeline_seen_ &&
    (now - last_pipeline_).seconds() >= 0.0 &&
    (now - last_pipeline_).seconds() <= pipeline_timeout_s_;
  Snapshot value;
  value.fcu_connected = fcu_connected_;
  value.armed = armed_;
  value.fcu_status_fresh = state_fresh;
  value.rc_fresh = rc_fresh_;
  value.operator_takeover = operator_takeover_;
  value.manual_interrupt = manual_interrupt_;
  value.abort_requested = abort_requested_;
  value.mission_started = mission_started_;
  value.gps_available = gps_valid_ && state_fresh;
  value.pipeline_fresh = pipeline_fresh && pipeline_tracked_;
  value.pipeline_lost = !value.pipeline_fresh;
  value.pipeline_ended = pipeline_ended_;
  value.red_pending = red_pending_;
  value.red_record_index = red_record_index_;
  value.ring_visible = ring_visible_;
  value.ring_color = ring_color_;
  value.mode = mode_;
  value.authority = authority_;
  value.run_id = run_id_;
  value.epoch = epoch_;
  return value;
}

bool MissionState::autonomyInterrupted() const
{
  const auto value = snapshot();
  if (!value.mission_started || lifecycle() != Lifecycle::Active) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (value.abort_requested || value.operator_takeover || value.manual_interrupt) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Autonomy interrupted: abort=%d, takeover=%d, manual_interrupt=%d",
      value.abort_requested, value.operator_takeover, value.manual_interrupt);
    return true;
  }

  if (autonomy_engaged_) {
    const bool mode_ok = (value.mode == autonomy_mode_ &&
      (autonomy_authority_.empty() || value.authority == autonomy_authority_));
    const bool ready = value.fcu_connected && value.fcu_status_fresh && value.rc_fresh &&
      value.armed && mode_ok;
    if (!ready) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Autonomy interrupted: connected=%d, state_fresh=%d, rc_fresh=%d, mode_ok=%d (mode=%s expected=%s)",
        value.fcu_connected, value.fcu_status_fresh, value.rc_fresh,
        mode_ok, value.mode.c_str(), autonomy_mode_.c_str());
      return true;
    }
  }

  return false;
}

bool MissionState::vehicleReady() const
{
  const auto value = snapshot();
  std::lock_guard<std::mutex> lock(mutex_);
  const bool mode_ok = !autonomy_engaged_ ||
    (value.armed && value.mode == autonomy_mode_ &&
    (autonomy_authority_.empty() || value.authority == autonomy_authority_));
  return value.fcu_connected && value.fcu_status_fresh && value.rc_fresh &&
         !value.operator_takeover && mode_ok;
}

bool MissionState::gpsAvailable() const {return snapshot().gps_available;}
bool MissionState::gpsRequired() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return gps_required_;
}
bool MissionState::pipelineLost() const {return snapshot().pipeline_lost;}
bool MissionState::pipelineEnded() const {return snapshot().pipeline_ended;}
bool MissionState::redPending() const {return snapshot().red_pending;}
uint32_t MissionState::pendingRedIndex() const {return snapshot().red_record_index;}

bool MissionState::acknowledgeRed(uint32_t index)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!red_pending_ || red_record_index_ != index) {
    return false;
  }
  red_pending_ = false;
  return true;
}

void MissionState::stateCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fcu_connected_ = msg->fcu_connected;
  armed_ = msg->armed;
  fcu_status_fresh_ = msg->fcu_status_fresh;
  rc_fresh_ = msg->rc_fresh;
  operator_takeover_ = msg->operator_takeover;
  mode_ = msg->mode;
  authority_ = msg->authority;
  gps_valid_ = msg->gps_valid && msg->gps_fresh;
  manual_interrupt_ = msg->operator_takeover;
  last_state_ = node_->now();
  state_received_at_ = std::chrono::steady_clock::now();
}

void MissionState::pipelineCallback(const banyu_msgs::msg::PipelineObservation::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (require_run_context_ && (!mission_started_ ||
    rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type()) < run_started_at_))
  {return;}
  pipeline_seen_ = true;
  pipeline_tracked_ = msg->status == banyu_msgs::msg::PipelineObservation::TRACKED && finite(
    msg->offset_x) && finite(msg->offset_y) &&
    finite(msg->heading_error_rad) && finite(msg->confidence) && msg->confidence >= 0.0F;
  last_pipeline_ = node_->now();
}

void MissionState::ringCallback(const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (require_run_context_ && (!mission_started_ ||
    rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type()) < run_started_at_))
  {return;}
  ring_visible_ = msg->visible;
  ring_color_ = msg->visible ? msg->color : banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
}

void MissionState::ringStateCallback(const banyu_msgs::msg::PipelineRingState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (require_run_context_ && (msg->run_id != run_id_ || msg->epoch != epoch_ ||
    !mission_started_)) {return;}
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
  if (require_run_context_) {return;}
  pipeline_ended_ = msg->data;
}

void MissionState::endStateCallback(const banyu_msgs::msg::PipelineEndState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!require_run_context_ || !mission_started_ || msg->run_id != run_id_ ||
    msg->epoch != epoch_) {return;}
  pipeline_ended_ = msg->ended;
}

}  // namespace banyu_mission
