#include "banyu_vehicle_interface/vehicle_status_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace banyu_vehicle_interface
{
namespace
{
// autonomy_mode may list several modes ("GUIDED,ALT_HOLD"): surface mode, then the underwater mode.
bool modeInList(const std::string & mode, const std::string & list)
{
  std::size_t start = 0;
  while (start <= list.size()) {
    const auto comma = list.find(',', start);
    const auto end = comma == std::string::npos ? list.size() : comma;
    if (list.compare(start, end - start, mode) == 0 && end - start == mode.size()) {return true;}
    if (comma == std::string::npos) {break;}
    start = comma + 1;
  }
  return false;
}
}  // namespace


VehicleStatusTracker::VehicleStatusTracker()
: VehicleStatusTracker(Config{}, nullptr)
{
}

VehicleStatusTracker::VehicleStatusTracker(Config config, rclcpp::Node * node)
: config_(std::move(config)), node_(node)
{
  config_.fcu_timeout_s = std::max(0.0, config_.fcu_timeout_s);
  config_.rc_timeout_s = std::max(0.0, config_.rc_timeout_s);
  config_.gps_timeout_s = std::max(0.0, config_.gps_timeout_s);
  config_.kill_switch_timeout_s = std::max(0.0, config_.kill_switch_timeout_s);
  current_hw_mode_ = static_cast<uint8_t>(std::clamp(config_.default_operational_mode, 0, 2));

  if (node_ != nullptr) {
    kill_switch_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/banyu/hardware/kill_switch", rclcpp::QoS(10),
      std::bind(&VehicleStatusTracker::killSwitchCallback, this, std::placeholders::_1));
    arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  }
}

VehicleStatusTracker::VehicleStatusTracker(rclcpp::Node * node)
: VehicleStatusTracker(Config{}, node)
{
}

void VehicleStatusTracker::updateState(
  const mavros_msgs::msg::State & state, const rclcpp::Time & received_at)
{
  state_received_ = true;
  state_connected_ = state.connected;
  state_armed_ = state.armed;
  mode_ = state.mode;
  last_state_ = received_at;
}

void VehicleStatusTracker::updateRc(
  const mavros_msgs::msg::RCIn & rc, const rclcpp::Time & received_at)
{
  rc_received_ = false;
  rc_takeover_ = false;
  if (rc.channels.empty()) {
    return;
  }
  // A negative takeover channel disables stick takeover only; RC freshness still counts.
  if (config_.rc_takeover_channel < 0) {
    rc_received_ = true;
    last_rc_ = received_at;
    return;
  }
  if (static_cast<size_t>(config_.rc_takeover_channel) >= rc.channels.size()) {
    return;
  }

  rc_received_ = true;
  rc_takeover_ = rc.channels[config_.rc_takeover_channel] >=
    static_cast<uint16_t>(std::clamp(config_.rc_takeover_threshold, 0, 65535));
  last_rc_ = received_at;
}

void VehicleStatusTracker::updateGps(
  const sensor_msgs::msg::NavSatFix & gps, const rclcpp::Time & received_at)
{
  gps_received_ = true;
  gps_valid_ = gps.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
    std::isfinite(gps.latitude) && std::isfinite(gps.longitude) &&
    gps.latitude >= -90.0 && gps.latitude <= 90.0 &&
    gps.longitude >= -180.0 && gps.longitude <= 180.0;
  last_gps_ = received_at;
}

void VehicleStatusTracker::killSwitchCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const auto now = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  updateKillSwitch(msg->data, now);
}

void VehicleStatusTracker::updateKillSwitch(uint8_t mode, const rclcpp::Time & received_at)
{
  const uint8_t previous_mode = current_hw_mode_;
  const bool first_message = !kill_switch_received_;
  current_hw_mode_ = mode;
  kill_switch_received_ = true;
  last_kill_switch_ = received_at;

  // Only trigger disarm request on transition to mode 0 or upon first explicit hardware disarm report
  if (current_hw_mode_ == 0 && (previous_mode != 0 || first_message) && arming_client_) {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;
    arming_client_->async_send_request(request);
  }
}

bool VehicleStatusTracker::isFresh(
  bool received, const rclcpp::Time & now, const rclcpp::Time & last_received,
  double timeout_s)
{
  if (!received) {
    return false;
  }
  const double age_s = (now - last_received).seconds();
  return age_s >= -1.0 && age_s <= timeout_s;
}

banyu_msgs::msg::VehicleStatus VehicleStatusTracker::status(const rclcpp::Time & now) const
{
  banyu_msgs::msg::VehicleStatus result;
  result.header.stamp = now;

  result.fcu_status_fresh = isFresh(state_received_, now, last_state_, config_.fcu_timeout_s);
  result.fcu_connected = result.fcu_status_fresh && state_connected_;
  result.armed = result.fcu_connected && state_armed_;
  result.rc_fresh = config_.bypass_rc ||
    isFresh(rc_received_, now, last_rc_, config_.rc_timeout_s);
  result.operator_takeover = !config_.bypass_rc && result.rc_fresh && rc_takeover_;
  result.mode = mode_;
  result.gps_fresh = isFresh(gps_received_, now, last_gps_, config_.gps_timeout_s) && gps_valid_;
  result.gps_valid = gps_received_ && gps_valid_;

  uint8_t effective_mode = current_hw_mode_;
  if (config_.bypass_rc) {
    const bool kill_fresh = kill_switch_received_ && last_kill_switch_.nanoseconds() > 0 &&
      isFresh(true, now, last_kill_switch_, config_.kill_switch_timeout_s);
    effective_mode = kill_fresh ?
      current_hw_mode_ :
      static_cast<uint8_t>(std::clamp(config_.bypass_operational_mode, 0, 2));
  } else if (kill_switch_received_ && config_.kill_switch_timeout_s > 0.0 &&
    last_kill_switch_.nanoseconds() > 0)
  {
    if (!isFresh(true, now, last_kill_switch_, config_.kill_switch_timeout_s)) {
      effective_mode = 0;
    }
  }

  result.operational_mode = effective_mode;
  result.hardware_kill_active = (effective_mode == 0);

  if (!result.fcu_connected || !result.rc_fresh) {
    result.authority = "UNKNOWN";
  } else if (effective_mode == 0) {
    result.authority = "DISARMED";
  } else if (result.operator_takeover) {
    result.authority = "OPERATOR";
  } else if (effective_mode == 1) {
    result.authority = "MANUAL";
  } else if (effective_mode == 2 && modeInList(result.mode, config_.autonomy_mode)) {
    result.authority = "AUTONOMY";
  } else {
    result.authority = "MANUAL";
  }

  return result;
}

}  // namespace banyu_vehicle_interface
