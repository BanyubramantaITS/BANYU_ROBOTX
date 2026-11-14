#include "banyu_vehicle_interface/vehicle_status_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

namespace banyu_vehicle_interface
{

class VehicleStatusNode : public rclcpp::Node
{
public:
  VehicleStatusNode()
  : Node("banyu_vehicle_status"), tracker_(loadConfig(), this)
  {
    const auto state_topic = declare_parameter<std::string>("state_topic", "/mavros/state");
    const auto rc_topic = declare_parameter<std::string>("rc_topic", "/mavros/rc/in");
    const auto gps_topic = declare_parameter<std::string>(
      "gps_topic", "/mavros/global_position/raw/fix");
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/banyu/vehicle_status");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);

    publisher_ = create_publisher<banyu_msgs::msg::VehicleStatus>(status_topic, rclcpp::QoS(10));
    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
      state_topic, rclcpp::QoS(10),
      std::bind(&VehicleStatusNode::stateCallback, this, std::placeholders::_1));
    rc_subscription_ = create_subscription<mavros_msgs::msg::RCIn>(
      rc_topic, rclcpp::QoS(10),
      std::bind(&VehicleStatusNode::rcCallback, this, std::placeholders::_1));
    gps_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&VehicleStatusNode::gpsCallback, this, std::placeholders::_1));

    const double safe_rate_hz = std::max(0.1, publish_rate_hz);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / safe_rate_hz));
    timer_ = create_wall_timer(
      std::max(std::chrono::milliseconds(1), period),
      std::bind(&VehicleStatusNode::publishStatus, this));
  }

private:
  VehicleStatusTracker::Config loadConfig()
  {
    VehicleStatusTracker::Config config;
    config.fcu_timeout_s = declare_parameter<double>("fcu_timeout_s", config.fcu_timeout_s);
    config.rc_timeout_s = declare_parameter<double>("rc_timeout_s", config.rc_timeout_s);
    config.gps_timeout_s = declare_parameter<double>("gps_timeout_s", config.gps_timeout_s);
    config.rc_takeover_channel = declare_parameter<int>(
      "rc_takeover_channel", config.rc_takeover_channel);
    config.rc_takeover_threshold = declare_parameter<int>(
      "rc_takeover_threshold", config.rc_takeover_threshold);
    config.autonomy_mode = declare_parameter<std::string>(
      "autonomy_mode", config.autonomy_mode);
    config.standby_mode = declare_parameter<std::string>("standby_mode", config.standby_mode);
    config.default_operational_mode = declare_parameter<int>(
      "default_operational_mode", config.default_operational_mode);
    config.kill_switch_timeout_s = declare_parameter<double>(
      "kill_switch_timeout_s", config.kill_switch_timeout_s);
    config.bypass_rc = declare_parameter<bool>("bypass_rc", config.bypass_rc);
    config.bypass_operational_mode = declare_parameter<int>(
      "bypass_operational_mode", config.bypass_operational_mode);
    return config;
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    tracker_.updateState(*msg, now());
  }

  void rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg)
  {
    tracker_.updateRc(*msg, now());
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    tracker_.updateGps(*msg, now());
  }

  void publishStatus()
  {
    publisher_->publish(tracker_.status(now()));
  }

  VehicleStatusTracker tracker_;
  rclcpp::Publisher<banyu_msgs::msg::VehicleStatus>::SharedPtr publisher_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace banyu_vehicle_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_vehicle_interface::VehicleStatusNode>());
  rclcpp::shutdown();
  return 0;
}
