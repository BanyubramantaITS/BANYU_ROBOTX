#pragma once

#include <memory>
#include <string>

#include "banyu_msgs/msg/vehicle_status.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace banyu_vehicle_interface
{

class VehicleStatusTracker
{
public:
  struct Config
  {
    double fcu_timeout_s{5.0};
    double rc_timeout_s{3.0};
    double gps_timeout_s{5.0};
    double kill_switch_timeout_s{1.0};
    int rc_takeover_channel{4};
    int rc_takeover_threshold{1700};
    std::string autonomy_mode{"GUIDED"};
    int default_operational_mode{0};
    // Run without a transmitter: RC counts as fresh, there is no operator takeover, and
    // the vehicle is AUTONOMOUS (bypass_operational_mode) unless a FRESH kill_switch
    // message says otherwise (teleop SB, ESP32 or `ros2 topic pub`). A stale or absent
    // kill_switch topic does NOT kill in this mode; stop with /banyu/mission/abort.
    bool bypass_rc{false};
    int bypass_operational_mode{2};
  };

  VehicleStatusTracker();
  explicit VehicleStatusTracker(Config config, rclcpp::Node * node = nullptr);
  explicit VehicleStatusTracker(rclcpp::Node * node);

  void updateState(const mavros_msgs::msg::State & state, const rclcpp::Time & received_at);
  void updateRc(const mavros_msgs::msg::RCIn & rc, const rclcpp::Time & received_at);
  void updateGps(const sensor_msgs::msg::NavSatFix & gps, const rclcpp::Time & received_at);
  void updateKillSwitch(
    uint8_t mode,
    const rclcpp::Time & received_at = rclcpp::Time(0, 0, RCL_ROS_TIME));

  void killSwitchCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  banyu_msgs::msg::VehicleStatus status(const rclcpp::Time & now) const;

private:
  static bool isFresh(
    bool received, const rclcpp::Time & now, const rclcpp::Time & last_received,
    double timeout_s);

  Config config_;
  rclcpp::Node * node_{nullptr};

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr kill_switch_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  uint8_t current_hw_mode_{0};
  bool kill_switch_received_{false};
  rclcpp::Time last_kill_switch_;

  bool state_received_{false};
  bool state_connected_{false};
  bool state_armed_{false};
  std::string mode_;
  rclcpp::Time last_state_;

  bool rc_received_{false};
  bool rc_takeover_{false};
  rclcpp::Time last_rc_;

  bool gps_received_{false};
  bool gps_valid_{false};
  rclcpp::Time last_gps_;
};

}  // namespace banyu_vehicle_interface
