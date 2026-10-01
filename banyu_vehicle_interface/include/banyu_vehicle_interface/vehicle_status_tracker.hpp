#pragma once

#include <string>

#include "banyu_msgs/msg/vehicle_status.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

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
    int rc_takeover_channel{4};
    int rc_takeover_threshold{1700};
    std::string autonomy_mode{"GUIDED"};
  };

  VehicleStatusTracker();
  explicit VehicleStatusTracker(Config config);

  void updateState(const mavros_msgs::msg::State & state, const rclcpp::Time & received_at);
  void updateRc(const mavros_msgs::msg::RCIn & rc, const rclcpp::Time & received_at);
  void updateGps(const sensor_msgs::msg::NavSatFix & gps, const rclcpp::Time & received_at);

  banyu_msgs::msg::VehicleStatus status(const rclcpp::Time & now) const;

private:
  static bool isFresh(
    bool received, const rclcpp::Time & now, const rclcpp::Time & last_received,
    double timeout_s);

  Config config_;
  bool state_received_{false};
  bool state_connected_{false};
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
