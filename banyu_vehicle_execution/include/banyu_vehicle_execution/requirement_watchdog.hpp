#ifndef BANYU_VEHICLE_EXECUTION__REQUIREMENT_WATCHDOG_HPP_
#define BANYU_VEHICLE_EXECUTION__REQUIREMENT_WATCHDOG_HPP_

#include "rclcpp/rclcpp.hpp"

namespace banyu_vehicle_execution
{

class RequirementWatchdog
{
public:
  void reset(const rclcpp::Time & now)
  {
    last_valid_time_ = now;
  }

  bool expired(const rclcpp::Time & now, double timeout_s) const
  {
    return timeout_s > 0.0 && (now - last_valid_time_).seconds() >= timeout_s;
  }

private:
  rclcpp::Time last_valid_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__REQUIREMENT_WATCHDOG_HPP_
