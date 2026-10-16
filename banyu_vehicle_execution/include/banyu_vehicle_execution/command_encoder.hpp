#ifndef BANYU_VEHICLE_EXECUTION__COMMAND_ENCODER_HPP_
#define BANYU_VEHICLE_EXECUTION__COMMAND_ENCODER_HPP_
#include "banyu_vehicle_execution/motion_intent.hpp"
#include <cstdint>
#include <mavros_msgs/msg/position_target.hpp>
namespace banyu_vehicle_execution
{
constexpr uint16_t kVelocityYawRateTypeMask =
  mavros_msgs::msg::PositionTarget::IGNORE_PX |
  mavros_msgs::msg::PositionTarget::IGNORE_PY |
  mavros_msgs::msg::PositionTarget::IGNORE_PZ |
  mavros_msgs::msg::PositionTarget::IGNORE_AFX |
  mavros_msgs::msg::PositionTarget::IGNORE_AFY |
  mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
  mavros_msgs::msg::PositionTarget::IGNORE_YAW;
static_assert(kVelocityYawRateTypeMask == 0x5C7, "velocity/yaw-rate mask changed");
mavros_msgs::msg::PositionTarget encodeVelocityYawRate(
  const MotionIntent & local_intent, const IntentLimits & limits = IntentLimits{});
}  // namespace banyu_vehicle_execution
#endif  // BANYU_VEHICLE_EXECUTION__COMMAND_ENCODER_HPP_
