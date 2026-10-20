#ifndef BANYU_VEHICLE_EXECUTION__COMMAND_ENCODER_HPP_
#define BANYU_VEHICLE_EXECUTION__COMMAND_ENCODER_HPP_
#include "banyu_vehicle_execution/motion_intent.hpp"
#include <cstdint>
#include <mavros_msgs/msg/override_rc_in.hpp>
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

// RC override output for flight modes that need no position estimate (ALT_HOLD,
// STABILIZE, MANUAL): the velocity intent becomes pilot "stick" PWM. Channels are
// 1-based Pixhawk RC channels and must match RCMAP_*; 0 disables an axis.
struct RcOverrideConfig
{
  int channel_forward{5};
  int channel_lateral{6};
  int channel_throttle{3};
  int channel_yaw{4};
  double gain_xy_pwm_per_mps{300.0};
  double gain_z_pwm_per_mps{300.0};
  double gain_yaw_pwm_per_rad_s{300.0};
  int max_delta_pwm{400};
  int neutral_pwm{1500};
  // ArduSub ignores this much stick around neutral (effective dead zone = RCx_DZ or
  // THR_DZ times the pilot gain). A non-zero command starts just outside it, so small
  // rates (e.g. a slow orbit yaw) are not swallowed. 0 = no compensation.
  int deadzone_xy_pwm{0};
  int deadzone_z_pwm{0};
  int deadzone_yaw_pwm{0};
};

// Rotates the local ENU intent back into the body frame with yaw_rad (ENU heading),
// then maps forward/left/up/CCW to ArduSub sticks (forward, right, up, clockwise).
mavros_msgs::msg::OverrideRCIn encodeRcOverride(
  const MotionIntent & local_intent, double yaw_rad, const RcOverrideConfig & config,
  const IntentLimits & limits = IntentLimits{});
// Controlled channels at neutral (stop), every other channel released.
mavros_msgs::msg::OverrideRCIn neutralRcOverride(const RcOverrideConfig & config);
// All channels released back to the transmitter.
mavros_msgs::msg::OverrideRCIn releasedRcOverride();
}  // namespace banyu_vehicle_execution
#endif  // BANYU_VEHICLE_EXECUTION__COMMAND_ENCODER_HPP_
