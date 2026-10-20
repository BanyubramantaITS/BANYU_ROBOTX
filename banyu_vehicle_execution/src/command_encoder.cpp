#include "banyu_vehicle_execution/command_encoder.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace banyu_vehicle_execution
{
mavros_msgs::msg::PositionTarget encodeVelocityYawRate(
  const MotionIntent & local_intent, const IntentLimits & limits)
{
  if (local_intent.frame != IntentFrame::LocalEnu) {
    throw std::invalid_argument("the encoder requires a local ENU intent");
  }
  const auto bounded = clampIntent(local_intent, limits);
  if (!bounded) {
    throw std::invalid_argument("intent and limits must contain finite values");
  }
  mavros_msgs::msg::PositionTarget target;
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = kVelocityYawRateTypeMask;
  target.position.x = 0.0;
  target.position.y = 0.0;
  target.position.z = 0.0;
  target.velocity.x = bounded->linear_x;
  target.velocity.y = bounded->linear_y;
  target.velocity.z = bounded->linear_z;
  target.acceleration_or_force.x = 0.0;
  target.acceleration_or_force.y = 0.0;
  target.acceleration_or_force.z = 0.0;
  target.yaw = 0.0;
  target.yaw_rate = bounded->yaw_rate;
  if (!std::isfinite(target.velocity.x) || !std::isfinite(target.velocity.y) ||
    !std::isfinite(target.velocity.z) || !std::isfinite(target.yaw_rate))
  {
    throw std::invalid_argument("encoded float32 command is not finite");
  }
  return target;
}
namespace
{
void setChannel(mavros_msgs::msg::OverrideRCIn & msg, int channel, double pwm)
{
  if (channel < 1 || channel > static_cast<int>(msg.channels.size())) {return;}
  msg.channels[static_cast<size_t>(channel - 1)] = static_cast<uint16_t>(std::lround(pwm));
}

double stick(double neutral, double value, double gain, int deadzone, int max_delta)
{
  if (std::abs(value) < 1e-4) {return neutral;}
  const double magnitude = std::min(
    static_cast<double>(std::max(0, deadzone)) + std::abs(gain * value),
    static_cast<double>(max_delta));
  return neutral + std::copysign(magnitude, value);
}
}  // namespace

mavros_msgs::msg::OverrideRCIn releasedRcOverride()
{
  mavros_msgs::msg::OverrideRCIn msg;
  msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  return msg;
}

mavros_msgs::msg::OverrideRCIn neutralRcOverride(const RcOverrideConfig & config)
{
  auto msg = releasedRcOverride();
  for (const int channel : {config.channel_forward, config.channel_lateral,
      config.channel_throttle, config.channel_yaw})
  {
    setChannel(msg, channel, config.neutral_pwm);
  }
  return msg;
}

mavros_msgs::msg::OverrideRCIn encodeRcOverride(
  const MotionIntent & local_intent, double yaw_rad, const RcOverrideConfig & config,
  const IntentLimits & limits)
{
  if (local_intent.frame != IntentFrame::LocalEnu) {
    throw std::invalid_argument("the RC override encoder requires a local ENU intent");
  }
  const auto bounded = clampIntent(local_intent, limits);
  if (!bounded || !std::isfinite(yaw_rad)) {
    throw std::invalid_argument("intent, heading and limits must be finite");
  }
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  const double body_forward = c * bounded->linear_x + s * bounded->linear_y;
  const double body_left = -s * bounded->linear_x + c * bounded->linear_y;
  const double neutral = static_cast<double>(config.neutral_pwm);
  auto msg = releasedRcOverride();
  setChannel(msg, config.channel_forward, stick(neutral, body_forward,
    config.gain_xy_pwm_per_mps, config.deadzone_xy_pwm, config.max_delta_pwm));
  setChannel(msg, config.channel_lateral, stick(neutral, -body_left,
    config.gain_xy_pwm_per_mps, config.deadzone_xy_pwm, config.max_delta_pwm));
  setChannel(msg, config.channel_throttle, stick(neutral, bounded->linear_z,
    config.gain_z_pwm_per_mps, config.deadzone_z_pwm, config.max_delta_pwm));
  setChannel(msg, config.channel_yaw, stick(neutral, -bounded->yaw_rate,
    config.gain_yaw_pwm_per_rad_s, config.deadzone_yaw_pwm, config.max_delta_pwm));
  return msg;
}

}  // namespace banyu_vehicle_execution
