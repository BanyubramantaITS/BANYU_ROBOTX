#include "banyu_vehicle_execution/command_encoder.hpp"
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
}  // namespace banyu_vehicle_execution
