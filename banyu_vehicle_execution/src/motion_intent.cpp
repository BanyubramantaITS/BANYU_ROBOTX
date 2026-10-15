#include "banyu_vehicle_execution/motion_intent.hpp"
#include <algorithm>
#include <cmath>
namespace banyu_vehicle_execution
{
namespace
{
bool finite(double value)
{
  return std::isfinite(value);
}
bool validLimits(const IntentLimits & limits)
{
  return finite(limits.max_linear_x) && finite(limits.max_linear_y) &&
         finite(limits.max_linear_z) && finite(limits.max_yaw_rate) &&
         limits.max_linear_x >= 0.0 && limits.max_linear_y >= 0.0 &&
         limits.max_linear_z >= 0.0 && limits.max_yaw_rate >= 0.0;
}
bool validFrame(IntentFrame frame)
{
  return frame == IntentFrame::LocalEnu || frame == IntentFrame::BodyFlu;
}
double bound(double value, double maximum)
{
  const double safe_maximum = std::min(maximum, kHardMaxLinearComponent);
  return std::clamp(value, -safe_maximum, safe_maximum);
}
double boundYawRate(double value, double maximum)
{
  return std::clamp(
    value, -std::min(maximum, kHardMaxYawRate),
    std::min(maximum, kHardMaxYawRate));
}
}  // namespace
MotionIntent MotionIntent::localEnu(double x, double y, double z, double yaw_rate_value)
{
  return {IntentFrame::LocalEnu, x, y, z, yaw_rate_value};
}
MotionIntent MotionIntent::bodyFlu(double x, double y, double z, double yaw_rate_value)
{
  return {IntentFrame::BodyFlu, x, y, z, yaw_rate_value};
}
IntentLimits::IntentLimits(double max_linear, double max_yaw)
: max_linear_x(max_linear), max_linear_y(max_linear), max_linear_z(max_linear),
  max_yaw_rate(max_yaw)
{
}
IntentLimits::IntentLimits(double max_x, double max_y, double max_z, double max_yaw)
: max_linear_x(max_x), max_linear_y(max_y), max_linear_z(max_z), max_yaw_rate(max_yaw)
{
}
IntentValidation validateIntent(const MotionIntent & intent, const IntentLimits & limits)
{
  if (!validLimits(limits)) {
    return {IntentValidationError::InvalidLimits, "limits must be finite and non-negative"};
  }
  if (!validFrame(intent.frame) || !finite(intent.linear_x) || !finite(intent.linear_y) ||
    !finite(intent.linear_z) || !finite(intent.yaw_rate))
  {
    return {IntentValidationError::NonFiniteComponent, "intent contains a non-finite value"};
  }
  if (std::abs(intent.linear_x) > std::min(limits.max_linear_x, kHardMaxLinearComponent) ||
    std::abs(intent.linear_y) > std::min(limits.max_linear_y, kHardMaxLinearComponent) ||
    std::abs(intent.linear_z) > std::min(limits.max_linear_z, kHardMaxLinearComponent) ||
    std::abs(intent.yaw_rate) > std::min(limits.max_yaw_rate, kHardMaxYawRate))
  {
    return {IntentValidationError::OutsideLimits, "intent exceeds a configured server limit"};
  }
  return {};
}
std::optional<MotionIntent> clampIntent(
  const MotionIntent & intent, const IntentLimits & limits)
{
  if (!validLimits(limits) || !validFrame(intent.frame) ||
    !finite(intent.linear_x) || !finite(intent.linear_y) ||
    !finite(intent.linear_z) || !finite(intent.yaw_rate))
  {
    return std::nullopt;
  }
  MotionIntent bounded = intent;
  bounded.linear_x = bound(bounded.linear_x, limits.max_linear_x);
  bounded.linear_y = bound(bounded.linear_y, limits.max_linear_y);
  bounded.linear_z = bound(bounded.linear_z, limits.max_linear_z);
  bounded.yaw_rate = boundYawRate(bounded.yaw_rate, limits.max_yaw_rate);
  return bounded;
}
std::optional<MotionIntent> bodyFluToLocalEnu(
  const MotionIntent & body_intent, double yaw_rad)
{
  if (body_intent.frame != IntentFrame::BodyFlu || !finite(yaw_rad) ||
    !finite(body_intent.linear_x) || !finite(body_intent.linear_y) ||
    !finite(body_intent.linear_z) || !finite(body_intent.yaw_rate))
  {
    return std::nullopt;
  }
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  const MotionIntent local = MotionIntent::localEnu(
    c * body_intent.linear_x - s * body_intent.linear_y,
    s * body_intent.linear_x + c * body_intent.linear_y,
    body_intent.linear_z,
    body_intent.yaw_rate);
  if (!finite(local.linear_x) || !finite(local.linear_y) ||
    !finite(local.linear_z) || !finite(local.yaw_rate))
  {
    return std::nullopt;
  }
  return local;
}
std::optional<MotionIntent> bodyFluToLocalEnu(
  const MotionIntent & body_intent, double yaw_rad, const IntentLimits & limits)
{
  const auto local = bodyFluToLocalEnu(body_intent, yaw_rad);
  if (!local) {
    return std::nullopt;
  }
  return clampIntent(*local, limits);
}
MotionIntent zeroIntent() noexcept
{
  return MotionIntent::localEnu(0.0, 0.0, 0.0, 0.0);
}
}  // namespace banyu_vehicle_execution
