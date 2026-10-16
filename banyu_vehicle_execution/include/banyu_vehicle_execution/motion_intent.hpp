#ifndef BANYU_VEHICLE_EXECUTION__MOTION_INTENT_HPP_
#define BANYU_VEHICLE_EXECUTION__MOTION_INTENT_HPP_
#include <optional>
#include <string>
#include <limits>
namespace banyu_vehicle_execution
{
constexpr double kHardMaxLinearComponent = 5.0;
constexpr double kHardMaxYawRate = 2.0;
static_assert(kHardMaxLinearComponent < std::numeric_limits<float>::max());
static_assert(kHardMaxYawRate < std::numeric_limits<float>::max());
enum class IntentFrame
{
  LocalEnu,
  BodyFlu
};
struct MotionIntent
{
  IntentFrame frame{IntentFrame::LocalEnu};
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double yaw_rate{0.0};
  static MotionIntent localEnu(double x, double y, double z, double yaw_rate = 0.0);
  static MotionIntent bodyFlu(double x, double y, double z, double yaw_rate = 0.0);
};
struct IntentLimits
{
  double max_linear_x{1.0};
  double max_linear_y{1.0};
  double max_linear_z{1.0};
  double max_yaw_rate{1.0};
  IntentLimits() = default;
  IntentLimits(double max_linear, double max_yaw);
  IntentLimits(double max_x, double max_y, double max_z, double max_yaw);
};
enum class IntentValidationError
{
  None,
  InvalidLimits,
  NonFiniteComponent,
  OutsideLimits
};
struct IntentValidation
{
  IntentValidationError error{IntentValidationError::None};
  std::string detail{};
  bool valid() const noexcept {return error == IntentValidationError::None;}
};
IntentValidation validateIntent(const MotionIntent & intent, const IntentLimits & limits);
std::optional<MotionIntent> clampIntent(const MotionIntent & intent, const IntentLimits & limits);
std::optional<MotionIntent> bodyFluToLocalEnu(
  const MotionIntent & body_intent, double yaw_rad);
std::optional<MotionIntent> bodyFluToLocalEnu(
  const MotionIntent & body_intent, double yaw_rad, const IntentLimits & limits);
MotionIntent zeroIntent() noexcept;
}  // namespace banyu_vehicle_execution
#endif  // BANYU_VEHICLE_EXECUTION__MOTION_INTENT_HPP_
