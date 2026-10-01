#ifndef BANYU_VEHICLE_EXECUTION__POR_VISUAL_HELPERS_HPP_
#define BANYU_VEHICLE_EXECUTION__POR_VISUAL_HELPERS_HPP_

#include <algorithm>
#include <cmath>

namespace banyu_vehicle_execution
{

struct VisualCorrection
{
  double lateral_flu_mps{0.0};
  double vertical_flu_mps{0.0};
  double yaw_rate_rad_s{0.0};
};

inline double visualCorrectionComponent(double gain, double error, double limit)
{
  if (!std::isfinite(gain) || !std::isfinite(error) || !std::isfinite(limit) || limit < 0.0) {
    return 0.0;
  }
  return std::clamp(-gain * error, -limit, limit);
}

inline VisualCorrection visualCorrection(
  double offset_x, double offset_y,
  double lateral_gain, double vertical_gain, double lateral_limit, double vertical_limit,
  double yaw_gain = 0.0, double yaw_limit = 0.0)
{
  return {
    visualCorrectionComponent(lateral_gain, offset_x, lateral_limit),
    visualCorrectionComponent(vertical_gain, offset_y, vertical_limit),
    visualCorrectionComponent(yaw_gain, offset_x, yaw_limit)};
}

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__POR_VISUAL_HELPERS_HPP_
