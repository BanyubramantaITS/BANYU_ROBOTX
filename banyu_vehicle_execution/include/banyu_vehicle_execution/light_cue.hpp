#ifndef BANYU_VEHICLE_EXECUTION__LIGHT_CUE_HPP_
#define BANYU_VEHICLE_EXECUTION__LIGHT_CUE_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

#include "banyu_msgs/msg/pipeline_ring_observation.hpp"
#include "banyu_vehicle_execution/vehicle_telemetry.hpp"

namespace banyu_vehicle_execution
{

// A pipeline light seen by the bottom camera while the pipe is not: the node sits on the pipe, so steering over the
// light brings the pipe into view. Used by AcquirePipeline and SearchPipeline.
struct LightCueConfig
{
  bool enabled{true};
  double min_confidence{0.5};
  // Lights above this depth are ignored: the start buoy's indicator is seen from the surface.
  double min_depth_m{1.0};
  double kp_sway{0.4};
  double kp_surge{0.3};
  double max_speed_mps{0.2};
  // Steering time per goal; after it lights are ignored, so a false light cannot hold the search.
  double budget_s{15.0};
};

struct LightCueVelocity
{
  double surge{0.0};
  double sway{0.0};
};

// Down-positive depth for the gate.
inline std::optional<double> lightCueDepth(const VehicleTelemetry & t)
{
  if (t.depth_valid) {return t.depth_m;}
  return std::nullopt;
}

// Body-frame velocity that moves the light toward the image centre (same signs as CenterOnRing), or nullopt when no
// usable light is in view.
inline std::optional<LightCueVelocity> lightCueVelocity(
  const VehicleTelemetry & t, const LightCueConfig & config)
{
  using Ring = banyu_msgs::msg::PipelineRingObservation;
  if (!config.enabled || !t.light_cue_valid || !t.light_cue_visible) {return std::nullopt;}
  // Before the repair a node is lit only RED or GREEN.
  if (t.light_cue_color != Ring::COLOR_RED && t.light_cue_color != Ring::COLOR_GREEN) {return std::nullopt;}
  if (!(t.light_cue_confidence >= config.min_confidence) ||
    !std::isfinite(t.light_cue_offset_x) || !std::isfinite(t.light_cue_offset_y))
  {
    return std::nullopt;
  }
  const auto depth = lightCueDepth(t);
  if (!depth || *depth < config.min_depth_m) {return std::nullopt;}
  const double max = config.max_speed_mps;
  return LightCueVelocity{
    -std::clamp(config.kp_surge * t.light_cue_offset_y, -max, max),
    -std::clamp(config.kp_sway * t.light_cue_offset_x, -max, max)};
}

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__LIGHT_CUE_HPP_
