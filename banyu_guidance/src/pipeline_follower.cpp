#include "banyu_guidance/pipeline_follower.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_guidance
{
namespace
{

constexpr double kHalfPi = 1.5707963267948966;
constexpr double kEpsilon = 1e-9;

double nonNegativeFinite(double value)
{
  return std::isfinite(value) && value > 0.0 ? value : 0.0;
}

double clampFinite(double value, double lower, double upper)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, lower, upper);
}

}  // namespace

PipelineFollower::PipelineFollower(GuidanceConfig config)
: config_(config)
{
  config_.surge_gain_mps = nonNegativeFinite(config_.surge_gain_mps);
  config_.max_surge_mps = nonNegativeFinite(config_.max_surge_mps);
  config_.sway_gain_mps = nonNegativeFinite(config_.sway_gain_mps);
  config_.max_sway_mps = nonNegativeFinite(config_.max_sway_mps);
  config_.yaw_gain_rad_s = nonNegativeFinite(config_.yaw_gain_rad_s);
  config_.max_yaw_rate_rad_s = nonNegativeFinite(config_.max_yaw_rate_rad_s);
  config_.min_confidence = clampFinite(config_.min_confidence, 0.0, 1.0);
  config_.max_observation_age_s = nonNegativeFinite(config_.max_observation_age_s);
  config_.max_future_skew_s = nonNegativeFinite(config_.max_future_skew_s);
}

VelocityIntent PipelineFollower::compute(
  const PipelineObservation & observation, std::int64_t now_ns) const
{
  VelocityIntent stopped;

  if (observation.status == PipelineStatus::NoPipe) {
    stopped.status = IntentStatus::Lost;
    return stopped;
  }
  if (observation.status != PipelineStatus::Tracked) {
    stopped.status = IntentStatus::Invalid;
    return stopped;
  }

  if (observation.stamp_ns <= 0 || now_ns <= 0) {
    stopped.status = IntentStatus::Invalid;
    return stopped;
  }

  const auto age_ns = now_ns >= observation.stamp_ns ?
    static_cast<std::uint64_t>(now_ns - observation.stamp_ns) :
    static_cast<std::uint64_t>(observation.stamp_ns - now_ns);
  const double age_s = static_cast<double>(age_ns) * 1e-9;
  if (observation.stamp_ns > now_ns && age_s > config_.max_future_skew_s + kEpsilon) {
    stopped.status = IntentStatus::Stale;
    return stopped;
  }
  if (observation.stamp_ns <= now_ns && age_s > config_.max_observation_age_s + kEpsilon) {
    stopped.status = IntentStatus::Stale;
    return stopped;
  }

  if (!std::isfinite(observation.offset_x) || !std::isfinite(observation.offset_y) ||
    !std::isfinite(observation.heading_error_rad) || !std::isfinite(observation.confidence))
  {
    stopped.status = IntentStatus::Invalid;
    return stopped;
  }
  if (observation.offset_x < -1.0 || observation.offset_x > 1.0 ||
    observation.offset_y < -1.0 || observation.offset_y > 1.0 ||
    observation.heading_error_rad < -kHalfPi || observation.heading_error_rad > kHalfPi ||
    observation.confidence < 0.0 || observation.confidence > 1.0)
  {
    stopped.status = IntentStatus::Invalid;
    return stopped;
  }
  if (observation.confidence + kEpsilon < config_.min_confidence) {
    stopped.status = IntentStatus::Lost;
    return stopped;
  }

  VelocityIntent intent;
  intent.status = IntentStatus::Tracking;
  intent.surge_mps = std::clamp(
    config_.surge_gain_mps * observation.confidence, 0.0, config_.max_surge_mps);
  intent.sway_mps = std::clamp(
    -config_.sway_gain_mps * observation.offset_x,
    -config_.max_sway_mps, config_.max_sway_mps);

  intent.yaw_control_active = config_.yaw_control_enabled;
  if (intent.yaw_control_active) {
    intent.yaw_rate_rad_s = std::clamp(
      -config_.yaw_gain_rad_s * observation.heading_error_rad,
      -config_.max_yaw_rate_rad_s, config_.max_yaw_rate_rad_s);
  } else {
    // Do not leave yaw unspecified: downstream setpoint adapters must see 0.
    intent.yaw_rate_rad_s = 0.0;
  }
  return intent;
}

const GuidanceConfig & PipelineFollower::config() const noexcept
{
  return config_;
}

}  // namespace banyu_guidance
