#ifndef BANYU_GUIDANCE__PIPELINE_FOLLOWER_HPP_
#define BANYU_GUIDANCE__PIPELINE_FOLLOWER_HPP_

#include <cstdint>

namespace banyu_guidance
{

enum class PipelineStatus : std::uint8_t
{
  Invalid = 0,
  NoPipe = 1,
  Tracked = 2,
};

enum class IntentStatus : std::uint8_t
{
  Stopped = 0,
  Stale = 1,
  Invalid = 2,
  Lost = 3,
  Tracking = 4,
};

// This is deliberately independent of ROS messages so it can be checked and
// used by an authority node without coupling guidance to MAVROS or a mission.
struct PipelineObservation
{
  std::uint64_t frame_sequence{0};
  PipelineStatus status{PipelineStatus::Invalid};
  std::int64_t stamp_ns{0};
  double offset_x{0.0};
  double offset_y{0.0};
  double heading_error_rad{0.0};
  double confidence{0.0};
};

struct GuidanceConfig
{
  // surge_gain_mps is the confidence-scaled nominal forward speed.
  double surge_gain_mps{0.6};
  double max_surge_mps{0.6};
  double sway_gain_mps{0.8};
  double max_sway_mps{0.4};
  double yaw_gain_rad_s{0.8};
  double max_yaw_rate_rad_s{0.35};
  double min_confidence{0.35};
  double max_observation_age_s{0.25};
  double max_future_skew_s{0.05};
  bool yaw_control_enabled{true};
};

// Body-frame FLU convention: +x is surge/forward, +y is sway/left, and +z
// is up. Pipeline offsets use image coordinates: +x is right and +y is down.
// Consequently, a positive image offset_x produces negative body sway. A
// positive image heading error is clockwise, while positive body yaw is
// counter-clockwise, so its correction is also negative.
// ROS world vectors in this stack are ENU. This component does not perform
// that conversion; a consumer with vehicle yaw psi can use
// v_enu.x = cos(psi) * surge - sin(psi) * sway and
// v_enu.y = sin(psi) * surge + cos(psi) * sway.
struct VelocityIntent
{
  double surge_mps{0.0};
  double sway_mps{0.0};
  double yaw_rate_rad_s{0.0};
  IntentStatus status{IntentStatus::Stopped};
  bool yaw_control_active{false};
};

class PipelineFollower
{
public:
  explicit PipelineFollower(GuidanceConfig config = {});

  // now_ns and observation.stamp_ns must come from the same ROS clock.
  // Every non-tracking result is a zero-velocity intent.
  VelocityIntent compute(const PipelineObservation & observation, std::int64_t now_ns) const;

  const GuidanceConfig & config() const noexcept;

private:
  GuidanceConfig config_;
};

}  // namespace banyu_guidance

#endif  // BANYU_GUIDANCE__PIPELINE_FOLLOWER_HPP_
