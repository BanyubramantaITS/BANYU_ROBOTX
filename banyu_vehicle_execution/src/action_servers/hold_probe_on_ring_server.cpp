#include "banyu_vehicle_execution/action_servers/hold_probe_on_ring_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "banyu_msgs/msg/pipeline_ring_observation.hpp"

namespace banyu_vehicle_execution
{

namespace
{
constexpr double kCreepSizeFactor = 1.2;  // keep creeping to this multiple of the contact size
constexpr double kCreepRateFactor = 0.3;  // of the descent rate
constexpr double kLitConfidence = 0.85;
constexpr double kOnNodeGainScale = 0.3;   // the lid is 0.4 m away, not 1.5 m: the same image offset is a smaller distance, so a gentler loop
constexpr double kLastOffsetHoldS = 3.0;   // keep correcting with the last offset this long after the lid goes dark
constexpr double kOnNodeMaxOffset = 0.5;    // on the node the lid is near the centre; a lit ring further off is another node   // the detector's confidence for a lit light; colour-threshold rings are weaker
constexpr double kGreenConfirmS = 1.5;  // s of steady GREEN that count as the repair
}  // namespace

HoldProbeOnRingServer::HoldProbeOnRingServer(
  rclcpp::Node * node,
  const std::string & action_name,
  ActionServerCoordinator * coordinator,
  double ring_kp_sway,
  double ring_kp_surge)
: TypedActionServer(node, action_name, coordinator),
  ring_kp_sway_(ring_kp_sway), ring_kp_surge_(ring_kp_surge)
{
}

void HoldProbeOnRingServer::onGoalStarted()
{
  elapsed_aligned_s_ = 0.0;
  max_alignment_error_ = 0.0;
  phase_ = Phase::Descend;
  start_depth_m_ = 0.0;
  saw_red_ = false;
  repaired_ = false;
  failure_.clear();
  hold_s_ = 0.0;
  lost_s_ = 0.0;
  green_s_ = 0.0;
  last_offset_x_ = 0.0;
  last_offset_y_ = 0.0;
  since_lit_s_ = 1.0e9;
  depth_captured_ = false;
}

bool HoldProbeOnRingServer::validateGoal(const Goal & goal, std::string & reason)
{
  if (goal.hold_duration_s <= 0.0 || goal.alignment_tolerance <= 0.0) {
    reason = "hold_duration_s and alignment_tolerance must be positive";
    return false;
  }
  if (goal.climb_only && goal.descent_rate_mps <= 0.0) {
    reason = "climb_only needs descent_rate_mps";
    return false;
  }
  if (goal.descent_rate_mps < 0.0 || goal.contact_ring_size < 0.0 || goal.max_descent_m < 0.0) {
    reason = "descent_rate_mps, contact_ring_size and max_descent_m must not be negative";
    return false;
  }
  if (!goal.climb_only && goal.descent_rate_mps > 0.0 &&
    (goal.contact_ring_size <= 0.0 || goal.max_descent_m <= 0.0))
  {
    reason = "approach mode needs contact_ring_size and max_descent_m";
    return false;
  }
  return true;
}

MotionIntent HoldProbeOnRingServer::tick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double dt)
{
  if (!active_handle_) {
    return zeroIntent();
  }
  if (active_handle_->is_canceling()) {
    cancelActive();
    return zeroIntent();
  }

  const auto goal = active_handle_->get_goal();
  if (goal->climb_only) {
    if (!telemetry.pose_valid || !telemetry.depth_valid) {return zeroIntent();}
    if (!std::isfinite(origin_depth_m_) || telemetry.depth_m <= origin_depth_m_ + 0.05) {
      finish(true, "Back at the depth where the approach started");
      return zeroIntent();
    }
    return MotionIntent::localEnu(0.0, 0.0, goal->descent_rate_mps);
  }
  if (goal->descent_rate_mps > 0.0) {
    return approachTick(telemetry, now, dt);
  }
  // On the node the ring is the lit lid; a weak colour-threshold match in the dark half of a flash is the water, and centring
  // on it walks the vehicle off the node.
  const bool visible = telemetry.ring_obs_valid && telemetry.ring_visible &&
    telemetry.ring_confidence >= kLitConfidence &&
    std::hypot(telemetry.ring_offset_x, telemetry.ring_offset_y) <= kOnNodeMaxOffset;
  const double offset_x = visible ? telemetry.ring_offset_x : 0.0;
  const double offset_y = visible ? telemetry.ring_offset_y : 0.0;
  const double error = visible ? std::hypot(offset_x, offset_y) : 1.0;
  const bool aligned = visible && (error <= goal->alignment_tolerance);

  if (aligned) {
    elapsed_aligned_s_ += dt;
  } else {
    elapsed_aligned_s_ = 0.0;
  }
  max_alignment_error_ = std::max(max_alignment_error_, error);

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->elapsed_s = elapsed_aligned_s_;
  feedback->alignment_error = error;
  feedback->aligned = aligned;
  active_handle_->publish_feedback(feedback);

  if (elapsed_aligned_s_ >= goal->hold_duration_s) {
    auto result = std::make_shared<Result>();
    result->success = true;
    result->reason = "Probe held on ring for required duration";
    result->held_duration_s = elapsed_aligned_s_;
    result->maximum_alignment_error = max_alignment_error_;
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
    return zeroIntent();
  }

  // Keep centering while holding, as the alignment actions do: the vehicle coasts after
  // CenterOnRing and a zero command would let the ring drift out of tolerance.
  if (visible) {
    last_offset_x_ = offset_x;
    last_offset_y_ = offset_y;
    since_lit_s_ = 0.0;
  } else {
    since_lit_s_ += dt;
  }
  if ((visible || since_lit_s_ <= kLastOffsetHoldS) && telemetry.pose_valid) {
    const double sway = -std::clamp(
      kOnNodeGainScale * ring_kp_sway_ * last_offset_x_, -kOnNodeGainScale * 0.5, kOnNodeGainScale * 0.5);
    const double surge = -std::clamp(
      kOnNodeGainScale * ring_kp_surge_ * last_offset_y_, -kOnNodeGainScale * 0.3, kOnNodeGainScale * 0.3);
    const auto local_intent = bodyFluToLocalEnu(
      MotionIntent::bodyFlu(surge, sway, 0.0, 0.0), telemetry.yaw);
    return local_intent.value_or(zeroIntent());
  }
  return zeroIntent();
}

void HoldProbeOnRingServer::finish(bool success, const std::string & reason)
{
  auto result = std::make_shared<Result>();
  result->success = success;
  result->reason = reason;
  result->held_duration_s = hold_s_;
  result->maximum_alignment_error = max_alignment_error_;
  result->ring_repaired = repaired_;
  if (success) {
    active_handle_->succeed(result);
    coordinator_->completeGoal(GoalHandle{active_identity_});
    active_handle_.reset();
    active_identity_ = {};
  } else {
    abortActiveWithResult(result);
  }
}

MotionIntent HoldProbeOnRingServer::approachTick(
  const VehicleTelemetry & telemetry, const rclcpp::Time & now, double dt)
{
  if (!telemetry.pose_valid || !telemetry.depth_valid) {
    return zeroIntent();
  }
  const auto goal = active_handle_->get_goal();
  if (!depth_captured_) {
    start_depth_m_ = telemetry.depth_m;
    origin_depth_m_ = telemetry.depth_m;
    depth_captured_ = true;
  }
  // Before contact any ring of the right colour guides the sink; once on the node only the lit lid counts.
  const bool lit = telemetry.ring_confidence >= kLitConfidence;
  const bool visible = telemetry.ring_obs_valid && telemetry.ring_visible &&
    (phase_ == Phase::Descend ||
    (lit && std::hypot(telemetry.ring_offset_x, telemetry.ring_offset_y) <= kOnNodeMaxOffset));
  const double offset_x = visible ? telemetry.ring_offset_x : 0.0;
  const double offset_y = visible ? telemetry.ring_offset_y : 0.0;
  const double error = visible ? std::hypot(offset_x, offset_y) : 1.0;
  max_alignment_error_ = std::max(max_alignment_error_, error);
  if (visible && telemetry.ring_color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
    saw_red_ = true;
  }
  // Up close the lid flickers between red and green in the detector, so one green frame proves nothing. The
  // switch turns the light GREEN and keeps it so for 5 s: count the time it stays GREEN with no red between.
  // Only on contact: before that a GREEN ring in view is a neighbour, not the repaired one.
  if (visible && saw_red_ && !repaired_ && phase_ == Phase::Hold) {
    if (telemetry.ring_color == banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN) {
      green_s_ += dt;
    } else {
      green_s_ = 0.0;
    }
    if (green_s_ >= kGreenConfirmS) {
      repaired_ = true;
      phase_ = Phase::Climb;
      if (!goal->climb_back) {
        // Stay on the node: the next action keeps the camera on the lid while the flash code is read.
        finish(true, "Ring repaired");
        return zeroIntent();
      }
    }
  }
  const double depth_below_start = telemetry.depth_m - start_depth_m_;

  auto feedback = std::make_shared<Feedback>();
  feedback->stamp = now;
  feedback->elapsed_s = hold_s_;
  feedback->alignment_error = error;
  feedback->aligned = visible && error <= goal->alignment_tolerance;
  active_handle_->publish_feedback(feedback);

  double heave = 0.0;
  switch (phase_) {
    case Phase::Descend:
      // Sink only onto the RED ring: with a GREEN one in view (a neighbour) the sink waits like a lost ring.
      if (visible && telemetry.ring_color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
        lost_s_ = 0.0;
        if (telemetry.ring_size >= goal->contact_ring_size) {
          phase_ = Phase::Hold;
        } else if (depth_below_start >= goal->max_descent_m) {
          failure_ = "Descent limit reached without contact";
          phase_ = Phase::Climb;
        } else if (error <= goal->alignment_tolerance) {
          heave = -goal->descent_rate_mps;
        }
      } else {
        // Without the ring the sink is blind: stop and wait for it to come back.
        lost_s_ += dt;
        if (lost_s_ >= 5.0) {
          failure_ = "Ring lost during the approach";
          phase_ = Phase::Climb;
        }
      }
      break;
    case Phase::Hold:
      hold_s_ += dt;
      // Contact is declared a little early (the size reading is noisy); keep creeping while the ring stays centred
      // until the lid is clearly the size it has when the tip touches the switch, or the ring turns GREEN.
      if (visible && error <= goal->alignment_tolerance &&
        telemetry.ring_size < kCreepSizeFactor * goal->contact_ring_size &&
        depth_below_start < goal->max_descent_m)
      {
        heave = -kCreepRateFactor * goal->descent_rate_mps;
      }
      if (hold_s_ >= goal->hold_duration_s) {
        failure_ = "Ring did not turn green on contact";
        phase_ = Phase::Climb;
      }
      break;
    case Phase::Climb:
      if (depth_below_start <= 0.05) {
        finish(failure_.empty() && repaired_, repaired_ ? "Ring repaired" : failure_);
        return zeroIntent();
      }
      heave = goal->descent_rate_mps;
      break;
  }

  const bool is_red = telemetry.ring_color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
  const bool tracking = visible && (phase_ != Phase::Descend || is_red);
  if (tracking) {
    last_offset_x_ = offset_x;
    last_offset_y_ = offset_y;
    since_lit_s_ = 0.0;
  } else {
    since_lit_s_ += dt;
  }
  if (tracking || (phase_ != Phase::Descend && since_lit_s_ <= kLastOffsetHoldS)) {
    const double scale = phase_ == Phase::Descend ? 1.0 : kOnNodeGainScale;
    const double sway = -std::clamp(scale * ring_kp_sway_ * last_offset_x_, -scale * 0.5, scale * 0.5);
    const double surge = -std::clamp(scale * ring_kp_surge_ * last_offset_y_, -scale * 0.3, scale * 0.3);
    auto local = bodyFluToLocalEnu(MotionIntent::bodyFlu(surge, sway, 0.0, 0.0), telemetry.yaw);
    if (local) {
      local->linear_z = heave;
      return *local;
    }
  }
  return MotionIntent::localEnu(0.0, 0.0, heave);
}

}  // namespace banyu_vehicle_execution
