#include "banyu_detection/ring_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_detection
{

RingTracker::RingTracker(RingTrackerConfig config)
: config_(config)
{
  config_.entry_confirm_frames = std::max<std::size_t>(1, config_.entry_confirm_frames);
  config_.exit_confirm_frames = std::max<std::size_t>(1, config_.exit_confirm_frames);
  config_.debounce_frames = std::max<std::size_t>(1, config_.debounce_frames);
  config_.maximum_sequence_gap = std::max<uint64_t>(1, config_.maximum_sequence_gap);
  config_.minimum_confidence = std::clamp(config_.minimum_confidence, 0.0, 1.0);
  config_.minimum_rearm_distance_m = std::max(1.0e-3, config_.minimum_rearm_distance_m);
  config_.pose_timestamp_tolerance_s = std::max(0.0, config_.pose_timestamp_tolerance_s);
}

bool RingTracker::isKnownColor(uint8_t color)
{
  return color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED ||
         color == banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN ||
         color == banyu_msgs::msg::PipelineRingObservation::COLOR_BLUE;
}

RingTracker::ObservationKind RingTracker::classifyObservation(
  const banyu_msgs::msg::PipelineRingObservation & observation) const
{
  if (!observation.visible) {
    const bool valid_invisible =
      observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN &&
      observation.confidence == 0.0F && std::isnan(observation.offset_x) &&
      std::isnan(observation.offset_y);
    return valid_invisible ? ObservationKind::INVISIBLE :
           ObservationKind::MALFORMED_OR_UNCERTAIN;
  }

  const bool valid_visible = isKnownColor(observation.color) &&
    std::isfinite(observation.offset_x) && std::isfinite(observation.offset_y) &&
    std::isfinite(observation.confidence) && observation.offset_x >= -1.0F &&
    observation.offset_x <= 1.0F && observation.offset_y >= -1.0F &&
    observation.offset_y <= 1.0F && observation.confidence >= config_.minimum_confidence &&
    observation.confidence <= 1.0F;
  return valid_visible ? ObservationKind::VISIBLE :
         ObservationKind::MALFORMED_OR_UNCERTAIN;
}

bool RingTracker::sameColor(uint8_t first, uint8_t second)
{
  return first == second && isKnownColor(first);
}

bool RingTracker::hasRecordedColor(uint8_t color) const
{
  return std::any_of(
    snapshot_.records.begin(), snapshot_.records.end(),
    [color](const banyu_msgs::msg::Task2RingRecord & record) {
      return record.color == color;
    });
}

void RingTracker::clearEntry()
{
  pending_color_ = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  has_pending_color_ = false;
}

void RingTracker::beginExit()
{
  clearEntry();
  snapshot_.phase = RingTrackerPhase::EXIT;
  snapshot_.consecutive_frames = 0;
  snapshot_.invisible_frames = 0;
}

void RingTracker::beginDebounce()
{
  clearEntry();
  snapshot_.phase = RingTrackerPhase::DEBOUNCE;
  snapshot_.consecutive_frames = 0;
  snapshot_.invisible_frames = 0;
}

bool RingTracker::rearmAllowed(const RingTrackerInput & input) const
{
  if (!validPoseForObservation(input)) {
    return false;
  }
  if (input.has_path_distance &&
    (!std::isfinite(input.path_distance_m) || input.path_distance_m < 0.0))
  {
    return false;
  }
  if (snapshot_.records.empty()) {
    return true;
  }
  if (!has_last_record_pose_) {
    return false;
  }

  const double dx = input.vehicle_pose_map.pose.position.x -
    last_record_pose_.pose.position.x;
  const double dy = input.vehicle_pose_map.pose.position.y -
    last_record_pose_.pose.position.y;
  const double dz = input.vehicle_pose_map.pose.position.z -
    last_record_pose_.pose.position.z;
  const double spatial_separation = std::sqrt(dx * dx + dy * dy + dz * dz);
  const bool spatially_separated = std::isfinite(spatial_separation) &&
    spatial_separation >= config_.minimum_rearm_distance_m;
  const bool progressed = input.has_path_distance && has_last_record_path_distance_ &&
    input.path_distance_m > last_record_path_distance_ + config_.minimum_rearm_distance_m;
  return spatially_separated || progressed;
}

bool RingTracker::validPoseForObservation(const RingTrackerInput & input) const
{
  if (!input.has_vehicle_pose || input.vehicle_pose_map.header.frame_id != "map") {
    return false;
  }
  const auto & stamp = input.observation.header.stamp;
  const auto & pose_stamp = input.vehicle_pose_map.header.stamp;
  if ((stamp.sec == 0 && stamp.nanosec == 0) ||
    (pose_stamp.sec == 0 && pose_stamp.nanosec == 0))
  {
    return false;
  }
  const double source_seconds = static_cast<double>(stamp.sec) +
    static_cast<double>(stamp.nanosec) * 1.0e-9;
  const double pose_seconds = static_cast<double>(pose_stamp.sec) +
    static_cast<double>(pose_stamp.nanosec) * 1.0e-9;
  const bool finite_pose =
    std::isfinite(input.vehicle_pose_map.pose.position.x) &&
    std::isfinite(input.vehicle_pose_map.pose.position.y) &&
    std::isfinite(input.vehicle_pose_map.pose.position.z) &&
    std::isfinite(input.vehicle_pose_map.pose.orientation.x) &&
    std::isfinite(input.vehicle_pose_map.pose.orientation.y) &&
    std::isfinite(input.vehicle_pose_map.pose.orientation.z) &&
    std::isfinite(input.vehicle_pose_map.pose.orientation.w);
  return finite_pose && std::isfinite(source_seconds) && std::isfinite(pose_seconds) &&
         std::abs(source_seconds - pose_seconds) <= config_.pose_timestamp_tolerance_s;
}

RingTrackerUpdate RingTracker::ignoredUpdate() const
{
  RingTrackerUpdate result;
  result.phase = snapshot_.phase;
  result.consecutive_frames = snapshot_.consecutive_frames;
  result.ignored = true;
  return result;
}

RingTrackerUpdate RingTracker::accept(const RingTrackerInput & input)
{
  const auto & observation = input.observation;
  banyu_msgs::msg::Task2RingRecord record;
  record.frame_sequence = observation.frame_sequence;
  record.color = observation.color;
  record.confidence = observation.confidence;
  record.vehicle_pose_map = input.vehicle_pose_map;

  snapshot_.records.push_back(record);
  snapshot_.phase = RingTrackerPhase::TRACKING;
  snapshot_.active_color = observation.color;
  snapshot_.consecutive_frames = 1;
  snapshot_.invisible_frames = 0;
  snapshot_.has_last_state = true;
  snapshot_.last_state.header = observation.header;
  snapshot_.last_state.index = static_cast<uint32_t>(snapshot_.records.size() - 1);
  snapshot_.last_state.color = observation.color;
  snapshot_.last_state.confidence = observation.confidence;
  if (observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
    snapshot_.red_pending = true;
    snapshot_.repaired = false;
  }

  if (input.has_path_distance && std::isfinite(input.path_distance_m)) {
    last_record_path_distance_ = input.path_distance_m;
    has_last_record_path_distance_ = true;
  }
  last_record_pose_ = input.vehicle_pose_map;
  has_last_record_pose_ = true;
  clearEntry();
  snapshot_.phase = RingTrackerPhase::TRACKING;
  snapshot_.consecutive_frames = 1;

  RingTrackerUpdate result;
  result.phase = snapshot_.phase;
  result.accepted = true;
  result.consecutive_frames = snapshot_.consecutive_frames;
  result.record_index = snapshot_.records.size() - 1;
  result.record = record;
  return result;
}

RingTrackerUpdate RingTracker::update(
  const banyu_msgs::msg::PipelineRingObservation & observation)
{
  RingTrackerInput input;
  input.observation = observation;
  return update(input);
}

RingTrackerUpdate RingTracker::update(const RingTrackerInput & input)
{
  const auto & observation = input.observation;
  if (has_sequence_ && observation.frame_sequence <= last_sequence_) {
    return ignoredUpdate();
  }
  const bool sequence_gap = has_sequence_ &&
    observation.frame_sequence - last_sequence_ >
    config_.maximum_sequence_gap;
  has_sequence_ = true;
  last_sequence_ = observation.frame_sequence;

  if (sequence_gap) {
    // A gap is unknown time, not confirmed absence.  Restart the exit window
    // rather than entering or bypassing debounce.
    if (snapshot_.phase == RingTrackerPhase::TRACKING ||
      snapshot_.phase == RingTrackerPhase::EXIT ||
      snapshot_.phase == RingTrackerPhase::DEBOUNCE ||
      (snapshot_.phase == RingTrackerPhase::ENTRY && !snapshot_.records.empty()))
    {
      beginExit();
    } else {
      snapshot_.phase = RingTrackerPhase::IDLE;
      snapshot_.invisible_frames = 0;
      clearEntry();
    }
    return ignoredUpdate();
  }

  const ObservationKind kind = classifyObservation(observation);
  if (kind == ObservationKind::MALFORMED_OR_UNCERTAIN) {
    if (snapshot_.phase == RingTrackerPhase::EXIT ||
      snapshot_.phase == RingTrackerPhase::DEBOUNCE)
    {
      beginExit();
    }
    return ignoredUpdate();
  }
  const bool visible = kind == ObservationKind::VISIBLE;

  if (snapshot_.phase == RingTrackerPhase::TRACKING) {
    if (visible && sameColor(observation.color, snapshot_.active_color)) {
      snapshot_.consecutive_frames = std::min(
        snapshot_.consecutive_frames + 1, config_.entry_confirm_frames);
      snapshot_.invisible_frames = 0;
      return ignoredUpdate();
    }
    beginExit();
  }

  if (snapshot_.phase == RingTrackerPhase::EXIT) {
    if (!visible) {
      ++snapshot_.invisible_frames;
      if (snapshot_.invisible_frames >= config_.exit_confirm_frames) {
        beginDebounce();
      }
      RingTrackerUpdate result = ignoredUpdate();
      result.exited = snapshot_.phase == RingTrackerPhase::DEBOUNCE;
      return result;
    }
    // A visible frame before exit confirmation is flicker from the current
    // ring.  Continue tracking it only when its color is unchanged.
    if (sameColor(observation.color, snapshot_.active_color)) {
      snapshot_.phase = RingTrackerPhase::TRACKING;
      snapshot_.invisible_frames = 0;
      snapshot_.consecutive_frames = 1;
      return ignoredUpdate();
    }
    snapshot_.invisible_frames = 0;
    return ignoredUpdate();
  }

  if (snapshot_.phase == RingTrackerPhase::DEBOUNCE) {
    if (!visible) {
      ++snapshot_.invisible_frames;
      if (snapshot_.invisible_frames < config_.debounce_frames) {
        return ignoredUpdate();
      }
      snapshot_.phase = RingTrackerPhase::IDLE;
      snapshot_.active_color =
        banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
      snapshot_.invisible_frames = 0;
    } else if (sameColor(observation.color, snapshot_.active_color)) {
      snapshot_.phase = RingTrackerPhase::TRACKING;
      snapshot_.invisible_frames = 0;
      snapshot_.consecutive_frames = 1;
      return ignoredUpdate();
    } else {
      // Any visible frame interrupts absence, even when it is another color.
      beginExit();
      return ignoredUpdate();
    }
  }

  if (!visible) {
    snapshot_.phase = RingTrackerPhase::IDLE;
    snapshot_.invisible_frames = 0;
    clearEntry();
    return ignoredUpdate();
  }
  if (!rearmAllowed(input)) {
    return ignoredUpdate();
  }

  if (!has_pending_color_ || !sameColor(pending_color_, observation.color)) {
    pending_color_ = observation.color;
    has_pending_color_ = true;
    snapshot_.phase = RingTrackerPhase::ENTRY;
    snapshot_.consecutive_frames = 1;
  } else {
    ++snapshot_.consecutive_frames;
  }

  if (snapshot_.consecutive_frames < config_.entry_confirm_frames) {
    return ignoredUpdate();
  }
  if (config_.reject_duplicate_red &&
    observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED &&
    hasRecordedColor(banyu_msgs::msg::PipelineRingObservation::COLOR_RED))
  {
    return ignoredUpdate();
  }
  return accept(input);
}

bool RingTracker::markRedRepaired()
{
  if (!snapshot_.red_pending || snapshot_.repaired) {
    return false;
  }
  snapshot_.red_pending = false;
  snapshot_.repaired = true;
  return true;
}

void RingTracker::reset()
{
  snapshot_.phase = RingTrackerPhase::IDLE;
  snapshot_.active_color =
    banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  snapshot_.consecutive_frames = 0;
  snapshot_.invisible_frames = 0;
  snapshot_.red_pending = false;
  snapshot_.repaired = false;
  snapshot_.records.clear();
  snapshot_.has_last_state = false;
  snapshot_.last_state = banyu_msgs::msg::PipelineRingState();
  has_sequence_ = false;
  last_sequence_ = 0;
  last_record_path_distance_ = 0.0;
  has_last_record_path_distance_ = false;
  last_record_pose_ = geometry_msgs::msg::PoseStamped();
  has_last_record_pose_ = false;
  clearEntry();
}

}  // namespace banyu_detection
