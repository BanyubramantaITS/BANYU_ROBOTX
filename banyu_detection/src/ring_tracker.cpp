#include "banyu_detection/ring_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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
      return !record.superseded && record.color == color;
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

void RingTracker::beginNewRingEntry()
{
  clearEntry();
  snapshot_.phase = RingTrackerPhase::IDLE;
  snapshot_.active_color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
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
  if (rearmed_) {
    return true;
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
    std::abs(input.path_distance_m - last_record_path_distance_) >=
    config_.minimum_rearm_distance_m;
  const bool centerline_progressed = hasCenterline() && has_last_record_path_distance_ &&
    std::abs(
      projectOntoCenterline(
        input.vehicle_pose_map.pose.position.x,
        input.vehicle_pose_map.pose.position.y) -
      last_record_path_distance_) >= config_.minimum_rearm_distance_m;
  return spatially_separated || progressed || centerline_progressed;
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

void RingTracker::setCenterline(const std::vector<std::pair<double, double>> & waypoints)
{
  centerline_waypoints_ = waypoints;
}

bool RingTracker::hasCenterline() const
{
  return centerline_waypoints_.size() >= 2;
}

double RingTracker::projectOntoCenterline(double x, double y) const
{
  if (!hasCenterline()) {
    return 0.0;
  }

  double best_dist_sq = std::numeric_limits<double>::max();
  double best_arc_length = 0.0;
  double cumulative_len = 0.0;

  for (std::size_t i = 0; i + 1 < centerline_waypoints_.size(); ++i) {
    const double x0 = centerline_waypoints_[i].first;
    const double y0 = centerline_waypoints_[i].second;
    const double x1 = centerline_waypoints_[i + 1].first;
    const double y1 = centerline_waypoints_[i + 1].second;

    const double vx = x1 - x0;
    const double vy = y1 - y0;
    const double seg_len_sq = vx * vx + vy * vy;
    const double seg_len = std::sqrt(seg_len_sq);

    if (seg_len_sq < 1e-9) {
      continue;
    }

    const double wx = x - x0;
    const double wy = y - y0;
    const double t = std::clamp((wx * vx + wy * vy) / seg_len_sq, 0.0, 1.0);

    const double proj_x = x0 + t * vx;
    const double proj_y = y0 + t * vy;
    const double dx = x - proj_x;
    const double dy = y - proj_y;
    const double dist_sq = dx * dx + dy * dy;

    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_arc_length = cumulative_len + t * seg_len;
    }

    cumulative_len += seg_len;
  }

  return best_arc_length;
}

bool RingTracker::cameraModelEnabled() const
{
  return config_.camera_height_m > 0.0 && config_.camera_half_vfov_rad > 0.0;
}

bool RingTracker::kind_is_visible(const banyu_msgs::msg::PipelineRingObservation & observation)
{
  return observation.visible && std::isfinite(observation.offset_y);
}

bool RingTracker::ringProgress(const RingTrackerInput & input, double & progress) const
{
  double x = input.vehicle_pose_map.pose.position.x;
  double y = input.vehicle_pose_map.pose.position.y;
  double ahead = 0.0;
  if (cameraModelEnabled() && input.has_vehicle_pose && kind_is_visible(input.observation)) {
    // Bottom camera pitched forward of vertical; image +y (down) is aft.
    const double angle = config_.camera_tilt_from_vertical_rad -
      config_.camera_half_vfov_rad * std::clamp(
      static_cast<double>(input.observation.offset_y), -1.0, 1.0);
    // The measured height (pipe width) when there is one: the fixed one is right only at the survey altitude, and a
    // vehicle 1.15 m above the nodes instead of 1.7 put far rings 50% too far ahead (two records for one node).
    double height = input.camera_height_m > 0.0 ? input.camera_height_m : config_.camera_height_m;
    // Only when the lid is big (close, about 0.8 m or less): at the survey altitude its size is a few pixels and the range from
    // it is too noisy, the fixed height does better.
    if (config_.lid_diameter_m > 0.0 && config_.focal_ratio > 0.0 && input.observation.size >= 0.08F) {
      const double range = config_.focal_ratio * config_.lid_diameter_m / input.observation.size;
      height = std::clamp(
        range * std::cos(config_.camera_tilt_from_vertical_rad), 0.1, 1.5 * config_.camera_height_m);
    }
    ahead = config_.camera_forward_m + height * std::tan(angle);
    const auto & q = input.vehicle_pose_map.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // Sideways too: a node off the vehicle's line sits on a different part of a bent pipe, and leaving the offset out put the same
    // node at progress values 0.9 m apart from one view to the next, which made duplicate records and pushed the last node out.
    double right = 0.0;
    if (config_.focal_ratio > 0.0) {
      const double slant = height / std::cos(angle);
      right = std::clamp(static_cast<double>(input.observation.offset_x), -1.0, 1.0) *
        0.5 / config_.focal_ratio * slant;
    }
    x += ahead * std::cos(yaw) + right * std::sin(yaw);
    y += ahead * std::sin(yaw) - right * std::cos(yaw);
  }
  if (hasCenterline() && input.has_vehicle_pose) {
    progress = projectOntoCenterline(x, y);
    return true;
  }
  if (input.has_path_distance && std::isfinite(input.path_distance_m)) {
    progress = input.path_distance_m + ahead;
    return true;
  }
  return false;
}

int RingTracker::findAssociatedRecord(
  double progress_m,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (snapshot_.records.empty()) {
    return -1;
  }

  int best_index = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < snapshot_.records.size(); ++i) {
    const auto & rec = snapshot_.records[i];
    if (rec.superseded) {
      continue;
    }
    if (hasCenterline()) {
      const double delta_progress = std::abs(progress_m - rec.pipeline_progress_m);
      if (delta_progress <= config_.node_association_distance_m && delta_progress < min_dist) {
        min_dist = delta_progress;
        best_index = static_cast<int>(i);
      }
    } else {
      const double dx = pose.pose.position.x - rec.vehicle_pose_map.pose.position.x;
      const double dy = pose.pose.position.y - rec.vehicle_pose_map.pose.position.y;
      const double dz = pose.pose.position.z - rec.vehicle_pose_map.pose.position.z;
      const double spatial_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (std::isfinite(spatial_dist) &&
        spatial_dist <= config_.node_association_distance_m &&
        spatial_dist < min_dist)
      {
        min_dist = spatial_dist;
        best_index = static_cast<int>(i);
      }
    }
  }

  return best_index;
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
  double calculated_progress = 0.0;
  const bool valid_progress = ringProgress(input, calculated_progress);

  const int match_idx = findAssociatedRecord(
    valid_progress ? calculated_progress : 0.0,
    input.vehicle_pose_map);

  if (match_idx >= 0) {
    // The same node can first look GREEN (bloom, a ring half in view) and then RED. A damaged node must not be
    // reported GREEN, and it must not take a second slot: RED upgrades the record, GREEN never downgrades it.
    if (observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED &&
      snapshot_.records[match_idx].color != banyu_msgs::msg::PipelineRingObservation::COLOR_RED)
    {
      snapshot_.records[match_idx].color = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
      snapshot_.red_pending = true;
      snapshot_.repaired = false;
    }
    const auto & existing = snapshot_.records[match_idx];
    banyu_msgs::msg::Task2RingRecord record = existing;

    snapshot_.phase = RingTrackerPhase::TRACKING;
    snapshot_.active_color = observation.color;
    snapshot_.consecutive_frames = 1;
    snapshot_.invisible_frames = 0;
    snapshot_.has_last_state = true;
    snapshot_.last_state.header = observation.header;
    snapshot_.last_state.index = static_cast<uint32_t>(match_idx);
    snapshot_.last_state.color = existing.color;
    snapshot_.last_state.confidence = observation.confidence;
    snapshot_.last_state.run_id = existing.run_id;
    snapshot_.last_state.epoch = existing.epoch;
    snapshot_.last_state.node_id = existing.node_id;
    snapshot_.last_state.ordering_verified = existing.ordering_verified;

    last_record_pose_ = input.vehicle_pose_map;
    has_last_record_pose_ = true;
    last_record_path_distance_ = existing.pipeline_progress_m;
    has_last_record_path_distance_ = true;
    rearmed_ = false;
    clearEntry();

    RingTrackerUpdate result;
    result.phase = snapshot_.phase;
    result.accepted = true;
    result.consecutive_frames = snapshot_.consecutive_frames;
    result.record_index = static_cast<std::size_t>(match_idx);
    result.record = record;
    return result;
  }

  // The pipe has four nodes: a fifth distinct sighting means two records are one node (seen from far, or from beside
  // the line). Merge the closest pair (the other is superseded, its id never reused), so a later node (the RED one in
  // seed 24) is not lost.
  if (config_.max_record_height_m > 0.0 && input.camera_height_m > config_.max_record_height_m) {
    clearEntry();
    return ignoredUpdate();
  }
  const std::size_t slot = snapshot_.records.size();
  std::size_t survivor = 0;
  std::size_t superseded = 0;
  bool merged = false;
  if (liveRecordCount() >= 4) {
    if (!valid_progress || !mergeClosestRecords(survivor, superseded)) {
      clearEntry();
      return ignoredUpdate();
    }
    merged = true;
  }

  banyu_msgs::msg::Task2RingRecord record;
  record.run_id = input.run_id;
  record.epoch = input.epoch;
  record.node_id = static_cast<uint32_t>(slot);
  record.frame_sequence = observation.frame_sequence;
  record.color = observation.color;
  record.confidence = observation.confidence;
  record.vehicle_pose_map = input.vehicle_pose_map;

  if (valid_progress) {
    record.pipeline_progress_m = calculated_progress;
    record.physical_identity_verified = true;
    if (snapshot_.records.empty()) {
      record.ordering_verified = true;
    } else {
      const double delta = calculated_progress - last_record_path_distance_;
      if (traverse_direction_ == 0 && delta != 0.0) {traverse_direction_ = delta > 0.0 ? 1 : -1;}
      record.ordering_verified = !config_.require_monotonic_order ||
        (traverse_direction_ != 0 && delta * traverse_direction_ > 0.0);
    }
    last_record_path_distance_ = calculated_progress;
    has_last_record_path_distance_ = true;
  } else {
    record.pipeline_progress_m = 0.0;
    record.physical_identity_verified = false;
    record.ordering_verified = false;
  }

  snapshot_.records.push_back(record);
  snapshot_.phase = RingTrackerPhase::TRACKING;
  snapshot_.active_color = observation.color;
  snapshot_.consecutive_frames = 1;
  snapshot_.invisible_frames = 0;
  snapshot_.has_last_state = true;
  snapshot_.last_state.header = observation.header;
  snapshot_.last_state.index = static_cast<uint32_t>(slot);
  snapshot_.last_state.color = observation.color;
  snapshot_.last_state.confidence = observation.confidence;
  snapshot_.last_state.run_id = record.run_id;
  snapshot_.last_state.epoch = record.epoch;
  snapshot_.last_state.node_id = record.node_id;
  snapshot_.last_state.ordering_verified = record.ordering_verified;
  if (observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
    snapshot_.red_pending = true;
    snapshot_.repaired = false;
  }

  last_record_pose_ = input.vehicle_pose_map;
  has_last_record_pose_ = true;
  rearmed_ = false;
  clearEntry();

  RingTrackerUpdate result;
  result.phase = snapshot_.phase;
  result.accepted = true;
  result.consecutive_frames = snapshot_.consecutive_frames;
  result.record_index = slot;
  result.record = record;
  result.merged = merged;
  if (merged) {
    result.merged_record = snapshot_.records[survivor];
    result.superseded_record = snapshot_.records[superseded];
  }
  return result;
}

std::size_t RingTracker::liveRecordCount() const
{
  return static_cast<std::size_t>(std::count_if(
      snapshot_.records.begin(), snapshot_.records.end(),
      [](const banyu_msgs::msg::Task2RingRecord & record) {return !record.superseded;}));
}

bool RingTracker::mergeClosestRecords(std::size_t & survivor, std::size_t & freed)
{
  auto & recs = snapshot_.records;
  std::vector<std::size_t> order;
  for (std::size_t i = 0; i < recs.size(); ++i) {
    if (recs[i].superseded) {continue;}
    if (!recs[i].physical_identity_verified) {return false;}
    order.push_back(i);
  }
  if (order.size() < 2 || !(config_.min_node_spacing_m > 0.0)) {return false;}
  std::sort(order.begin(), order.end(), [&recs](std::size_t a, std::size_t b) {
      return recs[a].pipeline_progress_m < recs[b].pipeline_progress_m;
    });
  std::size_t best = 0;
  for (std::size_t i = 1; i + 1 < order.size(); ++i) {
    if (recs[order[i + 1]].pipeline_progress_m - recs[order[i]].pipeline_progress_m <
      recs[order[best + 1]].pipeline_progress_m - recs[order[best]].pipeline_progress_m)
    {
      best = i;
    }
  }
  if (recs[order[best + 1]].pipeline_progress_m - recs[order[best]].pipeline_progress_m >= config_.min_node_spacing_m) {
    return false;
  }
  const std::size_t a = std::min(order[best], order[best + 1]);
  const std::size_t b = std::max(order[best], order[best + 1]);
  constexpr auto kRed = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
  // The RED record keeps its slot (a repair may refer to it); otherwise the one seen first.
  survivor = (recs[b].color == kRed && recs[a].color != kRed) ? b : a;
  freed = survivor == a ? b : a;
  if (recs[freed].color == kRed) {recs[survivor].color = kRed;}
  // The node's place in the order is its first sighting.
  recs[survivor].frame_sequence = std::min(recs[a].frame_sequence, recs[b].frame_sequence);
  recs[freed].superseded = true;
  return true;
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
  if (has_last_record_pose_ && validPoseForObservation(input)) {
    const double dx = input.vehicle_pose_map.pose.position.x -
      last_record_pose_.pose.position.x;
    const double dy = input.vehicle_pose_map.pose.position.y -
      last_record_pose_.pose.position.y;
    const double dz = input.vehicle_pose_map.pose.position.z -
      last_record_pose_.pose.position.z;
    const double sep = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (std::isfinite(sep) && sep >= config_.minimum_rearm_distance_m) {
      rearmed_ = true;
    }
    beyond_last_node_ = std::isfinite(sep) && sep > config_.node_association_distance_m;
    double ring_progress = 0.0;
    if (cameraModelEnabled() && has_last_record_path_distance_ &&
      kind_is_visible(input.observation) && ringProgress(input, ring_progress))
    {
      beyond_last_node_ = std::abs(ring_progress - last_record_path_distance_) >
        config_.node_association_distance_m;
    }
  } else {
    beyond_last_node_ = false;
  }
  if (kind == ObservationKind::MALFORMED_OR_UNCERTAIN) {
    if (snapshot_.phase == RingTrackerPhase::EXIT ||
      snapshot_.phase == RingTrackerPhase::DEBOUNCE)
    {
      beginExit();
    }
    return ignoredUpdate();
  }
  const bool visible = kind == ObservationKind::VISIBLE;

  // The node just recorded as GREEN now shows RED and the vehicle has not moved on: it is the same damaged node
  // (it can look green first). Upgrade its record instead of ignoring the colour change or taking a second slot.
  if (visible && observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED &&
    snapshot_.has_last_state && !beyond_last_node_ &&
    snapshot_.last_state.index < snapshot_.records.size() &&
    snapshot_.records[snapshot_.last_state.index].color !=
    banyu_msgs::msg::PipelineRingObservation::COLOR_RED &&
    snapshot_.phase != RingTrackerPhase::IDLE)
  {
    const std::size_t index = snapshot_.last_state.index;
    snapshot_.records[index].color = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
    snapshot_.red_pending = true;
    snapshot_.repaired = false;
    snapshot_.phase = RingTrackerPhase::TRACKING;
    snapshot_.active_color = observation.color;
    snapshot_.consecutive_frames = 1;
    snapshot_.invisible_frames = 0;
    snapshot_.last_state.color = observation.color;
    RingTrackerUpdate upgraded;
    upgraded.phase = snapshot_.phase;
    upgraded.accepted = true;
    upgraded.consecutive_frames = snapshot_.consecutive_frames;
    upgraded.record_index = index;
    upgraded.record = snapshot_.records[index];
    return upgraded;
  }

  if (snapshot_.phase == RingTrackerPhase::TRACKING && visible && cameraModelEnabled() &&
    beyond_last_node_)
  {
    // Consecutive same-color rings can be in view back to back; the estimated ring position
    // tells them apart.
    beginNewRingEntry();
  }
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
    // Rings can be closer than the camera footprint, so the next ring may be in view before
    // the last one leaves. Once the vehicle is beyond the last node's association radius, a
    // different color is a new ring and goes through normal entry confirmation.
    if (!beyond_last_node_) {
      snapshot_.invisible_frames = 0;
      return ignoredUpdate();
    }
    beginNewRingEntry();
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
    } else if (beyond_last_node_) {
      beginNewRingEntry();
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
    double calculated_prog = 0.0;
    ringProgress(input, calculated_prog);
    const int match = findAssociatedRecord(calculated_prog, input.vehicle_pose_map);
    if (match < 0) {
      return ignoredUpdate();
    }
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
  traverse_direction_ = 0;
  last_record_pose_ = geometry_msgs::msg::PoseStamped();
  has_last_record_pose_ = false;
  rearmed_ = false;
  clearEntry();
}

}  // namespace banyu_detection
