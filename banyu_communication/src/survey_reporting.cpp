#include "banyu_communication/survey_reporting.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace banyu_communication
{
namespace
{
int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.nanosec >= 1000000000U) {return -1;}
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool allowed(const std::vector<std::string> & sources, const std::string & source)
{
  return !source.empty() && std::find(sources.begin(), sources.end(), source) != sources.end();
}

bool historicalStamp(int64_t stamp, int64_t start, int64_t now)
{
  return stamp > 0 && stamp >= start && stamp <= now;
}

bool validReportId(const std::string & report_id)
{
  if (report_id.empty() || report_id.size() > 128) {return false;}
  for (const char character : report_id) {
    if (!((character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') || character == '-' || character == '_' ||
      character == '.')) {return false;}
  }
  return true;
}
}  // namespace

SurveyLedger::SurveyLedger()
: SurveyLedger(Config{}) {}

SurveyLedger::SurveyLedger(Config config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.minimum_confidence) || config_.minimum_confidence < 0.0 ||
    config_.minimum_confidence > 1.0 || !std::isfinite(config_.maximum_buoy_uncertainty_m) ||
    config_.maximum_buoy_uncertainty_m <= 0.0 || !std::isfinite(config_.ingress_max_age_s) ||
    config_.ingress_max_age_s <= 0.0 || !std::isfinite(config_.minimum_progress_separation_m) ||
    config_.minimum_progress_separation_m < 0.0 || config_.tier < 2 || config_.tier > 4)
  {throw std::invalid_argument("Invalid survey evidence validation configuration");}
}

void SurveyLedger::beginRun(uint32_t run_id, uint64_t epoch, int64_t source_start_ns)
{
  run_id_ = run_id;
  epoch_ = epoch;
  source_start_ns_ = source_start_ns;
  revision_ = 0;
  originals_.clear();
  evidence_ = banyu_msgs::msg::Task2SurveyEvidence{};
  buoy_pinned_ = false;
  course_changed_ = false;
  complete_ = false;
  reason_ = run_id == 0 || epoch == 0 || source_start_ns <= 0 ?
    "REPORT_RUN_CONTEXT_UNVERIFIED" : "SURVEY_INCOMPLETE observed=0 expected=4";
}

void SurveyLedger::setCourse(const std::string & course_id, uint32_t pinger_freq_hz)
{
  if (buoy_pinned_ && (course_id != course_id_ || pinger_freq_hz != frequency_)) {
    complete_ = false;
    course_changed_ = true;
    reason_ = "COURSE_CHANGED_DURING_SURVEY";
  }
  course_id_ = course_id;
  frequency_ = pinger_freq_hz;
}

std::string SurveyLedger::ingest(
  const banyu_msgs::msg::Task2SurveyEvidence & value, int64_t source_now_ns)
{
  if (source_start_ns_ <= 0) {return "REPORT_RUN_CONTEXT_UNVERIFIED";}
  if (run_id_ == 0 || epoch_ == 0 || value.run_id != run_id_ || value.epoch != epoch_) {
    return "WRONG_RUN";
  }
  if (!allowed(config_.evidence_sources, value.source_id)) {return "EVIDENCE_SOURCE_UNTRUSTED";}
  if (value.revision == 0 || value.revision <= revision_) {return "STALE_EVIDENCE";}
  revision_ = value.revision;
  complete_ = false;
  const auto fail = [this](std::string diagnostic) {
      reason_ = std::move(diagnostic);
      return reason_;
    };
  if (course_changed_) {return fail("COURSE_CHANGED_DURING_SURVEY");}
  const auto source_stamp = stampNs(value.header.stamp);
  if (!historicalStamp(source_stamp, source_start_ns_, source_now_ns) ||
    (source_now_ns - source_stamp) * 1.0e-9 > config_.ingress_max_age_s)
  {return fail("EVIDENCE_STAMP_INVALID_OR_STALE");}
  if (course_id_.empty() || frequency_ == 0) {return fail("COURSE_UNAVAILABLE");}
  const auto & buoy = value.active_buoy;
  const auto buoy_stamp_valid = [&](int64_t stamp) {
      return stamp > 0 && stamp <= source_now_ns &&
             (buoy_pinned_ || (source_now_ns - stamp) * 1.0e-9 <= config_.ingress_max_age_s);
    };
  if (buoy.run_id != run_id_ || buoy.epoch != epoch_ || buoy.revision == 0) {
    return fail("ACTIVE_BUOY_UNVERIFIED: run identity");
  }
  if (!allowed(config_.buoy_sources, buoy.source_id) || buoy.course_id != course_id_ ||
    buoy.endpoint_id.empty())
  {
    return fail("ACTIVE_BUOY_UNVERIFIED: source, course or endpoint");
  }
  if ((!buoy.green_confirmed && !buoy.pinger_confirmed) ||
    (buoy.pinger_confirmed && buoy.pinger_freq_hz != frequency_))
  {
    return fail("ACTIVE_BUOY_UNVERIFIED: not GREEN or pinger confirmed");
  }
  if (!buoy_stamp_valid(stampNs(buoy.header.stamp)) ||
    !buoy_stamp_valid(stampNs(buoy.fix.header.stamp)))
  {
    return fail("ACTIVE_BUOY_UNVERIFIED: stamp missing, future or stale");
  }
  if (buoy.fix.status.status < 0 || !std::isfinite(buoy.fix.latitude) ||
    std::abs(buoy.fix.latitude) > 90.0 || !std::isfinite(buoy.fix.longitude) ||
    std::abs(buoy.fix.longitude) > 180.0 || !std::isfinite(buoy.uncertainty_m) ||
    buoy.uncertainty_m < 0.0 || buoy.uncertainty_m > config_.maximum_buoy_uncertainty_m)
  {
    return fail("ACTIVE_BUOY_UNVERIFIED: invalid fix or uncertainty");
  }
  const bool low = buoy.endpoint_id == value.low_progress_endpoint_id;
  const bool high = buoy.endpoint_id == value.high_progress_endpoint_id;
  if (value.low_progress_endpoint_id.empty() || value.high_progress_endpoint_id.empty() ||
    value.low_progress_endpoint_id == value.high_progress_endpoint_id || low == high)
  {return fail("ORDER_UNVERIFIED: active endpoint does not define orientation");}
  if (buoy_pinned_ && (buoy.endpoint_id != evidence_.active_buoy.endpoint_id ||
    buoy.fix.latitude != evidence_.active_buoy.fix.latitude ||
    buoy.fix.longitude != evidence_.active_buoy.fix.longitude ||
    value.low_progress_endpoint_id != evidence_.low_progress_endpoint_id ||
    value.high_progress_endpoint_id != evidence_.high_progress_endpoint_id))
  {return fail("ACTIVE_ENDPOINT_CHANGED_DURING_SURVEY");}
  if (value.rings.size() > 4) {
    return fail("SURVEY_INCOMPLETE observed=" + std::to_string(value.rings.size()) + " expected=4");
  }

  std::vector<double> current_progress;
  std::set<uint32_t> identities;
  std::string snapshot_failure;

  for (const auto & ring : value.rings) {
    if (ring.run_id != run_id_ || ring.epoch != epoch_ || !identities.insert(ring.node_id).second) {
      if (snapshot_failure.empty()) {
        snapshot_failure = "NODE_IDENTITY_AMBIGUOUS";
      }
      continue;
    }
    if (!ring.physical_identity_verified) {
      if (snapshot_failure.empty()) {
        snapshot_failure = "NODE_IDENTITY_UNVERIFIED";
      }
      continue;
    }
    if (!ring.ordering_verified || !std::isfinite(ring.pipeline_progress_m)) {
      if (snapshot_failure.empty()) {
        snapshot_failure = "ORDER_UNVERIFIED";
      }
      continue;
    }
    current_progress.push_back(ring.pipeline_progress_m);
    if (ring.frame_sequence == 0 || !std::isfinite(ring.confidence) ||
      ring.confidence < config_.minimum_confidence || ring.confidence > 1.0F ||
      ring.vehicle_pose_map.header.frame_id != "map" ||
      !historicalStamp(stampNs(ring.vehicle_pose_map.header.stamp), source_start_ns_,
        source_now_ns))
    {
      if (snapshot_failure.empty()) {
        snapshot_failure = "OBSERVATION_PROVENANCE_INVALID";
      }
      continue;
    }
    const auto & position = ring.vehicle_pose_map.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      if (snapshot_failure.empty()) {
        snapshot_failure = "OBSERVATION_PROVENANCE_INVALID";
      }
      continue;
    }
    if (originals_.count(ring.node_id) == 0) {
      if (ring.color != 1 && ring.color != 2) {
        if (snapshot_failure.empty()) {
          snapshot_failure = "ORIGINAL_CONDITION_UNKNOWN";
        }
        continue;
      }
      originals_.emplace(ring.node_id, ring);
    } else {
      // A light only ever goes RED to GREEN (repair). One first seen GREEN from afar that later reads RED, before
      // any repair is verified, was RED all along: upgrade it. After the repair a RED flash on a GREEN node is the
      // resource code, not the original condition.
      if (!value.repair_verified && ring.color == 1 && originals_.at(ring.node_id).color != 1) {
        originals_.at(ring.node_id).color = ring.color;
      }
      if (originals_.at(ring.node_id).pipeline_progress_m != ring.pipeline_progress_m) {
        if (snapshot_failure.empty()) {
          snapshot_failure = "ORDER_UNVERIFIED: original progression changed";
        }
        continue;
      }
    }
  }

  if (originals_.size() > 4) {return fail("NODE_IDENTITY_AMBIGUOUS: first-pass identity changed");}

  std::sort(current_progress.begin(), current_progress.end());
  for (std::size_t i = 1; i < current_progress.size(); ++i) {
    if (current_progress[i] - current_progress[i - 1] <= config_.minimum_progress_separation_m) {
      if (snapshot_failure.empty()) {
        snapshot_failure = "ORDER_UNVERIFIED: coincident or ambiguous progression";
      }
      break;
    }
  }

  std::vector<double> progress;
  for (const auto & ring : originals_) {
    progress.push_back(ring.second.pipeline_progress_m);
  }
  std::sort(progress.begin(), progress.end());
  for (std::size_t i = 1; i < progress.size(); ++i) {
    if (progress[i] - progress[i - 1] <= config_.minimum_progress_separation_m) {
      return fail("ORDER_UNVERIFIED: coincident or ambiguous progression");
    }
  }

  evidence_ = value;
  buoy_pinned_ = buoy_pinned_ || !originals_.empty();

  if (!snapshot_failure.empty()) {
    return fail(snapshot_failure);
  }

  if (value.rings.size() != 4 || originals_.size() != 4) {
    return fail("SURVEY_INCOMPLETE observed=" + std::to_string(value.rings.size()) + " expected=4");
  }
  // A higher tier whose repair or flash code could not be confirmed reports as the Core survey only (the evidence
  // node then sends tier 2): the Core points do not depend on the code.
  const bool core_fallback = value.tier == 2 && config_.tier > 2;
  if (value.tier != config_.tier && !core_fallback) {return fail("TIER_CONTEXT_MISMATCH");}
  if (!value.traversal_complete) {return fail("TRAVERSAL_INCOMPLETE");}
  if (value.tier != 2) {
    if (!value.repair_verified) {return fail("REPAIR_UNVERIFIED");}
    if (!value.resource_code_complete || value.delivery_circle_color < 1 ||
      value.delivery_circle_color > 3 ||
      (value.tier == 3 && value.resource_color != 4) ||
      (value.tier == 4 && (value.resource_color < 1 || value.resource_color > 3)))
    {return fail("RESOURCE_CODE_INCOMPLETE");}
  }
  complete_ = true;
  reason_.clear();
  return {};
}

SurveyLedger::Report SurveyLedger::build(const std::string & report_id) const
{
  if (!complete_) {throw std::logic_error(reason_);}
  if (!validReportId(report_id)) {
    throw std::invalid_argument("Invalid report identity: " + report_id);
  }
  Report report;
  report.run_id = run_id_;
  report.epoch = epoch_;
  report.report_id = report_id;
  report.course_id = course_id_;
  report.tier = evidence_.tier;
  report.buoy_latitude = evidence_.active_buoy.fix.latitude;
  report.buoy_longitude = evidence_.active_buoy.fix.longitude;
  report.resource_color = report.tier == 2 ? 0 : evidence_.resource_color;
  report.delivery_circle_color = report.tier == 2 ? 0 : evidence_.delivery_circle_color;

  std::vector<banyu_msgs::msg::Task2RingRecord> ordered;
  for (const auto & ring : originals_) {
    ordered.push_back(ring.second);
  }
  const bool ascending = evidence_.active_buoy.endpoint_id == evidence_.low_progress_endpoint_id;
  std::sort(ordered.begin(), ordered.end(), [ascending](const auto & first, const auto & second) {
      return ascending ? first.pipeline_progress_m<second.pipeline_progress_m :
             first.pipeline_progress_m> second.pipeline_progress_m;
    });
  for (std::size_t i = 0; i < 4 && i < ordered.size(); ++i) {
    report.original_conditions[i] = ordered[i].color == 1 ? 2 : 1;
  }
  return report;
}

bool SurveyLedger::bestEffortAvailable() const
{
  return evidence_.revision != 0 && evidence_.run_id == run_id_ && evidence_.epoch == epoch_ &&
         !evidence_.active_buoy.endpoint_id.empty() && std::isfinite(evidence_.active_buoy.fix.latitude) &&
         std::isfinite(evidence_.active_buoy.fix.longitude);
}

SurveyLedger::Report SurveyLedger::buildBestEffort(const std::string & report_id) const
{
  if (complete_) {return build(report_id);}
  if (!bestEffortAvailable()) {throw std::logic_error("BEST_EFFORT_UNAVAILABLE: no verified active buoy");}
  if (!validReportId(report_id)) {
    throw std::invalid_argument("Invalid report identity: " + report_id);
  }
  Report report;
  report.run_id = run_id_;
  report.epoch = epoch_;
  report.report_id = report_id;
  report.course_id = course_id_;
  report.tier = 2;
  report.buoy_latitude = evidence_.active_buoy.fix.latitude;
  report.buoy_longitude = evidence_.active_buoy.fix.longitude;

  std::vector<banyu_msgs::msg::Task2RingRecord> seen;
  for (const auto & ring : originals_) {seen.push_back(ring.second);}
  if (seen.empty()) {
    for (const auto & ring : evidence_.rings) {
      if ((ring.color == 1 || ring.color == 2) && std::isfinite(ring.pipeline_progress_m)) {seen.push_back(ring);}
    }
  }
  const bool ascending = evidence_.active_buoy.endpoint_id == evidence_.low_progress_endpoint_id;
  std::sort(seen.begin(), seen.end(), [ascending](const auto & first, const auto & second) {
      return ascending ? first.pipeline_progress_m<second.pipeline_progress_m :
             first.pipeline_progress_m> second.pipeline_progress_m;
    });
  report.original_conditions.fill(1);  // GREEN where nothing was seen
  for (std::size_t i = 0; i < 4 && i < seen.size(); ++i) {
    report.original_conditions[i] = seen[i].color == 1 ? 2 : 1;
  }
  return report;
}

rx_msgs::msg::PipelineSurvey SurveyLedger::Report::toPipelineSurvey() const
{
  rx_msgs::msg::PipelineSurvey survey;
  survey.active_buoy_position.latitude = buoy_latitude;
  survey.active_buoy_position.longitude = buoy_longitude;
  survey.segments.assign(original_conditions.begin(), original_conditions.end());
  return survey;
}

rx_msgs::msg::ResourceDelivery SurveyLedger::Report::toResourceDelivery() const
{
  rx_msgs::msg::ResourceDelivery delivery;
  delivery.task = rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR;
  delivery.resource_color = resource_color;
  delivery.delivery_circle_color = delivery_circle_color;
  return delivery;
}

SurfaceGuard::SurfaceGuard()
: SurfaceGuard(Config{}) {}

SurfaceGuard::SurfaceGuard(Config config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.hold_s) || config_.hold_s <= 0.0 ||
    !std::isfinite(config_.surface_depth_m) || config_.surface_depth_m < 0.0 ||
    !std::isfinite(config_.tolerance_m) || config_.tolerance_m <= 0.0 ||
    !std::isfinite(config_.ttl_s) || config_.ttl_s <= 0.0)
  {throw std::invalid_argument("Invalid continuous surface guard configuration");}
}

void SurfaceGuard::beginRun(uint32_t run_id, uint64_t epoch)
{
  run_id_ = run_id;
  epoch_ = epoch;
  last_source_ns_ = 0;
  last_received_ns_ = 0;
  hold_started_ns_ = 0;
  hold_sample_count_ = 0;
  sample_received_ = false;
  calibration_id_.clear();
  source_id_.clear();
  invalidate("SURFACE_TELEMETRY_MISSING");
}

void SurfaceGuard::invalidate(const std::string & reason)
{
  near_surface_ = false;
  hold_started_ns_ = 0;
  hold_sample_count_ = 0;
  reason_ = reason;
}

void SurfaceGuard::observe(
  const banyu_msgs::msg::WaterDepth & depth, int64_t source_now_ns, int64_t steady_now_ns)
{
  if (run_id_ == 0 || epoch_ == 0 || depth.run_id != run_id_ || depth.epoch != epoch_) {return;}
  if (!allowed(config_.sources, depth.source_id)) {return;}
  const auto stamp = stampNs(depth.header.stamp);
  if (!depth.valid || !depth.calibrated || depth.datum != "water_surface" ||
    depth.calibration_id.empty() || !std::isfinite(depth.depth_m))
  {invalidate("SURFACE_DATUM_UNVERIFIED"); return;}
  if (stamp <= 0 || stamp > source_now_ns || (source_now_ns - stamp) * 1.0e-9 > config_.ttl_s) {
    invalidate("SURFACE_TELEMETRY_STAMP_INVALID");
    return;
  }
  if (stamp <= last_source_ns_) {return;}
  if (steady_now_ns < last_received_ns_ || (sample_received_ &&
    ((steady_now_ns - last_received_ns_) * 1.0e-9 > config_.ttl_s ||
    (stamp - last_source_ns_) * 1.0e-9 > config_.ttl_s)) ||
    (!calibration_id_.empty() && calibration_id_ != depth.calibration_id) ||
    (!source_id_.empty() && source_id_ != depth.source_id))
  {invalidate("SURFACE_CONTINUITY_LOST");}
  last_source_ns_ = stamp;
  last_received_ns_ = steady_now_ns;
  sample_received_ = true;
  calibration_id_ = depth.calibration_id;
  source_id_ = depth.source_id;
  if (std::abs(depth.depth_m - config_.surface_depth_m) > config_.tolerance_m) {
    invalidate("NOT_SURFACED");
    return;
  }
  if (!near_surface_) {
    hold_started_ns_ = steady_now_ns;
    hold_sample_count_ = 1;
  } else {
    hold_sample_count_++;
  }
  near_surface_ = true;
  reason_ = "SURFACE_HOLD_INCOMPLETE";
}

bool SurfaceGuard::ready(int64_t source_now_ns, int64_t steady_now_ns)
{
  if (!near_surface_) {return false;}
  if (steady_now_ns < last_received_ns_ || source_now_ns < last_source_ns_ ||
    (steady_now_ns - last_received_ns_) * 1.0e-9 > config_.ttl_s ||
    (source_now_ns - last_source_ns_) * 1.0e-9 > config_.ttl_s)
  {invalidate("SURFACE_TELEMETRY_STALE"); return false;}
  if (config_.hold_s > 0.0) {
    if (hold_sample_count_ < 2 ||
      (last_received_ns_ - hold_started_ns_) * 1.0e-9 < config_.hold_s)
    {
      reason_ = "SURFACE_HOLD_INCOMPLETE";
      return false;
    }
  }
  reason_.clear();
  return true;
}

}  // namespace banyu_communication
