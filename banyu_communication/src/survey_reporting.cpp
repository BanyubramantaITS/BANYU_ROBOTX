#include "banyu_communication/survey_reporting.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include "banyu_msgs/task2_submission.hpp"

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

std::string ioError(const std::string & operation)
{
  return operation + ": " + std::strerror(errno);
}

bool writeAtomic(const std::string & path, const std::string & contents, std::string & error)
{
  const auto temporary_name = path + ".tmp.XXXXXX";
  std::vector<char> temporary(temporary_name.begin(), temporary_name.end());
  temporary.push_back('\0');
  const int fd = ::mkstemp(temporary.data());
  if (fd == -1) {error = ioError("create outbox entry"); return false;}
  bool success = true;
  std::size_t offset = 0;
  while (offset < contents.size()) {
    const auto written = ::write(fd, contents.data() + offset, contents.size() - offset);
    if (written < 0 && errno == EINTR) {continue;}
    if (written <= 0) {error = ioError("write outbox entry"); success = false; break;}
    offset += static_cast<std::size_t>(written);
  }
  if (success && ::fsync(fd) != 0) {error = ioError("sync outbox entry"); success = false;}
  if (::close(fd) != 0 && success) {error = ioError("close outbox entry"); success = false;}
  if (!success) {::unlink(temporary.data()); return false;}
  if (::rename(temporary.data(), path.c_str()) != 0) {
    error = ioError("commit outbox entry");
    ::unlink(temporary.data());
    return false;
  }
  const auto parent = std::filesystem::path(path).parent_path();
  const int directory = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (directory == -1) {error = ioError("open outbox directory"); return false;}
  success = ::fsync(directory) == 0;
  if (!success) {error = ioError("sync outbox directory");}
  ::close(directory);
  return success;
}

bool parseSubmission(const std::string & payload, banyu_msgs::msg::Task2Submission & submission)
{
  return banyu_msgs::task2::parseSubmission(payload, submission);
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
  if (buoy.run_id != run_id_ || buoy.epoch != epoch_ || buoy.revision == 0 ||
    !allowed(config_.buoy_sources, buoy.source_id) || buoy.course_id != course_id_ ||
    buoy.endpoint_id.empty() || (!buoy.green_confirmed && !buoy.pinger_confirmed) ||
    (buoy.pinger_confirmed && buoy.pinger_freq_hz != frequency_) ||
    !buoy_stamp_valid(stampNs(buoy.header.stamp)) ||
    !buoy_stamp_valid(stampNs(buoy.fix.header.stamp)) ||
    buoy.fix.status.status < 0 || !std::isfinite(buoy.fix.latitude) ||
    std::abs(buoy.fix.latitude) > 90.0 || !std::isfinite(buoy.fix.longitude) ||
    std::abs(buoy.fix.longitude) > 180.0 || !std::isfinite(buoy.uncertainty_m) ||
    buoy.uncertainty_m < 0.0 || buoy.uncertainty_m > config_.maximum_buoy_uncertainty_m)
  {return fail("ACTIVE_BUOY_UNVERIFIED");}
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
  std::set<uint32_t> identities;
  std::vector<double> current_progress;
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
      if (originals_.at(ring.node_id).pipeline_progress_m != ring.pipeline_progress_m) {
        if (snapshot_failure.empty()) {
          snapshot_failure = "ORDER_UNVERIFIED: original progression changed";
        }
        continue;
      }
      // Qualified node already recorded: freeze first qualified condition.
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
  if (value.tier != config_.tier) {return fail("TIER_CONTEXT_MISMATCH");}
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

banyu_msgs::msg::Task2Submission SurveyLedger::build(const std::string & report_id) const
{
  if (!complete_) {throw std::logic_error(reason_);}
  banyu_msgs::msg::Task2Submission submission;
  submission.schema_version = 1;
  submission.run_id = run_id_;
  submission.epoch = epoch_;
  submission.report_id = report_id;
  submission.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  submission.course_id = course_id_;
  submission.tier = evidence_.tier;
  submission.buoy_latitude = evidence_.active_buoy.fix.latitude;
  submission.buoy_longitude = evidence_.active_buoy.fix.longitude;
  submission.resource_color = submission.tier == 2 ? 0 : evidence_.resource_color;
  submission.delivery_circle_color = submission.tier == 2 ? 0 : evidence_.delivery_circle_color;
  std::vector<banyu_msgs::msg::Task2RingRecord> ordered;
  for (const auto & ring : originals_) {
    ordered.push_back(ring.second);
  }
  const bool ascending = evidence_.active_buoy.endpoint_id == evidence_.low_progress_endpoint_id;
  std::sort(ordered.begin(), ordered.end(), [ascending](const auto & first, const auto & second) {
      return ascending ? first.pipeline_progress_m<second.pipeline_progress_m :
             first.pipeline_progress_m> second.pipeline_progress_m;
    });
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    submission.original_conditions[i] = ordered[i].color == 1 ? 2 : 1;
  }
  if (!banyu_msgs::task2::validSubmission(submission)) {
    throw std::invalid_argument("Invalid report identity or submission");
  }
  return submission;
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

SurveyOutbox::SurveyOutbox(std::string directory)
: directory_(std::move(directory))
{
  if (directory_.empty()) {error_ = "outbox directory is required"; return;}
  std::error_code error;
  std::filesystem::create_directories(directory_, error);
  if (error) {error_ = error.message(); return;}
  owner_fd_ = ::open((directory_ + "/.owner.lock").c_str(),
      O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (owner_fd_ == -1) {error_ = ioError("open outbox owner lock"); return;}
  if (::flock(owner_fd_, LOCK_EX | LOCK_NB) != 0) {
    error_ = ioError("acquire outbox ownership");
    ::close(owner_fd_);
    owner_fd_ = -1;
    return;
  }
  load();
}

SurveyOutbox::~SurveyOutbox()
{
  if (owner_fd_ != -1) {::close(owner_fd_);}
}

std::string SurveyOutbox::canonicalPayload(const banyu_msgs::msg::Task2Submission & submission)
{
  return banyu_msgs::task2::canonicalPayload(submission);
}

std::string SurveyOutbox::fileName(const banyu_msgs::msg::Task2Submission & submission) const
{
  return directory_ + "/run-" + std::to_string(submission.run_id) + "-epoch-" +
         std::to_string(submission.epoch) + "-kind-" + std::to_string(submission.kind) + ".report";
}

bool SurveyOutbox::persist(const Entry & entry, std::string & error) const
{
  std::ostringstream output;
  output << "BANYU_SURVEY_OUTBOX_V1\n" << std::quoted(canonicalPayload(entry.submission)) <<
    '\n' << entry.authorized << ' ' << entry.submission.authorization_revision << '\n';
  return writeAtomic(fileName(entry.submission), output.str(), error);
}

SurveyOutbox::Result SurveyOutbox::stage(const banyu_msgs::msg::Task2Submission & submission)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!error_.empty()) {return {false, false, "OUTBOX_UNAVAILABLE: " + error_, std::nullopt};}
  if (!banyu_msgs::task2::validSubmission(submission)) {
    return {false, false, "INVALID_SUBMISSION", std::nullopt};
  }
  const auto key = std::make_tuple(submission.run_id, submission.epoch, submission.kind);
  const auto existing = entries_.find(key);
  if (existing != entries_.end()) {
    if (canonicalPayload(existing->second.submission) != canonicalPayload(submission)) {
      return {false, false, "REPORT_CONFLICT", existing->second};
    }
    return {true, true, "REPORT_ALREADY_STAGED", existing->second};
  }
  for (const auto & entry : entries_) {
    if (entry.second.submission.report_id == submission.report_id) {
      return {false, false, "REPORT_ID_REUSED", std::nullopt};
    }
  }
  Entry entry{submission, false};
  entry.submission.authorization_revision = 0;
  if (!persist(entry, error_)) {
    return {false, false, "OUTBOX_UNAVAILABLE: " + error_, std::nullopt};
  }
  entries_.emplace(key, entry);
  return {true, false, "REPORT_STAGED_NOT_AUTHORIZED", entry};
}

SurveyOutbox::Result SurveyOutbox::authorize(
  const std::string & report_id, uint64_t authorization_revision)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!error_.empty()) {return {false, false, "OUTBOX_UNAVAILABLE: " + error_, std::nullopt};}
  if (authorization_revision == 0) {
    return {false, false, "INVALID_AUTHORIZATION_REVISION", std::nullopt};
  }
  for (auto & value : entries_) {
    if (value.second.submission.report_id != report_id) {continue;}
    if (value.second.authorized) {return {true, true, "REPORT_ALREADY_AUTHORIZED", value.second};}
    auto entry = value.second;
    entry.authorized = true;
    entry.submission.authorization_revision = authorization_revision;
    if (!persist(entry, error_)) {
      return {false, false, "OUTBOX_UNAVAILABLE: " + error_, std::nullopt};
    }
    value.second = entry;
    return {true, false, "REPORT_AUTHORIZED_LOCAL_ACCEPTANCE", entry};
  }
  return {false, false, "REPORT_NOT_STAGED", std::nullopt};
}

SurveyOutbox::Result SurveyOutbox::unstage(const std::string & report_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!error_.empty()) {return {false, false, "OUTBOX_UNAVAILABLE: " + error_, std::nullopt};}
  for (auto it = entries_.begin(); it != entries_.end(); ++it) {
    if (it->second.submission.report_id != report_id) {continue;}
    if (it->second.authorized) {
      return {false, false, "CANNOT_UNSTAGE_AUTHORIZED_REPORT", it->second};
    }
    const auto path = fileName(it->second.submission);
    ::unlink(path.c_str());
    const auto removed = it->second;
    entries_.erase(it);
    return {true, false, "REPORT_UNSTAGED", removed};
  }
  return {false, false, "REPORT_NOT_STAGED", std::nullopt};
}

std::optional<SurveyOutbox::Entry> SurveyOutbox::find(
  uint32_t run_id, uint64_t epoch, uint8_t kind) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = entries_.find(std::make_tuple(run_id, epoch, kind));
  return entry == entries_.end() ? std::nullopt : std::optional<Entry>(entry->second);
}

std::vector<SurveyOutbox::Entry> SurveyOutbox::entries() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Entry> result;
  for (const auto & entry : entries_) {
    result.push_back(entry.second);
  }
  return result;
}

void SurveyOutbox::load()
{
  std::error_code error;
  const auto finish_error = [this](const std::string & message) {
      entries_.clear();
      error_ = message;
    };
  std::set<std::string> report_ids;
  for (const auto & file : std::filesystem::directory_iterator(directory_, error)) {
    if (file.path().extension() != ".report") {continue;}
    if (file.is_symlink(error) || !file.is_regular_file(error) || error ||
      file.file_size(error) > 100000 || error)
    {finish_error("invalid outbox entry file"); return;}
    std::ifstream input(file.path());
    std::string format;
    std::string payload;
    unsigned authorized = 0;
    Entry entry;
    if (!(input >> format >> std::quoted(payload) >> authorized >>
      entry.submission.authorization_revision) || format != "BANYU_SURVEY_OUTBOX_V1" ||
      authorized > 1 || !parseSubmission(payload, entry.submission) ||
      ((authorized == 1) != (entry.submission.authorization_revision != 0)))
    {finish_error("invalid outbox entry contents"); return;}
    input >> std::ws;
    entry.authorized = authorized == 1;
    const auto key = std::make_tuple(
      entry.submission.run_id, entry.submission.epoch, entry.submission.kind);
    if (!input.eof() || std::filesystem::path(fileName(entry.submission)) != file.path() ||
      !report_ids.insert(entry.submission.report_id).second || !entries_.emplace(key, entry).second)
    {finish_error("inconsistent outbox entry identity"); return;}
  }
  if (error) {finish_error("read outbox directory: " + error.message());}
}

}  // namespace banyu_communication
