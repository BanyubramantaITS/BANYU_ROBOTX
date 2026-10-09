#ifndef BANYU_COMMUNICATION__SURVEY_REPORTING_HPP_
#define BANYU_COMMUNICATION__SURVEY_REPORTING_HPP_

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "banyu_msgs/msg/task2_submission.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "banyu_msgs/msg/water_depth.hpp"

namespace banyu_communication
{

// Ledger and guard calls are serialized by their owning node; clocks are injected.
class SurveyLedger
{
public:
  struct Config
  {
    std::vector<std::string> evidence_sources;
    std::vector<std::string> buoy_sources;
    double minimum_confidence{0.2};
    double maximum_buoy_uncertainty_m{5.0};
    double ingress_max_age_s{2.0};
    double minimum_progress_separation_m{0.0};
    uint8_t tier{2};
  };

  SurveyLedger();
  explicit SurveyLedger(Config config);
  void beginRun(uint32_t run_id, uint64_t epoch, int64_t source_start_ns);
  void setCourse(const std::string & course_id, uint32_t pinger_freq_hz);
  std::string ingest(const banyu_msgs::msg::Task2SurveyEvidence & evidence, int64_t source_now_ns);
  bool complete() const {return complete_;}
  std::string reason() const {return reason_;}
  banyu_msgs::msg::Task2Submission build(const std::string & report_id) const;

private:
  Config config_;
  uint32_t run_id_{0};
  uint64_t epoch_{0};
  int64_t source_start_ns_{0};
  uint64_t revision_{0};
  std::string course_id_;
  uint32_t frequency_{0};
  std::map<uint32_t, banyu_msgs::msg::Task2RingRecord> originals_;
  banyu_msgs::msg::Task2SurveyEvidence evidence_;
  bool buoy_pinned_{false};
  bool course_changed_{false};
  bool complete_{false};
  std::string reason_{"SURVEY_INCOMPLETE"};
};

class SurfaceGuard
{
public:
  struct Config
  {
    double hold_s{1.0};
    double surface_depth_m{0.0};
    double tolerance_m{0.2};
    double ttl_s{0.5};
    std::vector<std::string> sources;
  };

  SurfaceGuard();
  explicit SurfaceGuard(Config config);
  void beginRun(uint32_t run_id, uint64_t epoch);
  void observe(
    const banyu_msgs::msg::WaterDepth & depth, int64_t source_now_ns, int64_t steady_now_ns);
  bool ready(int64_t source_now_ns, int64_t steady_now_ns);
  std::string reason() const {return reason_;}

private:
  void invalidate(const std::string & reason);
  Config config_;
  uint32_t run_id_{0};
  uint64_t epoch_{0};
  int64_t last_source_ns_{0};
  int64_t last_received_ns_{0};
  int64_t hold_started_ns_{0};
  bool near_surface_{false};
  bool sample_received_{false};
  std::string calibration_id_;
  std::string source_id_;
  std::string reason_{"SURFACE_TELEMETRY_MISSING"};
};

class SurveyOutbox
{
public:
  struct Entry
  {
    banyu_msgs::msg::Task2Submission submission;
    bool authorized{false};
  };

  struct Result
  {
    bool success{false};
    bool duplicate{false};
    std::string message;
    std::optional<Entry> entry;
  };

  explicit SurveyOutbox(std::string directory);
  ~SurveyOutbox();
  Result stage(const banyu_msgs::msg::Task2Submission & submission);
  Result authorize(const std::string & report_id, uint64_t authorization_revision);
  std::optional<Entry> find(uint32_t run_id, uint64_t epoch, uint8_t kind) const;
  std::vector<Entry> entries() const;
  static std::string canonicalPayload(const banyu_msgs::msg::Task2Submission & submission);

private:
  using Key = std::tuple<uint32_t, uint64_t, uint8_t>;
  bool persist(const Entry & entry, std::string & error) const;
  void load();
  std::string fileName(const banyu_msgs::msg::Task2Submission & submission) const;
  std::string directory_;
  std::string error_;
  int owner_fd_{-1};
  mutable std::mutex mutex_;
  std::map<Key, Entry> entries_;
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION__SURVEY_REPORTING_HPP_
