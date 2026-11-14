#ifndef BANYU_COMMUNICATION__SURVEY_REPORTING_HPP_
#define BANYU_COMMUNICATION__SURVEY_REPORTING_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "banyu_msgs/msg/task2_ring_record.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"
#include "rx_msgs/msg/rx_task.hpp"

namespace banyu_communication
{

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
    // Best-effort report with no RED seen: put one RED at a random one of these positions (1 = the node closest to the active
    // buoy, 4 = the farthest; one position = always that one). The handbook lights one node RED.
    bool gamble_order{false};
    std::vector<int> gamble_red_positions{2, 3};
  };

  struct Report
  {
    uint32_t run_id{0};
    uint64_t epoch{0};
    std::string report_id;
    std::string course_id;
    uint8_t tier{2};
    double buoy_latitude{0.0};
    double buoy_longitude{0.0};
    std::array<uint8_t, 4> original_conditions{};
    uint8_t resource_color{0};
    uint8_t delivery_circle_color{0};

    rx_msgs::msg::PipelineSurvey toPipelineSurvey() const;
    rx_msgs::msg::ResourceDelivery toResourceDelivery() const;
  };

  SurveyLedger();
  explicit SurveyLedger(Config config);
  void beginRun(uint32_t run_id, uint64_t epoch, int64_t source_start_ns);
  void setCourse(const std::string & course_id, uint32_t pinger_freq_hz);
  std::string ingest(const banyu_msgs::msg::Task2SurveyEvidence & evidence, int64_t source_now_ns);
  bool complete() const {return complete_;}
  std::string reason() const {return reason_;}
  Report build(const std::string & report_id) const;
  // A Core survey from what was seen when the survey could not be completed (a pipe not found, a lost traverse): the
  // verified active buoy, the rings seen in progress order and GREEN for the rest. Keeps the heartbeat and start-buoy
  // points instead of sending nothing. Throws when no active buoy was ever verified.
  Report buildBestEffort(const std::string & report_id) const;
  bool bestEffortAvailable() const;

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
  uint32_t hold_sample_count_{0};
  bool near_surface_{false};
  bool sample_received_{false};
  std::string calibration_id_;
  std::string source_id_;
  std::string reason_{"SURFACE_TELEMETRY_MISSING"};
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION__SURVEY_REPORTING_HPP_
