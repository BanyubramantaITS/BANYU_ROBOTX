#ifndef BANYU_DETECTION__SURVEY_EVIDENCE_NODE_HPP_
#define BANYU_DETECTION__SURVEY_EVIDENCE_NODE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "banyu_msgs/msg/active_buoy_fix.hpp"
#include "banyu_msgs/msg/pipeline_end_state.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_ring_record.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/course.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace banyu_detection
{

class SurveyEvidenceNode : public rclcpp::Node
{
public:
  explicit SurveyEvidenceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SurveyEvidenceNode() = default;

  uint32_t runId() const;
  uint64_t epoch() const;
  std::size_t ringCount() const;
  bool isBuoyReceived() const;
  bool isTraversalComplete() const;

private:
  void handleAcceptedRing(const banyu_msgs::msg::Task2RingRecord::SharedPtr msg);
  void handleEndState(const banyu_msgs::msg::PipelineEndState::SharedPtr msg);
  void handleStartBuoyFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void handleActiveBuoyFix(const banyu_msgs::msg::ActiveBuoyFix::SharedPtr msg);
  void handleCourse(const rx_msgs::msg::Course::SharedPtr msg);
  void handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg);
  void publishEvidence();

  mutable std::mutex mutex_;
  std::string source_id_;
  std::string low_progress_endpoint_id_;
  std::string high_progress_endpoint_id_;
  std::string buoy_source_id_;
  std::string course_id_;
  uint32_t pinger_freq_hz_{25000};
  uint8_t tier_{2};

  uint32_t run_id_{0};
  uint64_t epoch_{0};
  uint64_t evidence_revision_{0};
  uint64_t buoy_revision_{0};
  bool buoy_received_{false};
  bool qualified_buoy_received_{false};
  bool traversal_complete_{false};
  bool repair_verified_{false};
  bool resource_code_complete_{false};
  uint8_t resource_color_{0};
  uint8_t delivery_circle_color_{0};
  bool start_buoy_trust_raw_fix_{false};

  banyu_msgs::msg::ActiveBuoyFix active_buoy_;
  std::map<uint32_t, banyu_msgs::msg::Task2RingRecord> rings_;

  rclcpp::Subscription<banyu_msgs::msg::Task2RingRecord>::SharedPtr accepted_ring_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineEndState>::SharedPtr end_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr start_buoy_fix_sub_;
  rclcpp::Subscription<banyu_msgs::msg::ActiveBuoyFix>::SharedPtr active_buoy_fix_sub_;
  rclcpp::Subscription<rx_msgs::msg::Course>::SharedPtr course_sub_;
  rclcpp::Subscription<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_sub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2SurveyEvidence>::SharedPtr evidence_pub_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION__SURVEY_EVIDENCE_NODE_HPP_
