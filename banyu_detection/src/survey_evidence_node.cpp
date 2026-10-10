#include "banyu_detection/survey_evidence_node.hpp"

#include <cmath>

namespace banyu_detection
{

SurveyEvidenceNode::SurveyEvidenceNode(const rclcpp::NodeOptions & options)
: Node("survey_evidence_node", options)
{
  source_id_ = declare_parameter<std::string>("source_id", "pipeline_topology");
  low_progress_endpoint_id_ = declare_parameter<std::string>(
    "low_progress_endpoint_id", "start_buoy");
  high_progress_endpoint_id_ = declare_parameter<std::string>(
    "high_progress_endpoint_id", "end_buoy");
  buoy_source_id_ = declare_parameter<std::string>(
    "buoy_source_id", "uuv_buoy_localizer");
  course_id_ = declare_parameter<std::string>("default_course_id", "course-a");
  pinger_freq_hz_ = static_cast<uint32_t>(
    declare_parameter<int64_t>("default_pinger_freq_hz", 25000));
  tier_ = static_cast<uint8_t>(declare_parameter<int64_t>("tier", 2));
  start_buoy_trust_raw_fix_ = declare_parameter<bool>("start_buoy_trust_raw_fix", false);

  const auto accepted_ring_topic = declare_parameter<std::string>(
    "topics.accepted_ring", "/banyu/pipeline/accepted_ring");
  const auto end_state_topic = declare_parameter<std::string>(
    "topics.end_state", "/banyu/pipeline/end_state");
  const auto start_buoy_fix_topic = declare_parameter<std::string>(
    "topics.start_buoy_fix", "/banyu/task2/start_buoy_fix");
  const auto active_buoy_fix_topic = declare_parameter<std::string>(
    "topics.active_buoy_fix", "/banyu/task2/active_buoy_fix");
  const auto course_topic = declare_parameter<std::string>(
    "topics.course", "/system/mission/course");
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");
  const auto evidence_topic = declare_parameter<std::string>(
    "topics.survey_evidence", "/banyu/pipeline/survey_evidence");

  evidence_pub_ = create_publisher<banyu_msgs::msg::Task2SurveyEvidence>(
    evidence_topic, rclcpp::QoS(1).reliable().transient_local());

  accepted_ring_sub_ = create_subscription<banyu_msgs::msg::Task2RingRecord>(
    accepted_ring_topic, rclcpp::QoS(20).reliable(),
    std::bind(&SurveyEvidenceNode::handleAcceptedRing, this, std::placeholders::_1));

  end_state_sub_ = create_subscription<banyu_msgs::msg::PipelineEndState>(
    end_state_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SurveyEvidenceNode::handleEndState, this, std::placeholders::_1));

  start_buoy_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    start_buoy_fix_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SurveyEvidenceNode::handleStartBuoyFix, this, std::placeholders::_1));

  active_buoy_fix_sub_ = create_subscription<banyu_msgs::msg::ActiveBuoyFix>(
    active_buoy_fix_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SurveyEvidenceNode::handleActiveBuoyFix, this, std::placeholders::_1));

  course_sub_ = create_subscription<rx_msgs::msg::Course>(
    course_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SurveyEvidenceNode::handleCourse, this, std::placeholders::_1));

  lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SurveyEvidenceNode::handleLifecycle, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "SurveyEvidenceNode initialized: source='%s', tier=%u, endpoints=['%s', '%s']",
    source_id_.c_str(), tier_, low_progress_endpoint_id_.c_str(),
    high_progress_endpoint_id_.c_str());
}

uint32_t SurveyEvidenceNode::runId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return run_id_;
}

uint64_t SurveyEvidenceNode::epoch() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return epoch_;
}

std::size_t SurveyEvidenceNode::ringCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rings_.size();
}

bool SurveyEvidenceNode::isBuoyReceived() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buoy_received_;
}

bool SurveyEvidenceNode::isTraversalComplete() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return traversal_complete_;
}

void SurveyEvidenceNode::handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  const bool new_run = (msg->run_id != run_id_ || msg->epoch != epoch_);
  run_id_ = msg->run_id;
  epoch_ = msg->epoch;

  if (new_run) {
    rings_.clear();
    buoy_received_ = false;
    traversal_complete_ = false;
    repair_verified_ = false;
    resource_code_complete_ = false;
    evidence_revision_ = 0;
    buoy_revision_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "Run context updated to run %u (epoch %lu); cleared previous evidence",
      run_id_, epoch_);
  }
}

void SurveyEvidenceNode::handleCourse(const rx_msgs::msg::Course::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  if (!rings_.empty() && !course_id_.empty() && course_id_ != msg->course_id) {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting conflicting mid-run course update: existing='%s', received='%s'",
      course_id_.c_str(), msg->course_id.c_str());
    return;
  }
  course_id_ = msg->course_id;
  pinger_freq_hz_ = msg->pinger_freq_hz;
  if (buoy_received_) {
    active_buoy_.course_id = course_id_;
    active_buoy_.pinger_freq_hz = pinger_freq_hz_;
    publishEvidence();
  }
}

void SurveyEvidenceNode::handleStartBuoyFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg || msg->status.status < 0 || !std::isfinite(msg->latitude) ||
    !std::isfinite(msg->longitude))
  {
    return;
  }

  if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
    active_buoy_.header.stamp = msg->header.stamp;
  } else {
    active_buoy_.header.stamp = now();
  }
  active_buoy_.header.frame_id = "map";
  active_buoy_.run_id = run_id_;
  active_buoy_.epoch = epoch_;
  active_buoy_.revision = ++buoy_revision_;
  active_buoy_.source_id = buoy_source_id_;
  active_buoy_.course_id = course_id_;
  active_buoy_.endpoint_id = low_progress_endpoint_id_;
  active_buoy_.fix = *msg;
  active_buoy_.uncertainty_m = 1.0;
  active_buoy_.green_confirmed = start_buoy_trust_raw_fix_;
  active_buoy_.pinger_confirmed = false;
  active_buoy_.pinger_freq_hz = pinger_freq_hz_;
  buoy_received_ = true;

  publishEvidence();
}

void SurveyEvidenceNode::handleActiveBuoyFix(const banyu_msgs::msg::ActiveBuoyFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  if (run_id_ != 0 && (msg->run_id != run_id_ || msg->epoch != epoch_)) {
    return;
  }

  if (!rings_.empty() && buoy_received_) {
    const std::string incoming_endpoint = msg->endpoint_id.empty() ?
      low_progress_endpoint_id_ : msg->endpoint_id;
    if (!active_buoy_.endpoint_id.empty() && active_buoy_.endpoint_id != incoming_endpoint) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting conflicting mid-run endpoint update: pinned='%s', received='%s'",
        active_buoy_.endpoint_id.c_str(), incoming_endpoint.c_str());
      return;
    }
  }

  active_buoy_ = *msg;
  if (active_buoy_.endpoint_id.empty()) {
    active_buoy_.endpoint_id = low_progress_endpoint_id_;
  }
  if (active_buoy_.course_id.empty() && !course_id_.empty()) {
    active_buoy_.course_id = course_id_;
  }
  if (active_buoy_.pinger_freq_hz == 0 && pinger_freq_hz_ != 0) {
    active_buoy_.pinger_freq_hz = pinger_freq_hz_;
  }
  buoy_received_ = true;
  publishEvidence();
}

void SurveyEvidenceNode::handleAcceptedRing(const banyu_msgs::msg::Task2RingRecord::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  if (run_id_ != 0 && (msg->run_id != run_id_ || msg->epoch != epoch_)) {
    return;
  }

  rings_[msg->node_id] = *msg;
  publishEvidence();
}

void SurveyEvidenceNode::handleEndState(const banyu_msgs::msg::PipelineEndState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  if (run_id_ != 0 && (msg->run_id != run_id_ || msg->epoch != epoch_)) {
    return;
  }

  traversal_complete_ = msg->ended;
  publishEvidence();
}

void SurveyEvidenceNode::publishEvidence()
{
  if (!buoy_received_) {
    return;
  }

  banyu_msgs::msg::Task2SurveyEvidence evidence;
  evidence.header.stamp = now();
  evidence.header.frame_id = "map";
  evidence.run_id = run_id_;
  evidence.epoch = epoch_;
  evidence.revision = ++evidence_revision_;
  evidence.source_id = source_id_;
  evidence.active_buoy = active_buoy_;
  evidence.low_progress_endpoint_id = low_progress_endpoint_id_;
  evidence.high_progress_endpoint_id = high_progress_endpoint_id_;

  for (const auto & item : rings_) {
    evidence.rings.push_back(item.second);
  }

  evidence.traversal_complete = traversal_complete_;
  evidence.tier = tier_;
  evidence.repair_verified = repair_verified_;
  evidence.resource_code_complete = resource_code_complete_;
  evidence.resource_color = resource_color_;
  evidence.delivery_circle_color = delivery_circle_color_;

  evidence_pub_->publish(evidence);
}

}  // namespace banyu_detection

#ifndef BANYU_DETECTION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_detection::SurveyEvidenceNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
