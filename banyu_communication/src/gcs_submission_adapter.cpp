#include "banyu_communication/gcs_submission_adapter.hpp"

#include "banyu_msgs/task2_submission.hpp"
#include "rx_msgs/msg/rx_task.hpp"

namespace banyu_communication
{

GcsSubmissionAdapter::GcsSubmissionAdapter(const rclcpp::NodeOptions & options)
: Node("gcs_submission_adapter", options)
{
  forward_to_rx_topics_ = declare_parameter<bool>("forward_to_rx_topics", true);
  const auto submission_topic = declare_parameter<std::string>(
    "topics.task2_submission", "/system/vehicle/uuv/task2/submission");
  const auto command_topic = declare_parameter<std::string>(
    "topics.command", "/system/mission/command");
  const auto survey_topic = declare_parameter<std::string>(
    "topics.pipeline_survey", "/system/vehicle/uuv/task2/pipeline_survey");
  const auto delivery_topic = declare_parameter<std::string>(
    "topics.delivery", "/system/vehicle/uuv/task2/delivery");

  submission_sub_ = create_subscription<banyu_msgs::msg::Task2Submission>(
    submission_topic, rclcpp::QoS(32).reliable().transient_local(),
    std::bind(&GcsSubmissionAdapter::handleSubmission, this, std::placeholders::_1));

  command_sub_ = create_subscription<rx_msgs::msg::Command>(
    command_topic, rclcpp::QoS(10).reliable(),
    std::bind(&GcsSubmissionAdapter::handleCommand, this, std::placeholders::_1));

  survey_pub_ = create_publisher<rx_msgs::msg::PipelineSurvey>(
    survey_topic, rclcpp::QoS(10).reliable().transient_local());

  delivery_pub_ = create_publisher<rx_msgs::msg::ResourceDelivery>(
    delivery_topic, rclcpp::QoS(10).reliable().transient_local());

  RCLCPP_INFO(
    get_logger(),
    "GcsSubmissionAdapter initialized: forward_to_rx=%d, submission_topic='%s'",
    forward_to_rx_topics_, submission_topic.c_str());
}

std::size_t GcsSubmissionAdapter::acceptedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return accepted_count_;
}

std::size_t GcsSubmissionAdapter::duplicateCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return duplicate_count_;
}

std::size_t GcsSubmissionAdapter::rejectedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rejected_count_;
}

bool GcsSubmissionAdapter::hasReport(const std::string & report_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return accepted_report_ids_.find(report_id) != accepted_report_ids_.end();
}

void GcsSubmissionAdapter::handleCommand(const rx_msgs::msg::Command::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  if (msg->command_type == rx_msgs::msg::Command::CMD_RUN_START) {
    RCLCPP_INFO(
      get_logger(),
      "Run start detected (seq=%u); retaining historical accepted report count=%zu",
      msg->command_seq, accepted_report_ids_.size());
  }
}

void GcsSubmissionAdapter::handleSubmission(const banyu_msgs::msg::Task2Submission::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }

  if (msg->run_id == 0 || msg->epoch == 0 || msg->report_id.empty() ||
    !banyu_msgs::task2::validSubmission(*msg))
  {
    ++rejected_count_;
    RCLCPP_WARN(
      get_logger(),
      "Rejected invalid Task2Submission (run_id=%u, epoch=%lu, report_id='%s')",
      msg->run_id, msg->epoch, msg->report_id.c_str());
    return;
  }

  if (accepted_report_ids_.find(msg->report_id) != accepted_report_ids_.end()) {
    ++duplicate_count_;
    RCLCPP_INFO(
      get_logger(),
      "Deduplicated retransmitted Task2Submission (report_id='%s')",
      msg->report_id.c_str());
    return;
  }

  accepted_report_ids_.insert(msg->report_id);
  ++accepted_count_;

  RCLCPP_INFO(
    get_logger(),
    "Accepted Task2Submission: report_id='%s', run_id=%u, epoch=%lu, kind=%u, tier=%u",
    msg->report_id.c_str(), msg->run_id, msg->epoch, msg->kind, msg->tier);

  if (!forward_to_rx_topics_) {
    return;
  }

  if (msg->kind == banyu_msgs::msg::Task2Submission::SURVEY) {
    rx_msgs::msg::PipelineSurvey survey;
    survey.header = msg->header;
    survey.active_buoy_position.latitude = msg->buoy_latitude;
    survey.active_buoy_position.longitude = msg->buoy_longitude;
    survey.segments.assign(
      msg->original_conditions.begin(),
      msg->original_conditions.end());
    survey_pub_->publish(survey);
  } else if (msg->kind == banyu_msgs::msg::Task2Submission::DELIVERY) {
    rx_msgs::msg::ResourceDelivery delivery;
    delivery.header = msg->header;
    delivery.task = rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR;
    delivery.resource_color = msg->resource_color;
    delivery.delivery_circle_color = msg->delivery_circle_color;
    delivery_pub_->publish(delivery);
  }
}

}  // namespace banyu_communication

#ifndef BANYU_COMMUNICATION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_communication::GcsSubmissionAdapter>());
  rclcpp::shutdown();
  return 0;
}
#endif
