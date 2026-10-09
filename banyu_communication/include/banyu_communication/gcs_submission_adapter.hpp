#ifndef BANYU_COMMUNICATION__GCS_SUBMISSION_ADAPTER_HPP_
#define BANYU_COMMUNICATION__GCS_SUBMISSION_ADAPTER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "banyu_msgs/msg/task2_submission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"

namespace banyu_communication
{

class GcsSubmissionAdapter : public rclcpp::Node
{
public:
  explicit GcsSubmissionAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GcsSubmissionAdapter() = default;

  std::size_t acceptedCount() const;
  std::size_t duplicateCount() const;
  std::size_t rejectedCount() const;
  bool hasReport(const std::string & report_id) const;

private:
  void handleSubmission(const banyu_msgs::msg::Task2Submission::SharedPtr msg);
  void handleCommand(const rx_msgs::msg::Command::SharedPtr msg);

  mutable std::mutex mutex_;
  bool forward_to_rx_topics_{true};
  std::set<std::string> accepted_report_ids_;
  std::map<std::string, std::string> accepted_payloads_;
  std::size_t accepted_count_{0};
  std::size_t duplicate_count_{0};
  std::size_t rejected_count_{0};

  rclcpp::Subscription<banyu_msgs::msg::Task2Submission>::SharedPtr submission_sub_;
  rclcpp::Subscription<rx_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Publisher<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_pub_;
  rclcpp::Publisher<rx_msgs::msg::ResourceDelivery>::SharedPtr delivery_pub_;
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION__GCS_SUBMISSION_ADAPTER_HPP_
