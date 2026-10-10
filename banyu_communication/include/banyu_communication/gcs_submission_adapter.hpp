#ifndef BANYU_COMMUNICATION__GCS_SUBMISSION_ADAPTER_HPP_
#define BANYU_COMMUNICATION__GCS_SUBMISSION_ADAPTER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "banyu_msgs/msg/task2_submission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"

namespace banyu_communication
{

/**
 * @brief UUV-side report publisher that converts internal Task2Submissions
 *        into standard rx_msgs public reports (PipelineSurvey and ResourceDelivery)
 *        with durable acceptance and publication intent recovery.
 */
class GcsSubmissionAdapter : public rclcpp::Node
{
public:
  enum class PublicationState : uint8_t
  {
    PENDING = 0,
    ATTEMPTED = 1,
    SUPPRESSED_NEW_RUN = 2
  };

  explicit GcsSubmissionAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GcsSubmissionAdapter() = default;

  std::size_t acceptedCount() const;
  std::size_t duplicateCount() const;
  std::size_t rejectedCount() const;
  std::size_t pendingCount() const;
  std::size_t attemptedCount() const;
  bool hasReport(const std::string & report_id) const;
  PublicationState publicationState(const std::string & report_id) const;

private:
  void handleSubmission(const banyu_msgs::msg::Task2Submission::SharedPtr msg);
  void handleCommand(const rx_msgs::msg::Command::SharedPtr msg);
  void publishPublicReport(const banyu_msgs::msg::Task2Submission & msg);

  struct AcceptedReport
  {
    uint32_t run_id{0};
    uint64_t epoch{0};
    uint8_t kind{0};
    std::string report_id;
    std::string payload;
    uint64_t authorization_revision{0};
    PublicationState pub_state{PublicationState::PENDING};
  };

  using LogicalKey = std::tuple<uint32_t, uint64_t, uint8_t>;
  void loadJournal();
  bool appendAcceptance(const AcceptedReport & report, std::string & error);
  bool appendPubAttempted(const AcceptedReport & report, std::string & error);
  bool appendJournalLine(const std::string & line, std::string & error);

  mutable std::mutex mutex_;
  std::string journal_path_;
  bool forward_to_rx_topics_{true};
  uint32_t active_run_id_{0};
  std::map<LogicalKey, AcceptedReport> accepted_by_key_;
  std::map<std::string, LogicalKey> report_id_to_key_;
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
