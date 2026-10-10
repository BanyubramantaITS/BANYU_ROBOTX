#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <vector>

#include "banyu_communication/gcs_submission_adapter.hpp"
#include "banyu_msgs/msg/task2_submission.hpp"
#include "banyu_msgs/task2_submission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"

using namespace std::chrono_literals;

class GcsSubmissionAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    std::error_code ec;
    std::filesystem::remove("/tmp/banyu_gcs_adapter_dedup.journal", ec);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    std::error_code ec;
    std::filesystem::remove("/tmp/banyu_gcs_adapter_dedup.journal", ec);
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    test_journal_path_ = (std::filesystem::temp_directory_path() /
      ("banyu_gcs_adapter_test_" + std::to_string(::getpid()) + ".journal")).string();
    std::error_code ec;
    std::filesystem::remove(test_journal_path_, ec);
    std::filesystem::remove("/tmp/banyu_gcs_adapter_dedup.journal", ec);

    rclcpp::NodeOptions options;
    options.append_parameter_override("journal_path", test_journal_path_);
    options.append_parameter_override(
      "topics.task2_submission", "/test_adapter/submission");
    options.append_parameter_override(
      "topics.pipeline_survey", "/test_adapter/pipeline_survey");
    options.append_parameter_override(
      "topics.delivery", "/test_adapter/delivery");
    options.append_parameter_override("forward_to_rx_topics", true);

    adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
    test_node_ = std::make_shared<rclcpp::Node>("gcs_adapter_test_peer");

    submission_pub_ = test_node_->create_publisher<banyu_msgs::msg::Task2Submission>(
      "/test_adapter/submission", rclcpp::QoS(10).reliable().transient_local());

    survey_sub_ = test_node_->create_subscription<rx_msgs::msg::PipelineSurvey>(
      "/test_adapter/pipeline_survey", rclcpp::QoS(10).reliable(),
      [this](const rx_msgs::msg::PipelineSurvey::SharedPtr msg) {
        received_surveys_.push_back(*msg);
      });

    delivery_sub_ = test_node_->create_subscription<rx_msgs::msg::ResourceDelivery>(
      "/test_adapter/delivery", rclcpp::QoS(10).reliable(),
      [this](const rx_msgs::msg::ResourceDelivery::SharedPtr msg) {
        received_deliveries_.push_back(*msg);
      });

    executor_.add_node(adapter_node_);
    executor_.add_node(test_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(adapter_node_);
    executor_.remove_node(test_node_);
    std::error_code ec;
    std::filesystem::remove(test_journal_path_, ec);
    std::filesystem::remove("/tmp/banyu_gcs_adapter_dedup.journal", ec);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<banyu_communication::GcsSubmissionAdapter> adapter_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<banyu_msgs::msg::Task2Submission>::SharedPtr submission_pub_;
  rclcpp::Subscription<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_sub_;
  rclcpp::Subscription<rx_msgs::msg::ResourceDelivery>::SharedPtr delivery_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<rx_msgs::msg::PipelineSurvey> received_surveys_;
  std::vector<rx_msgs::msg::ResourceDelivery> received_deliveries_;
  std::string test_journal_path_;
};

TEST_F(GcsSubmissionAdapterTest, IngestsAndForwardPipelineSurveyAndDeduplicates)
{
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "test-report-001";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};  // 4 light node conditions
  sub.authorization_revision = 1;

  submission_pub_->publish(sub);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_TRUE(adapter_node_->hasReport("test-report-001"));
  ASSERT_EQ(received_surveys_.size(), 1u);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.latitude, 33.810);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.longitude, -118.393);
  ASSERT_EQ(received_surveys_[0].segments.size(), 4u);
  EXPECT_EQ(received_surveys_[0].segments[0], 1u);
  EXPECT_EQ(received_surveys_[0].segments[1], 2u);

  // Duplicate submission with same report_id
  submission_pub_->publish(sub);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_EQ(adapter_node_->duplicateCount(), 1u);
  EXPECT_EQ(received_surveys_.size(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, IngestsAndForwardResourceDelivery)
{
  banyu_msgs::msg::Task2Submission del;
  del.header.stamp = test_node_->now();
  del.schema_version = 1;
  del.run_id = 15;
  del.epoch = 2;
  del.report_id = "test-delivery-001";
  del.kind = banyu_msgs::msg::Task2Submission::DELIVERY;
  del.course_id = "course-1";
  del.tier = 3;
  del.buoy_latitude = 33.810;
  del.buoy_longitude = -118.393;
  del.original_conditions = {1, 1, 1, 1};
  del.resource_color = 4;
  del.delivery_circle_color = 2;
  del.authorization_revision = 2;

  submission_pub_->publish(del);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_TRUE(adapter_node_->hasReport("test-delivery-001"));
  ASSERT_EQ(received_deliveries_.size(), 1u);
  EXPECT_EQ(received_deliveries_[0].resource_color, 4u);
  EXPECT_EQ(received_deliveries_[0].delivery_circle_color, 2u);
}

TEST_F(GcsSubmissionAdapterTest, RejectsInvalidSubmission)
{
  banyu_msgs::msg::Task2Submission invalid;
  invalid.run_id = 0;  // Invalid run_id
  invalid.epoch = 0;
  invalid.report_id = "invalid-report";

  submission_pub_->publish(invalid);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);
  EXPECT_EQ(adapter_node_->acceptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());
}

TEST_F(GcsSubmissionAdapterTest, UnsealedSubmissionMustNotBeForwarded)
{
  spinFor(100ms);
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "unsealed-audit";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.authorization_revision = 0;

  submission_pub_->publish(sub);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());
}

TEST_F(GcsSubmissionAdapterTest, RestartMustNotForwardRetainedReportTwice)
{
  spinFor(100ms);
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "restart-audit";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.authorization_revision = 1;

  submission_pub_->publish(sub);
  spinFor(100ms);
  ASSERT_EQ(received_surveys_.size(), 1u);

  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", test_journal_path_);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);
  spinFor(200ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, ConflictingPayloadForSameRunEpochKindIsRejected)
{
  spinFor(100ms);
  banyu_msgs::msg::Task2Submission sub1;
  sub1.header.stamp = test_node_->now();
  sub1.schema_version = 1;
  sub1.run_id = 20;
  sub1.epoch = 1;
  sub1.report_id = "report-original";
  sub1.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub1.course_id = "course-1";
  sub1.tier = 2;
  sub1.buoy_latitude = 33.810;
  sub1.buoy_longitude = -118.393;
  sub1.original_conditions = {1, 2, 1, 1};
  sub1.authorization_revision = 1;

  submission_pub_->publish(sub1);
  spinFor(100ms);
  ASSERT_EQ(adapter_node_->acceptedCount(), 1u);
  ASSERT_EQ(received_surveys_.size(), 1u);

  banyu_msgs::msg::Task2Submission sub2 = sub1;
  sub2.report_id = "report-conflict";
  sub2.original_conditions = {2, 2, 2, 2};
  submission_pub_->publish(sub2);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);
  EXPECT_EQ(received_surveys_.size(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, ReusedReportIdForDifferentRunIsRejected)
{
  spinFor(100ms);
  banyu_msgs::msg::Task2Submission sub1;
  sub1.header.stamp = test_node_->now();
  sub1.schema_version = 1;
  sub1.run_id = 30;
  sub1.epoch = 1;
  sub1.report_id = "reused-id";
  sub1.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub1.course_id = "course-1";
  sub1.tier = 2;
  sub1.buoy_latitude = 33.810;
  sub1.buoy_longitude = -118.393;
  sub1.original_conditions = {1, 2, 1, 1};
  sub1.authorization_revision = 1;

  submission_pub_->publish(sub1);
  spinFor(100ms);
  ASSERT_EQ(adapter_node_->acceptedCount(), 1u);

  banyu_msgs::msg::Task2Submission sub2 = sub1;
  sub2.run_id = 31;
  submission_pub_->publish(sub2);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, LocalAuditRecoversJournaledButUnpublishedReport)
{
  spinFor(100ms);
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "recovery-probe-001";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.authorization_revision = 1;

  // Pre-seed journal with accepted entry without publication attempt marker
  std::ofstream journal(test_journal_path_, std::ios::app);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V2 15 2 1 recovery-probe-001 1 "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub)) << "\n";
  journal.close();

  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", test_journal_path_);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());

  // Replay: recovers and publishes to public pipeline_survey topic
  submission_pub_->publish(sub);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.latitude, 33.810);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.longitude, -118.393);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);

  // Subsequent duplicate submission should deduplicate
  submission_pub_->publish(sub);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->duplicateCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, CrashBeforeDurableAcceptanceAllowsSubsequentRetry)
{
  spinFor(50ms);
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "retry-probe-001";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.authorization_revision = 1;

  EXPECT_EQ(adapter_node_->acceptedCount(), 0u);
  EXPECT_FALSE(adapter_node_->hasReport("retry-probe-001"));

  // First valid attempt succeeds
  submission_pub_->publish(sub);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_TRUE(adapter_node_->hasReport("retry-probe-001"));
  EXPECT_EQ(received_surveys_.size(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, JournalStorageFailureRejectsAndDoesNotClaimDurableSuccess)
{
  spinFor(50ms);
  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  std::string invalid_journal = "/dev/null/unwritable_dir/test.journal";

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", invalid_journal);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);
  spinFor(50ms);

  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "storage-fail-001";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.authorization_revision = 1;

  submission_pub_->publish(sub);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);
  EXPECT_EQ(adapter_node_->acceptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_FALSE(adapter_node_->hasReport("storage-fail-001"));
}

TEST_F(GcsSubmissionAdapterTest, NewRunBeginsSuppressesUnresolvedPendingReportFromOldRun)
{
  spinFor(50ms);
  banyu_msgs::msg::Task2Submission sub_old;
  sub_old.header.stamp = test_node_->now();
  sub_old.schema_version = 1;
  sub_old.run_id = 10;
  sub_old.epoch = 1;
  sub_old.report_id = "old-run-report";
  sub_old.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub_old.course_id = "course-1";
  sub_old.tier = 2;
  sub_old.buoy_latitude = 33.810;
  sub_old.buoy_longitude = -118.393;
  sub_old.original_conditions = {1, 2, 1, 1};
  sub_old.authorization_revision = 1;

  std::ofstream journal(test_journal_path_, std::ios::app);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 10 1 1 old-run-report 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub_old)) << "\n";
  journal.close();

  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", test_journal_path_);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  options.append_parameter_override("topics.command", "/test_adapter/command");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);

  auto command_pub = test_node_->create_publisher<rx_msgs::msg::Command>(
    "/test_adapter/command", rclcpp::QoS(10).reliable());
  spinFor(50ms);

  rx_msgs::msg::Command cmd;
  cmd.command_type = rx_msgs::msg::Command::CMD_RUN_START;
  cmd.command_seq = 1;
  cmd.run_id = 20;
  command_pub->publish(cmd);
  spinFor(50ms);

  // Replay of old run report must not be published
  submission_pub_->publish(sub_old);
  spinFor(100ms);

  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);

  // New report for current run 20 is accepted and published
  banyu_msgs::msg::Task2Submission sub_new = sub_old;
  sub_new.run_id = 20;
  sub_new.report_id = "new-run-report";
  submission_pub_->publish(sub_new);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_TRUE(adapter_node_->hasReport("new-run-report"));
}

TEST_F(GcsSubmissionAdapterTest, SurveyAndDeliveryEachRecoverIndependently)
{
  spinFor(50ms);
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "survey-indep-001";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 3;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.resource_color = 4;
  sub.delivery_circle_color = 2;
  sub.authorization_revision = 1;

  banyu_msgs::msg::Task2Submission del = sub;
  del.kind = banyu_msgs::msg::Task2Submission::DELIVERY;
  del.report_id = "delivery-indep-001";
  del.authorization_revision = 2;

  std::ofstream journal(test_journal_path_, std::ios::app);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 2 1 survey-indep-001 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub)) << "\n";
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 2 2 delivery-indep-001 2 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(del)) << "\n";
  journal.close();

  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", test_journal_path_);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->pendingCount(), 2u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);

  // Replay SURVEY only
  submission_pub_->publish(sub);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_TRUE(received_deliveries_.empty());
  EXPECT_EQ(
    adapter_node_->publicationState("survey-indep-001"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::ATTEMPTED);
  EXPECT_EQ(
    adapter_node_->publicationState("delivery-indep-001"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::PENDING);

  // Replay DELIVERY only
  submission_pub_->publish(del);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  ASSERT_EQ(received_deliveries_.size(), 1u);
  EXPECT_EQ(received_deliveries_[0].resource_color, 4u);
  EXPECT_EQ(
    adapter_node_->publicationState("delivery-indep-001"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::ATTEMPTED);
}

TEST_F(GcsSubmissionAdapterTest, AdapterRestartsAfterPublicationLateReceiverFollowsDocumentedPolicy)
{
  spinFor(50ms);
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = test_node_->now();
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = 2;
  sub.report_id = "late-recv-probe";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.authorization_revision = 1;

  submission_pub_->publish(sub);
  spinFor(100ms);
  ASSERT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(
    adapter_node_->publicationState("late-recv-probe"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::ATTEMPTED);

  // Restart adapter
  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", test_journal_path_);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);
  spinFor(50ms);

  // Late subscriber connects after restart
  std::vector<rx_msgs::msg::PipelineSurvey> late_surveys;
  auto late_sub = test_node_->create_subscription<rx_msgs::msg::PipelineSurvey>(
    "/test_adapter/pipeline_survey", rclcpp::QoS(10).reliable().transient_local(),
    [&late_surveys](const rx_msgs::msg::PipelineSurvey::SharedPtr msg) {
      late_surveys.push_back(*msg);
    });
  spinFor(100ms);

  // Restarted adapter does not spontaneously republish attempted reports.
  // Note: because submission_pub_ has transient_local QoS, adapter's new subscription
  // receives the cached submission upon match and deduplicates it immediately.
  EXPECT_TRUE(late_surveys.empty());
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->duplicateCount(), 1u);

  // Explicit retransmission of already attempted report is also deduplicated and not re-forwarded
  submission_pub_->publish(sub);
  spinFor(100ms);

  EXPECT_TRUE(late_surveys.empty());
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->duplicateCount(), 2u);
}

TEST_F(GcsSubmissionAdapterTest, NewRunSuppressionPersistsAcrossAdapterRestart)
{
  spinFor(50ms);
  banyu_msgs::msg::Task2Submission sub_old;
  sub_old.header.stamp = test_node_->now();
  sub_old.schema_version = 1;
  sub_old.run_id = 10;
  sub_old.epoch = 1;
  sub_old.report_id = "old-run-suppressed";
  sub_old.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub_old.course_id = "course-1";
  sub_old.tier = 2;
  sub_old.buoy_latitude = 33.810;
  sub_old.buoy_longitude = -118.393;
  sub_old.original_conditions = {1, 2, 1, 1};
  sub_old.authorization_revision = 1;

  // Pre-seed journal with PENDING entry
  std::ofstream journal(test_journal_path_, std::ios::app);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 10 1 1 old-run-suppressed 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub_old)) << "\n";
  journal.close();

  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", test_journal_path_);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  options.append_parameter_override("topics.command", "/test_adapter/command");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);

  auto command_pub = test_node_->create_publisher<rx_msgs::msg::Command>(
    "/test_adapter/command", rclcpp::QoS(10).reliable());
  spinFor(50ms);

  // CMD_RUN_START for run 20 suppresses old report
  rx_msgs::msg::Command cmd;
  cmd.command_type = rx_msgs::msg::Command::CMD_RUN_START;
  cmd.command_seq = 1;
  cmd.run_id = 20;
  command_pub->publish(cmd);
  spinFor(50ms);

  EXPECT_EQ(
    adapter_node_->publicationState("old-run-suppressed"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::SUPPRESSED_NEW_RUN);

  // Restart adapter - suppression state and active run must persist
  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);
  spinFor(50ms);

  EXPECT_EQ(
    adapter_node_->publicationState("old-run-suppressed"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::SUPPRESSED_NEW_RUN);

  // Replay of old report must still be rejected and not published
  submission_pub_->publish(sub_old);
  spinFor(100ms);

  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, JournalCorruptionMidLineLogsDiagnosticAndRecoversValidEntries)
{
  spinFor(50ms);
  banyu_msgs::msg::Task2Submission sub1;
  sub1.header.stamp = test_node_->now();
  sub1.schema_version = 1;
  sub1.run_id = 15;
  sub1.epoch = 1;
  sub1.report_id = "valid-before-corrupt";
  sub1.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub1.course_id = "course-1";
  sub1.tier = 2;
  sub1.buoy_latitude = 33.810;
  sub1.buoy_longitude = -118.393;
  sub1.original_conditions = {1, 2, 1, 1};
  sub1.authorization_revision = 1;

  banyu_msgs::msg::Task2Submission sub2 = sub1;
  sub2.epoch = 2;
  sub2.report_id = "valid-after-corrupt";

  // Write valid entry 1, a corrupt truncated entry, an unknown tag, and valid entry 2
  std::ofstream journal(test_journal_path_, std::ios::app);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 1 1 valid-before-corrupt 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub1)) << "\n";
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 1 corrupt_truncated_line_with_missing_fields\n";
  journal << "BANYU_GCS_ADAPTER_UNKNOWN_TAG 15 1 foo bar\n";
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 2 1 valid-after-corrupt 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub2)) << "\n";
  journal.close();

  executor_.remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions options;
  options.append_parameter_override("journal_path", test_journal_path_);
  options.append_parameter_override("topics.task2_submission", "/test_adapter/submission");
  options.append_parameter_override("topics.pipeline_survey", "/test_adapter/pipeline_survey");
  options.append_parameter_override("topics.delivery", "/test_adapter/delivery");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->corruptEntryCount(), 1u);
  EXPECT_EQ(adapter_node_->acceptedCount(), 2u);
  EXPECT_TRUE(adapter_node_->hasReport("valid-before-corrupt"));
  EXPECT_TRUE(adapter_node_->hasReport("valid-after-corrupt"));
  EXPECT_EQ(adapter_node_->pendingCount(), 2u);

  // Both valid entries can be recovered and published
  submission_pub_->publish(sub1);
  spinFor(100ms);
  EXPECT_EQ(received_surveys_.size(), 1u);

  submission_pub_->publish(sub2);
  spinFor(100ms);
  EXPECT_EQ(received_surveys_.size(), 2u);
}
