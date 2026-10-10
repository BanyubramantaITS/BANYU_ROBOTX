#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <vector>

#include "banyu_communication/gcs_submission_adapter.hpp"
#include "banyu_mission/run_coordinator.hpp"
#include "banyu_msgs/msg/task2_submission.hpp"
#include "banyu_msgs/task2_submission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"

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
      "topics.run_lifecycle", "/test_adapter/run_lifecycle");
    options.append_parameter_override(
      "topics.pipeline_survey", "/test_adapter/pipeline_survey");
    options.append_parameter_override(
      "topics.delivery", "/test_adapter/delivery");
    options.append_parameter_override(
      "topics.command", "/test_adapter/command");
    options.append_parameter_override("forward_to_rx_topics", true);

    adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
    test_node_ = std::make_shared<rclcpp::Node>("gcs_adapter_test_peer");

    submission_pub_ = test_node_->create_publisher<banyu_msgs::msg::Task2Submission>(
      "/test_adapter/submission", rclcpp::QoS(10).reliable().transient_local());

    lifecycle_pub_ = test_node_->create_publisher<banyu_msgs::msg::RunLifecycle>(
      "/test_adapter/run_lifecycle", rclcpp::QoS(10).reliable().transient_local());

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

  void restart(bool forward)
  {
    if (adapter_node_) {
      executor_.remove_node(adapter_node_);
      adapter_node_.reset();
    }
    rclcpp::NodeOptions options;
    options.append_parameter_override("journal_path", test_journal_path_);
    options.append_parameter_override(
      "topics.task2_submission", "/test_adapter/submission");
    options.append_parameter_override(
      "topics.run_lifecycle", "/test_adapter/run_lifecycle");
    options.append_parameter_override(
      "topics.pipeline_survey", "/test_adapter/pipeline_survey");
    options.append_parameter_override(
      "topics.delivery", "/test_adapter/delivery");
    options.append_parameter_override(
      "topics.command", "/test_adapter/command");
    options.append_parameter_override("forward_to_rx_topics", forward);
    adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
    executor_.add_node(adapter_node_);
    spinFor(150ms);
  }

  void publishLifecycle(
    uint32_t run_id = 15, uint64_t epoch = 2,
    uint8_t phase = banyu_msgs::msg::RunLifecycle::ACTIVE,
    uint8_t event = banyu_msgs::msg::RunLifecycle::START_ACCEPTED,
    uint32_t requested_run_id = 15)
  {
    banyu_msgs::msg::RunLifecycle msg;
    msg.header.stamp = test_node_->now();
    msg.header.frame_id = "uuv";
    msg.authority_instance = "test_authority";
    msg.revision = ++lifecycle_revision_;
    msg.run_id = run_id;
    msg.epoch = epoch;
    msg.phase = phase;
    msg.event = event;
    msg.requested_run_id = requested_run_id;
    msg.autonomous_ready = true;
    lifecycle_pub_->publish(msg);
    spinFor(50ms);
  }

  banyu_msgs::msg::Task2Submission createSurvey(
    uint32_t run_id = 15, uint64_t epoch = 1,
    const std::string & report_id = "local-audit-report")
  {
    banyu_msgs::msg::Task2Submission s;
    s.header.stamp = test_node_->now();
    s.schema_version = 1;
    s.run_id = run_id;
    s.epoch = epoch;
    s.report_id = report_id;
    s.kind = banyu_msgs::msg::Task2Submission::SURVEY;
    s.course_id = "course-1";
    s.tier = 2;
    s.buoy_latitude = 33.810;
    s.buoy_longitude = -118.393;
    s.original_conditions = {1, 2, 1, 1};
    s.authorization_revision = 1;
    return s;
  }

  std::shared_ptr<banyu_communication::GcsSubmissionAdapter> adapter_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<banyu_msgs::msg::Task2Submission>::SharedPtr submission_pub_;
  rclcpp::Publisher<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_pub_;
  uint64_t lifecycle_revision_{0};
  rclcpp::Subscription<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_sub_;
  rclcpp::Subscription<rx_msgs::msg::ResourceDelivery>::SharedPtr delivery_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<rx_msgs::msg::PipelineSurvey> received_surveys_;
  std::vector<rx_msgs::msg::ResourceDelivery> received_deliveries_;
  std::string test_journal_path_;
};

TEST_F(GcsSubmissionAdapterTest, IngestsAndForwardPipelineSurveyAndDeduplicates)
{
  publishLifecycle(15, 2);
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
  publishLifecycle(15, 2);
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
  publishLifecycle(15, 2);
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
  publishLifecycle(20, 1);
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
  publishLifecycle(30, 1);
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

  restart(true);

  EXPECT_EQ(adapter_node_->acceptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());

  // Establish authoritative lifecycle for run 15, epoch 2: recovers and publishes
  publishLifecycle(15, 2);
  spinFor(100ms);

  ASSERT_EQ(received_surveys_.size(), 1u);
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
  publishLifecycle(15, 2);
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
  options.append_parameter_override("topics.run_lifecycle", "/test_adapter/run_lifecycle");
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
  publishLifecycle(20, 1);
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

  restart(true);

  EXPECT_EQ(adapter_node_->pendingCount(), 2u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_TRUE(received_deliveries_.empty());

  // Establish authoritative lifecycle for run 15, epoch 2: recovers both to their respective public topics
  publishLifecycle(15, 2);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  ASSERT_EQ(received_deliveries_.size(), 1u);
  EXPECT_EQ(received_deliveries_[0].resource_color, 4u);
  EXPECT_EQ(
    adapter_node_->publicationState("survey-indep-001"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::ATTEMPTED);
  EXPECT_EQ(
    adapter_node_->publicationState("delivery-indep-001"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::ATTEMPTED);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 2u);

  // Retransmission of already attempted reports deduplicates independently
  submission_pub_->publish(sub);
  spinFor(100ms);
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->duplicateCount(), 1u);

  submission_pub_->publish(del);
  spinFor(100ms);
  EXPECT_EQ(received_deliveries_.size(), 1u);
  EXPECT_EQ(adapter_node_->duplicateCount(), 2u);
}

TEST_F(GcsSubmissionAdapterTest, AdapterRestartsAfterPublicationLateReceiverFollowsDocumentedPolicy)
{
  spinFor(50ms);
  publishLifecycle(15, 2);
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
  options.append_parameter_override("topics.run_lifecycle", "/test_adapter/run_lifecycle");
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(options);
  executor_.add_node(adapter_node_);

  auto command_pub = test_node_->create_publisher<rx_msgs::msg::Command>(
    "/test_adapter/command", rclcpp::QoS(10).reliable());
  spinFor(50ms);

  // Authoritative lifecycle for run 20 suppresses old report
  rx_msgs::msg::Command cmd;
  cmd.command_type = rx_msgs::msg::Command::CMD_RUN_START;
  cmd.command_seq = 1;
  cmd.run_id = 20;
  command_pub->publish(cmd);
  publishLifecycle(20, 1);
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

  restart(true);

  EXPECT_EQ(adapter_node_->corruptEntryCount(), 1u);
  EXPECT_EQ(adapter_node_->acceptedCount(), 2u);
  EXPECT_TRUE(adapter_node_->hasReport("valid-before-corrupt"));
  EXPECT_TRUE(adapter_node_->hasReport("valid-after-corrupt"));
  EXPECT_EQ(adapter_node_->pendingCount(), 2u);

  // Publish valid lifecycle for run 15, epoch 1 to recover valid epoch-1 report
  publishLifecycle(15, 1);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);

  // Publish valid lifecycle for run 15, epoch 2 to recover valid epoch-2 report
  publishLifecycle(15, 2);
  spinFor(100ms);

  EXPECT_EQ(received_surveys_.size(), 2u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 2u);
}

TEST_F(GcsSubmissionAdapterTest, ForeignVehicleStartMustNotBlockUuvReport)
{
  restart(true);
  publishLifecycle(15, 1);
  auto p = test_node_->create_publisher<rx_msgs::msg::Command>("/test_adapter/command", 10);
  spinFor(150ms);
  rx_msgs::msg::Command c;
  c.command_type = c.CMD_RUN_START;
  c.command_seq = 1;
  c.run_id = 99;
  c.target_vehicle = rx_msgs::msg::VehicleType::TYPE_UAV;
  p->publish(c);
  spinFor(150ms);
  submission_pub_->publish(createSurvey(15, 1, "local-audit-report"));
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 1u)
    << "A UAV-addressed start must not change UUV report context";
}

TEST_F(GcsSubmissionAdapterTest, RejectedConflictingStartMustNotBlockAcceptedRunReport)
{
  restart(true);
  banyu_mission::RunCoordinator coordinator;
  ASSERT_TRUE(coordinator.beginPreparation());
  ASSERT_TRUE(coordinator.markReady());
  auto accepted = coordinator.requestStart(15, 1, true);
  ASSERT_EQ(accepted.disposition, banyu_mission::RunCoordinator::Disposition::Accepted);
  ASSERT_TRUE(coordinator.completeInitialization(accepted.snapshot.epoch));
  publishLifecycle(15, accepted.snapshot.epoch, banyu_msgs::msg::RunLifecycle::ACTIVE,
    banyu_msgs::msg::RunLifecycle::START_ACCEPTED, 15);

  auto p = test_node_->create_publisher<rx_msgs::msg::Command>("/test_adapter/command", 10);
  spinFor(150ms);
  rx_msgs::msg::Command c;
  c.command_type = c.CMD_RUN_START;
  c.command_seq = 1;
  c.run_id = 15;
  c.target_vehicle = rx_msgs::msg::VehicleType::TYPE_UUV;
  p->publish(c);
  spinFor(150ms);
  auto rejected = coordinator.requestStart(99, 2, true);
  ASSERT_EQ(rejected.disposition, banyu_mission::RunCoordinator::Disposition::Rejected);
  EXPECT_EQ(coordinator.snapshot().run_id, 15u);
  publishLifecycle(15, accepted.snapshot.epoch, banyu_msgs::msg::RunLifecycle::ACTIVE,
    banyu_msgs::msg::RunLifecycle::START_REJECTED, 99);
  c.command_seq = 2;
  c.run_id = 99;
  p->publish(c);
  spinFor(150ms);
  auto s = createSurvey(15, accepted.snapshot.epoch, "local-audit-report");
  ASSERT_TRUE(coordinator.transition(s.epoch,
    banyu_mission::RunCoordinator::Phase::SurveyComplete));
  ASSERT_TRUE(coordinator.transition(s.epoch, banyu_mission::RunCoordinator::Phase::Surfaced));
  auto seal = coordinator.sealReport(
    s.run_id, s.epoch, s.kind, s.report_id, banyu_msgs::task2::canonicalPayload(s));
  ASSERT_EQ(seal.disposition, banyu_mission::RunCoordinator::Disposition::Accepted);
  submission_pub_->publish(s);
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 1u)
    << "Adapter must track mission-accepted context rather than raw conflicting starts";
}

TEST_F(GcsSubmissionAdapterTest, DisabledForwardingMustPreservePendingPublication)
{
  restart(false);
  publishLifecycle(15, 1);
  submission_pub_->publish(createSurvey(15, 1, "local-audit-report"));
  spinFor(200ms);
  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_EQ(adapter_node_->pendingCount(), 1u)
    << "No publication was attempted when forwarding is disabled";
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  restart(true);
  publishLifecycle(15, 1);
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 1u)
    << "Enabling forwarding must recover the unsent report";
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
}

TEST_F(GcsSubmissionAdapterTest, AlreadyPendingRecoveryWithForwardingDisabledRemainsPending)
{
  restart(false);
  publishLifecycle(15, 1);
  submission_pub_->publish(createSurvey(15, 1, "disabled-restart-pending"));
  spinFor(200ms);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());

  restart(false);
  publishLifecycle(15, 1);
  spinFor(200ms);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());
}

TEST_F(GcsSubmissionAdapterTest,
  DeliveryPreservesPendingPublicationWhenForwardingDisabledAndRecovers)
{
  restart(false);
  publishLifecycle(15, 1);
  banyu_msgs::msg::Task2Submission del = createSurvey(15, 1, "delivery-disabled-rep");
  del.kind = banyu_msgs::msg::Task2Submission::DELIVERY;
  del.resource_color = 4;
  del.delivery_circle_color = 2;
  del.tier = 3;
  submission_pub_->publish(del);
  spinFor(200ms);
  EXPECT_TRUE(received_deliveries_.empty());
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);

  restart(true);
  publishLifecycle(15, 1);
  spinFor(200ms);
  EXPECT_EQ(received_deliveries_.size(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
}

TEST_F(GcsSubmissionAdapterTest,
  DelayedOldAuthorityOrLowerEpochRejectedWithoutMutatingAcceptedContext)
{
  restart(true);
  publishLifecycle(15, 5, banyu_msgs::msg::RunLifecycle::ACTIVE,
    banyu_msgs::msg::RunLifecycle::START_ACCEPTED, 15);
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 5u);

  publishLifecycle(15, 3, banyu_msgs::msg::RunLifecycle::ACTIVE,
    banyu_msgs::msg::RunLifecycle::SNAPSHOT, 15);
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 5u);
}

TEST_F(GcsSubmissionAdapterTest, DuplicateStartRefreshesAuthorityWithoutSuppressingOrResetting)
{
  restart(true);
  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::ACTIVE,
    banyu_msgs::msg::RunLifecycle::START_ACCEPTED, 15);
  auto sub = createSurvey(15, 2, "dup-start-test");
  submission_pub_->publish(sub);
  spinFor(150ms);
  EXPECT_EQ(received_surveys_.size(), 1u);

  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::ACTIVE,
    banyu_msgs::msg::RunLifecycle::START_DUPLICATE, 15);
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_TRUE(adapter_node_->hasReport("dup-start-test"));
}

TEST_F(GcsSubmissionAdapterTest, StaleOrMismatchedCommandDoesNotSuppressReport)
{
  restart(true);
  publishLifecycle(15, 1);
  auto p = test_node_->create_publisher<rx_msgs::msg::Command>("/test_adapter/command", 10);
  spinFor(100ms);

  rx_msgs::msg::Command c;
  c.command_type = rx_msgs::msg::Command::CMD_RUN_START;
  c.command_seq = 0;
  c.run_id = 99;
  p->publish(c);
  spinFor(100ms);

  submission_pub_->publish(createSurvey(15, 1, "valid-report-under-stale-cmd"));
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, LegacyActiveRunJournalHintDoesNotAuthorizeRun)
{
  // Write legacy ACTIVE_RUN and a pending report to journal
  auto s = createSurvey(10, 1, "legacy-active-run-pending");
  std::ofstream journal(test_journal_path_, std::ios::trunc);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 10 1 1 legacy-active-run-pending 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(s)) << "\n";
  journal << "BANYU_GCS_ADAPTER_ACTIVE_RUN 10\n";
  journal.close();

  restart(true);
  EXPECT_EQ(adapter_node_->activeRunId(), 0u)
    << "Legacy ACTIVE_RUN must be treated as a hint, not proof of mission acceptance";
  EXPECT_EQ(adapter_node_->historicalHintRunId(), 10u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_TRUE(received_surveys_.empty());

  // Authoritative lifecycle for run 10 arrives
  publishLifecycle(10, 1);
  spinFor(150ms);
  EXPECT_EQ(adapter_node_->activeRunId(), 10u);
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, StaleEpochWithinSameRunRejected)
{
  restart(true);
  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::ACTIVE,
    banyu_msgs::msg::RunLifecycle::START_ACCEPTED, 15);
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 2u);

  // Submitting a report with stale epoch=1 for run 15 must be rejected
  auto stale_sub = createSurvey(15, 1, "stale-epoch-report");
  submission_pub_->publish(stale_sub);
  spinFor(150ms);

  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);
  EXPECT_FALSE(adapter_node_->hasReport("stale-epoch-report"));
  EXPECT_TRUE(received_surveys_.empty());

  // Submitting valid report for active epoch=2 is accepted and forwarded
  auto valid_sub = createSurvey(15, 2, "valid-epoch2-report");
  submission_pub_->publish(valid_sub);
  spinFor(150ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_TRUE(adapter_node_->hasReport("valid-epoch2-report"));
}

TEST_F(GcsSubmissionAdapterTest, RetransmissionWhileAuthorityUnavailableDoesNotPublish)
{
  restart(true);
  EXPECT_EQ(adapter_node_->activeRunId(), 0u);

  // Initial submission while active_run_id_ == 0 is accepted as PENDING, but not published
  auto sub = createSurvey(15, 1, "no-authority-pending");
  submission_pub_->publish(sub);
  spinFor(150ms);

  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());

  // Retransmission while active_run_id_ == 0 must still remain PENDING and not publish
  submission_pub_->publish(sub);
  spinFor(150ms);

  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());

  // Only when authoritative lifecycle arrives does it forward
  publishLifecycle(15, 1);
  spinFor(150ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, StaleLifecycleRejectedAfterJournalRestart)
{
  // Persist active context run 20, epoch 5, revision 10
  std::ofstream journal(test_journal_path_, std::ios::trunc);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ACTIVE_CONTEXT 20 5 10 coord_persisted_instance\n";
  journal.close();

  restart(true);
  EXPECT_EQ(adapter_node_->activeRunId(), 20u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 5u);

  // Stale lifecycle message with lower epoch=3 arriving as first lifecycle after restart
  banyu_msgs::msg::RunLifecycle stale_msg;
  stale_msg.header.stamp = test_node_->now();
  stale_msg.header.frame_id = "uuv";
  stale_msg.authority_instance = "coord_stale_instance";
  stale_msg.revision = 1;
  stale_msg.run_id = 20;
  stale_msg.epoch = 3;
  stale_msg.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  stale_msg.event = banyu_msgs::msg::RunLifecycle::SNAPSHOT;
  stale_msg.requested_run_id = 20;
  lifecycle_pub_->publish(stale_msg);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->activeRunId(), 20u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 5u);
}

TEST_F(GcsSubmissionAdapterTest, PersistenceFailurePreservesInMemoryAuthority)
{
  restart(true);
  publishLifecycle(15, 1);
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 1u);

  // Make journal read-only so appendActiveContext fails
  std::filesystem::permissions(
    test_journal_path_,
    std::filesystem::perms::owner_read | std::filesystem::perms::group_read,
    std::filesystem::perm_options::replace);

  // Transition to run 20 must fail persistence and preserve in-memory authority (run 15)
  publishLifecycle(20, 2);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->activeRunId(), 15u)
    << "In-memory authority must not change when required persistence fails";
  EXPECT_EQ(adapter_node_->activeEpoch(), 1u);

  // Restore permissions for teardown and verify previously accepted owner can still progress
  std::filesystem::permissions(
    test_journal_path_,
    std::filesystem::perms::all,
    std::filesystem::perm_options::replace);

  submission_pub_->publish(createSurvey(15, 1, "progress-after-persistence-failure"));
  spinFor(150ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
}

TEST_F(GcsSubmissionAdapterTest, RetainedSnapshotWithSameRevisionRecoversPendingReportOnRestart)
{
  auto sub = createSurvey(15, 2, "retained-same-rev-report");
  std::ofstream journal(test_journal_path_, std::ios::trunc);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ACTIVE_CONTEXT 15 2 10 test_authority\n";
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 2 1 retained-same-rev-report 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub)) << "\n";
  journal.close();

  restart(true);
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 2u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_FALSE(adapter_node_->lifecycleReceived());

  // Transient-local retained message arriving on restart has exact same revision=10
  banyu_msgs::msg::RunLifecycle msg;
  msg.header.stamp = test_node_->now();
  msg.header.frame_id = "uuv";
  msg.authority_instance = "test_authority";
  msg.revision = 10;
  msg.run_id = 15;
  msg.epoch = 2;
  msg.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  msg.event = banyu_msgs::msg::RunLifecycle::SNAPSHOT;
  msg.requested_run_id = 15;
  lifecycle_pub_->publish(msg);
  spinFor(150ms);

  EXPECT_TRUE(adapter_node_->lifecycleReceived());
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest,
  StartRejectedArrivingAsFirstMessageEstablishesAcceptedContextAndRecovers)
{
  auto sub = createSurvey(15, 1, "rejected-first-msg-report");
  std::ofstream journal(test_journal_path_, std::ios::trunc);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 1 1 rejected-first-msg-report 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub)) << "\n";
  journal.close();

  restart(true);
  EXPECT_EQ(adapter_node_->activeRunId(), 0u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);

  // START_REJECTED arriving as first lifecycle message describes accepted run 15 and rejected command 99
  banyu_msgs::msg::RunLifecycle msg;
  msg.header.stamp = test_node_->now();
  msg.header.frame_id = "uuv";
  msg.authority_instance = "coord_inst";
  msg.revision = 1;
  msg.run_id = 15;
  msg.epoch = 1;
  msg.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  msg.event = banyu_msgs::msg::RunLifecycle::START_REJECTED;
  msg.requested_run_id = 99;
  lifecycle_pub_->publish(msg);
  spinFor(150ms);

  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_TRUE(adapter_node_->lifecycleReceived());
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, TerminalLifecycleWithOlderSourceAgeAcceptedForReportRecovery)
{
  auto sub = createSurvey(15, 1, "terminal-old-source-report");
  std::ofstream journal(test_journal_path_, std::ios::trunc);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ENTRY_V3 15 1 1 terminal-old-source-report 1 PENDING "
          << std::quoted(banyu_msgs::task2::canonicalPayload(sub)) << "\n";
  journal.close();

  restart(true);

  // Terminal lifecycle with timestamp 10 seconds in the past (> 5s lifecycle_timeout_s)
  banyu_msgs::msg::RunLifecycle msg;
  msg.header.stamp = test_node_->now() - rclcpp::Duration::from_seconds(10.0);
  msg.header.frame_id = "uuv";
  msg.authority_instance = "coord_terminal";
  msg.revision = 1;
  msg.run_id = 15;
  msg.epoch = 1;
  msg.phase = banyu_msgs::msg::RunLifecycle::SUCCEEDED;
  msg.event = banyu_msgs::msg::RunLifecycle::TERMINAL;
  msg.requested_run_id = 15;
  lifecycle_pub_->publish(msg);
  spinFor(150ms);

  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_TRUE(adapter_node_->lifecycleReceived());
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest,
  SubmissionArrivingBeforeLifecycleOnRestartRemainsPendingUntilLifecycleArrives)
{
  std::ofstream journal(test_journal_path_, std::ios::trunc);
  ASSERT_TRUE(journal.is_open());
  journal << "BANYU_GCS_ADAPTER_ACTIVE_CONTEXT 15 1 5 coord_prev\n";
  journal.close();

  restart(true);
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
  EXPECT_FALSE(adapter_node_->lifecycleReceived());

  // Report arrives before any lifecycle message in this process
  auto sub = createSurvey(15, 1, "before-lifecycle-report");
  submission_pub_->publish(sub);
  spinFor(150ms);

  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());

  // Valid lifecycle arrives
  publishLifecycle(15, 1);
  spinFor(150ms);

  EXPECT_TRUE(adapter_node_->lifecycleReceived());
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest,
  DynamicParameterToggleEnablesForwardingAndRecoversPendingWithoutRestart)
{
  restart(false);
  publishLifecycle(15, 1);

  auto sub = createSurvey(15, 1, "param-toggle-report");
  submission_pub_->publish(sub);
  spinFor(150ms);

  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_TRUE(received_surveys_.empty());

  // Dynamically set parameter forward_to_rx_topics to true
  auto result = adapter_node_->set_parameter(rclcpp::Parameter("forward_to_rx_topics", true));
  EXPECT_TRUE(result.successful);
  spinFor(150ms);

  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, PhaseInitializingWithZeroRunIdOrEpochIsRejected)
{
  restart(true);

  banyu_msgs::msg::RunLifecycle msg;
  msg.header.stamp = test_node_->now();
  msg.header.frame_id = "uuv";
  msg.authority_instance = "coord_invalid";
  msg.revision = 1;
  msg.run_id = 0;  // invalid: zero in active phase
  msg.epoch = 1;
  msg.phase = banyu_msgs::msg::RunLifecycle::INITIALIZING;
  msg.event = banyu_msgs::msg::RunLifecycle::SNAPSHOT;
  msg.requested_run_id = 0;
  lifecycle_pub_->publish(msg);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->activeRunId(), 0u);
  EXPECT_FALSE(adapter_node_->lifecycleReceived());
}

TEST_F(GcsSubmissionAdapterTest, FutureEpochSubmissionCannotPublishUnderOlderAuthority)
{
  spinFor(150ms);
  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::SURFACED);
  ASSERT_EQ(adapter_node_->activeEpoch(), 2u);

  // Future epoch=3 submission cannot publish under epoch=2 authority
  submission_pub_->publish(createSurvey(15, 3, "future-epoch-fresh"));
  spinFor(200ms);
  EXPECT_TRUE(received_surveys_.empty())
    << "Authority for epoch 2 cannot publish a report from epoch 3";
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);

  // Retransmission still awaiting matching authority
  submission_pub_->publish(createSurvey(15, 3, "future-epoch-fresh"));
  spinFor(100ms);
  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);

  // Matching epoch=3 authority arrives -> published once
  publishLifecycle(15, 3, banyu_msgs::msg::RunLifecycle::SURFACED);
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);

  // Retransmission after publication is deduplicated
  submission_pub_->publish(createSurvey(15, 3, "future-epoch-fresh"));
  spinFor(100ms);
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->duplicateCount(), 1u);
}

TEST_F(GcsSubmissionAdapterTest, PendingFutureEpochCannotRecoverUnderOlderAuthority)
{
  spinFor(150ms);
  submission_pub_->publish(createSurvey(15, 3, "future-epoch-pending"));
  spinFor(150ms);
  ASSERT_EQ(adapter_node_->pendingCount(), 1u);
  ASSERT_TRUE(received_surveys_.empty());

  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::SURFACED);
  spinFor(200ms);
  EXPECT_TRUE(received_surveys_.empty())
    << "Pending epoch-3 report must await matching authority";
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);

  // Restart with forwarding disabled
  restart(false);
  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::SURFACED);
  spinFor(100ms);
  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);

  // Toggle forwarding to true while under older authority
  auto toggle_res = adapter_node_->set_parameter(rclcpp::Parameter("forward_to_rx_topics", true));
  EXPECT_TRUE(toggle_res.successful);
  spinFor(100ms);
  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);

  // Matching epoch=3 authority arrives
  publishLifecycle(15, 3, banyu_msgs::msg::RunLifecycle::SURFACED);
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
}

TEST_F(GcsSubmissionAdapterTest, RejectedAuthorityMustNotRetireCurrentMissionOwner)
{
  using L = banyu_msgs::msg::RunLifecycle;
  spinFor(150ms);
  publishLifecycle(15, 2, L::SUCCEEDED, L::TERMINAL);
  ASSERT_EQ(adapter_node_->activeRunId(), 15u);
  ASSERT_EQ(adapter_node_->activeEpoch(), 2u);

  // Rejected lower-epoch message from other authority
  L stale;
  stale.header.stamp = test_node_->now();
  stale.authority_instance = "stale_other_owner";
  stale.revision = 1;
  stale.run_id = 14;
  stale.epoch = 1;
  stale.phase = L::ACTIVE;
  stale.event = L::SNAPSHOT;
  lifecycle_pub_->publish(stale);
  spinFor(100ms);
  ASSERT_EQ(adapter_node_->activeRunId(), 15u);
  ASSERT_EQ(adapter_node_->activeEpoch(), 2u);

  // Rejected conflicting same-epoch run from other authority
  L conflict;
  conflict.header.stamp = test_node_->now();
  conflict.authority_instance = "conflicting_owner";
  conflict.revision = 1;
  conflict.run_id = 99;
  conflict.epoch = 2;
  conflict.phase = L::ACTIVE;
  conflict.event = L::SNAPSHOT;
  lifecycle_pub_->publish(conflict);
  spinFor(100ms);
  // Rejected unaccepted replacement without active/accepted context from other authority
  L unaccepted_prep;
  unaccepted_prep.header.stamp = test_node_->now();
  unaccepted_prep.authority_instance = "prep_only_owner";
  unaccepted_prep.revision = 1;
  unaccepted_prep.run_id = 0;
  unaccepted_prep.epoch = 3;
  unaccepted_prep.phase = L::PREPARING;
  unaccepted_prep.event = L::SNAPSHOT;
  lifecycle_pub_->publish(unaccepted_prep);
  spinFor(100ms);
  ASSERT_EQ(adapter_node_->activeRunId(), 15u);
  ASSERT_EQ(adapter_node_->activeEpoch(), 2u);

  // Stale same-owner revision is ignored without retiring owner
  L stale_rev;
  stale_rev.header.stamp = test_node_->now();
  stale_rev.authority_instance = "test_authority";
  stale_rev.revision = 1;  // stale revision
  stale_rev.run_id = 15;
  stale_rev.epoch = 2;
  stale_rev.phase = L::SUCCEEDED;
  stale_rev.event = L::SNAPSHOT;
  lifecycle_pub_->publish(stale_rev);
  spinFor(100ms);
  ASSERT_EQ(adapter_node_->activeRunId(), 15u);
  ASSERT_EQ(adapter_node_->activeEpoch(), 2u);

  // Valid next run from current owner test_authority succeeds
  publishLifecycle(16, 3, L::SURFACED, L::START_ACCEPTED, 16);
  spinFor(100ms);
  EXPECT_EQ(adapter_node_->activeRunId(), 16u)
    << "Rejected lower-epoch replacement must leave the current owner usable";
  EXPECT_EQ(adapter_node_->activeEpoch(), 3u);
  submission_pub_->publish(createSurvey(16, 3, "current-owner-next-run"));
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 1u);

  // Accepted replacement by a new authority
  L new_owner;
  new_owner.header.stamp = test_node_->now();
  new_owner.authority_instance = "new_valid_owner";
  new_owner.revision = 1;
  new_owner.run_id = 17;
  new_owner.epoch = 4;
  new_owner.phase = L::SURFACED;
  new_owner.event = L::START_ACCEPTED;
  new_owner.requested_run_id = 17;
  lifecycle_pub_->publish(new_owner);
  spinFor(100ms);
  EXPECT_EQ(adapter_node_->activeRunId(), 17u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 4u);

  // Previous owner test_authority is now retired; messages from it are rejected
  publishLifecycle(18, 5, L::SURFACED, L::START_ACCEPTED, 18);
  spinFor(100ms);
  EXPECT_EQ(adapter_node_->activeRunId(), 17u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 4u);

  // New owner submission publishes
  submission_pub_->publish(createSurvey(17, 4, "new-owner-report"));
  spinFor(200ms);
  EXPECT_EQ(received_surveys_.size(), 2u);
}

TEST_F(GcsSubmissionAdapterTest, LifecycleTimeoutConfigurationAndRuntimeValidation)
{
  rclcpp::NodeOptions bad_opts1;
  bad_opts1.append_parameter_override("lifecycle_timeout_s", -1.0);
  EXPECT_THROW(
    std::make_shared<banyu_communication::GcsSubmissionAdapter>(bad_opts1),
    std::invalid_argument);

  rclcpp::NodeOptions bad_opts2;
  bad_opts2.append_parameter_override("lifecycle_timeout_s", 0.0);
  EXPECT_THROW(
    std::make_shared<banyu_communication::GcsSubmissionAdapter>(bad_opts2),
    std::invalid_argument);

  auto res_bad = adapter_node_->set_parameter(rclcpp::Parameter("lifecycle_timeout_s", -2.0));
  EXPECT_FALSE(res_bad.successful);

  auto res_ok = adapter_node_->set_parameter(rclcpp::Parameter("lifecycle_timeout_s", 1.0));
  EXPECT_TRUE(res_ok.successful);
}

TEST_F(GcsSubmissionAdapterTest, LifecycleTimeoutFreshnessBoundary)
{
  restart(true);
  auto res_timeout = adapter_node_->set_parameter(rclcpp::Parameter("lifecycle_timeout_s", 0.2));
  ASSERT_TRUE(res_timeout.successful);

  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::ACTIVE);
  spinFor(50ms);

  // Wait past 0.2s timeout so authority becomes stale
  spinFor(250ms);

  submission_pub_->publish(createSurvey(15, 2, "stale-timeout-sub"));
  spinFor(100ms);
  EXPECT_TRUE(received_surveys_.empty())
    << "Stale active authority past timeout cannot publish fresh submission";
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);

  // Fresh active lifecycle within window reconciles and publishes
  publishLifecycle(15, 2, banyu_msgs::msg::RunLifecycle::ACTIVE);
  spinFor(150ms);
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}
