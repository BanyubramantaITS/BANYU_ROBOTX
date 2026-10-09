#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "banyu_communication/gcs_submission_adapter.hpp"
#include "banyu_msgs/msg/task2_submission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"

using namespace std::chrono_literals;

class GcsSubmissionAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
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
