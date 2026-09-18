#include "banyu_detection/pipeline_end_node.hpp"

#include <chrono>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <banyu_msgs/msg/pipeline_observation.hpp>

namespace
{

class PipelineEndNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

void spinFor(const rclcpp::Node::SharedPtr & node, int cycles = 10)
{
  for (int i = 0; i < cycles; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

banyu_msgs::msg::PipelineObservation makeObservation(
  uint64_t seq, uint8_t status, float offset_x = 0.05F, float conf = 0.9F)
{
  banyu_msgs::msg::PipelineObservation obs;
  obs.frame_sequence = seq;
  obs.status = status;
  obs.offset_x = offset_x;
  obs.offset_y = 0.1F;
  obs.heading_error_rad = 0.02F;
  obs.confidence = conf;
  return obs;
}

TEST_F(PipelineEndNodeTest, NoEndTopicEmittedFromLoss)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("publish_confirmed_only", true);
  auto node = std::make_shared<banyu_detection::PipelineEndNode>(options);

  auto obs_pub = node->create_publisher<banyu_msgs::msg::PipelineObservation>(
    "/banyu/pipeline/observation", rclcpp::SensorDataQoS());

  int received_count = 0;
  auto end_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/banyu/pipeline/end", rclcpp::QoS(5),
    [&received_count](const std_msgs::msg::Bool::SharedPtr) {
      ++received_count;
    });

  spinFor(node, 5);

  // Send NO_PIPE observations simulating loss
  for (uint64_t i = 1; i <= 5; ++i) {
    obs_pub->publish(makeObservation(i, banyu_msgs::msg::PipelineObservation::NO_PIPE));
    spinFor(node, 2);
  }

  EXPECT_EQ(received_count, 0);
  EXPECT_FALSE(node->adapter().isConfirmed());
}

TEST_F(PipelineEndNodeTest, CandidateEvidenceEmitsEndTopicWhenConfirmed)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("publish_confirmed_only", true);
  options.append_parameter_override("required_frames", 3);
  auto node = std::make_shared<banyu_detection::PipelineEndNode>(options);

  auto obs_pub = node->create_publisher<banyu_msgs::msg::PipelineObservation>(
    "/banyu/pipeline/observation", rclcpp::SensorDataQoS());
  auto cand_pub = node->create_publisher<std_msgs::msg::Bool>(
    "/banyu/pipeline/end_candidate", rclcpp::QoS(10));

  std::vector<bool> received_values;
  auto end_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/banyu/pipeline/end", rclcpp::QoS(5),
    [&received_values](const std_msgs::msg::Bool::SharedPtr msg) {
      received_values.push_back(msg->data);
    });

  spinFor(node, 5);

  // Signal end candidate
  std_msgs::msg::Bool cand_msg;
  cand_msg.data = true;
  cand_pub->publish(cand_msg);
  spinFor(node, 3);

  // Send 3 stable tracked observations to satisfy required_frames
  for (uint64_t i = 1; i <= 3; ++i) {
    cand_pub->publish(cand_msg);
    obs_pub->publish(makeObservation(i, banyu_msgs::msg::PipelineObservation::TRACKED));
    spinFor(node, 5);
  }

  EXPECT_TRUE(node->adapter().isConfirmed());
  ASSERT_FALSE(received_values.empty());
  EXPECT_TRUE(received_values.back());
}

TEST_F(PipelineEndNodeTest, ResetClearsStateAndSuppressesOutput)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("publish_confirmed_only", true);
  options.append_parameter_override("required_frames", 2);
  options.append_parameter_override("observation_topic", "/test_reset/obs");
  options.append_parameter_override("candidate_topic", "/test_reset/cand");
  options.append_parameter_override("end_topic", "/test_reset/end");
  auto node = std::make_shared<banyu_detection::PipelineEndNode>(options);

  auto obs_pub = node->create_publisher<banyu_msgs::msg::PipelineObservation>(
    "/test_reset/obs", rclcpp::SensorDataQoS());
  auto cand_pub = node->create_publisher<std_msgs::msg::Bool>(
    "/test_reset/cand", rclcpp::QoS(10));

  std_msgs::msg::Bool cand_msg;
  cand_msg.data = true;
  cand_pub->publish(cand_msg);
  spinFor(node, 2);

  obs_pub->publish(makeObservation(1, banyu_msgs::msg::PipelineObservation::TRACKED));
  spinFor(node, 2);
  cand_pub->publish(cand_msg);
  obs_pub->publish(makeObservation(2, banyu_msgs::msg::PipelineObservation::TRACKED));
  spinFor(node, 5);

  EXPECT_TRUE(node->adapter().isConfirmed());

  // Reset node
  node->reset();
  EXPECT_FALSE(node->adapter().isConfirmed());
}

TEST_F(PipelineEndNodeTest, ExplicitFalseCandidateSuppressesDefaultCandidate)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("publish_confirmed_only", true);
  options.append_parameter_override("required_frames", 2);
  options.append_parameter_override("default_candidate", true);
  options.append_parameter_override("observation_topic", "/test_override/obs");
  options.append_parameter_override("candidate_topic", "/test_override/cand");
  options.append_parameter_override("end_topic", "/test_override/end");
  auto node = std::make_shared<banyu_detection::PipelineEndNode>(options);

  auto obs_pub = node->create_publisher<banyu_msgs::msg::PipelineObservation>(
    "/test_override/obs", rclcpp::SensorDataQoS());
  auto cand_pub = node->create_publisher<std_msgs::msg::Bool>(
    "/test_override/cand", rclcpp::QoS(10));

  // Explicit false candidate overrides default_candidate
  std_msgs::msg::Bool cand_msg;
  cand_msg.data = false;
  cand_pub->publish(cand_msg);
  spinFor(node, 2);

  obs_pub->publish(makeObservation(1, banyu_msgs::msg::PipelineObservation::TRACKED));
  spinFor(node, 2);
  cand_pub->publish(cand_msg);
  obs_pub->publish(makeObservation(2, banyu_msgs::msg::PipelineObservation::TRACKED));
  spinFor(node, 5);

  EXPECT_FALSE(node->adapter().isConfirmed());
}

}  // namespace
