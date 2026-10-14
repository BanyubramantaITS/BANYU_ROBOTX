#include "banyu_detection/ring_tracker_node.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace
{

using ResetTask2 = banyu_msgs::srv::ResetTask2;
using Repair = banyu_msgs::srv::AcknowledgeRingRepair;
using namespace std::chrono_literals;

class RingTrackerResetTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void spinFor(rclcpp::Executor & executor, std::chrono::milliseconds duration = 40ms)
  {
    const auto until = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < until) {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }
};

TEST_F(RingTrackerResetTest, ResetIsIdempotentAndTagsCurrentObservations)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("require_run_context", true);
  options.append_parameter_override("entry_confirm_frames", 1);
  options.append_parameter_override("pose_topic", "/reset_test/pose");
  options.append_parameter_override("observation_topic", "/reset_test/observation");
  options.append_parameter_override("state_topic", "/reset_test/state");
  options.append_parameter_override("accepted_ring_topic", "/reset_test/accepted");
  options.append_parameter_override("reset_service", "/reset_test/reset");
  options.append_parameter_override("repair_service", "/reset_test/repair");
  auto tracker = std::make_shared<banyu_detection::RingTrackerNode>(options);
  auto peer = std::make_shared<rclcpp::Node>("ring_tracker_reset_peer");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(peer);
  auto reset = peer->create_client<ResetTask2>("/reset_test/reset");
  auto repair = peer->create_client<Repair>("/reset_test/repair");
  auto pose_pub = peer->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/reset_test/pose", rclcpp::SensorDataQoS());
  auto obs_pub = peer->create_publisher<banyu_msgs::msg::PipelineRingObservation>(
    "/reset_test/observation", rclcpp::SensorDataQoS());
  std::vector<banyu_msgs::msg::Task2RingRecord> records;
  auto accepted_sub = peer->create_subscription<banyu_msgs::msg::Task2RingRecord>(
    "/reset_test/accepted", banyu_detection::acceptedRingQoS(),
    [&records](const banyu_msgs::msg::Task2RingRecord & record) {records.push_back(record);});
  ASSERT_TRUE(reset->wait_for_service(1s));
  ASSERT_TRUE(repair->wait_for_service(1s));
  spinFor(executor, 200ms);

  auto publishRed = [&](const rclcpp::Time & stamp, uint64_t sequence) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1.0;
      pose_pub->publish(pose);
      spinFor(executor);
      banyu_msgs::msg::PipelineRingObservation obs;
      obs.header.stamp = stamp;
      obs.frame_sequence = sequence;
      obs.visible = true;
      obs.color = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
      obs.confidence = 0.9F;
      obs.offset_x = 0.0F;
      obs.offset_y = 0.0F;
      obs_pub->publish(obs);
      spinFor(executor);
    };
  auto initialize = [&](uint32_t run_id, uint64_t epoch) {
      auto request = std::make_shared<ResetTask2::Request>();
      request->run_id = run_id;
      request->epoch = epoch;
      auto future = reset->async_send_request(request);
      const auto status = executor.spin_until_future_complete(future, 1s);
      EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
      if (status != rclcpp::FutureReturnCode::SUCCESS) {
        return std::make_shared<ResetTask2::Response>();
      }
      return future.get();
    };
  auto acknowledge = [&](uint32_t run_id, uint64_t epoch) {
      auto request = std::make_shared<Repair::Request>();
      request->ring_index = 0;
      request->run_id = run_id;
      request->epoch = epoch;
      auto future = repair->async_send_request(request);
      const auto status = executor.spin_until_future_complete(future, 1s);
      EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
      if (status != rclcpp::FutureReturnCode::SUCCESS) {
        return std::make_shared<Repair::Response>();
      }
      return future.get();
    };

  publishRed(peer->now(), 1);
  EXPECT_TRUE(records.empty());
  EXPECT_FALSE(initialize(0, 10)->success);
  ASSERT_TRUE(initialize(12, 10)->success);
  const auto old_stamp = peer->now();
  publishRed(old_stamp, 1);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_EQ(records[0].run_id, 12U);
  EXPECT_EQ(records[0].epoch, 10U);
  EXPECT_EQ(records[0].node_id, 0U);
  EXPECT_FALSE(records[0].ordering_verified);

  EXPECT_TRUE(initialize(12, 10)->success);
  EXPECT_FALSE(initialize(0, 11)->success);
  EXPECT_FALSE(initialize(15, 9)->success);
  EXPECT_FALSE(acknowledge(12, 9)->success);
  EXPECT_TRUE(acknowledge(12, 10)->success);
  publishRed(peer->now(), 1);
  EXPECT_EQ(records.size(), 1U);

  ASSERT_TRUE(initialize(3, 11)->success);
  publishRed(old_stamp, 2);
  EXPECT_EQ(records.size(), 1U);
  publishRed(peer->now(), 1);
  ASSERT_EQ(records.size(), 2U);
  EXPECT_EQ(records[1].run_id, 3U);
  EXPECT_EQ(records[1].epoch, 11U);
  EXPECT_TRUE(acknowledge(3, 11)->success);
}

}  // namespace
