#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "banyu_communication/pipeline_buoys_node.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"

using namespace std::chrono_literals;
using banyu_msgs::msg::RunLifecycle;
using rx_msgs::msg::PipelineBuoys;

class PipelineBuoysNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(42);
    context_->init(0, nullptr, init_options);
    rclcpp::NodeOptions options;
    options.context(context_);
    options.parameter_overrides({
      rclcpp::Parameter("publish_rate_hz", 20.0),
      rclcpp::Parameter("lock_depth_m", 1.0)});
    node_ = std::make_shared<banyu_communication::PipelineBuoysNode>(options);
    peer_ = std::make_shared<rclcpp::Node>(
      "pipeline_buoys_test_peer", rclcpp::NodeOptions().context(context_));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    executor_->add_node(peer_);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    buoys_pub_ = peer_->create_publisher<PipelineBuoys>(
      "/system/vehicle/usv/task2/pipeline_buoys", latched);
    course_pub_ = peer_->create_publisher<rx_msgs::msg::Course>(
      "/system/mission/course", latched);
    lifecycle_pub_ = peer_->create_publisher<RunLifecycle>(
      "/banyu/mission/run_lifecycle", latched);
    depth_pub_ = peer_->create_publisher<banyu_msgs::msg::WaterDepth>(
      "/banyu/vehicle/water_depth", rclcpp::QoS(5).reliable());
    target_sub_ = peer_->create_subscription<banyu_msgs::msg::Task2BuoyTarget>(
      "/banyu/task2/buoy_target", latched,
      [this](banyu_msgs::msg::Task2BuoyTarget::SharedPtr msg) {targets_.push_back(*msg);});
    fix_sub_ = peer_->create_subscription<banyu_msgs::msg::ActiveBuoyFix>(
      "/banyu/task2/active_buoy_fix", latched,
      [this](banyu_msgs::msg::ActiveBuoyFix::SharedPtr msg) {fixes_.push_back(*msg);});

    rx_msgs::msg::Course course;
    course.course_id = "ITSN-SIM-A";
    course.pinger_freq_hz = 25000;
    course_pub_->publish(course);
    spinFor(200ms);
  }

  void TearDown() override
  {
    executor_.reset();
    node_.reset();
    peer_.reset();
    context_->shutdown("test complete");
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {executor_->spin_some(10ms);}
  }

  bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout = 3s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(10ms);
      if (done()) {return true;}
    }
    return false;
  }

  PipelineBuoys buoys(uint32_t seq, uint8_t active)
  {
    PipelineBuoys value;
    value.course_id = "ITSN-SIM-A";
    value.source_vehicle = rx_msgs::msg::VehicleType::TYPE_USV;
    value.seq = seq;
    value.buoy_a.latitude = 33.810438;
    value.buoy_a.longitude = -118.393392;
    value.buoy_b.latitude = 33.810438;
    value.buoy_b.longitude = -118.393272;
    value.buoy_b_valid = true;
    value.active_buoy = active;
    return value;
  }

  void lifecycle(uint32_t run_id, uint64_t epoch, uint8_t phase)
  {
    RunLifecycle value;
    value.header.stamp = node_->now();
    value.run_id = run_id;
    value.epoch = epoch;
    value.phase = phase;
    value.revision = ++revision_;
    value.authority_instance = "test";
    lifecycle_pub_->publish(value);
  }

  void depth(uint32_t run_id, uint64_t epoch, double depth_m)
  {
    banyu_msgs::msg::WaterDepth value;
    value.header.stamp = node_->now();
    value.run_id = run_id;
    value.epoch = epoch;
    value.depth_m = depth_m;
    value.valid = true;
    depth_pub_->publish(value);
  }

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<banyu_communication::PipelineBuoysNode> node_;
  std::shared_ptr<rclcpp::Node> peer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<PipelineBuoys>::SharedPtr buoys_pub_;
  rclcpp::Publisher<rx_msgs::msg::Course>::SharedPtr course_pub_;
  rclcpp::Publisher<RunLifecycle>::SharedPtr lifecycle_pub_;
  rclcpp::Publisher<banyu_msgs::msg::WaterDepth>::SharedPtr depth_pub_;
  rclcpp::Subscription<banyu_msgs::msg::Task2BuoyTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<banyu_msgs::msg::ActiveBuoyFix>::SharedPtr fix_sub_;
  std::vector<banyu_msgs::msg::Task2BuoyTarget> targets_;
  std::vector<banyu_msgs::msg::ActiveBuoyFix> fixes_;
  uint64_t revision_{0};
};

TEST_F(PipelineBuoysNodeTest, PublishesTargetBeforeTheRunAndEvidenceDuringIt)
{
  buoys_pub_->publish(buoys(1, PipelineBuoys::ACTIVE_A));
  ASSERT_TRUE(spinUntil([this]() {return !targets_.empty();}));
  EXPECT_DOUBLE_EQ(targets_.back().longitude, -118.393392);
  EXPECT_TRUE(targets_.back().heading_valid);
  spinFor(200ms);
  EXPECT_TRUE(fixes_.empty());   // no accepted run yet

  lifecycle(4, 2, RunLifecycle::ACTIVE);
  ASSERT_TRUE(spinUntil([this]() {return fixes_.size() >= 2;}));
  const auto & fix = fixes_.back();
  EXPECT_EQ(fix.run_id, 4u);
  EXPECT_EQ(fix.epoch, 2u);
  EXPECT_TRUE(fix.green_confirmed);
  EXPECT_EQ(fix.source_id, "usv_pipeline_buoys");
  EXPECT_EQ(fix.pinger_freq_hz, 25000u);
}

TEST_F(PipelineBuoysNodeTest, DiveLocksTheBuoyUntilTheRunEnds)
{
  buoys_pub_->publish(buoys(1, PipelineBuoys::ACTIVE_A));
  lifecycle(1, 1, RunLifecycle::ACTIVE);
  ASSERT_TRUE(spinUntil([this]() {return !fixes_.empty();}));

  depth(1, 1, 0.3);   // still on the surface
  buoys_pub_->publish(buoys(2, PipelineBuoys::ACTIVE_B));
  ASSERT_TRUE(spinUntil([this]() {
      return !targets_.empty() && targets_.back().source_seq == 2u;
    }));

  depth(1, 1, 2.5);
  ASSERT_TRUE(spinUntil([this]() {return targets_.back().locked;}));
  buoys_pub_->publish(buoys(3, PipelineBuoys::ACTIVE_A));
  fixes_.clear();
  spinFor(300ms);
  ASSERT_FALSE(fixes_.empty());
  for (const auto & fix : fixes_) {
    EXPECT_DOUBLE_EQ(fix.fix.longitude, -118.393272);
                                                                                    }
  EXPECT_EQ(targets_.back().source_seq, 2u);

  lifecycle(1, 1, RunLifecycle::SUCCEEDED);
  ASSERT_TRUE(spinUntil([this]() {return !targets_.back().locked;}));
  buoys_pub_->publish(buoys(4, PipelineBuoys::ACTIVE_A));
  ASSERT_TRUE(spinUntil([this]() {return targets_.back().source_seq == 4u;}));
  fixes_.clear();
  spinFor(200ms);
  EXPECT_TRUE(fixes_.empty());   // no evidence outside a live run
}

TEST_F(PipelineBuoysNodeTest, IgnoresOtherCourseAndDepthFromOtherRuns)
{
  auto practice = buoys(1, PipelineBuoys::ACTIVE_A);
  practice.course_id = "PRACTICE";
  buoys_pub_->publish(practice);
  spinFor(300ms);
  EXPECT_TRUE(targets_.empty());

  buoys_pub_->publish(buoys(2, PipelineBuoys::ACTIVE_A));
  lifecycle(2, 5, RunLifecycle::ACTIVE);
  ASSERT_TRUE(spinUntil([this]() {return !targets_.empty();}));
  depth(1, 4, 3.0);   // stale run
  spinFor(200ms);
  EXPECT_FALSE(targets_.back().locked);
}
