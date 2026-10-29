#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "banyu_vehicle_execution/calibrated_depth_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class CalibratedDepthNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("topics.rel_alt", "/test_depth/rel_alt");
    options.append_parameter_override("topics.water_depth", "/test_depth/water_depth");
    options.append_parameter_override("topics.run_lifecycle", "/test_depth/lifecycle");
    options.append_parameter_override("topics.gps_fix", "/test_depth/fix");
    options.append_parameter_override("auto_calibrate_surface_at_start", true);

    depth_node_ = std::make_shared<banyu_vehicle_execution::CalibratedDepthNode>(options);
    test_node_ = std::make_shared<rclcpp::Node>("calibrated_depth_test_peer");

    rel_alt_pub_ = test_node_->create_publisher<std_msgs::msg::Float64>(
      "/test_depth/rel_alt", rclcpp::SensorDataQoS());
    fix_pub_ = test_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/test_depth/fix", rclcpp::SensorDataQoS());
    lifecycle_pub_ = test_node_->create_publisher<banyu_msgs::msg::RunLifecycle>(
      "/test_depth/lifecycle", rclcpp::QoS(1).reliable().transient_local());

    depth_sub_ = test_node_->create_subscription<banyu_msgs::msg::WaterDepth>(
      "/test_depth/water_depth", rclcpp::QoS(10).reliable(),
      [this](const banyu_msgs::msg::WaterDepth::SharedPtr msg) {
        received_depths_.push_back(*msg);
      });

    executor_.add_node(depth_node_);
    executor_.add_node(test_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(depth_node_);
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

  std::shared_ptr<banyu_vehicle_execution::CalibratedDepthNode> depth_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rel_alt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_pub_;
  rclcpp::Subscription<banyu_msgs::msg::WaterDepth>::SharedPtr depth_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<banyu_msgs::msg::WaterDepth> received_depths_;
};

TEST_F(CalibratedDepthNodeTest, AutoCalibratesAtStartAndCalculatesDepth)
{
  // 1. Initial rel_alt at surface before run starts
  std_msgs::msg::Float64 alt;
  alt.data = -0.05;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  // 2. Lifecycle active for run 42, epoch 1
  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 42;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  EXPECT_TRUE(depth_node_->isCalibrated());
  EXPECT_DOUBLE_EQ(depth_node_->surfaceDatumOffset(), -0.05);

  // 3. Vehicle submerges to rel_alt = -1.55 m
  received_depths_.clear();
  alt.data = -1.55;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  ASSERT_FALSE(received_depths_.empty());
  const auto last = received_depths_.back();
  EXPECT_TRUE(last.valid);
  EXPECT_TRUE(last.calibrated);
  EXPECT_EQ(last.run_id, 42u);
  EXPECT_EQ(last.epoch, 1u);
  EXPECT_EQ(last.datum, "water_surface");
  EXPECT_EQ(last.source_id, "pressure_depth");
  // Depth should be surface_datum - rel_alt = (-0.05) - (-1.55) = 1.50 m
  EXPECT_NEAR(last.depth_m, 1.50, 1e-4);
}

TEST_F(CalibratedDepthNodeTest, HandlesInvalidAltProperly)
{
  std_msgs::msg::Float64 alt;
  alt.data = std::numeric_limits<double>::quiet_NaN();
  received_depths_.clear();
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  ASSERT_FALSE(received_depths_.empty());
  EXPECT_FALSE(received_depths_.back().valid);
  EXPECT_DOUBLE_EQ(received_depths_.back().depth_m, 0.0);
}

TEST_F(CalibratedDepthNodeTest, StartupUnderwaterMustNotBecomeSurfaceDatum)
{
  spinFor(100ms);
  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 42;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  std_msgs::msg::Float64 alt;
  alt.data = -2.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  ASSERT_FALSE(received_depths_.empty());
  EXPECT_FALSE(received_depths_.back().calibrated);
}

TEST_F(CalibratedDepthNodeTest, CalibratedSurfacingFollowedByDescentAndSurfacing)
{
  spinFor(100ms);
  std_msgs::msg::Float64 alt;
  alt.data = 0.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 42;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::READY;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  EXPECT_TRUE(depth_node_->isCalibrated());

  received_depths_.clear();
  alt.data = -1.5;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);
  ASSERT_FALSE(received_depths_.empty());
  EXPECT_TRUE(received_depths_.back().calibrated);
  EXPECT_NEAR(received_depths_.back().depth_m, 1.5, 1e-3);

  received_depths_.clear();
  alt.data = 0.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);
  ASSERT_FALSE(received_depths_.empty());
  EXPECT_TRUE(received_depths_.back().calibrated);
  EXPECT_NEAR(received_depths_.back().depth_m, 0.0, 1e-3);
}

TEST_F(CalibratedDepthNodeTest, MidRunRestartUnderwaterDoesNotRecalibrate)
{
  spinFor(100ms);
  std_msgs::msg::Float64 alt;
  alt.data = 0.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 42;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::PREPARING;
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  EXPECT_TRUE(depth_node_->isCalibrated());

  alt.data = -2.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  life.run_id = 43;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  EXPECT_FALSE(depth_node_->isCalibrated());
}

TEST_F(CalibratedDepthNodeTest, PreStartDatumSurvivesSurfaceDriftAtRunStart)
{
  spinFor(100ms);
  std_msgs::msg::Float64 alt;
  alt.data = 0.25;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 42;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::READY;
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  ASSERT_TRUE(depth_node_->isCalibrated());

  // Barometer drift moves the surfaced reading out of the raw surface band before the start.
  alt.data = 0.6;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  life.run_id = 43;
  life.epoch = 2;
  life.phase = banyu_msgs::msg::RunLifecycle::INITIALIZING;
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  EXPECT_TRUE(depth_node_->isCalibrated());
  EXPECT_NEAR(depth_node_->surfaceDatumOffset(), 0.25, 1e-6);
}

TEST_F(CalibratedDepthNodeTest, ActiveHeartbeatCannotReplaceSurfaceProof)
{
  spinFor(100ms);
  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 42;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  std_msgs::msg::Float64 alt;
  alt.data = 0.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);
  ASSERT_FALSE(depth_node_->isCalibrated());
  life.header.stamp = test_node_->now();
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  EXPECT_FALSE(depth_node_->isCalibrated());
}

TEST_F(CalibratedDepthNodeTest, EpochChangeWithinSameRunPreservesCalibration)
{
  spinFor(100ms);
  std_msgs::msg::Float64 alt;
  alt.data = 0.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 42;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::PREPARING;
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  ASSERT_TRUE(depth_node_->isCalibrated());

  alt.data = -1.5;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);

  life.epoch = 2;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  EXPECT_TRUE(depth_node_->isCalibrated());
  EXPECT_DOUBLE_EQ(depth_node_->surfaceDatumOffset(), 0.0);
}

TEST_F(CalibratedDepthNodeTest, GpsFixAtTheSurfaceKeepsTheDatumThroughABaroJumpBeforeTheRun)
{
  // The barometer jumps after arming, out of the surface band; a GPS fix (the vehicle is at the surface) keeps the datum following it
  // until the run starts.
  auto publish_fix = [this]() {
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = test_node_->now();
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.latitude = 1.28;
      fix.longitude = 103.86;
      fix_pub_->publish(fix);
    };
  std_msgs::msg::Float64 alt;
  alt.data = 0.14;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);
  ASSERT_TRUE(depth_node_->isCalibrated());

  publish_fix();
  alt.data = -0.40;   // outside the band, 0.54 m below the datum
  rel_alt_pub_->publish(alt);
  spinFor(50ms);
  publish_fix();
  rel_alt_pub_->publish(alt);
  spinFor(50ms);
  EXPECT_NEAR(depth_node_->surfaceDatumOffset(), -0.40, 1e-6);

  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 7;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::INITIALIZING;
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  EXPECT_TRUE(depth_node_->isCalibrated());

  // Once the run is active the datum is not touched any more.
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);
  publish_fix();
  alt.data = -3.0;
  rel_alt_pub_->publish(alt);
  spinFor(50ms);
  EXPECT_NEAR(depth_node_->surfaceDatumOffset(), -0.40, 1e-6);
}
