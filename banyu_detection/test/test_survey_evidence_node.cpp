#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "banyu_detection/survey_evidence_node.hpp"
#include "banyu_msgs/msg/pipeline_end_state.hpp"
#include "banyu_msgs/msg/resource_code.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_ring_record.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/course.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

class SurveyEvidenceNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  virtual int tier() const {return 2;}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("tier", tier());
    options.append_parameter_override("topics.resource_code", "/test_evidence/resource_code");
    options.append_parameter_override(
      "topics.accepted_ring", "/test_evidence/accepted_ring");
    options.append_parameter_override(
      "topics.end_state", "/test_evidence/end_state");
    options.append_parameter_override(
      "topics.start_buoy_fix", "/test_evidence/start_buoy_fix");
    options.append_parameter_override(
      "topics.active_buoy_fix", "/test_evidence/active_buoy_fix");
    options.append_parameter_override(
      "topics.course", "/test_evidence/course");
    options.append_parameter_override(
      "topics.run_lifecycle", "/test_evidence/lifecycle");
    options.append_parameter_override(
      "topics.survey_evidence", "/test_evidence/evidence");
    options.append_parameter_override(
      "start_buoy_trust_raw_fix", true);

    evidence_node_ = std::make_shared<banyu_detection::SurveyEvidenceNode>(options);
    test_node_ = std::make_shared<rclcpp::Node>("survey_evidence_test_peer");

    ring_pub_ = test_node_->create_publisher<banyu_msgs::msg::Task2RingRecord>(
      "/test_evidence/accepted_ring", rclcpp::QoS(10).reliable());
    end_pub_ = test_node_->create_publisher<banyu_msgs::msg::PipelineEndState>(
      "/test_evidence/end_state", rclcpp::QoS(1).reliable().transient_local());
    buoy_pub_ = test_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/test_evidence/start_buoy_fix", rclcpp::QoS(1).reliable().transient_local());
    course_pub_ = test_node_->create_publisher<rx_msgs::msg::Course>(
      "/test_evidence/course", rclcpp::QoS(1).reliable().transient_local());
    lifecycle_pub_ = test_node_->create_publisher<banyu_msgs::msg::RunLifecycle>(
      "/test_evidence/lifecycle", rclcpp::QoS(1).reliable().transient_local());
    resource_code_pub_ = test_node_->create_publisher<banyu_msgs::msg::ResourceCode>(
      "/test_evidence/resource_code", rclcpp::QoS(1).reliable().transient_local());

    evidence_sub_ = test_node_->create_subscription<banyu_msgs::msg::Task2SurveyEvidence>(
      "/test_evidence/evidence", rclcpp::QoS(1).reliable().transient_local(),
      [this](const banyu_msgs::msg::Task2SurveyEvidence::SharedPtr msg) {
        received_evidence_.push_back(*msg);
      });

    executor_.add_node(evidence_node_);
    executor_.add_node(test_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(evidence_node_);
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

  std::shared_ptr<banyu_detection::SurveyEvidenceNode> evidence_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<banyu_msgs::msg::Task2RingRecord>::SharedPtr ring_pub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineEndState>::SharedPtr end_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr buoy_pub_;
  rclcpp::Publisher<rx_msgs::msg::Course>::SharedPtr course_pub_;
  rclcpp::Publisher<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_pub_;
  rclcpp::Publisher<banyu_msgs::msg::ResourceCode>::SharedPtr resource_code_pub_;
  rclcpp::Subscription<banyu_msgs::msg::Task2SurveyEvidence>::SharedPtr evidence_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<banyu_msgs::msg::Task2SurveyEvidence> received_evidence_;
};

TEST_F(SurveyEvidenceNodeTest, AggregatesBuoyRingsAndEndStateIntoEvidence)
{
  // 1. Publish Run Lifecycle
  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 7;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  // 2. Publish Course
  rx_msgs::msg::Course course;
  course.course_id = "course-alpha";
  course.pinger_freq_hz = 37500;
  course_pub_->publish(course);
  spinFor(50ms);

  // 3. Publish Start Buoy Fix
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = test_node_->now();
  fix.status.status = 0;
  fix.latitude = 33.810;
  fix.longitude = -118.393;
  buoy_pub_->publish(fix);
  spinFor(50ms);

  EXPECT_TRUE(evidence_node_->isBuoyReceived());
  ASSERT_FALSE(received_evidence_.empty());
  EXPECT_EQ(received_evidence_.back().active_buoy.course_id, "course-alpha");
  EXPECT_EQ(received_evidence_.back().active_buoy.pinger_freq_hz, 37500u);
  EXPECT_TRUE(received_evidence_.back().active_buoy.green_confirmed);

  // 4. Publish 4 rings
  for (uint32_t i = 1; i <= 4; ++i) {
    banyu_msgs::msg::Task2RingRecord ring;
    ring.run_id = 7;
    ring.epoch = 1;
    ring.node_id = i;
    ring.color = (i == 2) ? 1 : 2;  // Node 2 damaged (red=1)
    ring.pipeline_progress_m = i * 2.5;
    ring.physical_identity_verified = true;
    ring.ordering_verified = true;
    ring_pub_->publish(ring);
    spinFor(20ms);
  }

  EXPECT_EQ(evidence_node_->ringCount(), 4u);

  // 5. Publish Pipeline End State
  banyu_msgs::msg::PipelineEndState end_state;
  end_state.run_id = 7;
  end_state.epoch = 1;
  end_state.ended = true;
  end_pub_->publish(end_state);
  spinFor(50ms);

  EXPECT_TRUE(evidence_node_->isTraversalComplete());
  ASSERT_FALSE(received_evidence_.empty());
  const auto latest = received_evidence_.back();
  EXPECT_EQ(latest.run_id, 7u);
  EXPECT_EQ(latest.epoch, 1u);
  EXPECT_EQ(latest.rings.size(), 4u);
  EXPECT_TRUE(latest.traversal_complete);
  EXPECT_EQ(latest.tier, 2u);
}

TEST_F(SurveyEvidenceNodeTest, ConflictingMidRunCourseAndEndpointUpdatesAreRejected)
{
  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 10;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  rx_msgs::msg::Course course;
  course.course_id = "course-alpha";
  course.pinger_freq_hz = 25000;
  course_pub_->publish(course);
  spinFor(50ms);

  banyu_msgs::msg::ActiveBuoyFix buoy;
  buoy.header.stamp = test_node_->now();
  buoy.run_id = 10;
  buoy.epoch = 1;
  buoy.endpoint_id = "start_buoy";
  buoy.green_confirmed = true;
  auto active_buoy_pub = test_node_->create_publisher<banyu_msgs::msg::ActiveBuoyFix>(
    "/test_evidence/active_buoy_fix", rclcpp::QoS(1).reliable().transient_local());
  active_buoy_pub->publish(buoy);
  spinFor(50ms);

  ASSERT_FALSE(received_evidence_.empty());
  EXPECT_EQ(received_evidence_.back().active_buoy.course_id, "course-alpha");
  EXPECT_EQ(received_evidence_.back().active_buoy.endpoint_id, "start_buoy");

  // Record a ring to freeze orientation and course context
  banyu_msgs::msg::Task2RingRecord ring;
  ring.run_id = 10;
  ring.epoch = 1;
  ring.node_id = 0;
  ring.color = 2;
  ring.pipeline_progress_m = 1.0;
  ring.physical_identity_verified = true;
  ring.ordering_verified = true;
  ring_pub_->publish(ring);
  spinFor(50ms);

  // Attempt to publish conflicting course
  rx_msgs::msg::Course conflicting_course;
  conflicting_course.course_id = "course-bravo";
  conflicting_course.pinger_freq_hz = 30000;
  course_pub_->publish(conflicting_course);
  spinFor(50ms);

  // Attempt to publish conflicting endpoint
  banyu_msgs::msg::ActiveBuoyFix conflicting_buoy = buoy;
  conflicting_buoy.endpoint_id = "end_buoy";
  active_buoy_pub->publish(conflicting_buoy);
  spinFor(50ms);

  // Evidence active_buoy must retain original pinned values
  EXPECT_EQ(received_evidence_.back().active_buoy.course_id, "course-alpha");
  EXPECT_EQ(received_evidence_.back().active_buoy.endpoint_id, "start_buoy");
}

TEST_F(SurveyEvidenceNodeTest, RawStartBuoyFixDoesNotReplaceQualifiedActiveBuoy)
{
  banyu_msgs::msg::RunLifecycle life;
  life.header.stamp = test_node_->now();
  life.run_id = 11;
  life.epoch = 1;
  life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
  lifecycle_pub_->publish(life);
  spinFor(50ms);

  banyu_msgs::msg::ActiveBuoyFix buoy;
  buoy.header.stamp = test_node_->now();
  buoy.run_id = 11;
  buoy.epoch = 1;
  buoy.revision = 1;
  buoy.endpoint_id = "start_buoy";
  buoy.green_confirmed = true;
  buoy.fix.latitude = 1.0;
  buoy.fix.longitude = 2.0;
  auto active_buoy_pub = test_node_->create_publisher<banyu_msgs::msg::ActiveBuoyFix>(
    "/test_evidence/active_buoy_fix", rclcpp::QoS(1).reliable().transient_local());
  active_buoy_pub->publish(buoy);
  spinFor(50ms);
  ASSERT_FALSE(received_evidence_.empty());
  ASSERT_TRUE(received_evidence_.back().active_buoy.green_confirmed);

  sensor_msgs::msg::NavSatFix raw;
  raw.header.stamp = test_node_->now();
  raw.latitude = 5.0;
  raw.longitude = 6.0;
  buoy_pub_->publish(raw);
  spinFor(50ms);
  EXPECT_DOUBLE_EQ(received_evidence_.back().active_buoy.fix.latitude, 1.0);
  EXPECT_TRUE(received_evidence_.back().active_buoy.green_confirmed);
}

// ---- flash code and repair verification (Advanced and Disruptive tiers)

class SurveyEvidenceDisruptiveTest : public SurveyEvidenceNodeTest
{
protected:
  int tier() const override {return 4;}

  void startRunWithBuoy(uint32_t run_id)
  {
    banyu_msgs::msg::RunLifecycle life;
    life.header.stamp = test_node_->now();
    life.run_id = run_id;
    life.epoch = 1;
    life.phase = banyu_msgs::msg::RunLifecycle::ACTIVE;
    lifecycle_pub_->publish(life);
    spinFor(50ms);
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = test_node_->now();
    fix.status.status = 0;
    fix.latitude = 33.810;
    fix.longitude = -118.393;
    buoy_pub_->publish(fix);
    spinFor(50ms);
  }

  banyu_msgs::msg::ResourceCode code(uint32_t run_id, bool complete, uint8_t tin, uint8_t circle)
  {
    banyu_msgs::msg::ResourceCode msg;
    msg.header.stamp = test_node_->now();
    msg.run_id = run_id;
    msg.epoch = 1;
    msg.repair_verified = true;
    msg.complete = complete;
    msg.resource_color = tin;
    msg.delivery_circle_color = circle;
    return msg;
  }
};

TEST_F(SurveyEvidenceDisruptiveTest, CompleteCodeFillsRepairAndResourceFields)
{
  startRunWithBuoy(20);
  resource_code_pub_->publish(code(20, true, 1, 3));   // red tin, blue circle
  spinFor(80ms);
  ASSERT_FALSE(received_evidence_.empty());
  const auto & e = received_evidence_.back();
  EXPECT_EQ(e.tier, 4u);
  EXPECT_TRUE(e.repair_verified);
  EXPECT_TRUE(e.resource_code_complete);
  EXPECT_EQ(e.resource_color, 1u);
  EXPECT_EQ(e.delivery_circle_color, 3u);
}

TEST_F(SurveyEvidenceDisruptiveTest, IncompleteCodeVerifiesRepairButNotTheCode)
{
  startRunWithBuoy(21);
  resource_code_pub_->publish(code(21, false, 0, 0));
  spinFor(80ms);
  ASSERT_FALSE(received_evidence_.empty());
  const auto & e = received_evidence_.back();
  EXPECT_TRUE(e.repair_verified);
  EXPECT_FALSE(e.resource_code_complete);
}

TEST_F(SurveyEvidenceDisruptiveTest, CodeFromAnotherRunIsIgnored)
{
  startRunWithBuoy(22);
  const auto before = received_evidence_.size();
  resource_code_pub_->publish(code(99, true, 1, 3));
  spinFor(80ms);
  EXPECT_EQ(received_evidence_.size(), before);
  EXPECT_FALSE(received_evidence_.back().resource_code_complete);
}

TEST_F(SurveyEvidenceDisruptiveTest, DisruptiveNeedsATinColour)
{
  startRunWithBuoy(23);
  resource_code_pub_->publish(code(23, true, 0, 2));   // complete but no tin
  spinFor(80ms);
  EXPECT_FALSE(received_evidence_.back().resource_code_complete);
}

class SurveyEvidenceAdvancedTest : public SurveyEvidenceDisruptiveTest
{
protected:
  int tier() const override {return 3;}
};

TEST_F(SurveyEvidenceAdvancedTest, AdvancedReportsAnyTinAndTheCircle)
{
  startRunWithBuoy(24);
  resource_code_pub_->publish(code(24, true, 0, 2));   // circle only: green
  spinFor(80ms);
  const auto & e = received_evidence_.back();
  EXPECT_EQ(e.tier, 3u);
  EXPECT_TRUE(e.resource_code_complete);
  EXPECT_EQ(e.resource_color, 4u);          // COLOR_ANY, as the report ledger requires at tier 3
  EXPECT_EQ(e.delivery_circle_color, 2u);
}

TEST_F(SurveyEvidenceNodeTest, CoreTierIgnoresTheFlashCode)
{
  banyu_msgs::msg::ResourceCode msg;
  msg.run_id = 0;
  msg.complete = true;
  msg.resource_color = 1;
  msg.delivery_circle_color = 2;
  resource_code_pub_->publish(msg);
  spinFor(80ms);
  for (const auto & e : received_evidence_) {
    EXPECT_FALSE(e.resource_code_complete);
    EXPECT_FALSE(e.repair_verified);
  }
}
