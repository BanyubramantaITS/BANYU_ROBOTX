#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "banyu_communication/uuv_comm_node.hpp"
#include "banyu_mission/run_coordinator.hpp"
#include "banyu_msgs/msg/pipeline_ring_state.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "banyu_msgs/srv/trigger_pipeline_survey.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"
#include "rx_msgs/msg/rx_task.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"

using namespace std::chrono_literals;

/**
 * @brief Integration test exercising real RunCoordinator and UuvCommNode
 *        without disk WAL journals or adapter layers.
 *
 * Substituted components:
 * - FCU/sensors: WaterDepth (/banyu/vehicle/water_depth),
 *   Task2SurveyEvidence (/banyu/pipeline/survey_evidence),
 *   Course (/system/mission/course).
 * - Standard GCS: peer subscribing to public rx_msgs reports
 *   (PipelineSurvey, ResourceDelivery) and sending Command messages.
 *
 * Real components under test:
 * - banyu_mission::RunCoordinator (authoritative run snapshot, transitions)
 * - banyu_communication::UuvCommNode (survey ledger, surface guard, reporting service, direct GCS pub)
 */
class MissionReportIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    coordinator_ = std::make_unique<banyu_mission::RunCoordinator>();

    mission_node_ = std::make_shared<rclcpp::Node>("mission_integration_node");
    test_node_ = std::make_shared<rclcpp::Node>("integration_test_peer");

    lifecycle_pub_ = mission_node_->create_publisher<banyu_msgs::msg::RunLifecycle>(
      "/banyu/mission/run_lifecycle", rclcpp::QoS(10).reliable().transient_local());

    // Substituted FCU/sensor publishers on test_node_
    course_pub_ = test_node_->create_publisher<rx_msgs::msg::Course>(
      "/system/mission/course", rclcpp::QoS(10).reliable());
    evidence_pub_ = test_node_->create_publisher<banyu_msgs::msg::Task2SurveyEvidence>(
      "/banyu/pipeline/survey_evidence", rclcpp::QoS(5).reliable().transient_local());
    depth_pub_ = test_node_->create_publisher<banyu_msgs::msg::WaterDepth>(
      "/banyu/vehicle/water_depth", rclcpp::QoS(5).reliable());
    command_pub_ = test_node_->create_publisher<rx_msgs::msg::Command>(
      "/system/mission/command", rclcpp::QoS(10).reliable());

    // Continuous surface telemetry publisher timer
    depth_timer_ = test_node_->create_wall_timer(20ms, [this]() {
          if (depth_enabled_) {
            banyu_msgs::msg::WaterDepth depth;
            depth.header.stamp = test_node_->now();
            depth.run_id = active_run_id_;
            depth.epoch = active_epoch_;
            depth.depth_m = 0.05;
            depth.valid = true;
            depth.calibrated = true;
            depth.datum = "water_surface";
            depth.source_id = "pressure_depth";
            depth.calibration_id = "calib-integ-1";
            depth_pub_->publish(depth);
          }
    });

    // Mock Team GCS subscribers on standard public topics
    survey_sub_ = test_node_->create_subscription<rx_msgs::msg::PipelineSurvey>(
      "/system/vehicle/uuv/task2/pipeline_survey", rclcpp::QoS(10).reliable(),
      [this](const rx_msgs::msg::PipelineSurvey::SharedPtr msg) {
        received_surveys_.push_back(*msg);
      });
    delivery_sub_ = test_node_->create_subscription<rx_msgs::msg::ResourceDelivery>(
      "/system/vehicle/uuv/task2/delivery", rclcpp::QoS(10).reliable(),
      [this](const rx_msgs::msg::ResourceDelivery::SharedPtr msg) {
        received_deliveries_.push_back(*msg);
      });

    // Client for UUV report triggering service
    survey_client_ = test_node_->create_client<banyu_msgs::srv::TriggerPipelineSurvey>(
      "/banyu/communication/trigger_pipeline_survey");

    startCommNode();

    executor_.add_node(mission_node_);
    executor_.add_node(test_node_);

    // Publish course boundary and pinger
    rx_msgs::msg::Course course;
    course.course_id = "course-integ-1";
    course.pinger_freq_hz = 30000;
    rx_msgs::msg::LatLng pt;
    pt.latitude = 33.810;
    pt.longitude = -118.393;
    course.boundary.push_back(pt);
    // Course is volatile: publish only once uuv_comm_node's reader has matched.
    ASSERT_TRUE(spinUntil([this]() {return course_pub_->get_subscription_count() > 0;}));
    course_pub_->publish(course);

    spinFor(50ms);
  }

  void TearDown() override
  {
    depth_enabled_ = false;
    depth_timer_.reset();
    executor_.remove_node(mission_node_);
    executor_.remove_node(test_node_);
    stopCommNode();
    mission_node_.reset();
    test_node_.reset();
    coordinator_.reset();
  }

  void startCommNode()
  {
    rclcpp::NodeOptions comm_opts;
    comm_opts.parameter_overrides({
      rclcpp::Parameter("heartbeat_rate_hz", 10.0),
      rclcpp::Parameter("survey.surface_hold_s", 0.05),
      rclcpp::Parameter("survey.surface_tolerance_m", 0.2),
      rclcpp::Parameter("survey.depth_timeout_s", 0.5),
      rclcpp::Parameter("survey.task2_tier", 2),
      rclcpp::Parameter("survey.publish_legacy_copies", false),
      rclcpp::Parameter("anchor_latitude", 33.810),
      rclcpp::Parameter("anchor_longitude", -118.393),
      rclcpp::Parameter("anchor_altitude_hae_m", 0.0)
    });
    comm_node_ = std::make_shared<banyu_communication::UuvCommNode>(comm_opts);
    executor_.add_node(comm_node_);
  }

  void stopCommNode()
  {
    if (comm_node_) {
      executor_.remove_node(comm_node_);
      comm_node_.reset();
    }
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(3ms);
    }
  }

  bool spinUntil(const std::function<bool()> & pred, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(3ms);
    }
    return pred();
  }

  void publishLifecycle(
    uint8_t event = banyu_msgs::msg::RunLifecycle::SNAPSHOT,
    uint32_t requested_run_id = 0)
  {
    banyu_msgs::msg::RunLifecycle msg;
    msg.header.stamp = mission_node_->now();
    msg.header.frame_id = "uuv";
    msg.authority_instance = "real_coordinator_owner";
    msg.revision = ++lifecycle_revision_;
    auto snap = coordinator_->snapshot();
    msg.run_id = snap.run_id;
    msg.epoch = snap.epoch;
    msg.command_seq = snap.command_seq;
    msg.phase = static_cast<uint8_t>(snap.phase);
    msg.event = event;
    msg.requested_run_id = requested_run_id != 0 ? requested_run_id : snap.run_id;
    if (accepted_at_.nanoseconds() == 0 || event == banyu_msgs::msg::RunLifecycle::START_ACCEPTED) {
      accepted_at_ = mission_node_->now();
    }
    msg.accepted_at = accepted_at_;
    msg.autonomous_ready = (snap.phase >= banyu_mission::RunCoordinator::Phase::Active &&
      snap.phase <= banyu_mission::RunCoordinator::Phase::Surfaced);
    msg.current_task = msg.autonomous_ready ? rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR : 0;
    lifecycle_pub_->publish(msg);
  }

  void startRun(uint32_t run_id, uint32_t cmd_seq = 1)
  {
    depth_enabled_ = false;
    ASSERT_TRUE(coordinator_->beginPreparation());
    ASSERT_TRUE(coordinator_->markReady());
    auto start = coordinator_->requestStart(run_id, cmd_seq, true);
    ASSERT_EQ(start.disposition, banyu_mission::RunCoordinator::Disposition::Accepted);
    ASSERT_TRUE(coordinator_->completeInitialization(start.snapshot.epoch));
    active_run_id_ = run_id;
    active_epoch_ = start.snapshot.epoch;
    accepted_at_ = mission_node_->now();
    publishLifecycle(banyu_msgs::msg::RunLifecycle::START_ACCEPTED);
    spinFor(30ms);
  }

  void surfaceVehicle()
  {
    depth_enabled_ = true;
    ASSERT_TRUE(coordinator_->transition(active_epoch_,
      banyu_mission::RunCoordinator::Phase::SurveyComplete));
    publishLifecycle(banyu_msgs::msg::RunLifecycle::SNAPSHOT);
    spinFor(120ms);
    ASSERT_TRUE(coordinator_->transition(active_epoch_,
      banyu_mission::RunCoordinator::Phase::Surfaced));
    publishLifecycle(banyu_msgs::msg::RunLifecycle::SNAPSHOT);
    spinFor(50ms);
  }

  void feedEvidence()
  {
    banyu_msgs::msg::Task2SurveyEvidence ev;
    ev.header.stamp = test_node_->now();
    ev.run_id = active_run_id_;
    ev.epoch = active_epoch_;
    ev.revision = ++evidence_revision_;
    ev.source_id = "pipeline_topology";
    ev.low_progress_endpoint_id = "low_end";
    ev.high_progress_endpoint_id = "high_end";
    ev.traversal_complete = true;
    ev.tier = 2;

    ev.active_buoy.header.stamp = ev.header.stamp;
    ev.active_buoy.run_id = active_run_id_;
    ev.active_buoy.epoch = active_epoch_;
    ev.active_buoy.revision = 1;
    ev.active_buoy.source_id = buoy_source_;
    ev.active_buoy.course_id = "course-integ-1";
    ev.active_buoy.endpoint_id = "low_end";
    ev.active_buoy.fix.header.stamp = ev.header.stamp;
    ev.active_buoy.fix.status.status = 1;
    ev.active_buoy.fix.latitude = 33.8105;
    ev.active_buoy.fix.longitude = -118.3935;
    ev.active_buoy.uncertainty_m = 0.3;
    ev.active_buoy.green_confirmed = true;
    ev.active_buoy.pinger_confirmed = true;
    ev.active_buoy.pinger_freq_hz = 30000;

    const std::array<double, 4> prog{0.5, 2.5, 5.0, 7.5};
    for (uint32_t i = 0; i < 4; ++i) {
      banyu_msgs::msg::Task2RingRecord ring;
      ring.run_id = active_run_id_;
      ring.epoch = active_epoch_;
      ring.node_id = 100 + i;
      ring.frame_sequence = i + 1;
      ring.physical_identity_verified = true;
      ring.ordering_verified = true;
      ring.pipeline_progress_m = prog[i];
      ring.color = (i == 1) ? 2 : 1;
      ring.confidence = 0.95F;
      ring.vehicle_pose_map.header.stamp = ev.header.stamp;
      ring.vehicle_pose_map.header.frame_id = "map";
      ring.vehicle_pose_map.pose.position.x = prog[i];
      ring.vehicle_pose_map.pose.position.y = 0.0;
      ring.vehicle_pose_map.pose.position.z = -1.5;
      ring.vehicle_pose_map.pose.orientation.w = 1.0;
      ev.rings.push_back(ring);
    }
    evidence_pub_->publish(ev);
    spinFor(60ms);
  }

  std::unique_ptr<banyu_mission::RunCoordinator> coordinator_;
  std::shared_ptr<banyu_communication::UuvCommNode> comm_node_;
  rclcpp::Node::SharedPtr mission_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Publisher<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_pub_;

  rclcpp::Publisher<rx_msgs::msg::Course>::SharedPtr course_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2SurveyEvidence>::SharedPtr evidence_pub_;
  rclcpp::Publisher<banyu_msgs::msg::WaterDepth>::SharedPtr depth_pub_;
  rclcpp::Publisher<rx_msgs::msg::Command>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr depth_timer_;

  rclcpp::Subscription<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_sub_;
  rclcpp::Subscription<rx_msgs::msg::ResourceDelivery>::SharedPtr delivery_sub_;
  rclcpp::Client<banyu_msgs::srv::TriggerPipelineSurvey>::SharedPtr survey_client_;

  std::vector<rx_msgs::msg::PipelineSurvey> received_surveys_;
  std::vector<rx_msgs::msg::ResourceDelivery> received_deliveries_;

  uint32_t active_run_id_{0};
  uint64_t active_epoch_{0};
  rclcpp::Time accepted_at_{0, 0, RCL_ROS_TIME};
  uint64_t lifecycle_revision_{0};
  uint64_t evidence_revision_{0};
  bool depth_enabled_{false};
  std::string buoy_source_{"uuv_buoy_localizer"};
};

// Case 1: Valid start and reporting flow from real coordinator -> comm -> mock GCS
TEST_F(MissionReportIntegrationTest, ValidStartAndReportFlowReachesStandardGcs)
{
  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "real-integ-report-001";
  auto future = survey_client_->async_send_request(req);

  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  auto res = future.get();
  EXPECT_TRUE(res->success) << res->message;

  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.latitude, 33.8105);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.longitude, -118.3935);
}

// Case 2: Rejected conflicting start does not block accepted run report
TEST_F(MissionReportIntegrationTest, RejectedConflictingStartDoesNotBlockAcceptedRunReport)
{
  startRun(15, 1);

  // Conflicting start for run 99 rejected by coordinator
  auto start99 = coordinator_->requestStart(99, 2, true);
  ASSERT_EQ(start99.disposition, banyu_mission::RunCoordinator::Disposition::Rejected);
  EXPECT_EQ(coordinator_->snapshot().run_id, 15u);
  publishLifecycle(banyu_msgs::msg::RunLifecycle::START_REJECTED, 99);

  // Raw command also issued to UUV
  rx_msgs::msg::Command cmd;
  cmd.command_type = rx_msgs::msg::Command::CMD_RUN_START;
  cmd.command_seq = 2;
  cmd.run_id = 99;
  cmd.target_vehicle = rx_msgs::msg::VehicleType::TYPE_UUV;
  command_pub_->publish(cmd);
  spinFor(50ms);

  // Run 15 completes and reports
  feedEvidence();
  surfaceVehicle();

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "run15-survives-rejected-cmd";
  auto future = survey_client_->async_send_request(req);
  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  auto res = future.get();
  EXPECT_TRUE(res->success) << res->message;

  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);
}

// Case 3: Duplicate start preserves observations
TEST_F(MissionReportIntegrationTest, DuplicateStartPreservesObservations)
{
  startRun(15, 1);
  feedEvidence();

  // Duplicate start for run 15
  auto dup = coordinator_->requestStart(15, 2, true);
  ASSERT_EQ(dup.disposition, banyu_mission::RunCoordinator::Disposition::Duplicate);
  publishLifecycle(banyu_msgs::msg::RunLifecycle::START_DUPLICATE, 15);
  spinFor(50ms);

  // Comm ledger observations preserved across duplicate start
  surfaceVehicle();

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "dup-start-preserves-obs";
  auto future = survey_client_->async_send_request(req);
  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  auto res = future.get();
  EXPECT_TRUE(res->success) << res->message;

  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);
}

// Case 4: Abort before report rejects survey and prevents report
TEST_F(MissionReportIntegrationTest, AbortBeforeReportRejectsSurveyAndPreventsReport)
{
  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();

  // Abort before report
  ASSERT_TRUE(coordinator_->transition(active_epoch_,
    banyu_mission::RunCoordinator::Phase::Interrupted));
  depth_enabled_ = false;
  publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL);

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "aborted-before-seal";
  auto future = survey_client_->async_send_request(req);
  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  auto res = future.get();
  EXPECT_FALSE(res->success);
  EXPECT_TRUE(received_surveys_.empty());
}

// Case 5: Duplicate survey trigger is deduplicated
TEST_F(MissionReportIntegrationTest, DuplicateSurveyTriggerIsDeduplicated)
{
  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "first-trigger";
  auto future = survey_client_->async_send_request(req);
  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  EXPECT_TRUE(future.get()->success);

  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);

  // Second trigger for the same run
  auto req2 = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req2->run_id = 15;
  req2->epoch = active_epoch_;
  req2->report_id = "second-trigger";
  auto future2 = survey_client_->async_send_request(req2);
  ASSERT_TRUE(spinUntil([&future2]() {return future2.wait_for(0s) == std::future_status::ready;}));
  EXPECT_TRUE(future2.get()->success);
  spinFor(50ms);

  // Still only 1 survey received
  EXPECT_EQ(received_surveys_.size(), 1u);
}

// Case 6: RoboCommand "Reset run ID & seq" between runs; run 1 / seq 1 is used twice.
TEST_F(MissionReportIntegrationTest, RunIdResetAfterPreparationReportsAgain)
{
  const auto report = [this](const std::string & report_id) {
      auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
      req->run_id = 1;
      req->epoch = active_epoch_;
      req->report_id = report_id;
      auto future = survey_client_->async_send_request(req);
      EXPECT_TRUE(spinUntil([&future]() {
          return future.wait_for(0s) == std::future_status::ready;
      }));
      return future.get();
    };

  startRun(1, 1);
  const auto first_epoch = active_epoch_;
  feedEvidence();
  surfaceVehicle();
  auto first = report("task2-survey-1-" + std::to_string(first_epoch));
  ASSERT_TRUE(first->success) << first->message;
  ASSERT_TRUE(spinUntil([this]() {return received_surveys_.size() == 1u;}));
  ASSERT_TRUE(coordinator_->transition(active_epoch_,
    banyu_mission::RunCoordinator::Phase::Succeeded, "MISSION_COMPLETED"));
  depth_enabled_ = false;
  publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL);
  spinFor(50ms);

  // Next day: RoboCommand restarts at run 1, seq 1; the operator prepares the vehicle again.
  buoy_source_ = "usv_pipeline_buoys";
  startRun(1, 1);
  EXPECT_GT(active_epoch_, first_epoch);
  feedEvidence();
  surfaceVehicle();
  auto second = report("task2-survey-1-" + std::to_string(active_epoch_));
  EXPECT_TRUE(second->success) << second->message;
  EXPECT_EQ(second->result_code, banyu_msgs::srv::TriggerPipelineSurvey::Response::ACCEPTED);
  ASSERT_TRUE(spinUntil([this]() {return received_surveys_.size() == 2u;}));
}
