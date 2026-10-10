#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "banyu_communication/gcs_submission_adapter.hpp"
#include "banyu_communication/uuv_comm_node.hpp"
#include "banyu_mission/run_coordinator.hpp"
#include "banyu_msgs/msg/pipeline_ring_state.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_submission.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "banyu_msgs/srv/seal_task2_report.hpp"
#include "banyu_msgs/srv/trigger_pipeline_survey.hpp"
#include "banyu_msgs/task2_submission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"
#include "rx_msgs/msg/rx_task.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"

using namespace std::chrono_literals;

/**
 * @brief Integration test exercising real RunCoordinator, UuvCommNode, and
 *        GcsSubmissionAdapter without mocks for lifecycle or sealing.
 *
 * Substituted components:
 * - FCU/sensors: WaterDepth (/banyu/vehicle/water_depth),
 *   Task2SurveyEvidence (/banyu/pipeline/survey_evidence),
 *   Course (/system/mission/course).
 * - Standard GCS: peer subscribing to public rx_msgs reports
 *   (PipelineSurvey, ResourceDelivery) and sending Command messages.
 *
 * Real components under test:
 * - banyu_mission::RunCoordinator (authoritative run snapshot, transitions, durable seals)
 * - banyu_communication::UuvCommNode (survey ledger, surface guard, outbox, reporting service)
 * - banyu_communication::GcsSubmissionAdapter (authoritative run context, journal, dedup, public forwarding)
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
    static uint64_t test_seq = 0;
    const std::string pid_str = std::to_string(::getpid()) + "_" + std::to_string(++test_seq);
    const auto temp_dir = std::filesystem::temp_directory_path();
    coordinator_journal_path_ = (temp_dir / ("banyu_integ_coord_" + pid_str + ".journal")).string();
    outbox_path_ = (temp_dir / ("banyu_integ_outbox_" + pid_str)).string();
    adapter_journal_path_ = (temp_dir / ("banyu_integ_adapter_" + pid_str + ".journal")).string();

    cleanupFiles();

    coordinator_ = std::make_unique<banyu_mission::RunCoordinator>(coordinator_journal_path_);

    mission_node_ = std::make_shared<rclcpp::Node>("mission_integration_node");
    test_node_ = std::make_shared<rclcpp::Node>("integration_test_peer");

    lifecycle_pub_ = mission_node_->create_publisher<banyu_msgs::msg::RunLifecycle>(
      "/banyu/mission/run_lifecycle", rclcpp::QoS(10).reliable().transient_local());

    seal_service_ = mission_node_->create_service<banyu_msgs::srv::SealTask2Report>(
      "/banyu/mission/seal_task2_report",
      [this](
        const std::shared_ptr<banyu_msgs::srv::SealTask2Report::Request> req,
        std::shared_ptr<banyu_msgs::srv::SealTask2Report::Response> resp)
      {
        ++seal_calls_;
        if (defer_seal_) {
          resp->success = false;
          resp->message = "DEFERRED_SEAL";
          return;
        }
        const std::string payload = banyu_msgs::task2::canonicalPayload(req->submission);
        auto result = coordinator_->sealReport(
          req->submission.run_id, req->submission.epoch, req->submission.kind,
          req->submission.report_id, payload);
        resp->success = (result.disposition ==
        banyu_mission::RunCoordinator::Disposition::Accepted);
        resp->message = result.reason;
        if (resp->success) {
          resp->authorization_revision = ++lifecycle_revision_;
          publishLifecycle(banyu_msgs::msg::RunLifecycle::SNAPSHOT);
        }
      });

    // Substituted FCU/sensor publishers on test_node_
    course_pub_ = test_node_->create_publisher<rx_msgs::msg::Course>(
      "/system/mission/course", rclcpp::QoS(1).reliable().transient_local());
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

    // Start comm node and adapter
    startCommNode();
    startAdapterNode(true);

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
    course_pub_->publish(course);

    spinFor(50ms);
  }

  void TearDown() override
  {
    depth_enabled_ = false;
    depth_timer_.reset();
    executor_.remove_node(mission_node_);
    executor_.remove_node(test_node_);
    stopAdapterNode();
    stopCommNode();
    mission_node_.reset();
    test_node_.reset();
    coordinator_.reset();
    cleanupFiles();
  }

  void cleanupFiles()
  {
    std::error_code ec;
    std::filesystem::remove(coordinator_journal_path_, ec);
    std::filesystem::remove(coordinator_journal_path_ + ".lock", ec);
    std::filesystem::remove_all(outbox_path_, ec);
    std::filesystem::remove(adapter_journal_path_, ec);
  }

  void startCommNode()
  {
    rclcpp::NodeOptions comm_opts;
    comm_opts.parameter_overrides({
      rclcpp::Parameter("heartbeat_rate_hz", 10.0),
      rclcpp::Parameter("survey.outbox_path", outbox_path_),
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

  void startAdapterNode(bool forward)
  {
    rclcpp::NodeOptions adapter_opts;
    adapter_opts.parameter_overrides({
      rclcpp::Parameter("journal_path", adapter_journal_path_),
      rclcpp::Parameter("topics.task2_submission", "/banyu/communication/task2_submission"),
      rclcpp::Parameter("topics.run_lifecycle", "/banyu/mission/run_lifecycle"),
      rclcpp::Parameter("topics.command", "/system/mission/command"),
      rclcpp::Parameter("topics.pipeline_survey", "/system/vehicle/uuv/task2/pipeline_survey"),
      rclcpp::Parameter("topics.delivery", "/system/vehicle/uuv/task2/delivery"),
      rclcpp::Parameter("forward_to_rx_topics", forward)
    });
    adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(adapter_opts);
    executor_.add_node(adapter_node_);
  }

  void stopAdapterNode()
  {
    if (adapter_node_) {
      executor_.remove_node(adapter_node_);
      adapter_node_.reset();
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
    ev.active_buoy.source_id = "uuv_buoy_localizer";
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
  std::shared_ptr<banyu_communication::GcsSubmissionAdapter> adapter_node_;
  rclcpp::Node::SharedPtr mission_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Publisher<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_pub_;
  rclcpp::Service<banyu_msgs::srv::SealTask2Report>::SharedPtr seal_service_;

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

  std::string coordinator_journal_path_;
  std::string outbox_path_;
  std::string adapter_journal_path_;
  uint32_t active_run_id_{0};
  uint64_t active_epoch_{0};
  rclcpp::Time accepted_at_{0, 0, RCL_ROS_TIME};
  uint64_t lifecycle_revision_{0};
  uint64_t evidence_revision_{0};
  std::size_t seal_calls_{0};
  bool depth_enabled_{false};
  bool defer_seal_{false};
};

// Case 1: Valid start and reporting flow from real coordinator -> comm -> adapter -> mock GCS
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
  EXPECT_EQ(seal_calls_, 1u);
  EXPECT_TRUE(coordinator_->hasSealedReport(15, active_epoch_, 1));

  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.latitude, 33.8105);
  EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.longitude, -118.3935);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
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

  EXPECT_EQ(adapter_node_->activeRunId(), 15u);

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

// Case 4: Abort before seal rejects survey and prevents report
TEST_F(MissionReportIntegrationTest, AbortBeforeSealRejectsSurveyAndPreventsReport)
{
  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();

  // Abort before seal
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
  EXPECT_FALSE(coordinator_->hasSealedReport(15, active_epoch_, 1));
  EXPECT_TRUE(received_surveys_.empty());
}

// Case 5: Durable seal before delayed abort preserves durable report
TEST_F(MissionReportIntegrationTest, DurableSealBeforeDelayedAbortPreservesReport)
{
  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "seal-before-delayed-abort";
  auto future = survey_client_->async_send_request(req);
  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  EXPECT_TRUE(future.get()->success);
  EXPECT_TRUE(coordinator_->hasSealedReport(15, active_epoch_, 1));

  // Immediate abort after seal
  ASSERT_TRUE(coordinator_->transition(active_epoch_,
    banyu_mission::RunCoordinator::Phase::Interrupted));
  depth_enabled_ = false;
  publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL);
  spinFor(50ms);

  // Report must still have reached public subscriber
  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_TRUE(adapter_node_->hasReport("seal-before-delayed-abort"));
}

// Case 6: Adapter restart does not duplicate publication
TEST_F(MissionReportIntegrationTest, AdapterRestartDoesNotDuplicatePublication)
{
  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "restart-dedup-integ";
  auto future = survey_client_->async_send_request(req);
  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  EXPECT_TRUE(future.get()->success);

  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);

  // Restart adapter with same journal
  stopAdapterNode();
  startAdapterNode(true);
  publishLifecycle(banyu_msgs::msg::RunLifecycle::SNAPSHOT);
  spinFor(100ms);

  // No duplicate received by GCS
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->publicationState("restart-dedup-integ"),
    banyu_communication::GcsSubmissionAdapter::PublicationState::ATTEMPTED);
}

// Case 7: Disabled forwarding recovery preserves pending and recovers on restart
TEST_F(MissionReportIntegrationTest, DisabledForwardingRecoveryPreservesPendingAndRecovers)
{
  // Start with forwarding disabled
  stopAdapterNode();
  startAdapterNode(false);

  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();

  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch_;
  req->report_id = "disabled-forwarding-integ";
  auto future = survey_client_->async_send_request(req);
  ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
  EXPECT_TRUE(future.get()->success);

  spinFor(100ms);
  EXPECT_TRUE(received_surveys_.empty());
  EXPECT_EQ(adapter_node_->pendingCount(), 1u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 0u);

  // Restart adapter with forwarding enabled
  stopAdapterNode();
  startAdapterNode(true);
  publishLifecycle(banyu_msgs::msg::RunLifecycle::SNAPSHOT);

  ASSERT_TRUE(spinUntil([this]() {return !received_surveys_.empty();}));
  EXPECT_EQ(received_surveys_.size(), 1u);
  EXPECT_EQ(adapter_node_->pendingCount(), 0u);
  EXPECT_EQ(adapter_node_->attemptedCount(), 1u);
}

// Case 8: Stale epoch and delayed previous-run response rejected without corrupting active run
TEST_F(MissionReportIntegrationTest, StaleEpochAndDelayedPreviousRunResponseRejected)
{
  startRun(15, 1);
  feedEvidence();
  surfaceVehicle();
  ASSERT_TRUE(coordinator_->transition(active_epoch_,
    banyu_mission::RunCoordinator::Phase::Interrupted));
  depth_enabled_ = false;

  // Coordinator advances to run 16 (epoch 2)
  startRun(16, 2);
  spinFor(50ms);

  EXPECT_EQ(adapter_node_->activeRunId(), 16u);
  EXPECT_EQ(adapter_node_->activeEpoch(), 2u);

  // Submitting delayed report from old run 15 is rejected
  auto sub_pub = test_node_->create_publisher<banyu_msgs::msg::Task2Submission>(
    "/banyu/communication/task2_submission", rclcpp::QoS(10).reliable().transient_local());
  spinFor(50ms);

  banyu_msgs::msg::Task2Submission old_sub;
  old_sub.header.stamp = test_node_->now();
  old_sub.schema_version = 1;
  old_sub.run_id = 15;
  old_sub.epoch = 1;
  old_sub.report_id = "delayed-stale-run15";
  old_sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  old_sub.course_id = "course-integ-1";
  old_sub.tier = 2;
  old_sub.buoy_latitude = 33.8105;
  old_sub.buoy_longitude = -118.3935;
  old_sub.original_conditions = {1, 2, 1, 1};
  old_sub.authorization_revision = 1;
  sub_pub->publish(old_sub);
  spinFor(100ms);

  EXPECT_EQ(adapter_node_->rejectedCount(), 1u);
  EXPECT_FALSE(adapter_node_->hasReport("delayed-stale-run15"));
  EXPECT_EQ(adapter_node_->activeRunId(), 16u);
}
