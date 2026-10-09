#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <unistd.h>

#include "banyu_communication/uuv_comm_node.hpp"
#include "banyu_msgs/msg/pipeline_ring_state.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_submission.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "banyu_msgs/msg/task2_survey_progress.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "banyu_msgs/srv/seal_task2_report.hpp"
#include "banyu_msgs/srv/trigger_pipeline_survey.hpp"
#include "rx_msgs/msg/color.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rx_msgs/msg/mission_status.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"
#include "rx_msgs/msg/rx_task.hpp"
#include "rx_msgs/msg/task_tier.hpp"

namespace
{
using namespace std::chrono_literals;
using Lifecycle = banyu_msgs::msg::RunLifecycle;
using Evidence = banyu_msgs::msg::Task2SurveyEvidence;
using Progress = banyu_msgs::msg::Task2SurveyProgress;
using Submission = banyu_msgs::msg::Task2Submission;
using Depth = banyu_msgs::msg::WaterDepth;
using Survey = banyu_msgs::srv::TriggerPipelineSurvey;
using Seal = banyu_msgs::srv::SealTask2Report;
using SurveyFuture = rclcpp::Client<Survey>::FutureAndRequestId;
using Tier = rx_msgs::msg::TaskTier;
using Color = rx_msgs::msg::Color;

class SurveyReportingTest : public ::testing::Test
{
protected:
  struct PendingSeal
  {
    std::shared_ptr<rclcpp::Service<Seal>> service;
    std::shared_ptr<rmw_request_id_t> header;
    std::shared_ptr<Seal::Request> request;
    uint64_t issued_revision;
  };

  void SetUp() override
  {
    std::string pattern = (std::filesystem::temp_directory_path() /
      "banyu-survey-integration-XXXXXX").string();
    std::vector<char> directory(pattern.begin(), pattern.end());
    directory.push_back('\0');
    ASSERT_NE(::mkdtemp(directory.data()), nullptr);
    directory_ = directory.data();
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.set_domain_id(42);
    context_->init(0, nullptr, init);
    peer_ = std::make_shared<rclcpp::Node>(
      "survey_reporting_fixture", rclcpp::NodeOptions().context(context_));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(peer_);
    lifecycle_pub_ = peer_->create_publisher<Lifecycle>(
      "/banyu/mission/run_lifecycle", rclcpp::QoS(1).reliable().transient_local());
    evidence_pub_ = peer_->create_publisher<Evidence>(
      "/banyu/pipeline/survey_evidence", rclcpp::QoS(5).reliable().transient_local());
    depth_pub_ = peer_->create_publisher<Depth>(
      "/banyu/vehicle/water_depth", rclcpp::QoS(5).reliable());
    course_pub_ = peer_->create_publisher<rx_msgs::msg::Course>(
      "/system/mission/course", rclcpp::QoS(1).reliable().transient_local());
    progress_sub_ = peer_->create_subscription<Progress>(
      "/banyu/communication/survey_progress", rclcpp::QoS(5).reliable().transient_local(),
      [this](Progress::SharedPtr value) {progress_.push_back(*value);});
    submission_sub_ = peer_->create_subscription<Submission>(
      "/system/vehicle/uuv/task2/submission", rclcpp::QoS(10).reliable().transient_local(),
      [this](Submission::SharedPtr value) {submissions_.push_back(*value);});
    legacy_survey_sub_ = peer_->create_subscription<rx_msgs::msg::PipelineSurvey>(
      "/system/vehicle/uuv/task2/pipeline_survey", rclcpp::QoS(10).reliable().transient_local(),
      [this](rx_msgs::msg::PipelineSurvey::SharedPtr value) {legacy_surveys_.push_back(*value);});
    legacy_delivery_sub_ = peer_->create_subscription<rx_msgs::msg::ResourceDelivery>(
      "/system/vehicle/uuv/task2/delivery", rclcpp::QoS(10).reliable().transient_local(),
      [this](rx_msgs::msg::ResourceDelivery::SharedPtr value) {
        legacy_deliveries_.push_back(*value);
                                                                                                    });
    status_sub_ = peer_->create_subscription<rx_msgs::msg::MissionStatus>(
      "/system/mission/status", 10,
      [this](rx_msgs::msg::MissionStatus::SharedPtr value) {statuses_.push_back(*value);});
    seal_server_ = peer_->create_service<Seal>(
      "/banyu/mission/seal_task2_report",
      [this](std::shared_ptr<rclcpp::Service<Seal>> service,
      std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Seal::Request> request) {
        ++seal_requests_;
        pending_seals_.push_back({service, header, request, lifecycle_.revision});
        if (!defer_seal_) {resolveSeals();}
      });
    survey_client_ = peer_->create_client<Survey>("/banyu/communication/trigger_pipeline_survey");
    refresh_timer_ = peer_->create_wall_timer(25ms, [this]() {
          if (has_run_) {refreshLifecycle();}
          if (depth_enabled_) {publishDepth(depth_template_);}
      });
    node_ = std::make_shared<banyu_communication::UuvCommNode>(nodeOptions());
    executor_->add_node(node_);
    ASSERT_TRUE(waitForDiscovery());
    course_.course_id = "course-observed-a";
    course_.pinger_freq_hz = 30000;
    for (const auto & point : std::vector<std::pair<double, double>>{
        {1.0, 2.0}, {1.02, 2.0}, {1.02, 2.02}, {1.0, 2.02}, {1.0, 2.0}})
    {
      rx_msgs::msg::LatLng coordinate;
      coordinate.latitude = point.first;
      coordinate.longitude = point.second;
      course_.boundary.push_back(coordinate);
    }
    course_pub_->publish(course_);
    spinFor(30ms);
  }

  void TearDown() override
  {
    pending_seals_.clear();
    refresh_timer_.reset();
    if (executor_ && node_) {executor_->remove_node(node_);}
    node_.reset();
    executor_.reset();
    peer_.reset();
    context_->shutdown("survey fixture finished");
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
    std::filesystem::remove_all(directory_.string() + ".moved", error);
  }

  rclcpp::NodeOptions nodeOptions()
  {
    rclcpp::NodeOptions options;
    options.context(context_);
    options.parameter_overrides({
        rclcpp::Parameter("heartbeat_rate_hz", 10.0),
        rclcpp::Parameter("survey.outbox_path", (directory_ / "outbox").string()),
        rclcpp::Parameter("survey.surface_hold_s", 0.15),
        rclcpp::Parameter("survey.surface_tolerance_m", 0.2),
        rclcpp::Parameter("survey.depth_timeout_s", 0.2),
        rclcpp::Parameter("survey.task2_tier", static_cast<int64_t>(tier_)),
        rclcpp::Parameter("survey.publish_legacy_copies", legacy_copies_)});
    return options;
  }

  bool spinUntil(const std::function<bool()> & condition, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      executor_->spin_some();
      if (condition()) {return true;}
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return condition();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    (void)spinUntil([]() {return false;}, duration);
  }

  bool waitForDiscovery()
  {
    return survey_client_->wait_for_service(2s) && spinUntil([this]() {
               return lifecycle_pub_->get_subscription_count() > 0 &&
                      evidence_pub_->get_subscription_count() > 0 &&
                      depth_pub_->get_subscription_count() > 0 &&
                      course_pub_->get_subscription_count() > 0;
      });
  }

  void refreshLifecycle()
  {
    lifecycle_.header.stamp = peer_->now();
    lifecycle_.revision = ++lifecycle_revision_;
    lifecycle_pub_->publish(lifecycle_);
    lifecycle_.event = Lifecycle::SNAPSHOT;
  }

  void phase(uint8_t phase_value)
  {
    lifecycle_.phase = phase_value;
    lifecycle_.current_task = phase_value >= Lifecycle::ACTIVE &&
      phase_value <= Lifecycle::SURFACED ?
      rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR : rx_msgs::msg::RxTask::TASK_NONE;
    if (phase_value == Lifecycle::FAILED || phase_value == Lifecycle::INTERRUPTED ||
      phase_value == Lifecycle::SUCCEEDED)
    {lifecycle_.event = Lifecycle::TERMINAL;}
    refreshLifecycle();
    spinFor(35ms);
  }

  void beginRun(uint32_t run_id = 17, uint64_t epoch = 1)
  {
    has_run_ = true;
    depth_enabled_ = false;
    lifecycle_.run_id = run_id;
    lifecycle_.epoch = epoch;
    lifecycle_.command_seq = static_cast<uint32_t>(epoch * 10);
    lifecycle_.accepted_at = peer_->now();
    lifecycle_.authority_instance = "observed-mission-authority";
    lifecycle_.autonomous_ready = true;
    lifecycle_.event = Lifecycle::START_ACCEPTED;
    phase(Lifecycle::INITIALIZING);
    phase(Lifecycle::ACTIVE);
  }

  Evidence evidence(uint32_t red_index = 1, bool high_endpoint = false)
  {
    Evidence value;
    value.header.stamp = peer_->now();
    value.run_id = lifecycle_.run_id;
    value.epoch = lifecycle_.epoch;
    value.source_id = "pipeline_topology";
    value.low_progress_endpoint_id = "observed-low-end";
    value.high_progress_endpoint_id = "observed-high-end";
    value.traversal_complete = true;
    value.tier = tier_;
    auto & buoy = value.active_buoy;
    buoy.header.stamp = peer_->now();
    buoy.run_id = value.run_id;
    buoy.epoch = value.epoch;
    buoy.revision = 1;
    buoy.source_id = "uuv_buoy_localizer";
    buoy.course_id = course_.course_id;
    buoy.endpoint_id =
      high_endpoint ? value.high_progress_endpoint_id : value.low_progress_endpoint_id;
    buoy.fix.header.stamp = buoy.header.stamp;
    buoy.fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    buoy.fix.latitude = 1.01;
    buoy.fix.longitude = 2.01;
    buoy.uncertainty_m = 0.4;
    buoy.green_confirmed = true;
    buoy.pinger_confirmed = true;
    buoy.pinger_freq_hz = course_.pinger_freq_hz;
    const std::array<double, 4> progression{0.6, 2.7, 7.1, 8.3};
    const std::array<std::pair<double, double>, 4> vehicle_positions{{
      {0.6, 0.0}, {2.7, 0.0}, {2.7, 0.3}, {0.7, 0.3}}};
    for (uint32_t i = 0; i < 4; ++i) {
      banyu_msgs::msg::Task2RingRecord ring;
      ring.run_id = value.run_id;
      ring.epoch = value.epoch;
      ring.node_id = 100 + i;
      ring.frame_sequence = 1 + i;
      ring.physical_identity_verified = true;
      ring.ordering_verified = true;
      ring.pipeline_progress_m = progression[i];
      ring.color = i == red_index ? 1 : 2;
      ring.confidence = 0.9F;
      ring.vehicle_pose_map.header.stamp = peer_->now();
      ring.vehicle_pose_map.header.frame_id = "map";
      ring.vehicle_pose_map.pose.position.x = vehicle_positions[i].first;
      ring.vehicle_pose_map.pose.position.y = vehicle_positions[i].second;
      ring.vehicle_pose_map.pose.position.z = -2.0;
      ring.vehicle_pose_map.pose.orientation.w = 1.0;
      value.rings.push_back(ring);
    }
    return value;
  }

  void publishEvidence(Evidence & value, bool expect_complete = true)
  {
    value.header.stamp = peer_->now();
    value.revision = ++evidence_revision_;
    const auto before = progress_.size();
    evidence_pub_->publish(value);
    ASSERT_TRUE(spinUntil([this, before]() {return progress_.size() > before;}));
    if (expect_complete) {
      ASSERT_TRUE(spinUntil([this, &value]() {
          return !progress_.empty() && progress_.back().survey_complete &&
                 progress_.back().evidence_revision == value.revision;
        })) << (progress_.empty() ? "missing progress" : progress_.back().reason);
    }
    spinFor(20ms);
  }

  Depth depth()
  {
    Depth value;
    value.header.stamp = peer_->now();
    value.run_id = lifecycle_.run_id;
    value.epoch = lifecycle_.epoch;
    value.depth_m = 0.0;
    value.valid = true;
    value.calibrated = true;
    value.datum = "water_surface";
    value.source_id = "pressure_depth";
    value.calibration_id = "measured-water-datum-fixture";
    return value;
  }

  void publishDepth(Depth value, bool refresh_stamp = true)
  {
    if (refresh_stamp) {value.header.stamp = peer_->now();}
    depth_pub_->publish(value);
  }

  void confirmSurface()
  {
    phase(Lifecycle::SURVEY_COMPLETE);
    depth_template_ = depth();
    depth_enabled_ = true;
    spinFor(230ms);
    phase(Lifecycle::SURFACED);
  }

  SurveyFuture requestAsync(std::string report_id = {}, uint32_t run_id = 0, uint64_t epoch = 0)
  {
    auto request = std::make_shared<Survey::Request>();
    request->run_id = run_id == 0 ? lifecycle_.run_id : run_id;
    request->epoch = epoch == 0 ? lifecycle_.epoch : epoch;
    request->report_id = report_id.empty() ? "observed-report-" + std::to_string(request->run_id) +
      "-" + std::to_string(request->epoch) : std::move(report_id);
    return survey_client_->async_send_request(request);
  }

  std::shared_ptr<Survey::Response> finish(SurveyFuture & future)
  {
    const bool done = spinUntil([&future]() {
          return future.wait_for(0ms) == std::future_status::ready;
                                                                                                      });
    EXPECT_TRUE(done);
    return done ? future.get() : nullptr;
  }

  std::shared_ptr<Survey::Response> request(
    std::string report_id = {}, uint32_t run_id = 0,
    uint64_t epoch = 0)
  {
    auto future = requestAsync(std::move(report_id), run_id, epoch);
    return finish(future);
  }

  void resolveSeals(bool stale_success = false)
  {
    auto pending = std::move(pending_seals_);
    pending_seals_.clear();
    for (auto & item : pending) {
      Seal::Response response;
      const auto & payload = item.request->submission;
      const auto key = std::make_tuple(payload.run_id, payload.epoch, payload.kind);
      const bool matching_run = payload.run_id == lifecycle_.run_id &&
        payload.epoch == lifecycle_.epoch;
      const bool same_identity = sealed_ids_.count(key) == 0 ||
        sealed_ids_[key] == payload.report_id;
      response.success = stale_success ||
        (matching_run && lifecycle_.phase == Lifecycle::SURFACED && same_identity);
      response.authorization_revision = stale_success ? item.issued_revision : lifecycle_.revision;
      response.message = response.success ? "mission authorization only; no GCS receipt" :
        "mission phase, epoch or report identity no longer eligible";
      if (response.success) {sealed_ids_[key] = payload.report_id;}
      item.service->send_response(*item.header, response);
    }
  }

  void restartNode()
  {
    executor_->remove_node(node_);
    node_.reset();
    node_ = std::make_shared<banyu_communication::UuvCommNode>(nodeOptions());
    executor_->add_node(node_);
    ASSERT_TRUE(waitForDiscovery());
    course_pub_->publish(course_);
    if (has_run_) {refreshLifecycle();}
    spinFor(60ms);
  }

  std::vector<Submission> surveys() const
  {
    std::vector<Submission> result;
    for (const auto & item : submissions_) {
      if (item.kind == Submission::SURVEY) {result.push_back(item);}
    }
    return result;
  }

  void expectRejected(const std::shared_ptr<Survey::Response> & response)
  {
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success) << response->message;
    EXPECT_FALSE(response->message.empty());
    EXPECT_TRUE(submissions_.empty());
    EXPECT_TRUE(legacy_surveys_.empty());
    EXPECT_TRUE(legacy_deliveries_.empty());
  }

  std::filesystem::path directory_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr peer_;
  std::shared_ptr<banyu_communication::UuvCommNode> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<Lifecycle>::SharedPtr lifecycle_pub_;
  rclcpp::Publisher<Evidence>::SharedPtr evidence_pub_;
  rclcpp::Publisher<Depth>::SharedPtr depth_pub_;
  rclcpp::Publisher<rx_msgs::msg::Course>::SharedPtr course_pub_;
  rclcpp::Subscription<Progress>::SharedPtr progress_sub_;
  rclcpp::Subscription<Submission>::SharedPtr submission_sub_;
  rclcpp::Subscription<rx_msgs::msg::PipelineSurvey>::SharedPtr legacy_survey_sub_;
  rclcpp::Subscription<rx_msgs::msg::ResourceDelivery>::SharedPtr legacy_delivery_sub_;
  rclcpp::Subscription<rx_msgs::msg::MissionStatus>::SharedPtr status_sub_;
  rclcpp::Service<Seal>::SharedPtr seal_server_;
  rclcpp::Client<Survey>::SharedPtr survey_client_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  Lifecycle lifecycle_;
  rx_msgs::msg::Course course_;
  Depth depth_template_;
  bool has_run_{false};
  bool depth_enabled_{false};
  bool defer_seal_{false};
  bool legacy_copies_{false};
  uint8_t tier_{Tier::TIER_CORE};
  uint64_t lifecycle_revision_{0};
  uint64_t evidence_revision_{0};
  std::size_t seal_requests_{0};
  std::vector<Progress> progress_;
  std::vector<Submission> submissions_;
  std::vector<rx_msgs::msg::PipelineSurvey> legacy_surveys_;
  std::vector<rx_msgs::msg::ResourceDelivery> legacy_deliveries_;
  std::vector<rx_msgs::msg::MissionStatus> statuses_;
  std::vector<PendingSeal> pending_seals_;
  std::map<std::tuple<uint32_t, uint64_t, uint8_t>, std::string> sealed_ids_;
};
}  // namespace

TEST_F(SurveyReportingTest, MissingObservationsNeverCreateFallbackPayload)
{
  beginRun();
  confirmSurface();
  const auto response = request();
  ASSERT_NO_FATAL_FAILURE(expectRejected(response));
  EXPECT_EQ(response->result_code, Survey::Response::SURVEY_INVALID);
  EXPECT_EQ(seal_requests_, 0U);
}

TEST_F(SurveyReportingTest, IncompleteAndUnverifiedOrderingDoNotPublish)
{
  beginRun();
  auto value = evidence();
  value.rings.pop_back();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  confirmSurface();
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  value = evidence();
  value.rings[2].ordering_verified = false;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  EXPECT_EQ(seal_requests_, 0U);
}

TEST_F(SurveyReportingTest, AllDamagedPositionsAndBothActiveEndpointsFollowPipeProgression)
{
  uint64_t epoch = 0;
  for (const bool high : {false, true}) {
    for (uint32_t red = 0; red < 4; ++red) {
      ++epoch;
      beginRun(static_cast<uint32_t>(100 + epoch), epoch);
      auto value = evidence(red, high);
      ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
      confirmSurface();
      const auto before = surveys().size();
      const auto response = request();
      ASSERT_NE(response, nullptr);
      ASSERT_TRUE(response->success) << response->message;
      ASSERT_TRUE(spinUntil([this, before]() {return surveys().size() > before;}));
      const auto payload = surveys().back();
      std::array<uint8_t, 4> expected{1, 1, 1, 1};
      expected[high ? 3 - red : red] = 2;
      EXPECT_EQ(payload.original_conditions, expected);
      EXPECT_EQ(payload.run_id, lifecycle_.run_id);
      EXPECT_EQ(payload.epoch, epoch);
      EXPECT_DOUBLE_EQ(payload.buoy_latitude, value.active_buoy.fix.latitude);
      EXPECT_DOUBLE_EQ(payload.buoy_longitude, value.active_buoy.fix.longitude);
      EXPECT_EQ(payload.course_id, course_.course_id);
      EXPECT_GT(payload.authorization_revision, 0U);
      EXPECT_EQ(lifecycle_.phase, Lifecycle::SURFACED);
      EXPECT_TRUE(legacy_surveys_.empty());
      phase(Lifecycle::SUCCEEDED);
    }
  }
}

TEST_F(SurveyReportingTest, BlueFirstObservationAndDuplicatePhysicalIdentityAreRejected)
{
  beginRun();
  auto value = evidence();
  value.rings[0].color = 3;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  confirmSurface();
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  value = evidence();
  value.rings[1].node_id = value.rings[0].node_id;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  value = evidence();
  value.rings[1].physical_identity_verified = false;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
}

TEST_F(SurveyReportingTest, BuoyFrequencyCourseAndEndpointProofAreRequired)
{
  beginRun();
  confirmSurface();
  for (int invalid = 0; invalid < 4; ++invalid) {
    auto value = evidence();
    if (invalid == 0) {
      value.active_buoy.green_confirmed = value.active_buoy.pinger_confirmed = false;
    }
    if (invalid == 1) {value.active_buoy.course_id = "another-course";}
    if (invalid == 2) {value.active_buoy.pinger_freq_hz += 1000;}
    if (invalid == 3) {value.active_buoy.endpoint_id = "unlocalized-end";}
    ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
    ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  }
  EXPECT_EQ(seal_requests_, 0U);
}

TEST_F(SurveyReportingTest, OriginalDamagedConditionSurvivesRepairAndOpticalColors)
{
  beginRun();
  auto value = evidence(2);
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  value.repair_verified = true;
  value.rings[2].color = 2;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  value.rings[2].color = 3;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  const auto response = request();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(spinUntil([this]() {return surveys().size() == 1;}));
  EXPECT_EQ(surveys().back().original_conditions, (std::array<uint8_t, 4>{1, 1, 2, 1}));
}

TEST_F(SurveyReportingTest, AdvancedRepairAndDecodedResourceAreSeparateMandatoryEvidence)
{
  tier_ = Tier::TIER_ADVANCED;
  ASSERT_NO_FATAL_FAILURE(restartNode());
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  confirmSurface();
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  value.repair_verified = true;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  value.resource_code_complete = true;
  value.resource_color = Color::COLOR_ANY;
  value.delivery_circle_color = Color::COLOR_BLUE;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  const auto response = request();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(spinUntil([this]() {return submissions_.size() == 2;}));
  EXPECT_EQ(surveys().size(), 1U);
  const auto & delivery = submissions_[0].kind == Submission::DELIVERY ?
    submissions_[0] : submissions_[1];
  EXPECT_EQ(delivery.kind, Submission::DELIVERY);
  EXPECT_EQ(delivery.resource_color, Color::COLOR_ANY);
  EXPECT_EQ(delivery.delivery_circle_color, Color::COLOR_BLUE);
  EXPECT_EQ(delivery.run_id, 17U);
  EXPECT_EQ(delivery.epoch, 1U);
  EXPECT_TRUE(legacy_deliveries_.empty());
}

TEST_F(SurveyReportingTest, DisruptiveDeliveryUsesBothObservedColors)
{
  tier_ = Tier::TIER_DISRUPTIVE;
  ASSERT_NO_FATAL_FAILURE(restartNode());
  beginRun();
  auto value = evidence();
  value.repair_verified = true;
  value.resource_code_complete = true;
  value.resource_color = Color::COLOR_RED;
  value.delivery_circle_color = Color::COLOR_GREEN;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  const auto response = request();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(spinUntil([this]() {return submissions_.size() == 2;}));
  for (const auto & item : submissions_) {
    if (item.kind == Submission::DELIVERY) {
      EXPECT_EQ(item.resource_color, Color::COLOR_RED);
      EXPECT_EQ(item.delivery_circle_color, Color::COLOR_GREEN);
    }
  }
}

TEST_F(SurveyReportingTest, WrongRunStaleEpochAndUntrustedSnapshotCannotEnterCurrentReport)
{
  beginRun(18, 2);
  confirmSurface();
  auto value = evidence();
  value.run_id = 17;
  value.epoch = 1;
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  value = evidence();
  value.source_id = "unconfigured-topology-source";
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value, false));
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  const auto wrong_request = request("old-epoch-request", 17, 1);
  ASSERT_NO_FATAL_FAILURE(expectRejected(wrong_request));
  EXPECT_EQ(wrong_request->result_code, Survey::Response::WRONG_RUN);
}

TEST_F(SurveyReportingTest, SurveyAndSurfaceMilestonesGuardPublicationBeforeBtSuccess)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  phase(Lifecycle::SURVEY_COMPLETE);
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  depth_template_ = depth();
  depth_enabled_ = true;
  spinFor(230ms);
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  phase(Lifecycle::SURFACED);
  const auto response = request();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  EXPECT_EQ(lifecycle_.phase, Lifecycle::SURFACED);
}

TEST_F(SurveyReportingTest, CalibratedContinuousSurfaceEvidenceIsRequired)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  phase(Lifecycle::SURVEY_COMPLETE);
  phase(Lifecycle::SURFACED);
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  publishDepth(depth());
  spinFor(40ms);
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  depth_template_ = depth();
  depth_template_.calibrated = false;
  depth_enabled_ = true;
  spinFor(230ms);
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  depth_template_.calibrated = true;
  spinFor(230ms);
  const auto response = request();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
}

TEST_F(SurveyReportingTest, SubmergenceBreaksContinuousSurfaceHold)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  phase(Lifecycle::SURVEY_COMPLETE);
  phase(Lifecycle::SURFACED);
  depth_template_ = depth();
  depth_enabled_ = true;
  spinFor(70ms);
  depth_template_.depth_m = 2.0;
  spinFor(35ms);
  depth_template_.depth_m = 0.0;
  spinFor(70ms);
  ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  spinFor(180ms);
  const auto response = request();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
}

TEST_F(SurveyReportingTest, InvalidDatumSourceCalibrationAndTimestampsCannotProveSurface)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  phase(Lifecycle::SURVEY_COMPLETE);
  phase(Lifecycle::SURFACED);
  for (int invalid = 0; invalid < 7; ++invalid) {
    auto sample = depth();
    if (invalid == 0) {sample.datum = "map-origin";}
    if (invalid == 1) {sample.source_id = "unconfigured-depth";}
    if (invalid == 2) {sample.calibration_id.clear();}
    if (invalid == 3) {sample.depth_m = std::numeric_limits<double>::quiet_NaN();}
    if (invalid == 4) {sample.header.stamp = builtin_interfaces::msg::Time{};}
    if (invalid == 5) {sample.header.stamp = peer_->now() + rclcpp::Duration::from_seconds(1.0);}
    if (invalid == 6) {sample.header.stamp = peer_->now() - rclcpp::Duration::from_seconds(1.0);}
    publishDepth(sample, false);
    spinFor(30ms);
    ASSERT_NO_FATAL_FAILURE(expectRejected(request()));
  }
}

TEST_F(SurveyReportingTest, RepeatedRequestReturnsLocalAcceptanceWithoutAnotherLogicalReport)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  const auto first = request();
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->success) << first->message;
  ASSERT_TRUE(spinUntil([this]() {return surveys().size() == 1;}));
  const auto original = surveys().back();
  const auto second = request();
  ASSERT_NE(second, nullptr);
  EXPECT_TRUE(second->success) << second->message;
  EXPECT_EQ(second->result_code, Survey::Response::ALREADY_ACCEPTED);
  spinFor(50ms);
  EXPECT_EQ(surveys().size(), 1U);
  EXPECT_EQ(surveys().back().report_id, original.report_id);
  EXPECT_TRUE(legacy_surveys_.empty());
}

TEST_F(SurveyReportingTest, ConcurrentDuplicateRequestsPublishOneLogicalSurvey)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  defer_seal_ = true;
  auto first = requestAsync();
  auto second = requestAsync();
  ASSERT_TRUE(spinUntil([this]() {return !pending_seals_.empty();}));
  spinFor(40ms);
  EXPECT_TRUE(submissions_.empty());
  resolveSeals();
  defer_seal_ = false;
  const auto first_result = finish(first);
  const auto second_result = finish(second);
  ASSERT_NE(first_result, nullptr);
  ASSERT_NE(second_result, nullptr);
  EXPECT_TRUE(first_result->success) << first_result->message;
  EXPECT_TRUE(second_result->success) << second_result->message;
  ASSERT_TRUE(spinUntil([this]() {return surveys().size() == 1;}));
  spinFor(50ms);
  EXPECT_EQ(surveys().size(), 1U);
}

TEST_F(SurveyReportingTest, AbortBeforeSealAuthorizationPublishesNothing)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  defer_seal_ = true;
  auto pending = requestAsync();
  ASSERT_TRUE(spinUntil([this]() {return pending_seals_.size() == 1;}));
  phase(Lifecycle::INTERRUPTED);
  resolveSeals();
  const auto response = finish(pending);
  ASSERT_NO_FATAL_FAILURE(expectRejected(response));
}

TEST_F(SurveyReportingTest, DelayedPositiveAuthorizationCannotPublishAfterAbort)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  defer_seal_ = true;
  auto pending = requestAsync();
  ASSERT_TRUE(spinUntil([this]() {return pending_seals_.size() == 1;}));
  phase(Lifecycle::INTERRUPTED);
  resolveSeals(true);
  const auto response = finish(pending);
  ASSERT_NO_FATAL_FAILURE(expectRejected(response));
}

TEST_F(SurveyReportingTest, DelayedPreviousRunAuthorizationCannotEnterNewEpoch)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  defer_seal_ = true;
  auto pending = requestAsync();
  ASSERT_TRUE(spinUntil([this]() {return pending_seals_.size() == 1;}));
  phase(Lifecycle::INTERRUPTED);
  beginRun(18, 2);
  resolveSeals(true);
  const auto response = finish(pending);
  ASSERT_NO_FATAL_FAILURE(expectRejected(response));
  EXPECT_EQ(lifecycle_.run_id, 18U);
  EXPECT_EQ(lifecycle_.epoch, 2U);
}

TEST_F(SurveyReportingTest, RecoverableOutboxRetainsIdentityAcrossCommunicationRestart)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  const auto first = request();
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->success) << first->message;
  ASSERT_TRUE(spinUntil([this]() {return surveys().size() == 1;}));
  const auto original = surveys().back();
  const auto seal_count = seal_requests_;
  ASSERT_NO_FATAL_FAILURE(restartNode());
  const auto repeated = request();
  ASSERT_NE(repeated, nullptr);
  EXPECT_TRUE(repeated->success) << repeated->message;
  EXPECT_EQ(repeated->result_code, Survey::Response::ALREADY_ACCEPTED);
  EXPECT_EQ(seal_requests_, seal_count);
  for (const auto & recovered : surveys()) {
    EXPECT_EQ(recovered.report_id, original.report_id);
    EXPECT_EQ(recovered.run_id, original.run_id);
    EXPECT_EQ(recovered.epoch, original.epoch);
    EXPECT_EQ(recovered.original_conditions, original.original_conditions);
  }
}

TEST_F(SurveyReportingTest, OutboxWriteFailurePublishesNoSuccessfulReport)
{
  beginRun();
  auto value = evidence();
  ASSERT_NO_FATAL_FAILURE(publishEvidence(value));
  confirmSurface();
  std::filesystem::rename(directory_, directory_.string() + ".moved");
  const auto response = request();
  ASSERT_NO_FATAL_FAILURE(expectRejected(response));
  EXPECT_EQ(response->result_code, Survey::Response::OUTBOX_UNAVAILABLE);
}
