#include "banyu_vehicle_execution/vehicle_execution_node.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <optional>

#include <cmath>
#include <rcl/time.h>
#include <rclcpp_action/rclcpp_action.hpp>

namespace
{

using Lifecycle = banyu_msgs::msg::RunLifecycle;
using Search = banyu_msgs::action::SearchPipeline;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Search>;
using namespace std::chrono_literals;

class RunAuthorizationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  virtual bool enableMavros() const {return true;}
  virtual bool calibratedSurface() const {return false;}
  virtual double lifecycleTimeout() const {return 0.25;}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("control.require_run_authorization", true);
    options.append_parameter_override("control.run_lifecycle_timeout_s", lifecycleTimeout());
    options.append_parameter_override("control.require_lease", false);
    options.append_parameter_override("control.enable_mavros_publish", enableMavros());
    options.append_parameter_override("topics.run_lifecycle", "/authorization/lifecycle");
    options.append_parameter_override("topics.status", "/authorization/status");
    options.append_parameter_override("topics.pose", "/authorization/pose");
    options.append_parameter_override("topics.mavros_target", "/authorization/target");
    options.append_parameter_override("topics.preview_target", "/authorization/preview");
    options.append_parameter_override("topics.execution_readiness", "/authorization/readiness");
    options.append_parameter_override("topics.water_depth", "/authorization/water_depth");
    options.append_parameter_override("control.require_calibrated_surface_depth",
        calibratedSurface());
    options.append_parameter_override("surface.depth_sources",
        std::vector<std::string>{"bench_depth"});
    options.append_parameter_override("actions.search_pipeline", "/authorization/search");
    options.append_parameter_override("actions.surface", "/authorization/surface");
    node_ = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
    peer_ = std::make_shared<rclcpp::Node>("run_authorization_peer");
    executor_.add_node(node_);
    executor_.add_node(peer_);
    lifecycle_pub_ = peer_->create_publisher<Lifecycle>(
      "/authorization/lifecycle", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = peer_->create_publisher<banyu_msgs::msg::VehicleStatus>(
      "/authorization/status", rclcpp::QoS(10));
    pose_pub_ = peer_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/authorization/pose", rclcpp::SensorDataQoS());
    water_depth_pub_ = peer_->create_publisher<banyu_msgs::msg::WaterDepth>(
      "/authorization/water_depth", rclcpp::SensorDataQoS());
    target_sub_ = peer_->create_subscription<mavros_msgs::msg::PositionTarget>(
      "/authorization/target", rclcpp::QoS(10),
      [this](const mavros_msgs::msg::PositionTarget & target) {targets_.push_back(target);});
    readiness_sub_ = peer_->create_subscription<banyu_msgs::msg::ExecutionReadiness>(
      "/authorization/readiness", rclcpp::QoS(1).reliable().transient_local(),
      [this](const banyu_msgs::msg::ExecutionReadiness & readiness) {readiness_ = readiness;});
    client_ = rclcpp_action::create_client<Search>(peer_, "/authorization/search");
    ASSERT_TRUE(client_->wait_for_action_server(1s));
    pump(200ms, false, false);
  }

  void TearDown() override
  {
    executor_.remove_node(node_);
    executor_.remove_node(peer_);
  }

  void publishInputs()
  {
    banyu_msgs::msg::VehicleStatus status;
    status.header.stamp = node_->now();
    status.fcu_connected = true;
    status.armed = armed_;
    status.fcu_status_fresh = true;
    status.rc_fresh = rc_fresh_;
    status.operator_takeover = takeover_;
    status.mode = "GUIDED";
    status_pub_->publish(status);
    if (publish_pose_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1.0;
      pose.pose.position.x = x_;
      pose.pose.position.y = y_;
      pose.pose.position.z = z_;
      pose_pub_->publish(pose);
    }
    if (water_stream_) {
      auto depth = makeWaterDepth();
      depth.depth_m = water_depth_m_;
      water_depth_pub_->publish(depth);
    }
  }

  banyu_msgs::msg::WaterDepth makeWaterDepth()
  {
    banyu_msgs::msg::WaterDepth depth;
    depth.header.stamp = node_->now();
    depth.run_id = run_id_;
    depth.epoch = epoch_;
    depth.depth_m = 0.0;
    depth.valid = true;
    depth.calibrated = true;
    depth.datum = "water_surface";
    depth.source_id = "bench_depth";
    depth.calibration_id = "bench-water-zero";
    return depth;
  }

  void publishLifecycle()
  {
    Lifecycle state;
    state.header.stamp = node_->now();
    state.authority_instance = authority_;
    state.revision = ++revision_;
    state.phase = phase_;
    state.autonomous_ready = ready_flag_;
    if (phase_ >= Lifecycle::INITIALIZING) {has_run_context_ = true;}
    if (has_run_context_) {
      state.run_id = run_id_;
      state.epoch = epoch_;
    }
    lifecycle_pub_->publish(state);
    last_lifecycle_ = state;
  }

  void pump(
    std::chrono::milliseconds duration, bool inputs = true, bool lifecycle = true)
  {
    const auto until = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < until) {
      if (inputs) {publishInputs();}
      if (lifecycle) {publishLifecycle();}
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
  }

  ClientGoalHandle::SharedPtr sendSearch(
    std::optional<uint32_t> run_id = std::nullopt,
    std::optional<uint64_t> epoch = std::nullopt)
  {
    Search::Goal goal;
    goal.run_id = run_id.value_or(run_id_);
    goal.epoch = epoch.value_or(epoch_);
    goal.search_span_rad = 100.0;
    goal.sweep_rate_rad_s = 0.2;
    auto future = client_->async_send_goal(goal);
    const auto status = executor_.spin_until_future_complete(future, 1s);
    EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {return nullptr;}
    return future.get();
  }

  void authorize()
  {
    phase_ = Lifecycle::READY;
    pump(100ms);
    phase_ = Lifecycle::ACTIVE;
    pump(80ms);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<banyu_vehicle_execution::VehicleExecutionNode> node_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::Publisher<Lifecycle>::SharedPtr lifecycle_pub_;
  rclcpp::Publisher<banyu_msgs::msg::VehicleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<banyu_msgs::msg::WaterDepth>::SharedPtr water_depth_pub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<banyu_msgs::msg::ExecutionReadiness>::SharedPtr readiness_sub_;
  rclcpp_action::Client<Search>::SharedPtr client_;
  std::vector<mavros_msgs::msg::PositionTarget> targets_;
  Lifecycle last_lifecycle_;
  banyu_msgs::msg::ExecutionReadiness readiness_;
  uint8_t phase_{Lifecycle::READY};
  uint64_t revision_{0};
  std::string authority_{"authority-A"};
  bool rc_fresh_{true};
  bool armed_{true};
  bool has_run_context_{false};
  uint64_t epoch_{100};
  uint32_t run_id_{13};
  bool takeover_{false};
  bool publish_pose_{true};
  bool ready_flag_{true};
  bool water_stream_{false};
  double water_depth_m_{0.0};
  double x_{0.0};
  double y_{0.0};
  double z_{-2.0};
};

TEST_F(RunAuthorizationTest, ReadyAndInitializingHoldWithoutAdmittingMissionActions)
{
  ready_flag_ = false;
  pump(200ms);
  EXPECT_EQ(sendSearch(), nullptr);
  ASSERT_FALSE(targets_.empty());
  EXPECT_DOUBLE_EQ(targets_.back().velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(targets_.back().velocity.y, 0.0);
  EXPECT_TRUE(readiness_.holding);
  EXPECT_EQ(readiness_.authority_instance, authority_);
  EXPECT_EQ(readiness_.run_id, 0U);
  EXPECT_LE(readiness_.lifecycle_revision, last_lifecycle_.revision);
  x_ = 1.0;
  y_ = -2.0;
  z_ = -1.0;
  pump(120ms);
  const auto held = targets_.back();
  EXPECT_LT(held.velocity.x, 0.0);
  EXPECT_GT(held.velocity.y, 0.0);
  EXPECT_LT(held.velocity.z, 0.0);
  EXPECT_LE(std::sqrt(held.velocity.x * held.velocity.x + held.velocity.y * held.velocity.y +
    held.velocity.z * held.velocity.z), 0.300001);
  EXPECT_EQ(held.type_mask, 0x5C7);
  EXPECT_DOUBLE_EQ(held.yaw_rate, 0.0);
  phase_ = Lifecycle::INITIALIZING;
  pump(120ms);
  EXPECT_EQ(sendSearch(), nullptr);
  ASSERT_FALSE(targets_.empty());
  EXPECT_LT(targets_.back().velocity.x, 0.0);
  EXPECT_GT(targets_.back().velocity.y, 0.0);
}

TEST_F(RunAuthorizationTest, ActiveNeedsHandshakeAndTerminalPublishesOnlyFiniteZeroBarrier)
{
  phase_ = Lifecycle::ACTIVE;
  pump(80ms);
  EXPECT_EQ(sendSearch(), nullptr);
  EXPECT_TRUE(targets_.empty());
  ++epoch_;
  ++run_id_;
  authorize();
  auto handle = sendSearch();
  ASSERT_NE(handle, nullptr);
  auto result = client_->async_get_result(handle);
  pump(120ms);
  ASSERT_FALSE(targets_.empty());
  // Search starts with a retrace (surge) and then sweeps (yaw); either is commanded motion.
  const auto & moving = targets_.back();
  EXPECT_GT(std::abs(moving.velocity.x) + std::abs(moving.velocity.y) +
    std::abs(moving.yaw_rate), 0.0);
  phase_ = Lifecycle::FAILED;
  publishLifecycle();
  const auto abort_deadline = std::chrono::steady_clock::now() + 500ms;
  while (result.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < abort_deadline)
  {
    executor_.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  targets_.clear();
  pump(220ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_GE(targets_.size(), 1U);
  EXPECT_LE(targets_.size(), 2U);
  for (const auto & target : targets_) {
    EXPECT_DOUBLE_EQ(target.velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(target.velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(target.velocity.z, 0.0);
    EXPECT_DOUBLE_EQ(target.yaw_rate, 0.0);
  }
  targets_.clear();
  pump(150ms);
  EXPECT_TRUE(targets_.empty());
  EXPECT_EQ(sendSearch(), nullptr);
  EXPECT_FALSE(readiness_.holding);
}

TEST_F(RunAuthorizationTest, RcLossAbortsActionAndImmediatelyRevokesHardwareOutput)
{
  authorize();
  auto handle = sendSearch();
  ASSERT_NE(handle, nullptr);
  auto result = client_->async_get_result(handle);
  pump(100ms);
  rc_fresh_ = false;
  pump(80ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  targets_.clear();
  pump(100ms);
  EXPECT_TRUE(targets_.empty());
  EXPECT_EQ(sendSearch(), nullptr);
  rc_fresh_ = true;
  pump(100ms);
  EXPECT_EQ(sendSearch(), nullptr);
  EXPECT_TRUE(targets_.empty());
  ++epoch_;
  ++run_id_;
  authorize();
  EXPECT_NE(sendSearch(), nullptr);
}

TEST_F(RunAuthorizationTest, MissingOrStalePoseCannotMaintainReadinessHold)
{
  publish_pose_ = false;
  pump(120ms);
  EXPECT_TRUE(targets_.empty());
  publish_pose_ = true;
  pump(150ms);
  ASSERT_FALSE(targets_.empty());
  publish_pose_ = false;
  pump(600ms);
  targets_.clear();
  pump(100ms);
  EXPECT_TRUE(targets_.empty());
}

TEST_F(RunAuthorizationTest, OldRevisionsAndRetiredAuthoritiesCannotReopenTerminalRun)
{
  authorize();
  const auto old_authority = last_lifecycle_;
  authority_ = "authority-B";
  revision_ = 0;
  ++epoch_;
  ++run_id_;
  authorize();
  phase_ = Lifecycle::INTERRUPTED;
  pump(180ms);
  targets_.clear();
  auto replay = old_authority;
  replay.header.stamp = node_->now();
  replay.revision += 1000;
  lifecycle_pub_->publish(replay);
  pump(100ms, true, false);
  EXPECT_TRUE(targets_.empty());
  EXPECT_EQ(sendSearch(), nullptr);
  replay = last_lifecycle_;
  replay.header.stamp = node_->now();
  replay.phase = Lifecycle::ACTIVE;
  replay.revision -= 1;
  lifecycle_pub_->publish(replay);
  pump(100ms, true, false);
  EXPECT_TRUE(targets_.empty());
  EXPECT_EQ(sendSearch(), nullptr);
}

TEST_F(RunAuthorizationTest, InvalidLifecyclePayloadsPreserveCurrentAuthorization)
{
  authorize();
  std::vector<Lifecycle> invalid(6, last_lifecycle_);
  invalid[0].phase = 255;
  invalid[1].run_id = 0;
  invalid[2].epoch = 0;
  invalid[3].run_id = 14;
  invalid[4].authority_instance = "unknown-authority";
  invalid[4].epoch -= 1;
  invalid[5].header.stamp.nanosec = 1000000000U;
  for (std::size_t i = 0; i < invalid.size(); ++i) {
    if (i != 5) {invalid[i].header.stamp = node_->now();}
    invalid[i].revision = revision_ + i + 100;
    lifecycle_pub_->publish(invalid[i]);
    pump(10ms, true, false);
  }
  EXPECT_NE(sendSearch(), nullptr);
}

TEST_F(RunAuthorizationTest, DisarmingRevokesHardwareOutputAndRunPermission)
{
  authorize();
  auto handle = sendSearch();
  ASSERT_NE(handle, nullptr);
  auto result = client_->async_get_result(handle);
  pump(80ms);
  armed_ = false;
  pump(80ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  targets_.clear();
  pump(100ms);
  EXPECT_TRUE(targets_.empty());
  EXPECT_EQ(sendSearch(), nullptr);
}

TEST_F(RunAuthorizationTest, FrozenRosClockCannotKeepLifecyclePermissionFresh)
{
  auto clock = node_->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 50000000000LL), RCL_RET_OK);
  phase_ = Lifecycle::READY;
  pump(100ms);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 51000000000LL), RCL_RET_OK);
  phase_ = Lifecycle::ACTIVE;
  pump(80ms);
  auto handle = sendSearch();
  ASSERT_NE(handle, nullptr);
  auto result = client_->async_get_result(handle);
  pump(400ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  targets_.clear();
  pump(100ms);
  EXPECT_TRUE(targets_.empty());
  EXPECT_EQ(sendSearch(), nullptr);
}

class PreviewReadinessTest : public RunAuthorizationTest
{
protected:
  bool enableMavros() const override {return false;}
};

TEST_F(PreviewReadinessTest, PreviewOutputCannotClaimActualReadinessHold)
{
  pump(250ms);
  EXPECT_TRUE(targets_.empty());
  EXPECT_FALSE(readiness_.holding);
  EXPECT_EQ(readiness_.reason, "MAVROS output disabled");
}

class SurfaceVerificationTest : public RunAuthorizationTest
{
protected:
  bool calibratedSurface() const override {return true;}
  double lifecycleTimeout() const override {return 2.0;}
  using Surface = banyu_msgs::action::Surface;
  using SurfaceHandle = rclcpp_action::ClientGoalHandle<Surface>;

  SurfaceHandle::SharedPtr sendSurface(
    double tolerance = 0.2, double watchdog = 2.0,
    uint32_t run_id = 0, uint64_t epoch = 0)
  {
    surface_client_ = rclcpp_action::create_client<Surface>(peer_, "/authorization/surface");
    EXPECT_TRUE(surface_client_->wait_for_action_server(1s));
    Surface::Goal goal;
    goal.run_id = (run_id == 0) ? run_id_ : run_id;
    goal.epoch = (epoch == 0) ? epoch_ : epoch;
    goal.target_depth_m = 0.0;
    goal.depth_tolerance_m = tolerance;
    goal.watchdog_timeout_s = watchdog;
    auto future = surface_client_->async_send_goal(goal);
    const auto status = executor_.spin_until_future_complete(future, 1s);
    EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {return nullptr;}
    return future.get();
  }

  rclcpp_action::Client<Surface>::SharedPtr surface_client_;
};

TEST_F(SurfaceVerificationTest, OneNearSurfaceSampleCannotCompleteButFreshStreamCan)
{
  authorize();
  auto handle = sendSurface();
  ASSERT_NE(handle, nullptr);
  auto result = surface_client_->async_get_result(handle);
  water_depth_pub_->publish(makeWaterDepth());
  pump(750ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  water_stream_ = true;
  pump(1200ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(SurfaceVerificationTest, InvalidEvidenceAndRawMapDepthCannotAuthorizeSurfacing)
{
  z_ = 0.0;
  authorize();
  auto handle = sendSurface();
  ASSERT_NE(handle, nullptr);
  auto result = surface_client_->async_get_result(handle);
  auto raw = peer_->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());
  std_msgs::msg::Float64 rel_alt;
  rel_alt.data = 0.0;
  raw->publish(rel_alt);
  std::vector<banyu_msgs::msg::WaterDepth> invalid(9, makeWaterDepth());
  invalid[0].valid = false;
  invalid[1].calibrated = false;
  invalid[2].datum = "map";
  invalid[3].source_id = "untrusted";
  invalid[4].calibration_id.clear();
  invalid[5].depth_m = std::numeric_limits<double>::quiet_NaN();
  invalid[6].epoch -= 1;
  invalid[7].header.stamp = builtin_interfaces::msg::Time();
  invalid[8].header.stamp = node_->now() + rclcpp::Duration::from_seconds(1.0);
  targets_.clear();
  for (std::size_t i = 0; i < invalid.size(); ++i) {
    if (i < 7) {invalid[i].header.stamp = node_->now();}
    water_depth_pub_->publish(invalid[i]);
    pump(60ms);
    EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  }
  for (const auto & target : targets_) {
    EXPECT_DOUBLE_EQ(target.velocity.z, 0.0);
  }
  water_stream_ = true;
  pump(1200ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(SurfaceVerificationTest, BriefInvalidSampleResetsDwellEvenBetweenActionTicks)
{
  authorize();
  auto handle = sendSurface();
  ASSERT_NE(handle, nullptr);
  auto result = surface_client_->async_get_result(handle);
  water_stream_ = true;
  pump(700ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  water_stream_ = false;
  auto invalid = makeWaterDepth();
  invalid.depth_m = std::numeric_limits<double>::quiet_NaN();
  water_depth_pub_->publish(invalid);
  pump(20ms);
  water_stream_ = true;
  pump(450ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  pump(700ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(SurfaceVerificationTest, NearSurfaceExcursionResetsHoldAndToleranceCannotBeWidened)
{
  authorize();
  auto handle = sendSurface(99.0);
  ASSERT_NE(handle, nullptr);
  auto result = surface_client_->async_get_result(handle);
  water_stream_ = true;
  water_depth_m_ = 0.4;
  pump(1100ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  ASSERT_FALSE(targets_.empty());
  EXPECT_GT(targets_.back().velocity.z, 0.0);
  water_depth_m_ = 0.0;
  pump(600ms);
  water_depth_m_ = 0.3;
  pump(100ms);
  water_depth_m_ = 0.0;
  pump(600ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  pump(550ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(SurfaceVerificationTest, FailedRunAndPreviousEpochCannotSupplySurfaceProof)
{
  authorize();
  auto handle = sendSurface();
  ASSERT_NE(handle, nullptr);
  auto result = surface_client_->async_get_result(handle);
  water_stream_ = true;
  pump(300ms);
  const auto old_depth = makeWaterDepth();
  phase_ = Lifecycle::FAILED;
  pump(100ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(sendSurface(), nullptr);
  water_stream_ = false;
  ++epoch_;
  ++run_id_;
  authorize();
  handle = sendSurface();
  ASSERT_NE(handle, nullptr);
  result = surface_client_->async_get_result(handle);
  auto stale = old_depth;
  stale.header.stamp = node_->now();
  water_depth_pub_->publish(stale);
  pump(600ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  water_stream_ = true;
  pump(1200ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(SurfaceVerificationTest, FrozenSourceClockExpiresDepthAndSteadyWatchdog)
{
  auto clock = node_->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 50000000000LL), RCL_RET_OK);
  phase_ = Lifecycle::READY;
  pump(150ms);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 51000000000LL), RCL_RET_OK);
  phase_ = Lifecycle::ACTIVE;
  pump(80ms);
  auto handle = sendSurface(0.2, 0.2);
  ASSERT_NE(handle, nullptr);
  auto result = surface_client_->async_get_result(handle);
  water_stream_ = true;
  pump(800ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(finished.result, nullptr);
  EXPECT_NE(finished.result->reason.find("water-depth"), std::string::npos);
}

TEST_F(RunAuthorizationTest, MismatchedRunIdOrEpochGoalIsRejected)
{
  authorize();
  // Valid goal succeeds
  auto handle = sendSearch(run_id_, epoch_);
  ASSERT_NE(handle, nullptr);

  // Mismatched run_id is rejected
  EXPECT_EQ(sendSearch(run_id_ + 1, epoch_), nullptr);

  // Mismatched epoch is rejected
  EXPECT_EQ(sendSearch(run_id_, epoch_ + 1), nullptr);

  // Zero run context is rejected
  EXPECT_EQ(sendSearch(0, 0), nullptr);
}

TEST_F(SurfaceVerificationTest, SubmergedDipBetweenTicksImmediatelyResetsHold)
{
  authorize();
  auto handle = sendSurface();
  ASSERT_NE(handle, nullptr);
  auto result = surface_client_->async_get_result(handle);

  // Surface water depth stream for 700 ms (hold requires 1000 ms)
  water_stream_ = true;
  water_depth_m_ = 0.0;
  pump(700ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  water_stream_ = false;

  // Intermediate sample exceeds surface tolerance (0.35m > 0.2m)
  auto dip = makeWaterDepth();
  dip.depth_m = 0.35;
  water_depth_pub_->publish(dip);
  pump(20ms);

  // Resume surface stream; because of dip, the hold was reset
  water_stream_ = true;
  water_depth_m_ = 0.0;
  pump(500ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);

  // Full hold completion
  pump(700ms);
  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

}  // namespace
