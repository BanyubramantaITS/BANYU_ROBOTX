#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "banyu_mission/banyu_mission.hpp"
#include "banyu_msgs/action/navigate_surface_gps.hpp"
#include "banyu_msgs/msg/execution_readiness.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_buoy_target.hpp"
#include "banyu_msgs/msg/vehicle_status.hpp"
#include "banyu_msgs/srv/reset_task2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/rx_task.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
using namespace std::chrono_literals;
using Lifecycle = banyu_msgs::msg::RunLifecycle;
using Reset = banyu_msgs::srv::ResetTask2;
using Navigate = banyu_msgs::action::NavigateSurfaceGps;
using NavigateHandle = rclcpp_action::ServerGoalHandle<Navigate>;

bool waitUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(5ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return predicate();
}

class RunLifecycleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  // The Task 2 tree stands by until the USV's active buoy arrives; most tests start with one already known.
  virtual bool publishDefaultBuoyTarget() const {return true;}
  // These tests check the target plumbing: no standoff unless a test asks for it.
  virtual double buoyStandoff() const {return 0.0;}
  virtual std::string divePoint() const {return "buoy";}

  void SetUp() override
  {
    std::string pattern = (std::filesystem::temp_directory_path() /
      "banyu-run-lifecycle-XXXXXX").string();
    std::vector<char> directory(pattern.begin(), pattern.end());
    directory.push_back('\0');
    ASSERT_NE(::mkdtemp(directory.data()), nullptr);
    directory_ = directory.data();

    fixture_ = std::make_shared<rclcpp::Node>("run_lifecycle_fixture");
    reset_callback_group_ = fixture_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    status_pub_ = fixture_->create_publisher<banyu_msgs::msg::VehicleStatus>(
      "/banyu/vehicle_status", rclcpp::QoS(5));
    pose_pub_ = fixture_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS());
    command_pub_ = fixture_->create_publisher<rx_msgs::msg::Command>(
      "/system/mission/command", rclcpp::QoS(10));
    execution_readiness_pub_ = fixture_->create_publisher<banyu_msgs::msg::ExecutionReadiness>(
      "/banyu/execution/readiness", rclcpp::QoS(5).reliable());
    target_pub_ = fixture_->create_publisher<banyu_msgs::msg::Task2BuoyTarget>(
      "/banyu/task2/buoy_target", rclcpp::QoS(1).reliable().transient_local());
    lifecycle_sub_ = fixture_->create_subscription<Lifecycle>(
      "/banyu/mission/run_lifecycle", rclcpp::QoS(10).reliable().transient_local(),
      [this](const Lifecycle::SharedPtr value) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        lifecycle_ = *value;
        lifecycle_seen_ = true;
        lifecycle_history_.push_back(*value);
      });
    mode_server_ = fixture_->create_service<mavros_msgs::srv::SetMode>(
      "/mavros/set_mode",
      [this](const std::shared_ptr<mavros_msgs::srv::SetMode::Request> request,
      std::shared_ptr<mavros_msgs::srv::SetMode::Response> response) {
        std::lock_guard<std::mutex> lock(vehicle_mutex_);
        requested_modes_.push_back(request->custom_mode);
        mode_ = request->custom_mode;
        response->mode_sent = true;
      });
    arming_server_ = fixture_->create_service<mavros_msgs::srv::CommandBool>(
      "/mavros/cmd/arming",
      [this](const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request,
      std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response) {
        armed_.store(request->value);
        ++arming_requests_;
        response->success = true;
      });
    tracker_reset_server_ = resetServer("/banyu/pipeline/reset_tracker", true);
    end_reset_server_ = resetServer("/banyu/pipeline/reset_end", false);
    navigation_server_ = rclcpp_action::create_server<Navigate>(
      fixture_, "/banyu/actions/navigate_surface_gps",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<NavigateHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<NavigateHandle> handle) {
        ++navigation_goals_;
        std::lock_guard<std::mutex> lock(navigation_mutex_);
        navigation_handles_.push_back(handle);
      });
    telemetry_timer_ = fixture_->create_wall_timer(20ms, [this]() {publishTelemetry();});

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("mission_mode", "task2"),
        rclcpp::Parameter("mission.start_latitude", 1.0),
        rclcpp::Parameter("mission.start_longitude", 2.0),
        rclcpp::Parameter("mission.target_latitude", 1.001),
        rclcpp::Parameter("mission.target_longitude", 2.001),
        rclcpp::Parameter("mission.end_latitude", 1.002),
        rclcpp::Parameter("mission.end_longitude", 2.002),
        rclcpp::Parameter("mission.buoy_standoff_m", buoyStandoff()),
        rclcpp::Parameter("mission.dive_point", divePoint()),
        rclcpp::Parameter("run.journal_path", (directory_ / "runs.journal").string()),
        rclcpp::Parameter("timing.run_reset_timeout_s", 2.0),
        rclcpp::Parameter("timing.execution_readiness_timeout_s", 0.2),
        rclcpp::Parameter("groot_port", next_groot_port_.fetch_add(4))});
    mission_ = std::make_shared<banyu_mission::BanyuMission>(options);
    mission_->initialize();
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, 4);
    executor_->add_node(fixture_);
    executor_->add_node(mission_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
    ASSERT_TRUE(waitUntil([this]() {return command_pub_->get_subscription_count() > 0;}));
    ASSERT_TRUE(waitUntil([this]() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        return lifecycle_seen_;
      }));
    if (publishDefaultBuoyTarget()) {
      ASSERT_TRUE(waitUntil([this]() {return target_pub_->get_subscription_count() > 0;}));
      publishBuoyTarget(1.001, 2.001, 0);
      std::this_thread::sleep_for(100ms);
    }
  }

  void publishBuoyTarget(double latitude, double longitude, uint32_t seq)
  {
    banyu_msgs::msg::Task2BuoyTarget target;
    target.course_id = "course-a";
    target.source_seq = seq;
    target.latitude = latitude;
    target.longitude = longitude;
    target_pub_->publish(target);
  }

  void TearDown() override
  {
    {
      std::lock_guard<std::mutex> lock(reset_mutex_);
      permit_tracker_reset_ = true;
      permit_end_reset_ = true;
      terminating_ = true;
    }
    reset_condition_.notify_all();
    if (fixture_ && executor_) {
      auto abort = fixture_->create_client<std_srvs::srv::Trigger>("/banyu/mission/abort");
      if (abort->wait_for_service(200ms)) {
        auto request = abort->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
        (void)request.wait_for(500ms);
      }
    }
    if (executor_) {executor_->cancel();}
    if (spin_thread_.joinable()) {spin_thread_.join();}
    if (executor_ && mission_) {executor_->remove_node(mission_);}
    if (executor_ && fixture_) {executor_->remove_node(fixture_);}
    mission_.reset();
    executor_.reset();
    telemetry_timer_.reset();
    lifecycle_sub_.reset();
    target_pub_.reset();
    navigation_server_.reset();
    tracker_reset_server_.reset();
    end_reset_server_.reset();
    mode_server_.reset();
    arming_server_.reset();
    fixture_.reset();
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
  }

  rclcpp::Service<Reset>::SharedPtr resetServer(const std::string & name, bool tracker)
  {
    return fixture_->create_service<Reset>(
      name, [this, tracker](const std::shared_ptr<Reset::Request> request,
      std::shared_ptr<Reset::Response> response) {
        {
          std::unique_lock<std::mutex> lock(reset_mutex_);
          (tracker ? tracker_requests_ : end_requests_).push_back(*request);
          reset_condition_.wait_for(lock, 4s, [this, tracker]() {
            return terminating_ || (tracker ? permit_tracker_reset_ : permit_end_reset_);
            });
          response->success = !terminating_ &&
          (tracker ? tracker_reset_success_ : end_reset_success_);
        }
        response->message = response->success ? "reset complete" : "reset rejected";
      }, rclcpp::ServicesQoS(), reset_callback_group_);
  }

  void publishTelemetry()
  {
    banyu_msgs::msg::VehicleStatus status;
    status.header.stamp = fixture_->now();
    status.fcu_connected = true;
    status.armed = armed_.load();
    status.fcu_status_fresh = true;
    status.rc_fresh = rc_fresh_.load();
    status.gps_valid = true;
    status.gps_fresh = true;
    status.operator_takeover = takeover_.load();
    {
      std::lock_guard<std::mutex> lock(vehicle_mutex_);
      status.mode = mode_;
    }
    status.authority = takeover_.load() ? "MANUAL" : "AUTONOMY";
    status.operational_mode = 2;
    status_pub_->publish(status);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = status.header.stamp;
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.0;
    if (publish_pose_.load()) {pose_pub_->publish(pose);}
    const auto context = lifecycle();
    if (publish_hold_ack_.load() && context.phase == Lifecycle::READY) {
      banyu_msgs::msg::ExecutionReadiness readiness;
      readiness.header.stamp = fixture_->now() +
        rclcpp::Duration::from_seconds(hold_stamp_offset_s_.load());
      readiness.header.frame_id = "vehicle_execution";
      readiness.authority_instance = corrupt_hold_authority_.load() ?
        context.authority_instance + "_wrong_authority" : context.authority_instance;
      readiness.run_id = context.run_id;
      readiness.epoch = context.epoch;
      readiness.lifecycle_revision = context.revision;
      readiness.holding = true;
      readiness.reason = "mock executor holding after preparation";
      execution_readiness_pub_->publish(readiness);
    }

    std::lock_guard<std::mutex> lock(navigation_mutex_);
    for (auto & handle : navigation_handles_) {
      if (!handle->is_active()) {continue;}
      auto result = std::make_shared<Navigate::Result>();
      result->success = false;
      if (handle->is_canceling()) {
        result->reason = "fixture cancellation";
        handle->canceled(result);
      } else if (fail_navigation_.load()) {
        result->reason = "navigation execution failure";
        handle->abort(result);
      }
    }
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> trigger(const std::string & name)
  {
    auto client = fixture_->create_client<std_srvs::srv::Trigger>(name);
    if (!client->wait_for_service(2s)) {return nullptr;}
    auto request = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (request.wait_for(3s) != std::future_status::ready) {return nullptr;}
    return request.get();
  }

  void prepare(bool require_execution_readiness = true)
  {
    ASSERT_TRUE(waitUntil([this]() {
        return status_pub_->get_subscription_count() > 0 && pose_pub_->get_subscription_count() > 0;
      }));
    std::shared_ptr<std_srvs::srv::Trigger::Response> response;
    const bool accepted = waitUntil([this, &response]() {
          response = trigger("/banyu/mission/prepare_autonomy");
          return response && response->success;
      });
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(accepted) << response->message;
    ASSERT_TRUE(waitUntil([this, require_execution_readiness]() {
        const auto value = lifecycle();
        return value.phase == Lifecycle::READY &&
               (!require_execution_readiness || value.autonomous_ready);
      }, 6s));
    EXPECT_GT(arming_requests_.load(), 0U);
    EXPECT_EQ(navigation_goals_.load(), 0U);
  }

  void command(
    uint32_t run_id, uint32_t seq, uint8_t type = rx_msgs::msg::Command::CMD_RUN_START,
    uint8_t target = rx_msgs::msg::VehicleType::TYPE_UUV)
  {
    rx_msgs::msg::Command value;
    value.header.stamp = fixture_->now();
    value.header.frame_id = "gcs";
    value.run_id = run_id;
    value.command_seq = seq;
    value.command_type = type;
    value.target_vehicle = target;
    command_pub_->publish(value);
  }

  Lifecycle lifecycle()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    return lifecycle_;
  }

  bool sawEvent(uint8_t event, uint32_t requested_run_id)
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    for (const auto & value : lifecycle_history_) {
      if (value.event == event && value.requested_run_id == requested_run_id) {return true;}
    }
    return false;
  }

  void start()
  {
    ASSERT_NO_FATAL_FAILURE(prepare());
    command(13, 10);
    ASSERT_TRUE(waitUntil([this]() {
        const auto value = lifecycle();
        return value.run_id == 13 && value.phase == Lifecycle::ACTIVE;
      }));
    ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 1U;}));
    EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR);
  }

  void expectContextUnchanged(const Lifecycle & before)
  {
    const auto after = lifecycle();
    EXPECT_EQ(after.run_id, before.run_id);
    EXPECT_EQ(after.epoch, before.epoch);
    EXPECT_EQ(after.command_seq, before.command_seq);
    EXPECT_EQ(after.phase, before.phase);
    EXPECT_EQ(after.current_task, before.current_task);
  }

  std::filesystem::path directory_;
  rclcpp::Node::SharedPtr fixture_;
  std::shared_ptr<banyu_mission::BanyuMission> mission_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<banyu_msgs::msg::VehicleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<rx_msgs::msg::Command>::SharedPtr command_pub_;
  rclcpp::Publisher<banyu_msgs::msg::ExecutionReadiness>::SharedPtr execution_readiness_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2BuoyTarget>::SharedPtr target_pub_;
  rclcpp::Subscription<Lifecycle>::SharedPtr lifecycle_sub_;
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr mode_server_;
  rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_server_;
  rclcpp::Service<Reset>::SharedPtr tracker_reset_server_;
  rclcpp::Service<Reset>::SharedPtr end_reset_server_;
  rclcpp::CallbackGroup::SharedPtr reset_callback_group_;
  rclcpp_action::Server<Navigate>::SharedPtr navigation_server_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  std::mutex lifecycle_mutex_;
  Lifecycle lifecycle_;
  bool lifecycle_seen_{false};
  std::vector<Lifecycle> lifecycle_history_;
  std::mutex vehicle_mutex_;
  std::string mode_{"MANUAL"};
  std::vector<std::string> requested_modes_;
  std::atomic_bool armed_{false};
  std::atomic_bool rc_fresh_{true};
  std::atomic_bool takeover_{false};
  std::atomic<std::size_t> arming_requests_{0};
  std::mutex reset_mutex_;
  std::condition_variable reset_condition_;
  bool permit_tracker_reset_{true};
  bool permit_end_reset_{true};
  bool tracker_reset_success_{true};
  bool end_reset_success_{true};
  bool terminating_{false};
  std::vector<Reset::Request> tracker_requests_;
  std::vector<Reset::Request> end_requests_;
  std::mutex navigation_mutex_;
  std::vector<std::shared_ptr<NavigateHandle>> navigation_handles_;
  std::atomic<std::size_t> navigation_goals_{0};
  std::atomic_bool fail_navigation_{false};
  std::atomic_bool publish_hold_ack_{true};
  std::atomic_bool corrupt_hold_authority_{false};
  std::atomic<double> hold_stamp_offset_s_{0.0};
  std::atomic_bool publish_pose_{true};
  inline static std::atomic<int> next_groot_port_{31000};
};

class RunLifecycleNoBuoyTargetTest : public RunLifecycleTest
{
protected:
  bool publishDefaultBuoyTarget() const override {return false;}
};
}  // namespace

TEST_F(RunLifecycleTest, DirectTask2TriggerCannotBypassGcsAuthorization)
{
  const auto before = lifecycle();
  const auto response = trigger("/banyu/mission/start");
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("GCS_START_REQUIRED"), std::string::npos);
  expectContextUnchanged(before);
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, StartBeforePreparationAndZeroIdentityDoNotInitialize)
{
  command(13, 1);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 13);}));
  EXPECT_EQ(lifecycle().run_id, 0U);
  ASSERT_NO_FATAL_FAILURE(prepare());
  const auto before = lifecycle();
  command(0, 2);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 0);}));
  expectContextUnchanged(before);
  std::lock_guard<std::mutex> lock(reset_mutex_);
  EXPECT_TRUE(tracker_requests_.empty());
  EXPECT_TRUE(end_requests_.empty());
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, DuplicateAndConflictingStartsPreserveAcceptedEpoch)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const auto before = lifecycle();
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_DUPLICATE, 13);}));
  expectContextUnchanged(before);
  command(14, 11);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 14);}));
  expectContextUnchanged(before);
  command(0, 12);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 0);}));
  expectContextUnchanged(before);
  std::lock_guard<std::mutex> lock(reset_mutex_);
  ASSERT_EQ(tracker_requests_.size(), 1U);
  ASSERT_EQ(end_requests_.size(), 1U);
  EXPECT_EQ(tracker_requests_[0].run_id, 13U);
  EXPECT_EQ(tracker_requests_[0].epoch, before.epoch);
  EXPECT_EQ(end_requests_[0].epoch, before.epoch);
  EXPECT_EQ(navigation_goals_.load(), 1U);
}

TEST_F(RunLifecycleTest, CommandAddressedToAnotherVehicleCannotInitializeUuv)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  const auto before = lifecycle();
  command(13, 10, rx_msgs::msg::Command::CMD_RUN_START, rx_msgs::msg::VehicleType::TYPE_USV);
  std::this_thread::sleep_for(150ms);
  expectContextUnchanged(before);
  std::lock_guard<std::mutex> lock(reset_mutex_);
  EXPECT_TRUE(tracker_requests_.empty());
  EXPECT_TRUE(end_requests_.empty());
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, NavigationWaitsForBothResetAcknowledgements)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    permit_tracker_reset_ = false;
    permit_end_reset_ = false;
  }
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(reset_mutex_);
      return !tracker_requests_.empty();
    }));
  const auto initializing = lifecycle();
  EXPECT_EQ(initializing.phase, Lifecycle::INITIALIZING);
  EXPECT_EQ(initializing.current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_EQ(navigation_goals_.load(), 0U);
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    permit_tracker_reset_ = true;
  }
  reset_condition_.notify_all();
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(reset_mutex_);
      return !end_requests_.empty();
    }));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(navigation_goals_.load(), 0U);
  EXPECT_EQ(lifecycle().phase, Lifecycle::INITIALIZING);
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    permit_end_reset_ = true;
  }
  reset_condition_.notify_all();
  ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 1U;}));
  EXPECT_EQ(lifecycle().phase, Lifecycle::ACTIVE);
  EXPECT_EQ(lifecycle().epoch, initializing.epoch);
}

TEST_F(RunLifecycleTest, RejectedResetFailsClosedBeforeAnyNavigation)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    end_reset_success_ = false;
  }
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::FAILED;}));
  EXPECT_EQ(lifecycle().run_id, 13U);
  EXPECT_GT(lifecycle().epoch, 0U);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, AbortDuringInitializationFencesDelayedAcknowledgement)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    permit_end_reset_ = false;
  }
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(reset_mutex_);
      return !end_requests_.empty();
    }));
  const auto response = trigger("/banyu/mission/abort");
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));
  const auto interrupted = lifecycle();
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    permit_end_reset_ = true;
  }
  reset_condition_.notify_all();
  std::this_thread::sleep_for(150ms);
  expectContextUnchanged(interrupted);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, MissingResetAcknowledgementFailsClosedAndLateSuccessCannotRevive)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    permit_end_reset_ = false;
  }
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(reset_mutex_);
      return !end_requests_.empty();
    }));
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::FAILED;}, 3s));
  const auto failed = lifecycle();
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    permit_end_reset_ = true;
  }
  reset_condition_.notify_all();
  std::this_thread::sleep_for(150ms);
  expectContextUnchanged(failed);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, MissionDoneStopsWithoutManufacturingSuccess)
{
  ASSERT_NO_FATAL_FAILURE(start());
  command(13, 11, rx_msgs::msg::Command::CMD_MISSION_DONE);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));
  const auto terminal = lifecycle();
  EXPECT_EQ(terminal.current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_EQ(terminal.run_id, 13U);
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_DUPLICATE, 13);}));
  expectContextUnchanged(terminal);
  EXPECT_EQ(navigation_goals_.load(), 1U);
}

TEST_F(RunLifecycleTest, NavigationExecutionFailureClearsTask)
{
  ASSERT_NO_FATAL_FAILURE(start());
  fail_navigation_.store(true);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::FAILED;}));
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_EQ(lifecycle().run_id, 13U);
}

TEST_F(RunLifecycleTest, StaleOrDifferentRunStopCannotTerminateAcceptedRun)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const auto before = lifecycle();
  command(14, 11, rx_msgs::msg::Command::CMD_MISSION_DONE);
  std::this_thread::sleep_for(150ms);
  expectContextUnchanged(before);
  command(13, 9, rx_msgs::msg::Command::CMD_MISSION_DONE);
  std::this_thread::sleep_for(150ms);
  expectContextUnchanged(before);
  EXPECT_EQ(navigation_goals_.load(), 1U);
  command(13, 11, rx_msgs::msg::Command::CMD_MISSION_DONE);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
}

TEST_F(RunLifecycleTest, DisarmingDuringAnActionInterruptsRunAndClearsTask)
{
  ASSERT_NO_FATAL_FAILURE(start());
  armed_.store(false);
  ASSERT_TRUE(waitUntil([this]() {return !lifecycle().autonomous_ready;}));
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));
  EXPECT_EQ(lifecycle().run_id, 13U);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
}

TEST_F(RunLifecycleTest, AcceptedContextKeepsMissionStateDisarmProtectionEngaged)
{
  banyu_mission::MissionState state(fixture_.get());
  ASSERT_TRUE(state.configureMissionCoordinates(1.0, 2.0, 1.001, 2.001));
  {
    std::lock_guard<std::mutex> lock(vehicle_mutex_);
    mode_ = "GUIDED";
  }
  armed_.store(true);
  ASSERT_TRUE(state.acceptRun(13, 1));
  ASSERT_TRUE(state.autonomyEngaged());
  ASSERT_TRUE(waitUntil([&state]() {return state.vehicleReady();}));
  armed_.store(false);
  ASSERT_TRUE(waitUntil([&state]() {return !state.snapshot().armed;}));
  EXPECT_TRUE(state.autonomyEngaged());
  EXPECT_TRUE(state.autonomyInterrupted());
}

TEST_F(RunLifecycleTest, StaleMapPoseDuringAnActionInterruptsRunAndClearsTask)
{
  ASSERT_NO_FATAL_FAILURE(start());
  publish_pose_.store(false);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));
  EXPECT_EQ(lifecycle().run_id, 13U);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_FALSE(lifecycle().autonomous_ready);
  EXPECT_NE(lifecycle().reason.find("AUTONOMY_READINESS_LOST"), std::string::npos);
}

TEST_F(RunLifecycleTest, OperatorTakeoverClearsTaskWithoutOverridingPilotMode)
{
  ASSERT_NO_FATAL_FAILURE(start());
  {
    std::lock_guard<std::mutex> lock(vehicle_mutex_);
    requested_modes_.clear();
    mode_ = "MANUAL";
  }
  takeover_.store(true);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
  std::lock_guard<std::mutex> lock(vehicle_mutex_);
  for (const auto & mode : requested_modes_) {
    EXPECT_NE(mode, "SURFACE");
    EXPECT_NE(mode, "GUIDED");
  }
}

TEST_F(RunLifecycleTest, RcFreshnessLossBeforeStartRevokesReadiness)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  rc_fresh_.store(false);
  ASSERT_TRUE(waitUntil([this]() {return !lifecycle().autonomous_ready;}));
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 13);}));
  EXPECT_EQ(lifecycle().run_id, 0U);
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, PreparedVehicleNeedsLiveExecutionHoldAcknowledgementBeforeStart)
{
  publish_hold_ack_.store(false);
  ASSERT_NO_FATAL_FAILURE(prepare(false));
  EXPECT_FALSE(lifecycle().autonomous_ready);
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 13);}));
  EXPECT_EQ(lifecycle().run_id, 0U);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_NONE);
  EXPECT_EQ(navigation_goals_.load(), 0U);
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    EXPECT_TRUE(tracker_requests_.empty());
    EXPECT_TRUE(end_requests_.empty());
  }
  publish_hold_ack_.store(true);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().autonomous_ready;}));
  command(13, 11);
  ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 1U;}));
  EXPECT_EQ(lifecycle().phase, Lifecycle::ACTIVE);
  EXPECT_EQ(lifecycle().current_task, rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR);
}

TEST_F(RunLifecycleTest, WrongAuthorityHoldAcknowledgementCannotAuthorizeStart)
{
  corrupt_hold_authority_.store(true);
  ASSERT_NO_FATAL_FAILURE(prepare(false));
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(lifecycle().autonomous_ready);
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 13);}));
  EXPECT_EQ(lifecycle().run_id, 0U);
  EXPECT_EQ(navigation_goals_.load(), 0U);
  std::lock_guard<std::mutex> lock(reset_mutex_);
  EXPECT_TRUE(tracker_requests_.empty());
  EXPECT_TRUE(end_requests_.empty());
}

TEST_F(RunLifecycleTest, ExpiredHoldAcknowledgementRevokesStartEligibility)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  publish_hold_ack_.store(false);
  ASSERT_TRUE(waitUntil([this]() {return !lifecycle().autonomous_ready;}));
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::IDLE;}));
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 13);}));
  EXPECT_EQ(lifecycle().run_id, 0U);
  EXPECT_EQ(navigation_goals_.load(), 0U);
}

TEST_F(RunLifecycleTest, FutureAndStaleHoldSourceStampsCannotAuthorizeStart)
{
  hold_stamp_offset_s_.store(1.0);
  ASSERT_NO_FATAL_FAILURE(prepare(false));
  std::this_thread::sleep_for(100ms);
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 13);}));
  EXPECT_FALSE(lifecycle().autonomous_ready);
  hold_stamp_offset_s_.store(-1.0);
  std::this_thread::sleep_for(100ms);
  command(14, 11);
  ASSERT_TRUE(waitUntil([this]() {return sawEvent(Lifecycle::START_REJECTED, 14);}));
  EXPECT_FALSE(lifecycle().autonomous_ready);
  EXPECT_EQ(lifecycle().run_id, 0U);
  EXPECT_EQ(navigation_goals_.load(), 0U);
  std::lock_guard<std::mutex> lock(reset_mutex_);
  EXPECT_TRUE(tracker_requests_.empty());
  EXPECT_TRUE(end_requests_.empty());
}

// RoboCommand "Reset run ID & seq": after re-preparation run 13 / seq 1 starts a new epoch.
TEST_F(RunLifecycleTest, ReusedRunIdAndSeqStartAgainAfterPreparation)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const auto first = lifecycle();
  command(13, 11, rx_msgs::msg::Command::CMD_MISSION_DONE);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));

  navigation_goals_.store(0);
  ASSERT_NO_FATAL_FAILURE(prepare());
  command(13, 1);
  ASSERT_TRUE(waitUntil([this, &first]() {
      const auto value = lifecycle();
      return value.run_id == 13 && value.epoch > first.epoch && value.phase == Lifecycle::ACTIVE;
    }));
  const auto second = lifecycle();
  EXPECT_EQ(second.command_seq, 1U);
  ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 1U;}));

  // Within the new window a retransmission is still only a duplicate.
  command(13, 1);
  std::this_thread::sleep_for(150ms);
  expectContextUnchanged(second);
  EXPECT_EQ(navigation_goals_.load(), 1U);

  // The stop sequence restarted with the window too.
  command(13, 2, rx_msgs::msg::Command::CMD_MISSION_DONE);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::INTERRUPTED;}));
  EXPECT_EQ(lifecycle().epoch, second.epoch);
}

TEST_F(RunLifecycleTest, UsvBuoyTargetDrivesTransitAndRetargetsUntilLocked)
{
  auto & target_pub = target_pub_;
  banyu_msgs::msg::Task2BuoyTarget target;
  target.course_id = "course-a";
  target.source_seq = 1;
  target.latitude = 1.0011;
  target.longitude = 2.0012;
  target.heading_valid = true;
  target.heading_enu_rad = 0.5;
  target_pub->publish(target);
  ASSERT_TRUE(waitUntil([&target_pub]() {return target_pub->get_subscription_count() > 0;}));
  std::this_thread::sleep_for(200ms);

  ASSERT_NO_FATAL_FAILURE(start());
  const auto goal = [this](std::size_t index) {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      return navigation_handles_.at(index)->get_goal();
    };
  EXPECT_DOUBLE_EQ(goal(0)->target_latitude_deg, 1.0011);
  EXPECT_DOUBLE_EQ(goal(0)->target_longitude_deg, 2.0012);

  // A newer USV estimate about 3 m away re-sends the transit goal.
  target.source_seq = 2;
  target.latitude = 1.00113;
  target_pub->publish(target);
  ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 2U;}));
  EXPECT_DOUBLE_EQ(goal(1)->target_latitude_deg, 1.00113);

  // Sub-metre jitter does not.
  target.source_seq = 3;
  target.latitude = 1.001132;
  target_pub->publish(target);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(navigation_goals_.load(), 2U);
}

TEST_F(RunLifecycleNoBuoyTargetTest, StandsByOnTheSurfaceUntilTheUsvBuoyTargetArrives)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {
      const auto value = lifecycle();
      return value.run_id == 13 && value.phase == Lifecycle::ACTIVE;
    }));
  // No transit to the configured mission.target_* coordinates while the USV has not reported the active buoy.
  std::this_thread::sleep_for(1500ms);
  EXPECT_EQ(navigation_goals_.load(), 0U);
  EXPECT_EQ(lifecycle().phase, Lifecycle::ACTIVE);

  ASSERT_TRUE(waitUntil([this]() {return target_pub_->get_subscription_count() > 0;}));
  publishBuoyTarget(1.0011, 2.0012, 1);
  ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 1U;}));
  std::lock_guard<std::mutex> lock(navigation_mutex_);
  EXPECT_DOUBLE_EQ(navigation_handles_.at(0)->get_goal()->target_latitude_deg, 1.0011);
  EXPECT_DOUBLE_EQ(navigation_handles_.at(0)->get_goal()->target_longitude_deg, 2.0012);
}

class RunLifecycleStandoffTest : public RunLifecycleNoBuoyTargetTest
{
protected:
  double buoyStandoff() const override {return 1.5;}
};

TEST_F(RunLifecycleStandoffTest, SurfaceGoalIsTheStandoffInsideTheStartBuoyNeverTheBuoy)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::ACTIVE;}));
  ASSERT_TRUE(waitUntil([this]() {return target_pub_->get_subscription_count() > 0;}));
  banyu_msgs::msg::Task2BuoyTarget target;
  target.course_id = "course-a";
  target.source_seq = 1;
  target.latitude = 1.0011;
  target.longitude = 2.0012;
  target.heading_valid = true;
  target.heading_enu_rad = 0.0;   // the end buoy is due east
  target_pub_->publish(target);
  ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 1U;}));
  std::lock_guard<std::mutex> lock(navigation_mutex_);
  const auto goal = navigation_handles_.at(0)->get_goal();
  constexpr double kEarthRadiusM = 6378137.0;
  const double east_m = (goal->target_longitude_deg - 2.0012) * M_PI / 180.0 * kEarthRadiusM *
    std::cos(1.0011 * M_PI / 180.0);
  EXPECT_NEAR(east_m, 1.5, 0.01);
  EXPECT_NEAR(goal->target_latitude_deg, 1.0011, 1e-9);
}

class RunLifecycleMidpointTest : public RunLifecycleNoBuoyTargetTest
{
protected:
  double buoyStandoff() const override {return 1.5;}
  std::string divePoint() const override {return "midpoint";}
};

TEST_F(RunLifecycleMidpointTest, SurfaceGoalIsMidwayBetweenTheBuoys)
{
  ASSERT_NO_FATAL_FAILURE(prepare());
  command(13, 10);
  ASSERT_TRUE(waitUntil([this]() {return lifecycle().phase == Lifecycle::ACTIVE;}));
  ASSERT_TRUE(waitUntil([this]() {return target_pub_->get_subscription_count() > 0;}));
  banyu_msgs::msg::Task2BuoyTarget target;
  target.course_id = "course-a";
  target.source_seq = 1;
  target.latitude = 1.0011;
  target.longitude = 2.0012;
  target.heading_valid = true;
  target.heading_enu_rad = 0.0;
  target.other_valid = true;
  target.other_latitude = 1.0013;
  target.other_longitude = 2.0016;
  target_pub_->publish(target);
  ASSERT_TRUE(waitUntil([this]() {return navigation_goals_.load() == 1U;}));
  std::lock_guard<std::mutex> lock(navigation_mutex_);
  const auto goal = navigation_handles_.at(0)->get_goal();
  EXPECT_NEAR(goal->target_latitude_deg, 1.0012, 1e-9);
  EXPECT_NEAR(goal->target_longitude_deg, 2.0014, 1e-9);
}
