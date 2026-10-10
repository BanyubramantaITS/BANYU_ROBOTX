#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "banyu_communication/gcs_submission_adapter.hpp"
#include "banyu_communication/uuv_comm_node.hpp"
#include "banyu_mission/banyu_mission.hpp"
#include "banyu_mission/run_coordinator.hpp"
#include "banyu_msgs/action/navigate_surface_gps.hpp"
#include "banyu_msgs/msg/active_buoy_fix.hpp"
#include "banyu_msgs/msg/execution_readiness.hpp"
#include "banyu_msgs/msg/pipeline_end_state.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_ring_record.hpp"
#include "banyu_msgs/msg/task2_submission.hpp"
#include "banyu_msgs/msg/task2_survey_evidence.hpp"
#include "banyu_msgs/msg/task2_survey_progress.hpp"
#include "banyu_msgs/msg/vehicle_status.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "banyu_msgs/srv/reset_task2.hpp"
#include "banyu_msgs/srv/seal_task2_report.hpp"
#include "banyu_msgs/srv/trigger_pipeline_survey.hpp"
#include "banyu_msgs/task2_submission.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rx_msgs/msg/command.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rx_msgs/msg/pipeline_survey.hpp"
#include "rx_msgs/msg/resource_delivery.hpp"
#include "rx_msgs/msg/rx_task.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace
{

using Lifecycle = banyu_msgs::msg::RunLifecycle;
using Reset = banyu_msgs::srv::ResetTask2;
using Navigate = banyu_msgs::action::NavigateSurfaceGps;
using NavigateHandle = rclcpp_action::ServerGoalHandle<Navigate>;

bool waitUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return predicate();
}

static std::atomic<int> next_groot_port_{2666};

class ProductionMissionBoundaryTest : public ::testing::Test
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
    mission_journal_path_ = (temp_dir / ("banyu_prod_mission_" + pid_str + ".journal")).string();
    comm_outbox_path_ = (temp_dir / ("banyu_prod_outbox_" + pid_str)).string();
    adapter_journal_path_ = (temp_dir / ("banyu_prod_adapter_" + pid_str + ".journal")).string();
    test_bt_path_ = (temp_dir / ("banyu_prod_bt_" + pid_str + ".xml")).string();

    cleanupFiles();

    {
      std::ofstream bt_file(test_bt_path_);
      bt_file <<
        R"(<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <ReactiveFallback name="mission_root">
      <Sequence name="interrupt_sequence">
        <AutonomyInterrupted />
        <HandleAbortOrTeleop />
      </Sequence>
      <Sequence name="mission_phases">
        <SubTree ID="MissionPreflight" _autoremap="true" />
        <WaitSurveyComplete />
        <ConfirmSurfaceAction />
        <Sleep msec="10000" />
      </Sequence>
    </ReactiveFallback>
  </BehaviorTree>

  <BehaviorTree ID="MissionPreflight">
    <Sequence>
      <RequireMissionStarted />
      <RequireVehicleReady />
    </Sequence>
  </BehaviorTree>
</root>
)";
    }

    fixture_ = std::make_shared<rclcpp::Node>("prod_boundary_fixture");

    // Mock FCU service servers
    mode_server_ = fixture_->create_service<mavros_msgs::srv::SetMode>(
      "/mavros/set_mode",
      [this](
        const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
        std::shared_ptr<mavros_msgs::srv::SetMode::Response> resp)
      {
        std::lock_guard<std::mutex> lock(vehicle_mutex_);
        mode_ = req->custom_mode;
        resp->mode_sent = true;
      });

    arming_server_ = fixture_->create_service<mavros_msgs::srv::CommandBool>(
      "/mavros/cmd/arming",
      [this](
        const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,
        std::shared_ptr<mavros_msgs::srv::CommandBool::Response> resp)
      {
        armed_.store(req->value);
        resp->success = true;
      });

    // Mock reset servers
    tracker_reset_server_ = fixture_->create_service<Reset>(
      "/banyu/pipeline/reset_tracker",
      [](const std::shared_ptr<Reset::Request>/*req*/, std::shared_ptr<Reset::Response> resp)
      {
        resp->success = true;
        resp->message = "ok";
      });

    end_reset_server_ = fixture_->create_service<Reset>(
      "/banyu/pipeline/reset_end",
      [](const std::shared_ptr<Reset::Request>/*req*/, std::shared_ptr<Reset::Response> resp)
      {
        resp->success = true;
        resp->message = "ok";
      });

    // Mock navigation action server
    navigation_server_ = rclcpp_action::create_server<Navigate>(
      fixture_, "/banyu/actions/navigate_surface_gps",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<NavigateHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](const std::shared_ptr<NavigateHandle>) {
      });

    // Fixture publishers
    status_pub_ = fixture_->create_publisher<banyu_msgs::msg::VehicleStatus>(
      "/banyu/vehicle_status", rclcpp::QoS(5));
    pose_pub_ = fixture_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS());
    command_pub_ = fixture_->create_publisher<rx_msgs::msg::Command>(
      "/system/mission/command", rclcpp::QoS(10));
    execution_readiness_pub_ = fixture_->create_publisher<banyu_msgs::msg::ExecutionReadiness>(
      "/banyu/execution/readiness", rclcpp::QoS(5).reliable());
    survey_progress_pub_ = fixture_->create_publisher<banyu_msgs::msg::Task2SurveyProgress>(
      "/banyu/communication/survey_progress", rclcpp::QoS(5).reliable().transient_local());
    course_pub_ = fixture_->create_publisher<rx_msgs::msg::Course>(
      "/system/mission/course", rclcpp::QoS(1).reliable().transient_local());
    depth_pub_ = fixture_->create_publisher<banyu_msgs::msg::WaterDepth>(
      "/banyu/vehicle/water_depth", rclcpp::QoS(5).reliable());
    evidence_pub_ = fixture_->create_publisher<banyu_msgs::msg::Task2SurveyEvidence>(
      "/banyu/pipeline/survey_evidence", rclcpp::QoS(5).reliable().transient_local());
    end_state_pub_ = fixture_->create_publisher<banyu_msgs::msg::PipelineEndState>(
      "/banyu/pipeline/end_state", rclcpp::QoS(5).reliable().transient_local());

    fixture_cbg_ = fixture_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Fixture subscribers
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = fixture_cbg_;

    lifecycle_sub_ = fixture_->create_subscription<Lifecycle>(
      "/banyu/mission/run_lifecycle", rclcpp::QoS(10).reliable().transient_local(),
      [this](const Lifecycle::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        lifecycle_ = *msg;
        lifecycle_history_.push_back(*msg);
      },
      sub_opts);

    survey_sub_ = fixture_->create_subscription<rx_msgs::msg::PipelineSurvey>(
      "/system/vehicle/uuv/task2/pipeline_survey", rclcpp::QoS(10).reliable(),
      [this](const rx_msgs::msg::PipelineSurvey::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(survey_mutex_);
        received_surveys_.push_back(*msg);
      },
      sub_opts);

    delivery_sub_ = fixture_->create_subscription<rx_msgs::msg::ResourceDelivery>(
      "/system/vehicle/uuv/task2/delivery", rclcpp::QoS(10).reliable(),
      [this](const rx_msgs::msg::ResourceDelivery::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(survey_mutex_);
        received_deliveries_.push_back(*msg);
      },
      sub_opts);

    // Fixture service clients
    prepare_client_ = fixture_->create_client<std_srvs::srv::Trigger>(
      "/banyu/mission/prepare_autonomy", rclcpp::ServicesQoS(), fixture_cbg_);
    start_client_ = fixture_->create_client<std_srvs::srv::Trigger>(
      "/banyu/mission/start", rclcpp::ServicesQoS(), fixture_cbg_);
    abort_client_ = fixture_->create_client<std_srvs::srv::Trigger>(
      "/banyu/mission/abort", rclcpp::ServicesQoS(), fixture_cbg_);
    trigger_survey_client_ = fixture_->create_client<banyu_msgs::srv::TriggerPipelineSurvey>(
      "/banyu/communication/trigger_pipeline_survey", rclcpp::ServicesQoS(), fixture_cbg_);

    // Dedicated proxy node enabling controlled transport scheduling between UuvCommNode and BanyuMission
    proxy_node_ = std::make_shared<rclcpp::Node>("proxy_seal_node");
    proxy_cbg_ = proxy_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    seal_client_ = proxy_node_->create_client<banyu_msgs::srv::SealTask2Report>(
      "/banyu/mission/seal_task2_report", rclcpp::ServicesQoS(), proxy_cbg_);

    proxy_seal_server_ = proxy_node_->create_service<banyu_msgs::srv::SealTask2Report>(
      "/test_proxy/seal_task2_report",
      [this](
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<banyu_msgs::srv::SealTask2Report::Request> req)
      {
        if (!seal_client_->service_is_ready()) {
          banyu_msgs::srv::SealTask2Report::Response resp;
          resp.success = false;
          resp.message = "PROXY_BACKEND_UNAVAILABLE";
          proxy_seal_server_->send_response(*header, resp);
          return;
        }
        seal_client_->async_send_request(
          req,
          [this, header](rclcpp::Client<banyu_msgs::srv::SealTask2Report>::SharedFuture future) {
            auto resp = future.get();
            if (on_seal_response_hook_) {
              on_seal_response_hook_();
            }
            proxy_seal_server_->send_response(*header, *resp);
          });
      },
      rclcpp::ServicesQoS(),
      proxy_cbg_);

    // Periodic telemetry timer (vehicle status & pose)
    telemetry_timer_ = fixture_->create_wall_timer(20ms, [this]() {
          publishTelemetry();
    });

    // Publish initial course
    rx_msgs::msg::Course course;
    course.course_id = "course-prod-1";
    course.pinger_freq_hz = 25000;
    rx_msgs::msg::LatLng pt;
    pt.latitude = 33.810;
    pt.longitude = -118.393;
    course.boundary.push_back(pt);
    course_pub_->publish(course);

    // Initialize the three production nodes
    startProductionStack();

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, 4);
    executor_->add_node(fixture_);
    executor_->add_node(proxy_node_);
    executor_->add_node(mission_node_);
    executor_->add_node(comm_node_);
    executor_->add_node(adapter_node_);

    spin_thread_ = std::thread([this]() {executor_->spin();});

    // Wait until command publisher and lifecycle subscriber match
    ASSERT_TRUE(waitUntil([this]() {
        return command_pub_->get_subscription_count() > 0;
    }));
    ASSERT_TRUE(waitUntil([this]() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        return !lifecycle_history_.empty();
    }));
  }

  void TearDown() override
  {
    if (abort_client_->wait_for_service(100ms)) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      (void)abort_client_->async_send_request(req);
    }
    std::this_thread::sleep_for(50ms);

    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    if (executor_) {
      executor_->remove_node(fixture_);
      if (proxy_node_) {executor_->remove_node(proxy_node_);}
      if (mission_node_) {executor_->remove_node(mission_node_);}
      if (comm_node_) {executor_->remove_node(comm_node_);}
      if (adapter_node_) {executor_->remove_node(adapter_node_);}
    }

    mission_node_.reset();
    comm_node_.reset();
    adapter_node_.reset();
    telemetry_timer_.reset();
    on_seal_response_hook_ = nullptr;
    proxy_seal_server_.reset();
    seal_client_.reset();
    proxy_node_.reset();
    fixture_.reset();
    executor_.reset();

    cleanupFiles();
  }

  void cleanupFiles()
  {
    std::error_code ec;
    std::filesystem::remove(mission_journal_path_, ec);
    std::filesystem::remove(mission_journal_path_ + ".lock", ec);
    std::filesystem::remove_all(comm_outbox_path_, ec);
    std::filesystem::remove(adapter_journal_path_, ec);
    std::filesystem::remove(test_bt_path_, ec);
  }

  void startProductionStack()
  {
    // 1. Production BanyuMission Node
    rclcpp::NodeOptions mission_opts;
    mission_opts.parameter_overrides({
        rclcpp::Parameter("mission_mode", "task2"),
        rclcpp::Parameter("mission.start_latitude", 33.810),
        rclcpp::Parameter("mission.start_longitude", -118.393),
        rclcpp::Parameter("mission.target_latitude", 33.811),
        rclcpp::Parameter("mission.target_longitude", -118.392),
        rclcpp::Parameter("mission.end_latitude", 33.812),
        rclcpp::Parameter("mission.end_longitude", -118.391),
        rclcpp::Parameter("mission.task2_tier", 2),
        rclcpp::Parameter("run.journal_path", mission_journal_path_),
        rclcpp::Parameter("bt_xml", test_bt_path_),
        rclcpp::Parameter("timing.preparation_timeout_s", 10.0),
        rclcpp::Parameter("timing.run_reset_timeout_s", 2.0),
        rclcpp::Parameter("timing.execution_readiness_timeout_s", 1.0),
        rclcpp::Parameter("groot_port", next_groot_port_.fetch_add(4))
    });
    mission_node_ = std::make_shared<banyu_mission::BanyuMission>(mission_opts);
    mission_node_->initialize();

    // 2. Production UuvCommNode
    rclcpp::NodeOptions comm_opts;
    comm_opts.parameter_overrides({
        rclcpp::Parameter("heartbeat_rate_hz", 10.0),
        rclcpp::Parameter("survey.outbox_path", comm_outbox_path_),
        rclcpp::Parameter("survey.surface_hold_s", 0.05),
        rclcpp::Parameter("survey.surface_tolerance_m", 0.2),
        rclcpp::Parameter("survey.depth_timeout_s", 0.5),
        rclcpp::Parameter("survey.task2_tier", 2),
        rclcpp::Parameter("survey.publish_legacy_copies", false),
        rclcpp::Parameter("anchor_latitude", 33.810),
        rclcpp::Parameter("anchor_longitude", -118.393),
        rclcpp::Parameter("anchor_altitude_hae_m", 0.0),
        rclcpp::Parameter("topics.water_depth", "/banyu/vehicle/water_depth"),
        rclcpp::Parameter("topics.survey_evidence", "/banyu/pipeline/survey_evidence"),
        rclcpp::Parameter("topics.course", "/system/mission/course"),
        rclcpp::Parameter("topics.run_lifecycle", "/banyu/mission/run_lifecycle"),
        rclcpp::Parameter("topics.task2_submission", "/banyu/communication/task2_submission"),
        rclcpp::Parameter("services.seal_task2_report", "/test_proxy/seal_task2_report"),
        rclcpp::Parameter("services.trigger_pipeline_survey",
          "/banyu/communication/trigger_pipeline_survey")
    });
    comm_node_ = std::make_shared<banyu_communication::UuvCommNode>(comm_opts);

    // 3. Production GcsSubmissionAdapter
    rclcpp::NodeOptions adapter_opts;
    adapter_opts.parameter_overrides({
        rclcpp::Parameter("journal_path", adapter_journal_path_),
        rclcpp::Parameter("forward_to_rx_topics", true),
        rclcpp::Parameter("lifecycle_timeout_s", 5.0),
        rclcpp::Parameter("topics.task2_submission", "/banyu/communication/task2_submission"),
        rclcpp::Parameter("topics.run_lifecycle", "/banyu/mission/run_lifecycle"),
        rclcpp::Parameter("topics.command", "/system/mission/command"),
        rclcpp::Parameter("topics.pipeline_survey", "/system/vehicle/uuv/task2/pipeline_survey"),
        rclcpp::Parameter("topics.delivery", "/system/vehicle/uuv/task2/delivery")
    });
    adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(adapter_opts);
  }

  void publishTelemetry()
  {
    if (!fixture_) {return;}
    const auto stamp = fixture_->now();

    banyu_msgs::msg::VehicleStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = "uuv";
    {
      std::lock_guard<std::mutex> lock(vehicle_mutex_);
      status.mode = mode_;
      status.fcu_connected = fcu_connected_;
      status.fcu_status_fresh = fcu_status_fresh_;
      status.rc_fresh = rc_fresh_;
      status.armed = armed_.load();
      status.operator_takeover = operator_takeover_;
      status.authority = operator_takeover_ ? "MANUAL" : "AUTONOMY";
      status.gps_valid = gps_available_;
      status.gps_fresh = gps_available_;
    }
    status_pub_->publish(status);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = 10.0;
    pose.pose.position.y = 5.0;
    pose.pose.position.z = -0.1;
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);

    if (publish_hold_ack_.load()) {
      banyu_msgs::msg::RunLifecycle context;
      {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        context = lifecycle_;
      }
      if (context.phase == Lifecycle::READY) {
        banyu_msgs::msg::ExecutionReadiness readiness;
        readiness.header.stamp = stamp;
        readiness.header.frame_id = "uuv";
        readiness.authority_instance = context.authority_instance;
        readiness.run_id = context.run_id;
        readiness.epoch = context.epoch;
        readiness.lifecycle_revision = context.revision;
        readiness.holding = true;
        readiness.reason = "mock executor holding after preparation";
        execution_readiness_pub_->publish(readiness);
      }
    }

    if (depth_enabled_) {
      banyu_msgs::msg::WaterDepth depth;
      depth.header.stamp = stamp;
      {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        depth.run_id = lifecycle_.run_id;
        depth.epoch = lifecycle_.epoch;
      }
      depth.depth_m = current_depth_m_;
      depth.valid = true;
      depth.calibrated = true;
      depth.datum = "water_surface";
      depth.source_id = "pressure_depth";
      depth.calibration_id = "calib-prod-1";
      depth_pub_->publish(depth);
    }
  }

  void sendCommand(
    uint32_t run_id, uint32_t seq, uint8_t type,
    uint8_t target = rx_msgs::msg::VehicleType::TYPE_UUV)
  {
    rx_msgs::msg::Command cmd;
    cmd.header.stamp = fixture_->now();
    cmd.run_id = run_id;
    cmd.command_seq = seq;
    cmd.command_type = type;
    cmd.target_vehicle = target;
    command_pub_->publish(cmd);
  }

  bool executePreparation()
  {
    if (!prepare_client_->wait_for_service(2s)) {return false;}
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp;
    const bool accepted = waitUntil([this, &resp]() {
          auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
          auto future = prepare_client_->async_send_request(req);
          if (future.wait_for(500ms) == std::future_status::ready) {
            resp = future.get();
            return resp && resp->success;
          }
          return false;
    }, 4s);
    if (!accepted) {return false;}

    // Wait until mission tree ticks through arming and set_mode, publishing Ready phase with autonomous_ready
    return waitUntil([this]() {
               std::lock_guard<std::mutex> lock(lifecycle_mutex_);
               return lifecycle_.phase == Lifecycle::READY && lifecycle_.autonomous_ready;
    }, 4s);
  }

  void publishExecutionReadiness(bool holding = true)
  {
    banyu_msgs::msg::ExecutionReadiness msg;
    msg.header.stamp = fixture_->now();
    msg.header.frame_id = "uuv";
    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      msg.authority_instance = lifecycle_.authority_instance;
      msg.run_id = lifecycle_.run_id;
      msg.epoch = lifecycle_.epoch;
      msg.lifecycle_revision = lifecycle_.revision;
    }
    msg.holding = holding;
    execution_readiness_pub_->publish(msg);
  }

  std::shared_ptr<rclcpp::Node> fixture_;
  std::shared_ptr<rclcpp::Node> proxy_node_;
  rclcpp::CallbackGroup::SharedPtr fixture_cbg_;
  rclcpp::CallbackGroup::SharedPtr proxy_cbg_;
  std::shared_ptr<banyu_mission::BanyuMission> mission_node_;
  std::shared_ptr<banyu_communication::UuvCommNode> comm_node_;
  std::shared_ptr<banyu_communication::GcsSubmissionAdapter> adapter_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::string mission_journal_path_;
  std::string comm_outbox_path_;
  std::string adapter_journal_path_;

  // Fixture communication
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr mode_server_;
  rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_server_;
  rclcpp::Service<Reset>::SharedPtr tracker_reset_server_;
  rclcpp::Service<Reset>::SharedPtr end_reset_server_;
  rclcpp::Service<banyu_msgs::srv::SealTask2Report>::SharedPtr proxy_seal_server_;
  std::function<void()> on_seal_response_hook_;
  rclcpp_action::Server<Navigate>::SharedPtr navigation_server_;

  rclcpp::Publisher<banyu_msgs::msg::VehicleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<rx_msgs::msg::Command>::SharedPtr command_pub_;
  rclcpp::Publisher<banyu_msgs::msg::ExecutionReadiness>::SharedPtr execution_readiness_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2SurveyProgress>::SharedPtr survey_progress_pub_;
  rclcpp::Publisher<rx_msgs::msg::Course>::SharedPtr course_pub_;
  rclcpp::Publisher<banyu_msgs::msg::WaterDepth>::SharedPtr depth_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2SurveyEvidence>::SharedPtr evidence_pub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineEndState>::SharedPtr end_state_pub_;
  std::string test_bt_path_;
  std::atomic<bool> publish_hold_ack_{true};

  rclcpp::Subscription<Lifecycle>::SharedPtr lifecycle_sub_;
  rclcpp::Subscription<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_sub_;
  rclcpp::Subscription<rx_msgs::msg::ResourceDelivery>::SharedPtr delivery_sub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr prepare_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr abort_client_;
  rclcpp::Client<banyu_msgs::srv::TriggerPipelineSurvey>::SharedPtr trigger_survey_client_;
  rclcpp::Client<banyu_msgs::srv::SealTask2Report>::SharedPtr seal_client_;

  rclcpp::TimerBase::SharedPtr telemetry_timer_;

  std::mutex vehicle_mutex_;
  std::string mode_{"GUIDED"};
  bool fcu_connected_{true};
  bool fcu_status_fresh_{true};
  bool rc_fresh_{true};
  std::atomic<bool> armed_{true};
  bool operator_takeover_{false};
  bool gps_available_{true};

  std::atomic<bool> depth_enabled_{true};
  double current_depth_m_{0.05};

  std::mutex lifecycle_mutex_;
  Lifecycle lifecycle_;
  std::vector<Lifecycle> lifecycle_history_;

  std::mutex survey_mutex_;
  std::vector<rx_msgs::msg::PipelineSurvey> received_surveys_;
  std::vector<rx_msgs::msg::ResourceDelivery> received_deliveries_;
};

// 1. Ready/preflight setup through real interfaces; addressed nonzero CMD_RUN_START accepted
TEST_F(ProductionMissionBoundaryTest, ReadyPreflightSetupAcceptsRunStartAndAttributesContext)
{
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);

  // Send addressed nonzero CMD_RUN_START
  sendCommand(15, 1, rx_msgs::msg::Command::CMD_RUN_START);

  // Mission must accept start, execute reset consumers, and transition to ACTIVE
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.run_id, 15u);
    EXPECT_EQ(lifecycle_.current_task, rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR);
    EXPECT_TRUE(lifecycle_.autonomous_ready);
    EXPECT_GT(lifecycle_.epoch, 0u);
  }

  // Adapter must have accepted run 15 context
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
}

// 2. Direct local start cannot bypass Task 2 GCS gate; zero, foreign-target, duplicate and conflicting starts handled
TEST_F(ProductionMissionBoundaryTest, DirectLocalStartCannotBypassGcsGateAndStartVariationsHandled)
{
  // Direct local start service must be rejected in Task 2 mode
  ASSERT_TRUE(start_client_->wait_for_service(500ms));
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = start_client_->async_send_request(req);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_NE(resp->message.find("GCS_START_REQUIRED"), std::string::npos);

  // Zero run_id CMD_RUN_START is rejected
  sendCommand(0, 1, rx_msgs::msg::Command::CMD_RUN_START);
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.run_id, 0u);
  }

  // Foreign-target (TYPE_UAV) is ignored
  sendCommand(15, 2, rx_msgs::msg::Command::CMD_RUN_START, rx_msgs::msg::VehicleType::TYPE_UAV);
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.run_id, 0u);
  }

  // Valid preflight & start
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(15, 3, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  // Duplicate start for same run_id & seq
  sendCommand(15, 3, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.event == banyu_msgs::msg::RunLifecycle::START_DUPLICATE;
  }, 1s));
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);

  // Conflicting start for different run_id with same seq
  sendCommand(99, 3, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.event == banyu_msgs::msg::RunLifecycle::START_REJECTED;
  }, 1s));
  // Accepted run 15 preserved
  EXPECT_EQ(adapter_node_->activeRunId(), 15u);
}

// 3. Abort, internal stop/mission-done and readiness loss clear current_task to TASK_NONE
TEST_F(ProductionMissionBoundaryTest, AbortStopAndReadinessLossClearCurrentTaskToTaskNone)
{
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(15, 1, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  // 3a. Call /banyu/mission/abort service
  ASSERT_TRUE(abort_client_->wait_for_service(500ms));
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = abort_client_->async_send_request(req);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);

  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.phase == Lifecycle::INTERRUPTED;
  }, 2s));

  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.current_task, rx_msgs::msg::RxTask::TASK_NONE);
    EXPECT_EQ(lifecycle_.event, banyu_msgs::msg::RunLifecycle::TERMINAL);
  }

  // 3b. CMD_MISSION_DONE stop command terminates run and clears current_task to TASK_NONE
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(16, 2, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 16u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.current_task, rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR);
  }

  sendCommand(16, 3, rx_msgs::msg::Command::CMD_MISSION_DONE);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.phase == Lifecycle::INTERRUPTED;
  }, 2s));
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.current_task, rx_msgs::msg::RxTask::TASK_NONE);
    EXPECT_EQ(lifecycle_.event, banyu_msgs::msg::RunLifecycle::TERMINAL);
  }

  // 3c. Readiness loss clears current_task to TASK_NONE and stale callbacks cannot restore activity
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(17, 4, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 17u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  // Drop FCU fresh and RC fresh status
  {
    std::lock_guard<std::mutex> lock(vehicle_mutex_);
    rc_fresh_ = false;
    fcu_status_fresh_ = false;
  }
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.phase == Lifecycle::INTERRUPTED;
  }, 2s));
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.current_task, rx_msgs::msg::RxTask::TASK_NONE);
  }

  // Restore readiness - verify stale telemetry callbacks cannot restore active status
  {
    std::lock_guard<std::mutex> lock(vehicle_mutex_);
    rc_fresh_ = true;
    fcu_status_fresh_ = true;
  }
  std::this_thread::sleep_for(150ms);
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    EXPECT_EQ(lifecycle_.phase, Lifecycle::INTERRUPTED);
    EXPECT_EQ(lifecycle_.current_task, rx_msgs::msg::RxTask::TASK_NONE);
  }
}

// 4. Missing/incomplete evidence returns failure diagnostic and emits no public survey
TEST_F(ProductionMissionBoundaryTest, IncompleteEvidenceReturnsDiagnosticAndEmitsNoSurvey)
{
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(15, 1, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  uint64_t active_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    active_epoch = lifecycle_.epoch;
  }

  // 4a. Missing evidence: trigger survey with no evidence published
  ASSERT_TRUE(trigger_survey_client_->wait_for_service(500ms));
  auto req_missing = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req_missing->run_id = 15;
  req_missing->epoch = active_epoch;
  req_missing->report_id = "test-fail-missing-report";
  auto fut_missing = trigger_survey_client_->async_send_request(req_missing);
  ASSERT_EQ(fut_missing.wait_for(1s), std::future_status::ready);
  auto resp_missing = fut_missing.get();
  EXPECT_FALSE(resp_missing->success);
  EXPECT_NE(resp_missing->result_code, banyu_msgs::srv::TriggerPipelineSurvey::Response::ACCEPTED);

  // 4b. Incomplete evidence: only 2 rings out of 4 published
  banyu_msgs::msg::Task2SurveyEvidence inc_ev;
  inc_ev.header.stamp = fixture_->now();
  inc_ev.header.frame_id = "uuv";
  inc_ev.run_id = 15;
  inc_ev.epoch = active_epoch;
  inc_ev.revision = 1;
  inc_ev.source_id = "pipeline_topology";
  inc_ev.low_progress_endpoint_id = "start_buoy";
  inc_ev.high_progress_endpoint_id = "end_buoy";
  inc_ev.traversal_complete = false;
  inc_ev.tier = 2;
  inc_ev.active_buoy.header.stamp = inc_ev.header.stamp;
  inc_ev.active_buoy.run_id = 15;
  inc_ev.active_buoy.epoch = active_epoch;
  inc_ev.active_buoy.revision = 1;
  inc_ev.active_buoy.source_id = "uuv_buoy_localizer";
  inc_ev.active_buoy.course_id = "course-prod-1";
  inc_ev.active_buoy.endpoint_id = "start_buoy";
  inc_ev.active_buoy.fix.latitude = 33.810;
  inc_ev.active_buoy.fix.longitude = -118.393;
  inc_ev.active_buoy.uncertainty_m = 0.3;
  inc_ev.active_buoy.green_confirmed = true;
  inc_ev.active_buoy.pinger_confirmed = true;
  inc_ev.active_buoy.pinger_freq_hz = 25000;
  for (uint32_t i = 0; i < 2; ++i) {
    banyu_msgs::msg::Task2RingRecord ring;
    ring.run_id = 15;
    ring.epoch = active_epoch;
    ring.node_id = 100 + i;
    ring.frame_sequence = i + 1;
    ring.physical_identity_verified = true;
    ring.ordering_verified = true;
    ring.pipeline_progress_m = 1.0 + i * 2.0;
    ring.color = 2;
    ring.confidence = 0.95F;
    ring.vehicle_pose_map.header.stamp = inc_ev.header.stamp;
    ring.vehicle_pose_map.header.frame_id = "map";
    ring.vehicle_pose_map.pose.position.x = 1.0 + i * 2.0;
    ring.vehicle_pose_map.pose.position.z = -1.5;
    ring.vehicle_pose_map.pose.orientation.w = 1.0;
    inc_ev.rings.push_back(ring);
  }
  evidence_pub_->publish(inc_ev);
  std::this_thread::sleep_for(100ms);

  auto req_inc = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req_inc->run_id = 15;
  req_inc->epoch = active_epoch;
  req_inc->report_id = "test-fail-incomplete-report";
  auto fut_inc = trigger_survey_client_->async_send_request(req_inc);
  ASSERT_EQ(fut_inc.wait_for(1s), std::future_status::ready);
  auto resp_inc = fut_inc.get();
  EXPECT_FALSE(resp_inc->success);
  EXPECT_NE(resp_inc->result_code, banyu_msgs::srv::TriggerPipelineSurvey::Response::ACCEPTED);

  // 4c. Submerged depth (5.0m) / unverified surfacing
  current_depth_m_ = 5.0;
  std::this_thread::sleep_for(100ms);

  auto req_sub = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req_sub->run_id = 15;
  req_sub->epoch = active_epoch;
  req_sub->report_id = "test-fail-depth-report";
  auto fut_sub = trigger_survey_client_->async_send_request(req_sub);
  ASSERT_EQ(fut_sub.wait_for(1s), std::future_status::ready);
  auto resp_sub = fut_sub.get();
  EXPECT_FALSE(resp_sub->success);
  EXPECT_EQ(resp_sub->result_code,
      banyu_msgs::srv::TriggerPipelineSurvey::Response::SURVEY_INVALID);

  // Ensure no public survey was published across all failure paths
  {
    std::lock_guard<std::mutex> lock(survey_mutex_);
    EXPECT_TRUE(received_surveys_.empty());
  }
}

// 5. Qualified four-node evidence and verified surfacing allows real seal and publication
TEST_F(ProductionMissionBoundaryTest, CompleteQualifiedEvidenceAndSurfacingPublishesSurvey)
{
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(15, 1, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  uint64_t active_epoch = 0;
  std::string auth_inst;
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    active_epoch = lifecycle_.epoch;
    auth_inst = lifecycle_.authority_instance;
  }

  // Publish qualified four-node survey evidence to UuvCommNode
  banyu_msgs::msg::Task2SurveyEvidence ev;
  ev.header.stamp = fixture_->now();
  ev.header.frame_id = "uuv";
  ev.run_id = 15;
  ev.epoch = active_epoch;
  ev.revision = 1;
  ev.source_id = "pipeline_topology";
  ev.low_progress_endpoint_id = "start_buoy";
  ev.high_progress_endpoint_id = "end_buoy";
  ev.traversal_complete = true;
  ev.tier = 2;

  ev.active_buoy.header.stamp = ev.header.stamp;
  ev.active_buoy.run_id = 15;
  ev.active_buoy.epoch = active_epoch;
  ev.active_buoy.revision = 1;
  ev.active_buoy.source_id = "uuv_buoy_localizer";
  ev.active_buoy.course_id = "course-prod-1";
  ev.active_buoy.endpoint_id = "start_buoy";
  ev.active_buoy.fix.header.stamp = ev.header.stamp;
  ev.active_buoy.fix.status.status = 1;
  ev.active_buoy.fix.latitude = 33.810;
  ev.active_buoy.fix.longitude = -118.393;
  ev.active_buoy.uncertainty_m = 0.3;
  ev.active_buoy.green_confirmed = true;
  ev.active_buoy.pinger_confirmed = true;
  ev.active_buoy.pinger_freq_hz = 25000;

  const std::array<double, 4> ring_progress{0.5, 2.5, 5.0, 7.5};
  for (uint32_t i = 0; i < 4; ++i) {
    banyu_msgs::msg::Task2RingRecord ring;
    ring.run_id = 15;
    ring.epoch = active_epoch;
    ring.node_id = 100 + i;
    ring.frame_sequence = i + 1;
    ring.physical_identity_verified = true;
    ring.ordering_verified = true;
    ring.pipeline_progress_m = ring_progress[i];
    ring.color = (i == 1) ? 1 : 2;
    ring.confidence = 0.95F;
    ring.vehicle_pose_map.header.stamp = ev.header.stamp;
    ring.vehicle_pose_map.header.frame_id = "map";
    ring.vehicle_pose_map.pose.position.x = ring_progress[i];
    ring.vehicle_pose_map.pose.position.y = 0.0;
    ring.vehicle_pose_map.pose.position.z = -1.5;
    ring.vehicle_pose_map.pose.orientation.w = 1.0;
    ev.rings.push_back(ring);
  }
  evidence_pub_->publish(ev);

  banyu_msgs::msg::PipelineEndState end_state;
  end_state.header.stamp = fixture_->now();
  end_state.run_id = 15;
  end_state.epoch = active_epoch;
  end_state.ended = true;
  end_state_pub_->publish(end_state);

  // Publish survey progress to BanyuMission
  banyu_msgs::msg::Task2SurveyProgress progress_msg;
  progress_msg.header.stamp = fixture_->now();
  progress_msg.authority_instance = auth_inst;
  progress_msg.run_id = 15;
  progress_msg.epoch = active_epoch;
  progress_msg.evidence_revision = 1;
  progress_msg.survey_complete = true;
  survey_progress_pub_->publish(progress_msg);

  // Surface depth telemetry
  current_depth_m_ = 0.05;

  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.phase == Lifecycle::SURFACED;
  }, 4s));

  std::this_thread::sleep_for(200ms);

  // Trigger survey reporting through UuvCommNode -> real seal_task2_report service on BanyuMission -> GcsSubmissionAdapter
  ASSERT_TRUE(trigger_survey_client_->wait_for_service(500ms));
  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch;
  req->report_id = "task2-survey-15-" + std::to_string(active_epoch);
  auto future = trigger_survey_client_->async_send_request(req);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  auto resp = future.get();
  EXPECT_TRUE(resp->success) << "Trigger pipeline survey failed: " << resp->message;

  // Verify public survey received on GCS topic
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(survey_mutex_);
      return !received_surveys_.empty();
  }, 2s));

  {
    std::lock_guard<std::mutex> lock(survey_mutex_);
    ASSERT_EQ(received_surveys_.size(), 1u);
    EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.latitude, 33.810);
    EXPECT_DOUBLE_EQ(received_surveys_[0].active_buoy_position.longitude, -118.393);
    ASSERT_EQ(received_surveys_[0].segments.size(), 4u);
    EXPECT_EQ(received_surveys_[0].segments[0], 1u);
    EXPECT_EQ(received_surveys_[0].segments[1], 2u);
    EXPECT_EQ(received_surveys_[0].segments[2], 1u);
    EXPECT_EQ(received_surveys_[0].segments[3], 1u);
  }

  // Terminate run 15 cleanly before preparing run 16
  sendCommand(15, 2, rx_msgs::msg::Command::CMD_MISSION_DONE);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.phase == Lifecycle::INTERRUPTED;
  }, 2s));

  // 5b. Cover the other active end: high-progress endpoint ("end_buoy")
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(16, 3, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 16u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  uint64_t active_epoch2 = 0;
  std::string auth_inst2;
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    active_epoch2 = lifecycle_.epoch;
    auth_inst2 = lifecycle_.authority_instance;
  }

  // Active buoy at high progress endpoint ("end_buoy")
  banyu_msgs::msg::Task2SurveyEvidence ev2;
  ev2.header.stamp = fixture_->now();
  ev2.header.frame_id = "uuv";
  ev2.run_id = 16;
  ev2.epoch = active_epoch2;
  ev2.revision = 1;
  ev2.source_id = "pipeline_topology";
  ev2.low_progress_endpoint_id = "start_buoy";
  ev2.high_progress_endpoint_id = "end_buoy";
  ev2.traversal_complete = true;
  ev2.tier = 2;

  ev2.active_buoy.header.stamp = ev2.header.stamp;
  ev2.active_buoy.run_id = 16;
  ev2.active_buoy.epoch = active_epoch2;
  ev2.active_buoy.revision = 1;
  ev2.active_buoy.source_id = "uuv_buoy_localizer";
  ev2.active_buoy.course_id = "course-prod-1";
  ev2.active_buoy.endpoint_id = "end_buoy";
  ev2.active_buoy.fix.header.stamp = ev2.header.stamp;
  ev2.active_buoy.fix.status.status = 1;
  ev2.active_buoy.fix.latitude = 33.812;
  ev2.active_buoy.fix.longitude = -118.391;
  ev2.active_buoy.uncertainty_m = 0.3;
  ev2.active_buoy.green_confirmed = true;
  ev2.active_buoy.pinger_confirmed = true;
  ev2.active_buoy.pinger_freq_hz = 25000;

  for (uint32_t i = 0; i < 4; ++i) {
    banyu_msgs::msg::Task2RingRecord ring;
    ring.run_id = 16;
    ring.epoch = active_epoch2;
    ring.node_id = 200 + i;
    ring.frame_sequence = i + 1;
    ring.physical_identity_verified = true;
    ring.ordering_verified = true;
    ring.pipeline_progress_m = ring_progress[i];
    ring.color = (i == 1) ? 1 : 2;
    ring.confidence = 0.95F;
    ring.vehicle_pose_map.header.stamp = ev2.header.stamp;
    ring.vehicle_pose_map.header.frame_id = "map";
    ring.vehicle_pose_map.pose.position.x = ring_progress[i];
    ring.vehicle_pose_map.pose.orientation.w = 1.0;
    ev2.rings.push_back(ring);
  }
  evidence_pub_->publish(ev2);

  banyu_msgs::msg::PipelineEndState end_state2;
  end_state2.header.stamp = fixture_->now();
  end_state2.run_id = 16;
  end_state2.epoch = active_epoch2;
  end_state2.ended = true;
  end_state_pub_->publish(end_state2);

  banyu_msgs::msg::Task2SurveyProgress progress_msg2;
  progress_msg2.header.stamp = fixture_->now();
  progress_msg2.authority_instance = auth_inst2;
  progress_msg2.run_id = 16;
  progress_msg2.epoch = active_epoch2;
  progress_msg2.evidence_revision = 1;
  progress_msg2.survey_complete = true;
  survey_progress_pub_->publish(progress_msg2);

  current_depth_m_ = 0.05;

  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 16u && lifecycle_.phase == Lifecycle::SURFACED;
  }, 4s));

  std::this_thread::sleep_for(200ms);

  auto req2 = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req2->run_id = 16;
  req2->epoch = active_epoch2;
  req2->report_id = "task2-survey-16-" + std::to_string(active_epoch2);
  auto future2 = trigger_survey_client_->async_send_request(req2);
  ASSERT_EQ(future2.wait_for(2s), std::future_status::ready);
  auto resp2 = future2.get();
  EXPECT_TRUE(resp2->success) << "Trigger pipeline survey failed: " << resp2->message;

  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(survey_mutex_);
      return received_surveys_.size() >= 2u;
  }, 2s));

  {
    std::lock_guard<std::mutex> lock(survey_mutex_);
    ASSERT_EQ(received_surveys_.size(), 2u);
    EXPECT_DOUBLE_EQ(received_surveys_[1].active_buoy_position.latitude, 33.812);
    EXPECT_DOUBLE_EQ(received_surveys_[1].active_buoy_position.longitude, -118.391);
    ASSERT_EQ(received_surveys_[1].segments.size(), 4u);
    // Descending order from end_buoy: [1, 1, 2, 1]
    EXPECT_EQ(received_surveys_[1].segments[0], 1u);
    EXPECT_EQ(received_surveys_[1].segments[1], 1u);
    EXPECT_EQ(received_surveys_[1].segments[2], 2u);
    EXPECT_EQ(received_surveys_[1].segments[3], 1u);
  }
}

// 6. Abort before durable sealing rejects reporting; abort after durable sealing preserves record
TEST_F(ProductionMissionBoundaryTest, AbortBeforeSealingRejectsReportAndAbortAfterPreserves)
{
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(15, 1, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  uint64_t active_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    active_epoch = lifecycle_.epoch;
  }

  // Abort run before sealing
  auto abort_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto abort_future = abort_client_->async_send_request(abort_req);
  ASSERT_EQ(abort_future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(abort_future.get()->success);

  // Calling seal service on BanyuMission must be rejected
  ASSERT_TRUE(seal_client_->wait_for_service(500ms));
  auto seal_req = std::make_shared<banyu_msgs::srv::SealTask2Report::Request>();
  seal_req->submission.schema_version = 1;
  seal_req->submission.run_id = 15;
  seal_req->submission.epoch = active_epoch;
  seal_req->submission.report_id = "aborted-seal-report";
  seal_req->submission.course_id = "course-prod-1";
  seal_req->submission.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  seal_req->submission.tier = 2;
  seal_req->submission.buoy_latitude = 33.810;
  seal_req->submission.buoy_longitude = -118.393;
  seal_req->submission.original_conditions = {1, 2, 1, 1};
  seal_req->submission.authorization_revision = 1;

  auto seal_future = seal_client_->async_send_request(seal_req);
  ASSERT_EQ(seal_future.wait_for(1s), std::future_status::ready);
  auto seal_resp = seal_future.get();
  EXPECT_FALSE(seal_resp->success);
  EXPECT_EQ(seal_resp->message, "RUN_NOT_ELIGIBLE_FOR_REPORT");
}

// 7. Controlled response scheduling distinguishing seal-then-abort from delayed transport
TEST_F(ProductionMissionBoundaryTest, DurableSealPreservedWhenAbortFollowsDurableSeal)
{
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(15, 1, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  uint64_t active_epoch = 0;
  std::string auth_inst;
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    active_epoch = lifecycle_.epoch;
    auth_inst = lifecycle_.authority_instance;
  }

  // Provide evidence and surfacing
  banyu_msgs::msg::Task2SurveyEvidence ev;
  ev.header.stamp = fixture_->now();
  ev.header.frame_id = "uuv";
  ev.run_id = 15;
  ev.epoch = active_epoch;
  ev.revision = 1;
  ev.source_id = "pipeline_topology";
  ev.low_progress_endpoint_id = "start_buoy";
  ev.high_progress_endpoint_id = "end_buoy";
  ev.traversal_complete = true;
  ev.tier = 2;

  ev.active_buoy.header.stamp = ev.header.stamp;
  ev.active_buoy.run_id = 15;
  ev.active_buoy.epoch = active_epoch;
  ev.active_buoy.revision = 1;
  ev.active_buoy.source_id = "uuv_buoy_localizer";
  ev.active_buoy.course_id = "course-prod-1";
  ev.active_buoy.endpoint_id = "start_buoy";
  ev.active_buoy.fix.header.stamp = ev.header.stamp;
  ev.active_buoy.fix.status.status = 1;
  ev.active_buoy.fix.latitude = 33.810;
  ev.active_buoy.fix.longitude = -118.393;
  ev.active_buoy.uncertainty_m = 0.3;
  ev.active_buoy.green_confirmed = true;
  ev.active_buoy.pinger_confirmed = true;
  ev.active_buoy.pinger_freq_hz = 25000;

  const std::array<double, 4> ring_progress{0.5, 2.5, 5.0, 7.5};
  for (uint32_t i = 0; i < 4; ++i) {
    banyu_msgs::msg::Task2RingRecord ring;
    ring.run_id = 15;
    ring.epoch = active_epoch;
    ring.node_id = 100 + i;
    ring.frame_sequence = i + 1;
    ring.physical_identity_verified = true;
    ring.ordering_verified = true;
    ring.pipeline_progress_m = ring_progress[i];
    ring.color = (i == 1) ? 1 : 2;
    ring.confidence = 0.95F;
    ring.vehicle_pose_map.header.stamp = ev.header.stamp;
    ring.vehicle_pose_map.header.frame_id = "map";
    ring.vehicle_pose_map.pose.position.x = ring_progress[i];
    ring.vehicle_pose_map.pose.position.y = 0.0;
    ring.vehicle_pose_map.pose.position.z = -1.5;
    ring.vehicle_pose_map.pose.orientation.w = 1.0;
    ev.rings.push_back(ring);
  }
  evidence_pub_->publish(ev);

  banyu_msgs::msg::PipelineEndState end_state;
  end_state.header.stamp = fixture_->now();
  end_state.run_id = 15;
  end_state.epoch = active_epoch;
  end_state.ended = true;
  end_state_pub_->publish(end_state);

  banyu_msgs::msg::Task2SurveyProgress progress_msg;
  progress_msg.header.stamp = fixture_->now();
  progress_msg.authority_instance = auth_inst;
  progress_msg.run_id = 15;
  progress_msg.epoch = active_epoch;
  progress_msg.evidence_revision = 1;
  progress_msg.survey_complete = true;
  survey_progress_pub_->publish(progress_msg);

  current_depth_m_ = 0.05;

  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.phase == Lifecycle::SURFACED;
  }, 4s));

  std::this_thread::sleep_for(200ms);

  // Controlled response scheduling at the transport boundary:
  // Hook executes after BanyuMission durably seals the report, but before the seal response is returned to UuvCommNode
  std::atomic<bool> seal_hook_executed{false};
  on_seal_response_hook_ = [this, &seal_hook_executed]() {
      auto abort_req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto abort_fut = abort_client_->async_send_request(abort_req);
      (void)abort_fut.wait_for(1s);
      (void)waitUntil([this]() {
          std::lock_guard<std::mutex> lock(lifecycle_mutex_);
          return lifecycle_.phase == Lifecycle::INTERRUPTED;
    }, 2s);
      seal_hook_executed.store(true);
    };

  // Trigger survey reporting through UuvCommNode -> proxy -> real BanyuMission seal service
  auto req = std::make_shared<banyu_msgs::srv::TriggerPipelineSurvey::Request>();
  req->run_id = 15;
  req->epoch = active_epoch;
  req->report_id = "delayed-transport-abort-report";
  auto future = trigger_survey_client_->async_send_request(req);
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  auto resp = future.get();

  EXPECT_TRUE(seal_hook_executed.load());
  // Because the mission aborted while the seal response was delayed at the transport boundary,
  // UuvCommNode preserves the report in outbox but returns RUN_NOT_ACTIVE
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->result_code, banyu_msgs::srv::TriggerPipelineSurvey::Response::RUN_NOT_ACTIVE);
  EXPECT_NE(resp->message.find("RUN_NOT_ACTIVE"), std::string::npos);

  // Public survey was NOT published immediately
  {
    std::lock_guard<std::mutex> lock(survey_mutex_);
    EXPECT_TRUE(received_surveys_.empty());
  }

  // Stop mission node to release the exclusive journal lock before inspecting journal
  executor_->remove_node(mission_node_);
  mission_node_.reset();

  // But the report was durably sealed in BanyuMission's RunCoordinator journal
  banyu_mission::RunCoordinator coord(mission_journal_path_);
  EXPECT_TRUE(coord.hasSealedReport(15, active_epoch, 1));

  on_seal_response_hook_ = nullptr;
}

// 8. Adapter restart and matching retained mission snapshot recover eligible PENDING data; ATTEMPTED deduplicated
TEST_F(ProductionMissionBoundaryTest, AdapterRestartRecoversPendingUnderMatchingRetainedSnapshot)
{
  ASSERT_TRUE(executePreparation());
  publishExecutionReadiness(true);
  sendCommand(15, 1, rx_msgs::msg::Command::CMD_RUN_START);
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return lifecycle_.run_id == 15u && lifecycle_.phase == Lifecycle::ACTIVE;
  }, 3s));

  uint64_t active_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    active_epoch = lifecycle_.epoch;
  }

  // Pre-seed adapter journal with pending report
  banyu_msgs::msg::Task2Submission sub;
  sub.header.stamp = fixture_->now();
  sub.header.frame_id = "uuv";
  sub.schema_version = 1;
  sub.run_id = 15;
  sub.epoch = active_epoch;
  sub.report_id = "restart-retained-recovery";
  sub.kind = banyu_msgs::msg::Task2Submission::SURVEY;
  sub.course_id = "course-prod-1";
  sub.tier = 2;
  sub.buoy_latitude = 33.810;
  sub.buoy_longitude = -118.393;
  sub.original_conditions = {1, 2, 1, 1};
  sub.authorization_revision = 1;

  std::ofstream j(adapter_journal_path_, std::ios::app);
  ASSERT_TRUE(j.is_open());
  j << "BANYU_GCS_ADAPTER_ENTRY_V3 15 " << active_epoch << " 1 restart-retained-recovery 1 PENDING "
    << std::quoted(banyu_msgs::task2::canonicalPayload(sub)) << "\n";
  j.close();

  // Stop and restart adapter node
  executor_->remove_node(adapter_node_);
  adapter_node_.reset();

  rclcpp::NodeOptions adapter_opts;
  adapter_opts.parameter_overrides({
      rclcpp::Parameter("journal_path", adapter_journal_path_),
      rclcpp::Parameter("forward_to_rx_topics", true),
      rclcpp::Parameter("lifecycle_timeout_s", 5.0),
      rclcpp::Parameter("topics.task2_submission", "/banyu/communication/task2_submission"),
      rclcpp::Parameter("topics.run_lifecycle", "/banyu/mission/run_lifecycle"),
      rclcpp::Parameter("topics.command", "/system/mission/command"),
      rclcpp::Parameter("topics.pipeline_survey", "/system/vehicle/uuv/task2/pipeline_survey"),
      rclcpp::Parameter("topics.delivery", "/system/vehicle/uuv/task2/delivery")
  });
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(adapter_opts);
  executor_->add_node(adapter_node_);

  // Adapter receives transient_local retained RunLifecycle from BanyuMission and recovers report
  ASSERT_TRUE(waitUntil([this]() {
      std::lock_guard<std::mutex> lock(survey_mutex_);
      return !received_surveys_.empty();
  }, 3s));

  {
    std::lock_guard<std::mutex> lock(survey_mutex_);
    EXPECT_EQ(received_surveys_.size(), 1u);
  }

  // 8b. Already ATTEMPTED report is deduplicated upon subsequent restart (not re-forwarded)
  executor_->remove_node(adapter_node_);
  adapter_node_.reset();
  adapter_node_ = std::make_shared<banyu_communication::GcsSubmissionAdapter>(adapter_opts);
  executor_->add_node(adapter_node_);
  std::this_thread::sleep_for(200ms);
  {
    std::lock_guard<std::mutex> lock(survey_mutex_);
    EXPECT_EQ(received_surveys_.size(), 1u);
  }

  // 8c. Submissions with wrong epoch remain blocked in integrated path
  EXPECT_FALSE(adapter_node_->hasValidAuthority(15, active_epoch + 1));
}

} // namespace
