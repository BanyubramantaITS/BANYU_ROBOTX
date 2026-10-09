#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <rx_msgs/msg/robot_state.hpp>
#include <rx_msgs/msg/vehicle_type.hpp>

#include "banyu_communication/uuv_comm_node.hpp"

using namespace std::chrono_literals;
using banyu_communication::UuvCommNode;
using rx_msgs::msg::Command;
using rx_msgs::msg::RobotState;
using rx_msgs::msg::RxTask;
using rx_msgs::msg::VehicleType;

namespace
{
template<typename CommandMessage>
auto runStopCommand(int) -> decltype(CommandMessage::CMD_RUN_STOP, std::optional<uint8_t>())
{
  return CommandMessage::CMD_RUN_STOP;
}

template<typename CommandMessage>
std::optional<uint8_t> runStopCommand(long)
{
  return std::nullopt;
}
}  // namespace

class UuvCommNodeTest : public ::testing::Test
{
protected:
  using Survey = banyu_msgs::srv::TriggerPipelineSurvey;

  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(42);
    context_->init(0, nullptr, init_options);
    options_.context(context_);
    options_.parameter_overrides({
      rclcpp::Parameter("heartbeat_rate_hz", 10.0),
      rclcpp::Parameter("anchor_latitude", 33.0),
      rclcpp::Parameter("anchor_longitude", -118.0),
      rclcpp::Parameter("anchor_altitude_hae_m", 4.0),
      rclcpp::Parameter("active_buoy_latitude", 33.1),
      rclcpp::Parameter("active_buoy_longitude", -118.1)});
    node_ = std::make_shared<UuvCommNode>(options_);
    peer_ = std::make_shared<rclcpp::Node>(
      "uuv_comm_test_peer", rclcpp::NodeOptions().context(context_));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    executor_->add_node(peer_);
    heartbeat_sub_ = peer_->create_subscription<rx_msgs::msg::Heartbeat>(
      "/system/vehicle/uuv/heartbeat", 10,
      [this](rx_msgs::msg::Heartbeat::SharedPtr msg) {heartbeats_.push_back(*msg);});
    status_sub_ = peer_->create_subscription<rx_msgs::msg::MissionStatus>(
      "/system/mission/status", 10,
      [this](rx_msgs::msg::MissionStatus::SharedPtr msg) {statuses_.push_back(*msg);});
    survey_sub_ = peer_->create_subscription<rx_msgs::msg::PipelineSurvey>(
      "/system/vehicle/uuv/task2/pipeline_survey", rclcpp::QoS(10).transient_local(),
      [this](rx_msgs::msg::PipelineSurvey::SharedPtr msg) {surveys_.push_back(*msg);});
    ack_sub_ = peer_->create_subscription<rx_msgs::msg::IncidentAck>(
      "/system/vehicle/uuv/task4/incident_ack", 10,
      [this](rx_msgs::msg::IncidentAck::SharedPtr msg) {acks_.push_back(*msg);});
    readiness_sub_ = peer_->create_subscription<rx_msgs::msg::ReadinessReport>(
      "/system/vehicle/uuv/task4/readiness", 10,
      [this](rx_msgs::msg::ReadinessReport::SharedPtr msg) {readiness_.push_back(*msg);});
    start_server_ = peer_->create_service<std_srvs::srv::Trigger>(
      "/banyu/mission/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ++start_requests_;
        response->success = start_success_;
        response->message = start_success_ ? "accepted" : "preflight failed";
      });
    abort_server_ = peer_->create_service<std_srvs::srv::Trigger>(
      "/banyu/mission/abort",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ++abort_requests_;
        response->success = true;
        response->message = "aborted";
      });
    command_pub_ = peer_->create_publisher<Command>("/system/mission/command", 10);
    survey_client_ = peer_->create_client<Survey>("/banyu/communication/trigger_pipeline_survey");
    ASSERT_TRUE(spinUntil([this]() {return command_pub_->get_subscription_count() > 0;}));
    ASSERT_TRUE(survey_client_->wait_for_service(2s));
    ASSERT_TRUE(spinUntil([this]() {return !heartbeats_.empty();}));
  }

  void TearDown() override
  {
    executor_.reset();
    node_.reset();
    peer_.reset();
    context_->shutdown("test complete");
  }

  bool spinUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    spinUntil([]() {return false;}, duration);
  }

  template<typename Message>
  typename rclcpp::Publisher<Message>::SharedPtr publisher(
    const std::string & topic, const rclcpp::QoS & qos = rclcpp::QoS(10))
  {
    auto pub = peer_->create_publisher<Message>(topic, qos);
    EXPECT_TRUE(spinUntil([pub]() {return pub->get_subscription_count() > 0;}));
    return pub;
  }

  void requestSurvey()
  {
    const auto count = surveys_.size();
    auto future = survey_client_->async_send_request(std::make_shared<Survey::Request>());
    ASSERT_TRUE(spinUntil([&future]() {return future.wait_for(0s) == std::future_status::ready;}));
    EXPECT_TRUE(future.get()->success);
    ASSERT_TRUE(spinUntil([this, count]() {return surveys_.size() > count;}));
  }

  void deferMissionStart()
  {
    start_server_ = peer_->create_service<std_srvs::srv::Trigger>(
      "/banyu/mission/start",
      [this](const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request>) {
        ++start_requests_;
        pending_start_request_ = header;
      });
  }

  void acceptMissionStart()
  {
    ASSERT_TRUE(pending_start_request_);
    std_srvs::srv::Trigger::Response response;
    response.success = true;
    response.message = "accepted";
    start_server_->send_response(*pending_start_request_, response);
    pending_start_request_.reset();
  }

  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::NodeOptions options_;
  std::shared_ptr<UuvCommNode> node_;
  rclcpp::Node::SharedPtr peer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<rx_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<rx_msgs::msg::MissionStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<rx_msgs::msg::PipelineSurvey>::SharedPtr survey_sub_;
  rclcpp::Subscription<rx_msgs::msg::IncidentAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<rx_msgs::msg::ReadinessReport>::SharedPtr readiness_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_server_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Client<Survey>::SharedPtr survey_client_;
  std::shared_ptr<rmw_request_id_t> pending_start_request_;
  std::vector<rx_msgs::msg::Heartbeat> heartbeats_;
  std::vector<rx_msgs::msg::MissionStatus> statuses_;
  std::vector<rx_msgs::msg::PipelineSurvey> surveys_;
  std::vector<rx_msgs::msg::IncidentAck> acks_;
  std::vector<rx_msgs::msg::ReadinessReport> readiness_;
  size_t start_requests_{0};
  size_t abort_requests_{0};
  bool start_success_{true};
};

TEST_F(UuvCommNodeTest, InstantiatesWithTwoHzDefaultAndInitialTask)
{
  auto default_node = std::make_shared<UuvCommNode>(
    rclcpp::NodeOptions().context(context_).enable_rosout(false));
  EXPECT_STREQ(default_node->get_name(), "uuv_comm_node");
  EXPECT_DOUBLE_EQ(default_node->get_parameter("heartbeat_rate_hz").as_double(), 2.0);
  const auto & heartbeat = heartbeats_.back();
  EXPECT_EQ(heartbeat.header.frame_id, "uuv");
  EXPECT_EQ(heartbeat.vehicle_type, VehicleType::TYPE_UUV);
  EXPECT_EQ(heartbeat.current_task, RxTask::TASK_NONE);
  EXPECT_EQ(heartbeat.state, RobotState::STATE_KILLED);
}

TEST_F(UuvCommNodeTest, PublishesAtConfiguredHeartbeatRate)
{
  heartbeats_.clear();
  ASSERT_TRUE(spinUntil([this]() {return heartbeats_.size() >= 5;}));
  const auto elapsed = (rclcpp::Time(heartbeats_[4].header.stamp) -
    rclcpp::Time(heartbeats_[0].header.stamp)).seconds();
  EXPECT_NEAR(elapsed / 4.0, 0.1, 0.04);
}

TEST_F(UuvCommNodeTest, RejectsInvalidHeartbeatRates)
{
  for (const double rate : {0.0, -1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    auto options = rclcpp::NodeOptions().context(context_).enable_rosout(false);
    options.parameter_overrides({rclcpp::Parameter("heartbeat_rate_hz", rate)});
    EXPECT_THROW(std::make_shared<UuvCommNode>(options), std::invalid_argument);
  }
}

TEST_F(UuvCommNodeTest, FiltersCommandsAndStartsAddressedOrBroadcastRuns)
{
  Command command;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 17;
  for (const uint8_t target : {VehicleType::TYPE_USV, VehicleType::TYPE_UAV}) {
    command.target_vehicle = target;
    command_pub_->publish(command);
  }
  spinFor(150ms);
  EXPECT_EQ(start_requests_, 0u);
  EXPECT_TRUE(statuses_.empty());
  EXPECT_EQ(heartbeats_.back().current_task, RxTask::TASK_NONE);

  command.target_vehicle = VehicleType::TYPE_UUV;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return start_requests_ == 1 && !statuses_.empty() &&
             statuses_.back().message == "Mission started: accepted" &&
             heartbeats_.back().current_task == RxTask::TASK_INFRA_SURVEY_REPAIR;
    }));
  EXPECT_EQ(statuses_.back().run_id, 17u);
  EXPECT_EQ(statuses_.back().vehicle_id, "uuv");
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_INFRA_SURVEY_REPAIR);
  EXPECT_EQ(statuses_.front().message, "Mission start requested");
  EXPECT_EQ(statuses_.front().current_task, RxTask::TASK_NONE);

  command.target_vehicle = VehicleType::TYPE_UNKNOWN;
  command.run_id = 18;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return start_requests_ == 2 && statuses_.back().run_id == 18;
    }));
}

TEST_F(UuvCommNodeTest, ReportsRejectedMissionStart)
{
  start_success_ = false;
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 7;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return !statuses_.empty() &&
             statuses_.back().message == "Mission start rejected: preflight failed";
    }));
  EXPECT_EQ(statuses_.back().run_id, 7u);
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
  spinFor(150ms);
  EXPECT_EQ(heartbeats_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, KeepsTaskNoneUntilMissionStartSucceeds)
{
  deferMissionStart();
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 17;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return pending_start_request_ && !statuses_.empty() &&
             statuses_.back().message == "Mission start requested";
    }));
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
  spinFor(150ms);
  EXPECT_EQ(heartbeats_.back().current_task, RxTask::TASK_NONE);

  acceptMissionStart();
  ASSERT_TRUE(spinUntil([this]() {
      return statuses_.back().message == "Mission started: accepted" &&
             heartbeats_.back().current_task == RxTask::TASK_INFRA_SURVEY_REPAIR;
    }));
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_INFRA_SURVEY_REPAIR);
}

TEST_F(UuvCommNodeTest, ReportsUnavailableMissionStartWithTaskNone)
{
  start_server_.reset();
  ASSERT_TRUE(spinUntil([this]() {return peer_->count_services("/banyu/mission/start") == 0;}));
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 7;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return !statuses_.empty() &&
             statuses_.back().message == "Mission start service unavailable";
    }));
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
  spinFor(150ms);
  EXPECT_EQ(heartbeats_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, MissionDoneTriggersAbortResetsActiveTaskAndPublishesStatus)
{
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 17;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return !statuses_.empty() && statuses_.back().message == "Mission started: accepted" &&
             heartbeats_.back().current_task == RxTask::TASK_INFRA_SURVEY_REPAIR;
    }));

  command.command_type = Command::CMD_MISSION_DONE;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return abort_requests_ == 1 && statuses_.back().message == "Mission stopped" &&
             heartbeats_.back().current_task == RxTask::TASK_NONE;
    }));
  EXPECT_EQ(statuses_.back().run_id, 17u);
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, RunStopTriggersAbortAndResetsActiveTask)
{
  const auto run_stop = runStopCommand<Command>(0);
  if (!run_stop) {
    GTEST_SKIP() << "Installed rx_msgs schema has no CMD_RUN_STOP constant";
  }
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 17;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return !statuses_.empty() && statuses_.back().message == "Mission started: accepted" &&
             heartbeats_.back().current_task == RxTask::TASK_INFRA_SURVEY_REPAIR;
    }));

  command.command_type = *run_stop;
  command.target_vehicle = VehicleType::TYPE_UAV;
  command_pub_->publish(command);
  spinFor(150ms);
  EXPECT_EQ(abort_requests_, 0u);
  EXPECT_EQ(heartbeats_.back().current_task, RxTask::TASK_INFRA_SURVEY_REPAIR);

  command.target_vehicle = VehicleType::TYPE_UNKNOWN;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return abort_requests_ == 1 && statuses_.back().message == "Mission stopped" &&
             heartbeats_.back().current_task == RxTask::TASK_NONE;
    }));
  EXPECT_EQ(statuses_.back().run_id, 17u);
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, MissionDoneClearsTaskWhenAbortServiceIsUnavailable)
{
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return heartbeats_.back().current_task == RxTask::TASK_INFRA_SURVEY_REPAIR;
    }));
  abort_server_.reset();
  ASSERT_TRUE(spinUntil([this]() {return peer_->count_services("/banyu/mission/abort") == 0;}));

  command.command_type = Command::CMD_MISSION_DONE;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return !statuses_.empty() && statuses_.back().message == "Mission stopped" &&
             heartbeats_.back().current_task == RxTask::TASK_NONE;
    }));
  EXPECT_EQ(abort_requests_, 0u);
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, MissionDoneIgnoresLateMissionStartResponse)
{
  deferMissionStart();
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 17;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {return pending_start_request_ != nullptr;}));
  command.command_type = Command::CMD_MISSION_DONE;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return !statuses_.empty() && statuses_.back().message == "Mission stopped";
    }));

  acceptMissionStart();
  spinFor(150ms);
  EXPECT_EQ(statuses_.back().message, "Mission stopped");
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
  EXPECT_EQ(heartbeats_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, AcknowledgesTask4CommandsWithMatchingSequence)
{
  Command command;
  command.target_vehicle = VehicleType::TYPE_UAV;
  command.command_type = Command::CMD_TASK4_ASSISTANCE;
  command.command_seq = 10;
  command_pub_->publish(command);
  spinFor(100ms);
  EXPECT_TRUE(acks_.empty());
  EXPECT_TRUE(readiness_.empty());

  command.target_vehicle = VehicleType::TYPE_UNKNOWN;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {return acks_.size() == 1 && readiness_.size() == 1;}));
  EXPECT_EQ(acks_.back().command_seq, 10u);
  EXPECT_EQ(readiness_.back().command_seq, 10u);
  EXPECT_EQ(acks_.back().header.frame_id, "uuv");

  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_TASK4_KEEP_OUT_ZONE;
  command.command_seq = 11;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {return acks_.size() == 2;}));
  EXPECT_EQ(acks_.back().command_seq, 11u);
  command.command_type = Command::CMD_TASK4_ALL_CLEAR;
  command.command_seq = 12;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {return acks_.size() == 3;}));
  EXPECT_EQ(acks_.back().command_seq, 12u);
  EXPECT_EQ(readiness_.size(), 1u);
}

TEST_F(UuvCommNodeTest, HeartbeatUsesTelemetryAndFallsBackForInvalidGps)
{
  auto gps_pub = publisher<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", rclcpp::SensorDataQoS());
  auto pose_pub = publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto velocity_pub = publisher<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS());
  sensor_msgs::msg::NavSatFix gps;
  gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  gps.latitude = 34.0;
  gps.longitude = -119.0;
  gps.altitude = 12.0;
  gps_pub->publish(gps);
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.z = -2.5;
  const double roll = 30.0 * std::acos(-1.0) / 180.0;
  const double pitch = -20.0 * std::acos(-1.0) / 180.0;
  const double yaw = -90.0 * std::acos(-1.0) / 180.0;
  pose.pose.orientation.x = std::sin(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) -
    std::cos(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
  pose.pose.orientation.y = std::cos(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2) +
    std::sin(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2);
  pose.pose.orientation.z = std::cos(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2) -
    std::sin(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2);
  pose.pose.orientation.w = std::cos(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) +
    std::sin(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
  pose_pub->publish(pose);
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.linear.x = 0.3;
  velocity.twist.linear.y = 0.4;
  velocity_pub->publish(velocity);
  ASSERT_TRUE(spinUntil([&]() {
      gps_pub->publish(gps);
      pose_pub->publish(pose);
      velocity_pub->publish(velocity);
      return heartbeats_.back().position.latitude == 34.0 &&
             heartbeats_.back().depth_m == 2.5F && heartbeats_.back().spd_mps == 0.5F;
    }));
  const auto heartbeat = heartbeats_.back();
  EXPECT_DOUBLE_EQ(heartbeat.position.longitude, -119.0);
  EXPECT_FLOAT_EQ(heartbeat.altitude_hae_m, 12.0F);
  EXPECT_FLOAT_EQ(heartbeat.spd_mps, 0.5F);
  EXPECT_NEAR(heartbeat.heading_deg, 180.0F, 1e-4);
  EXPECT_NEAR(heartbeat.roll_deg, 30.0F, 1e-4);
  EXPECT_NEAR(heartbeat.pitch_deg, -20.0F, 1e-4);

  gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  ASSERT_TRUE(spinUntil([&]() {
      gps_pub->publish(gps);
      return heartbeats_.back().position.latitude == 33.0;
    }));
  EXPECT_DOUBLE_EQ(heartbeats_.back().position.longitude, -118.0);
  EXPECT_FLOAT_EQ(heartbeats_.back().altitude_hae_m, 4.0F);
  gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  gps.latitude = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 5; ++i) {
    gps_pub->publish(gps);
    spinFor(30ms);
  }
  EXPECT_DOUBLE_EQ(heartbeats_.back().position.latitude, 33.0);
}

TEST_F(UuvCommNodeTest, HeartbeatFallsBackToAnchorWhenGpsIsStaleAndRecovers)
{
  auto gps_pub = publisher<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", rclcpp::SensorDataQoS());
  sensor_msgs::msg::NavSatFix gps;
  gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  gps.latitude = 34.0;
  gps.longitude = -119.0;
  gps.altitude = 12.0;
  auto last_publish_time = node_->now();
  ASSERT_TRUE(spinUntil([&]() {
      last_publish_time = node_->now();
      gps_pub->publish(gps);
      return heartbeats_.back().position.latitude == 34.0;
    }));

  spinFor(2500ms);
  EXPECT_DOUBLE_EQ(heartbeats_.back().position.latitude, 34.0);
  ASSERT_TRUE(spinUntil([this]() {
      return heartbeats_.back().position.latitude == 33.0;
    }, 1500ms));
  EXPECT_GT((rclcpp::Time(heartbeats_.back().header.stamp) - last_publish_time).seconds(), 3.0);
  EXPECT_DOUBLE_EQ(heartbeats_.back().position.longitude, -118.0);
  EXPECT_FLOAT_EQ(heartbeats_.back().altitude_hae_m, 4.0F);

  ASSERT_TRUE(spinUntil([&]() {
      gps_pub->publish(gps);
      return heartbeats_.back().position.latitude == 34.0;
    }));
  EXPECT_DOUBLE_EQ(heartbeats_.back().position.longitude, -119.0);
  EXPECT_FLOAT_EQ(heartbeats_.back().altitude_hae_m, 12.0F);
}

TEST_F(UuvCommNodeTest, HeartbeatConvertsEnuYawToCompassCardinalHeadings)
{
  auto pose_pub = publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.z = -1.0;
  const std::vector<double> yaw_degrees{90.0, 0.0, -90.0, 180.0};
  const std::vector<float> compass_headings{0.0F, 90.0F, 180.0F, 270.0F};
  for (size_t i = 0; i < yaw_degrees.size(); ++i) {
    const double yaw = yaw_degrees[i] * std::acos(-1.0) / 180.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    ASSERT_TRUE(spinUntil([&]() {
        pose_pub->publish(pose);
        return heartbeats_.back().depth_m == 1.0F &&
               std::abs(heartbeats_.back().heading_deg - compass_headings[i]) < 1e-4;
      }));
    EXPECT_NEAR(heartbeats_.back().heading_deg, compass_headings[i], 1e-4);
  }
}

TEST_F(UuvCommNodeTest, HeartbeatResetsStaleVelocityAndPoseAndRecovers)
{
  auto velocity_pub = publisher<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS());
  auto pose_pub = publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.linear.x = 0.3;
  velocity.twist.linear.y = 0.4;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.z = -2.0;
  // A 30-degree roll with zero ENU yaw gives a 90-degree compass heading.
  pose.pose.orientation.x = std::sin(std::acos(-1.0) / 12.0);
  pose.pose.orientation.w = std::cos(std::acos(-1.0) / 12.0);
  auto last_publish_time = node_->now();
  ASSERT_TRUE(spinUntil([&]() {
      last_publish_time = node_->now();
      velocity_pub->publish(velocity);
      pose_pub->publish(pose);
      return heartbeats_.back().spd_mps == 0.5F && heartbeats_.back().depth_m == 2.0F;
    }));
  EXPECT_NEAR(heartbeats_.back().roll_deg, 30.0F, 1e-4);
  EXPECT_NEAR(heartbeats_.back().heading_deg, 90.0F, 1e-4);

  spinFor(2500ms);
  EXPECT_FLOAT_EQ(heartbeats_.back().spd_mps, 0.5F);
  EXPECT_FLOAT_EQ(heartbeats_.back().depth_m, 2.0F);
  ASSERT_TRUE(spinUntil([this]() {
      return heartbeats_.back().spd_mps == 0.0F && heartbeats_.back().depth_m == 0.0F;
    }, 1500ms));
  EXPECT_GT((rclcpp::Time(heartbeats_.back().header.stamp) - last_publish_time).seconds(), 3.0);
  EXPECT_FLOAT_EQ(heartbeats_.back().roll_deg, 0.0F);
  EXPECT_FLOAT_EQ(heartbeats_.back().pitch_deg, 0.0F);
  EXPECT_FLOAT_EQ(heartbeats_.back().heading_deg, 0.0F);

  ASSERT_TRUE(spinUntil([&]() {
      velocity_pub->publish(velocity);
      pose_pub->publish(pose);
      return heartbeats_.back().spd_mps == 0.5F && heartbeats_.back().depth_m == 2.0F;
    }));
  EXPECT_NEAR(heartbeats_.back().roll_deg, 30.0F, 1e-4);
  EXPECT_NEAR(heartbeats_.back().heading_deg, 90.0F, 1e-4);
}

TEST_F(UuvCommNodeTest, HeartbeatDistinguishesAutoManualDisarmedAndFailsafe)
{
  auto state_pub = publisher<mavros_msgs::msg::State>("/mavros/state");
  mavros_msgs::msg::State state;
  state.connected = true;
  state.armed = true;
  state.system_status = 4;
  for (const auto & mode : {"GUIDED", "AUTO", "MANUAL"}) {
    state.mode = mode;
    state_pub->publish(state);
    const uint8_t expected = state.mode == "MANUAL" ?
      RobotState::STATE_MANUAL : RobotState::STATE_AUTO;
    ASSERT_TRUE(spinUntil([this, expected]() {return heartbeats_.back().state == expected;}));
  }
  state.armed = false;
  state_pub->publish(state);
  ASSERT_TRUE(spinUntil([this]() {return heartbeats_.back().state == RobotState::STATE_KILLED;}));
  state.armed = true;
  state.mode = "GUIDED";
  state.system_status = 5;
  state_pub->publish(state);
  spinFor(150ms);
  EXPECT_EQ(heartbeats_.back().state, RobotState::STATE_KILLED);

  state.system_status = 4;
  state_pub->publish(state);
  ASSERT_TRUE(spinUntil([this]() {return heartbeats_.back().state == RobotState::STATE_AUTO;}));
  auto status_pub = publisher<banyu_msgs::msg::VehicleStatus>("/banyu/vehicle_status");
  banyu_msgs::msg::VehicleStatus status;
  status.fcu_connected = true;
  status.fcu_status_fresh = true;
  status.operator_takeover = true;
  status_pub->publish(status);
  ASSERT_TRUE(spinUntil([this]() {return heartbeats_.back().state == RobotState::STATE_MANUAL;}));
  status.fcu_status_fresh = false;
  status_pub->publish(status);
  ASSERT_TRUE(spinUntil([this]() {return heartbeats_.back().state == RobotState::STATE_KILLED;}));
}

TEST_F(UuvCommNodeTest, SurveyServicePublishesDefaultSegmentsAndMissionStatus)
{
  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 17;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return !statuses_.empty() && statuses_.back().message == "Mission started: accepted";
    }));
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_INFRA_SURVEY_REPAIR);
  requestSurvey();
  ASSERT_FALSE(surveys_.empty());
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{1, 1, 1, 2}));
  EXPECT_DOUBLE_EQ(surveys_.back().active_buoy_position.latitude, 33.1);
  EXPECT_DOUBLE_EQ(surveys_.back().active_buoy_position.longitude, -118.1);
  EXPECT_EQ(surveys_.back().header.frame_id, "uuv");
  ASSERT_TRUE(spinUntil([this]() {
      return statuses_.back().message == "Pipeline survey published" &&
             heartbeats_.back().current_task == RxTask::TASK_NONE;
    }));
  EXPECT_EQ(statuses_.back().message, "Pipeline survey published");
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, SurveyOrdersAndDeduplicatesRingObservationsAndResetsForNewRun)
{
  using banyu_msgs::msg::PipelineRingState;
  auto ring_pub = publisher<PipelineRingState>(
    "/banyu/pipeline/ring_state", rclcpp::QoS(10).reliable().transient_local());
  PipelineRingState ring;
  ring.index = 1;
  ring.color = PipelineRingState::COLOR_RED;
  ring_pub->publish(ring);
  ring.index = 0;
  ring.color = PipelineRingState::COLOR_GREEN;
  ring_pub->publish(ring);
  ring_pub->publish(ring);
  ring.index = 2;
  ring.color = PipelineRingState::COLOR_BLUE;
  ring_pub->publish(ring);
  ring.index = 3;
  ring.color = PipelineRingState::COLOR_UNKNOWN;
  ring_pub->publish(ring);
  spinFor(100ms);
  requestSurvey();
  ASSERT_FALSE(surveys_.empty());
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{1, 2, 1, 0}));

  Command command;
  command.target_vehicle = VehicleType::TYPE_UUV;
  command.command_type = Command::CMD_RUN_START;
  command.run_id = 9;
  command_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() {
      return start_requests_ == 1 && statuses_.back().message == "Mission started: accepted";
    }));
  requestSurvey();
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{1, 1, 1, 2}));
  ASSERT_TRUE(spinUntil([this]() {
      return statuses_.back().run_id == 9 &&
             statuses_.back().message == "Pipeline survey published";
    }));
  EXPECT_EQ(statuses_.back().current_task, RxTask::TASK_NONE);
}

TEST_F(UuvCommNodeTest, SurveyPadsSparseIndicesAndPreservesIndicesBeyondFourSections)
{
  using banyu_msgs::msg::PipelineRingState;
  auto ring_pub = publisher<PipelineRingState>("/banyu/pipeline/ring_state");
  PipelineRingState ring;
  ring.index = 3;
  ring.color = PipelineRingState::COLOR_BLUE;
  ring_pub->publish(ring);
  ring.index = 0;
  ring.color = PipelineRingState::COLOR_RED;
  ring_pub->publish(ring);
  spinFor(100ms);
  requestSurvey();
  ASSERT_FALSE(surveys_.empty());
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{2, 0, 0, 1}));

  ring.index = 5;
  ring_pub->publish(ring);
  spinFor(100ms);
  requestSurvey();
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{2, 0, 0, 1, 0, 2}));

  ring.index = 19;
  ring_pub->publish(ring);
  spinFor(100ms);
  requestSurvey();
  ASSERT_EQ(surveys_.back().segments.size(), 20u);
  EXPECT_EQ(surveys_.back().segments[19], 2u);
  EXPECT_EQ(surveys_.back().segments[18], 0u);
}

TEST_F(UuvCommNodeTest, SurveyRejectsOutOfRangeRingIndicesWithoutExpandingSegments)
{
  using banyu_msgs::msg::PipelineRingState;
  auto ring_pub = publisher<PipelineRingState>("/banyu/pipeline/ring_state");
  PipelineRingState ring;
  ring.color = PipelineRingState::COLOR_RED;
  for (const uint32_t index : {20u, 21u, std::numeric_limits<uint32_t>::max()}) {
    ring.index = index;
    ring_pub->publish(ring);
  }
  spinFor(100ms);
  requestSurvey();
  ASSERT_FALSE(surveys_.empty());
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{1, 1, 1, 2}));

  ring.index = 0;
  ring.color = PipelineRingState::COLOR_BLUE;
  ring_pub->publish(ring);
  ring.index = 20;
  ring_pub->publish(ring);
  spinFor(100ms);
  requestSurvey();
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{1, 0, 0, 0}));
}

TEST_F(UuvCommNodeTest, LateSurveySubscriberReceivesRetainedReport)
{
  requestSurvey();
  survey_sub_.reset();
  surveys_.clear();
  survey_sub_ = peer_->create_subscription<rx_msgs::msg::PipelineSurvey>(
    "/system/vehicle/uuv/task2/pipeline_survey", rclcpp::QoS(10).transient_local(),
    [this](rx_msgs::msg::PipelineSurvey::SharedPtr msg) {surveys_.push_back(*msg);});
  ASSERT_TRUE(spinUntil([this]() {return !surveys_.empty();}));
  EXPECT_EQ(surveys_.back().segments, (std::vector<uint8_t>{1, 1, 1, 2}));
}
