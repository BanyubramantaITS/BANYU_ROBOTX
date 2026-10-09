#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "banyu_mission/action_nodes.hpp"
#include "banyu_mission/banyu_mission.hpp"
#include "banyu_mission/por_mission_config.hpp"
#include "banyu_msgs/msg/pipeline_ring_state.hpp"
#include "banyu_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
class MissionTest : public ::testing::Test
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
};

void spinFor(const rclcpp::Node::SharedPtr & node, int cycles = 5)
{
  for (int i = 0; i < cycles; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

std::string missionTreePath()
{
#ifdef BANYU_MISSION_SOURCE_DIR
  const auto source_path = std::string(BANYU_MISSION_SOURCE_DIR) + "/bt/banyu_mission.xml";
  if (std::filesystem::exists(source_path)) {
    return source_path;
  }
#endif
  return ament_index_cpp::get_package_share_directory("banyu_mission") +
         "/bt/banyu_mission.xml";
}

std::string porMissionTreePath()
{
#ifdef BANYU_MISSION_SOURCE_DIR
  const auto source_path = std::string(BANYU_MISSION_SOURCE_DIR) + "/bt/por_mission.xml";
  if (std::filesystem::exists(source_path)) {
    return source_path;
  }
#endif
  return ament_index_cpp::get_package_share_directory("banyu_mission") +
         "/bt/por_mission.xml";
}

std::string porParamsPath()
{
  return ament_index_cpp::get_package_share_directory("banyu_bringup") +
         "/config/por_params.yaml";
}

std::string readFile(const std::string & path)
{
  std::ifstream stream(path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void spinUntilReady(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture & future)
{
  for (int i = 0; i < 100 && future.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready; ++i)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void abortMission(const rclcpp::Node::SharedPtr & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("/banyu/mission/abort");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto request = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  auto future = request.future.share();
  spinUntilReady(node, future);
  ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);
}
}  // namespace

TEST_F(MissionTest, LoadsAndRegistersAllNodes)
{
  auto node = std::make_shared<rclcpp::Node>("mission_tree_parser_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get(), false);
  BT::BehaviorTreeFactory factory;
  banyu_mission::registerMissionNodes(factory, node, state);

  EXPECT_NO_THROW(factory.registerBehaviorTreeFromFile(missionTreePath()));
  EXPECT_NO_THROW(
  {
    auto tree = factory.createTree("Main", BT::Blackboard::create());
    (void)tree;
  });

  const auto & builders = factory.builders();
  for (const auto & id : {"NavigateSurfaceGps", "AlignHeading", "DiveToDepth",
      "AcquirePipeline", "SearchPipeline", "FollowPipeline",
      "CenterOnRing", "HoldProbeOnRing", "Surface",
      "PublishPipelineSurvey", "AutonomyInterrupted", "PipelineEnded", "ArmAndSetGuided"})
  {
    EXPECT_NE(builders.find(id), builders.end()) << id;
  }
}

TEST_F(MissionTest, VehicleSafetyFailsClosedAndManualBeforeStartDoesNotAbort)
{
  auto node = std::make_shared<rclcpp::Node>("mission_state_safety_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get());
  ASSERT_TRUE(state->configureMissionCoordinates(1.0, 2.0, 1.001, 2.001));
  auto publisher = node->create_publisher<banyu_msgs::msg::VehicleStatus>(
    "/banyu/vehicle_status", rclcpp::QoS(5));
  banyu_msgs::msg::VehicleStatus status;
  status.fcu_connected = true;
  status.fcu_status_fresh = true;
  status.rc_fresh = true;
  status.operator_takeover = true;
  status.mode = "GUIDED";
  publisher->publish(status);
  spinFor(node);
  EXPECT_FALSE(state->autonomyInterrupted());
  ASSERT_TRUE(state->requestStart());
  EXPECT_TRUE(state->autonomyInterrupted());
}

TEST_F(MissionTest, PreflightDoesNotInterruptAutonomyWhenNotReady)
{
  auto node = std::make_shared<rclcpp::Node>("preflight_no_interrupt_test");
  banyu_mission::MissionState state(node.get(), false);
  ASSERT_TRUE(state.configureMissionCoordinates(1.0, 2.0, 1.001, 2.001));
  ASSERT_TRUE(state.requestStart());
  EXPECT_FALSE(state.autonomyEngaged());
  EXPECT_FALSE(state.autonomyInterrupted());

  state.markAutonomyEngaged();
  EXPECT_TRUE(state.autonomyEngaged());
  EXPECT_TRUE(state.autonomyInterrupted());
}

TEST_F(MissionTest, TerminalLifecycleIsOneShot)
{
  auto node = std::make_shared<rclcpp::Node>("mission_state_lifecycle_test");
  banyu_mission::MissionState state(node.get(), false);
  ASSERT_TRUE(state.configureMissionCoordinates(1.0, 2.0, 1.001, 2.001));
  ASSERT_TRUE(state.requestStart());
  state.markSucceeded();
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Succeeded);
  EXPECT_FALSE(state.requestStart());
  state.markFailed();
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Succeeded);
}

TEST_F(MissionTest, MissionCanBeRestartedAfterAbortOrFailure)
{
  auto node = std::make_shared<rclcpp::Node>("mission_state_restart_test");
  banyu_mission::MissionState state(node.get(), false);
  ASSERT_TRUE(state.configureMissionCoordinates(1.0, 2.0, 1.001, 2.001));

  ASSERT_TRUE(state.requestStart());
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Active);

  state.markInterrupted();
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Interrupted);

  EXPECT_TRUE(state.requestStart());
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Active);

  state.markFailed();
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Failed);

  EXPECT_TRUE(state.requestStart());
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Active);
}

TEST_F(MissionTest, BearingValidationUsesEnuAndRejectsInvalidCoordinates)
{
  double bearing = 0.0;
  EXPECT_TRUE(banyu_mission::MissionState::deriveEnuInitialBearing(0.0, 0.0, 0.0, 1.0, bearing));
  EXPECT_NEAR(bearing, 0.0, 1e-9);
  EXPECT_FALSE(banyu_mission::MissionState::deriveEnuInitialBearing(91.0, 0.0, 0.0, 1.0, bearing));
  EXPECT_FALSE(banyu_mission::MissionState::validateMissionCoordinates(0.0, 0.0, 0.0, 0.0));
}

TEST_F(MissionTest, RingAcknowledgementMustMatchPendingRecord)
{
  auto node = std::make_shared<rclcpp::Node>("mission_state_ack_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get());
  auto publisher = node->create_publisher<banyu_msgs::msg::PipelineRingState>(
    "/banyu/pipeline/ring_state", rclcpp::QoS(5));
  banyu_msgs::msg::PipelineRingState ring;
  ring.index = 7;
  ring.color = banyu_msgs::msg::PipelineRingState::COLOR_RED;
  ring.confidence = 1.0F;
  publisher->publish(ring);
  spinFor(node);
  ASSERT_TRUE(state->redPending());
  EXPECT_EQ(state->pendingRedIndex(), 7U);
  EXPECT_FALSE(state->acknowledgeRed(6));
  EXPECT_TRUE(state->redPending());
  EXPECT_TRUE(state->acknowledgeRed(7));
  EXPECT_FALSE(state->redPending());
}

TEST_F(MissionTest, ActionResultSemanticFailureIsNotTransportSuccess)
{
  struct Result {bool success{false};};
  struct SuccessfulResult {bool success{true};};
  EXPECT_FALSE(
    banyu_mission::ActionClientNode<banyu_msgs::action::FollowPipeline>::semanticSuccess(Result{}));
  EXPECT_TRUE(
    banyu_mission::ActionClientNode<banyu_msgs::action::FollowPipeline>::semanticSuccess(
      SuccessfulResult{}));
}

TEST_F(MissionTest, PublishPipelineSurveyNodeProvidedPortsIsEmpty)
{
  EXPECT_TRUE(banyu_mission::PublishPipelineSurveyNode::providedPorts().empty());
}

TEST_F(MissionTest, PublishPipelineSurveyWaitsForDelayedServiceDiscovery)
{
  using Survey = banyu_msgs::srv::TriggerPipelineSurvey;
  auto node = std::make_shared<rclcpp::Node>("survey_service_discovery_test");
  auto server_node = std::make_shared<rclcpp::Node>("delayed_survey_service_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get(), false);
  ASSERT_TRUE(state->configureMissionCoordinates(0.0, 0.0, 0.0, 0.0, false));
  ASSERT_TRUE(state->acceptRun(1u, 1u));
  banyu_mission::PublishPipelineSurveyNode action(
    "PublishPipelineSurvey", BT::NodeConfig{}, node, state, "/test/delayed_pipeline_survey");
  rclcpp::Service<Survey>::SharedPtr server;
  size_t requests = 0;
  std::thread advertise_service([&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      server = server_node->create_service<Survey>(
        "/test/delayed_pipeline_survey",
        [&](const std::shared_ptr<Survey::Request> request,
        std::shared_ptr<Survey::Response> response) {
          ++requests;
          response->success = true;
          response->report_id = request->report_id;
          response->result_code = Survey::Response::ACCEPTED;
        });
    });
  const auto start_status = action.onStart();
  advertise_service.join();
  ASSERT_EQ(start_status, BT::NodeStatus::RUNNING);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(server_node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (action.onRunning() == BT::NodeStatus::RUNNING &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(action.onRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(requests, 1u);
}

TEST_F(MissionTest, PublishPipelineSurveyFailsAfterFiveSecondsWithoutServiceResponse)
{
  using Survey = banyu_msgs::srv::TriggerPipelineSurvey;
  auto node = std::make_shared<rclcpp::Node>(
    "survey_service_timeout_test",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
  auto server_node = std::make_shared<rclcpp::Node>("unresponsive_survey_service_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get(), false);
  ASSERT_TRUE(state->configureMissionCoordinates(0.0, 0.0, 0.0, 0.0, false));
  ASSERT_TRUE(state->acceptRun(1u, 1u));
  std::vector<std::shared_ptr<rmw_request_id_t>> requests;
  auto server = server_node->create_service<Survey>(
    "/test/unresponsive_pipeline_survey",
    [&](const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<Survey::Request>) {
      requests.push_back(header);
    });
  banyu_mission::PublishPipelineSurveyNode action(
    "PublishPipelineSurvey", BT::NodeConfig{}, node, state, "/test/unresponsive_pipeline_survey");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(server_node);

  const auto started = std::chrono::steady_clock::now();
  ASSERT_EQ(action.onStart(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(action.onRunning(), BT::NodeStatus::RUNNING);
  auto status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING &&
    std::chrono::steady_clock::now() - started < std::chrono::seconds(7))
  {
    executor.spin_some();
    status = action.onRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_GE(std::chrono::steady_clock::now() - started, std::chrono::seconds(5));
  EXPECT_EQ(node->now().nanoseconds(), 0);
  ASSERT_EQ(requests.size(), 1u);

  // A late successful reply must not change the timed-out result.
  Survey::Response response;
  response.success = true;
  response.report_id = "task2-survey-1-1";
  response.result_code = Survey::Response::ACCEPTED;
  server->send_response(*requests.front(), response);
  executor.spin_some();
  EXPECT_EQ(action.onRunning(), BT::NodeStatus::FAILURE);

  ASSERT_EQ(action.onStart(), BT::NodeStatus::RUNNING);
  const auto retry_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (requests.size() < 2 && std::chrono::steady_clock::now() < retry_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_EQ(action.onRunning(), BT::NodeStatus::RUNNING);
  server->send_response(*requests.back(), response);
  while (action.onRunning() == BT::NodeStatus::RUNNING &&
    std::chrono::steady_clock::now() < retry_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(action.onRunning(), BT::NodeStatus::SUCCESS);
}

TEST_F(MissionTest, AcquirePipelineRequiresAcquiredResult)
{
  using Node = banyu_mission::AcquirePipelineNode;
  Node::Result result;
  result.success = true;
  result.acquired = true;
  EXPECT_TRUE(Node::semanticResultSuccess(result));
  result.acquired = false;
  EXPECT_FALSE(Node::semanticResultSuccess(result));
  result.acquired = true;
  result.success = false;
  EXPECT_FALSE(Node::semanticResultSuccess(result));
}

TEST_F(MissionTest, SearchPipelineRequiresFoundResult)
{
  using Node = banyu_mission::SearchPipelineNode;
  Node::Result result;
  result.success = true;
  result.found = true;
  EXPECT_TRUE(Node::semanticResultSuccess(result));
  result.found = false;
  EXPECT_FALSE(Node::semanticResultSuccess(result));
  result.found = true;
  result.success = false;
  EXPECT_FALSE(Node::semanticResultSuccess(result));
}

TEST_F(MissionTest, FollowPipelineRequiresExpectedEnabledResult)
{
  using Node = banyu_mission::FollowPipelineNode;
  Node::Result result;
  result.success = true;
  result.enabled = true;
  EXPECT_TRUE(Node::semanticResultSuccess(result, true));
  EXPECT_FALSE(Node::semanticResultSuccess(result, false));
  result.enabled = false;
  EXPECT_TRUE(Node::semanticResultSuccess(result, false));
  result.success = false;
  EXPECT_FALSE(Node::semanticResultSuccess(result, false));
}

TEST_F(MissionTest, TreeInstantiatesWithSubtreeAutoremap)
{
  auto node = std::make_shared<rclcpp::Node>("mission_tree_autoremap_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get(), false);
  BT::BehaviorTreeFactory factory;
  banyu_mission::registerMissionNodes(factory, node, state);

  factory.registerBehaviorTreeFromFile(missionTreePath());
  auto blackboard = BT::Blackboard::create();
  blackboard->set("target_latitude", 33.81);
  blackboard->set("target_longitude", -118.39);
  blackboard->set("start_latitude", 33.81);
  blackboard->set("start_longitude", -118.39);
  blackboard->set("gps_tolerance", 1.0);
  blackboard->set("heading", 0.0);
  blackboard->set("heading_tolerance", 0.1);
  blackboard->set("inspection_depth", 1.5);
  blackboard->set("depth_tolerance", 0.2);
  blackboard->set("camera_name", "front_camera");
  blackboard->set("heading_prior", 0.0);
  blackboard->set("acquire_timeout_ms", static_cast<unsigned>(15000));
  blackboard->set("search_span", 1.0);
  blackboard->set("sweep_rate", 0.2);
  blackboard->set("ring_tolerance", 0.1);
  blackboard->set("hold_duration", 5.0);
  blackboard->set("mission_id", std::string("test_mission"));
  blackboard->set("crc32", static_cast<uint32_t>(12345));
  blackboard->set("max_retries", static_cast<uint8_t>(3));
  blackboard->set("surface_depth", 0.0);
  blackboard->set("surface_tolerance", 0.2);
  blackboard->set("ring_order", std::vector<uint8_t>{1});

  auto tree = factory.createTree("Main", blackboard);
  bool found_nav_node = false;
  bool found_surface_node = false;
  tree.applyVisitor(
    [&](BT::TreeNode * tree_node) {
      if (auto nav = dynamic_cast<banyu_mission::NavigateSurfaceGpsNode *>(tree_node)) {
        found_nav_node = true;
        double lat = 0.0;
        EXPECT_TRUE(nav->getInput("target_latitude", lat));
        EXPECT_NEAR(lat, 33.81, 1e-4);
      }
      if (auto surface = dynamic_cast<banyu_mission::SurfaceNode *>(tree_node)) {
        found_surface_node = true;
        double watchdog_timeout_s = -1.0;
        EXPECT_TRUE(surface->getInput("watchdog_timeout_s", watchdog_timeout_s));
        EXPECT_DOUBLE_EQ(watchdog_timeout_s, 0.0);
      }
    });
  EXPECT_TRUE(found_nav_node);
  EXPECT_TRUE(found_surface_node);
}

TEST_F(MissionTest, PorTreeLoadsAndInstantiates)
{
  auto node = std::make_shared<rclcpp::Node>("por_mission_tree_parser_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get(), false);
  BT::BehaviorTreeFactory factory;
  banyu_mission::registerMissionNodes(factory, node, state);

  EXPECT_NO_THROW(factory.registerBehaviorTreeFromFile(porMissionTreePath()));
  EXPECT_NO_THROW(
  {
    auto tree = factory.createTree("Main", BT::Blackboard::create());
    (void)tree;
  });
}

TEST_F(MissionTest, PorConfigLoadsInstalledSafetyValuesIntoBlackboard)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", porParamsPath()});
  auto node = std::make_shared<rclcpp::Node>("banyu_mission", options);
  const auto config = banyu_mission::PorMissionConfig::load(*node);
  EXPECT_NO_THROW(config.validate());

  auto blackboard = BT::Blackboard::create();
  config.apply(blackboard);
  EXPECT_DOUBLE_EQ(blackboard->get<double>("gate_depth_m"), config.gate_depth_m);
  EXPECT_DOUBLE_EQ(blackboard->get<double>("marker_align_standoff_min_m"), 2.2);
  EXPECT_DOUBLE_EQ(blackboard->get<double>("marker_align_standoff_max_m"), 2.5);
  EXPECT_DOUBLE_EQ(config.surface_watchdog_timeout_s, 5.0);
  EXPECT_DOUBLE_EQ(
    blackboard->get<double>("surface_watchdog_timeout_s"),
    config.surface_watchdog_timeout_s);
}

TEST_F(MissionTest, PorSurfaceWatchdogPropagatesToSurfaceGoalPort)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", porParamsPath()});
  auto node = std::make_shared<rclcpp::Node>("por_surface_watchdog_propagation_test", options);
  const auto config = banyu_mission::PorMissionConfig::load(*node);
  ASSERT_NO_THROW(config.validate());

  auto state = std::make_shared<banyu_mission::MissionState>(node.get(), false);
  BT::BehaviorTreeFactory factory;
  banyu_mission::registerMissionNodes(factory, node, state);
  factory.registerBehaviorTreeFromFile(porMissionTreePath());
  auto blackboard = BT::Blackboard::create();
  config.apply(blackboard);
  auto tree = factory.createTree("Main", blackboard);

  bool found_surface_node = false;
  tree.applyVisitor(
    [&](BT::TreeNode * tree_node) {
      if (auto surface = dynamic_cast<banyu_mission::SurfaceNode *>(tree_node)) {
        found_surface_node = true;
        double watchdog_timeout_s = 0.0;
        EXPECT_TRUE(surface->getInput("watchdog_timeout_s", watchdog_timeout_s));
        EXPECT_DOUBLE_EQ(watchdog_timeout_s, config.surface_watchdog_timeout_s);
      }
    });
  EXPECT_TRUE(found_surface_node);
}

TEST_F(MissionTest, PorInstalledConfigBindsAcquireGateDepth)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", porParamsPath()});
  options.parameter_overrides({rclcpp::Parameter("groot_port", 1766)});

  auto config_node = std::make_shared<rclcpp::Node>("por_installed_config_test", options);
  const auto config = banyu_mission::PorMissionConfig::load(*config_node);
  ASSERT_NO_THROW(config.validate());
  auto blackboard = BT::Blackboard::create();
  config.apply(blackboard);
  EXPECT_DOUBLE_EQ(blackboard->get<double>("gate_depth_m"), config.gate_depth_m);
  EXPECT_NE(
    readFile(porMissionTreePath()).find("target_depth_m=\"{gate_depth_m}\""),
    std::string::npos);
  config_node.reset();
}

TEST_F(MissionTest, PorTwoMeterGateDepthValidatesAndStarts)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", porParamsPath()});
  options.parameter_overrides(
  {
    rclcpp::Parameter("groot_port", 2769),
    rclcpp::Parameter("mission.gate_depth_m", 2.0)});
  auto node = std::make_shared<rclcpp::Node>("banyu_mission", options);
  const auto config = banyu_mission::PorMissionConfig::load(*node);
  EXPECT_NO_THROW(config.validate());
  auto blackboard = BT::Blackboard::create();
  config.apply(blackboard);
  EXPECT_DOUBLE_EQ(blackboard->get<double>("gate_depth_m"), 2.0);
  node.reset();

  auto mission = std::make_shared<banyu_mission::BanyuMission>(options);
  mission->initialize();
  auto client = mission->create_client<std_srvs::srv::Trigger>("/banyu/mission/start");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto request = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  auto future = request.future.share();
  spinUntilReady(mission, future);
  ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);
  EXPECT_EQ(
    mission->lifecycle(), banyu_mission::MissionState::Lifecycle::Active);
  abortMission(mission);
  mission.reset();
  node.reset();
}

TEST_F(MissionTest, PorInvalidGateDepthsRejectValidation)
{
  const std::vector<double> invalid_depths {
    0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()};
  for (const double depth : invalid_depths) {
    banyu_mission::PorMissionConfig config;
    config.gate_depth_m = depth;
    EXPECT_THROW(config.validate(), std::invalid_argument);
  }
}

TEST_F(MissionTest, Task2InvalidCoordinatesRejectMissionStart)
{
  auto node = std::make_shared<rclcpp::Node>("task2_coordinate_validation_test");
  banyu_mission::MissionState state(node.get(), false);
  state.configure(5.0, 1.0, 5.0, 4, 1700, "GUIDED", "", true);
  EXPECT_FALSE(state.configureMissionCoordinates(0.0, 0.0, 0.0, 0.0, true));
  EXPECT_FALSE(state.requestStart());
  EXPECT_EQ(state.lifecycle(), banyu_mission::MissionState::Lifecycle::Idle);
}

TEST_F(MissionTest, PorTreeUsesOrderedMemorySequence)
{
  const auto xml = readFile(porMissionTreePath());
  EXPECT_EQ(xml.find("<ReactiveSequence"), std::string::npos);
  const auto sequence_start = xml.find("<Sequence name=\"por_mission_phases\">");
  ASSERT_NE(sequence_start, std::string::npos);
  const auto sequence_end = xml.find("</BehaviorTree>", sequence_start);
  ASSERT_NE(sequence_end, std::string::npos);
  const auto sequence = xml.substr(sequence_start, sequence_end - sequence_start);
  const std::vector<std::string> phases {
    "AcquireGate", "AlignGate", "PassGate", "SearchMarker", "AlignMarker",
    "OrbitMarker", "TurnAround", "SearchGate", "AlignGate", "PassGate", "Surface"};
  std::size_t previous = 0;
  for (const auto & phase : phases) {
    const auto current = sequence.find("<" + phase, previous);
    ASSERT_NE(current, std::string::npos) << phase;
    previous = current + phase.size();
  }
}

TEST_F(MissionTest, FallbackModeParameterDefaultsToSurface)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"mission_mode", std::string("por")},
  });
  auto mission = std::make_shared<banyu_mission::BanyuMission>(options);
  std::string fallback_mode;
  EXPECT_TRUE(mission->get_parameter("fallback_mode", fallback_mode));
  EXPECT_EQ(fallback_mode, "SURFACE");
}

TEST_F(MissionTest, HandleAbortOrTeleopInvokesAbortCallback)
{
  auto node = std::make_shared<rclcpp::Node>("handle_abort_cb_test");
  auto state = std::make_shared<banyu_mission::MissionState>(node.get());
  ASSERT_TRUE(state->configureMissionCoordinates(1.0, 2.0, 1.001, 2.001));
  ASSERT_TRUE(state->requestStart());
  bool abort_called = false;
  banyu_mission::HandleAbortOrTeleopNode abort_node(
    "HandleAbortOrTeleop", {}, state, [&abort_called]() {abort_called = true;});

  EXPECT_EQ(abort_node.executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(abort_called);
  EXPECT_TRUE(state->snapshot().abort_requested);
}

TEST_F(MissionTest, AbortServiceCommandsFallbackModeSurface)
{
  auto helper_node = std::make_shared<rclcpp::Node>("set_mode_helper_node");
  std::string received_mode;
  auto set_mode_server = helper_node->create_service<mavros_msgs::srv::SetMode>(
    "/mavros/set_mode",
    [&received_mode](
      const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
      std::shared_ptr<mavros_msgs::srv::SetMode::Response> res) {
      received_mode = req->custom_mode;
      res->mode_sent = true;
    });

  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"mission_mode", std::string("por")},
  });
  auto mission = std::make_shared<banyu_mission::BanyuMission>(options);

  auto abort_client = helper_node->create_client<std_srvs::srv::Trigger>("/banyu/mission/abort");
  ASSERT_TRUE(abort_client->wait_for_service(std::chrono::seconds(1)));
  auto future = abort_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  for (int i = 0; i < 50 && received_mode.empty(); ++i) {
    rclcpp::spin_some(mission);
    rclcpp::spin_some(helper_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(received_mode, "SURFACE");
}
