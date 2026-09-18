#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "banyu_mission/action_nodes.hpp"
#include "banyu_mission/banyu_mission.hpp"
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
      "SendMissionReport", "AutonomyInterrupted", "PipelineEnded"})
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

TEST_F(MissionTest, SendMissionReportRequiresAcknowledgedMatchingCrc)
{
  using Result = banyu_msgs::action::SendMissionReport::Result;
  constexpr uint32_t expected_crc = 0x12345678U;
  Result result;
  result.success = true;
  result.acknowledged = true;
  result.acknowledged_crc32 = expected_crc;
  EXPECT_TRUE(banyu_mission::SendMissionReportNode::semanticResultSuccess(result, expected_crc));
  result.acknowledged = false;
  EXPECT_FALSE(banyu_mission::SendMissionReportNode::semanticResultSuccess(result, expected_crc));
  result.acknowledged = true;
  result.acknowledged_crc32 = expected_crc + 1U;
  EXPECT_FALSE(banyu_mission::SendMissionReportNode::semanticResultSuccess(result, expected_crc));
  result.success = false;
  result.acknowledged_crc32 = expected_crc;
  EXPECT_FALSE(banyu_mission::SendMissionReportNode::semanticResultSuccess(result, expected_crc));
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
