#include "banyu_vehicle_execution/command_encoder.hpp"
#include "banyu_vehicle_execution/goal_arbiter.hpp"
#include "banyu_vehicle_execution/motion_intent.hpp"
#include "banyu_vehicle_execution/vehicle_telemetry.hpp"
#include "banyu_vehicle_execution/action_servers.hpp"
#include "banyu_vehicle_execution/vehicle_execution_node.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <chrono>
using namespace std::chrono_literals;
using banyu_vehicle_execution::GoalAdmission;
using banyu_vehicle_execution::GoalArbiter;
using banyu_vehicle_execution::GoalIdentity;
using banyu_vehicle_execution::GoalState;
using banyu_vehicle_execution::GoalUuid;
using banyu_vehicle_execution::IntentFrame;
using banyu_vehicle_execution::IntentLimits;
using banyu_vehicle_execution::IntentValidationError;
using banyu_vehicle_execution::Lease;
using banyu_vehicle_execution::MotionIntent;
using banyu_vehicle_execution::SteadyTime;
using banyu_vehicle_execution::SteadyTimeSource;
using banyu_vehicle_execution::StopReason;

class FakeClock : public SteadyTimeSource
{
public:
  SteadyTime current{};
  SteadyTime now() const override {return current;}
  void advance(std::chrono::seconds amount) {current += amount;}
};

GoalIdentity identity(uint8_t last_byte, const std::string & owner, uint64_t generation)
{
  GoalUuid uuid{};
  uuid[15] = last_byte;
  return {uuid, owner, generation};
}

Lease leaseFor(const GoalIdentity & goal, const FakeClock & clock, bool active = true)
{
  return {goal, clock.now(), clock.now() + 10s, active};
}

TEST(CommandEncoder, UsesVelocityYawRateMaskFrameAndExplicitZeros)
{
  const auto target = banyu_vehicle_execution::encodeVelocityYawRate(
    MotionIntent::localEnu(0.2, -0.3, 0.4, 0.7));
  EXPECT_EQ(target.coordinate_frame, mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(target.type_mask, 0x5C7);
  EXPECT_EQ(target.type_mask & mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE, 0U);
  EXPECT_DOUBLE_EQ(target.position.x, 0.0);
  EXPECT_DOUBLE_EQ(target.position.y, 0.0);
  EXPECT_DOUBLE_EQ(target.position.z, 0.0);
  EXPECT_DOUBLE_EQ(target.acceleration_or_force.x, 0.0);
  EXPECT_DOUBLE_EQ(target.acceleration_or_force.y, 0.0);
  EXPECT_DOUBLE_EQ(target.acceleration_or_force.z, 0.0);
  EXPECT_DOUBLE_EQ(target.yaw, 0.0);
  EXPECT_FLOAT_EQ(target.yaw_rate, 0.7F);
}

TEST(MotionIntent, BodyFluUsesFreshYawAndPreservesVerticalSign)
{
  const auto local = banyu_vehicle_execution::bodyFluToLocalEnu(
    MotionIntent::bodyFlu(1.0, 0.0, 0.7, -0.2), M_PI / 2.0);
  ASSERT_TRUE(local.has_value());
  EXPECT_EQ(local->frame, IntentFrame::LocalEnu);
  EXPECT_NEAR(local->linear_x, 0.0, 1e-12);
  EXPECT_NEAR(local->linear_y, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(local->linear_z, 0.7);
  EXPECT_DOUBLE_EQ(local->yaw_rate, -0.2);
  const auto second_yaw = banyu_vehicle_execution::bodyFluToLocalEnu(
    MotionIntent::bodyFlu(1.0, 0.0, -0.7), 0.0);
  ASSERT_TRUE(second_yaw.has_value());
  EXPECT_DOUBLE_EQ(second_yaw->linear_x, 1.0);
  EXPECT_DOUBLE_EQ(second_yaw->linear_z, -0.7);
}

TEST(MotionIntent, ClampsToPhysicalLimitsAndRejectsNonfiniteValues)
{
  const IntentLimits limits{1.0, 2.0, 3.0, 0.5};
  const auto bounded = banyu_vehicle_execution::clampIntent(
    MotionIntent::localEnu(4.0, -4.0, 2.0, -1.0), limits);
  ASSERT_TRUE(bounded.has_value());
  EXPECT_DOUBLE_EQ(bounded->linear_x, 1.0);
  EXPECT_DOUBLE_EQ(bounded->linear_y, -2.0);
  EXPECT_DOUBLE_EQ(bounded->linear_z, 2.0);
  EXPECT_DOUBLE_EQ(bounded->yaw_rate, -0.5);
  EXPECT_EQ(
    banyu_vehicle_execution::validateIntent(MotionIntent::localEnu(4.0, 0.0, 0.0), limits).error,
    IntentValidationError::OutsideLimits);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    banyu_vehicle_execution::clampIntent(MotionIntent::localEnu(nan, 0.0, 0.0), limits));
  EXPECT_EQ(
    banyu_vehicle_execution::validateIntent(MotionIntent::localEnu(0.0, nan, 0.0), limits).error,
    IntentValidationError::NonFiniteComponent);
  EXPECT_THROW(
    banyu_vehicle_execution::encodeVelocityYawRate(MotionIntent::localEnu(nan, 0.0, 0.0)),
    std::invalid_argument);
}

TEST(CommandEncoder, HardLimitsKeepFloat32FieldsFinite)
{
  const double huge = std::numeric_limits<double>::max();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto target = banyu_vehicle_execution::encodeVelocityYawRate(
    MotionIntent::localEnu(huge, -huge, huge, huge),
    IntentLimits{huge, huge, huge, huge});
  EXPECT_TRUE(std::isfinite(target.velocity.x));
  EXPECT_TRUE(std::isfinite(target.velocity.y));
  EXPECT_TRUE(std::isfinite(target.velocity.z));
  EXPECT_TRUE(std::isfinite(target.yaw_rate));
  EXPECT_LE(std::abs(target.velocity.x), banyu_vehicle_execution::kHardMaxLinearComponent);
  EXPECT_LE(std::abs(target.yaw_rate), banyu_vehicle_execution::kHardMaxYawRate);
  EXPECT_THROW(
    banyu_vehicle_execution::encodeVelocityYawRate(
      MotionIntent::localEnu(0.0, 0.0, 0.0), IntentLimits{nan, 1.0, 1.0, 1.0}),
    std::invalid_argument);
}

TEST(GoalArbiter, RequiresExactFirstActiveLeaseForNonzeroOutput)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 2);
  const auto goal_identity = identity(1, "pilot", 10);
  const auto goal = arbiter.requestGoal(goal_identity);
  ASSERT_EQ(goal.admission, GoalAdmission::Activated);
  EXPECT_DOUBLE_EQ(
    arbiter.selectOutput(goal.handle, MotionIntent::localEnu(1, 0, 0)).linear_x, 0.0);
  auto wrong_owner = leaseFor(goal_identity, clock);
  wrong_owner.goal.owner = "other";
  EXPECT_FALSE(arbiter.establishLease(goal.handle, wrong_owner));
  auto wrong_uuid = leaseFor(goal_identity, clock);
  wrong_uuid.goal.uuid[0] = 1;
  EXPECT_FALSE(arbiter.establishLease(goal.handle, wrong_uuid));
  EXPECT_TRUE(arbiter.establishLease(goal.handle, leaseFor(goal_identity, clock)));
  EXPECT_DOUBLE_EQ(
    arbiter.selectOutput(goal.handle, MotionIntent::localEnu(1, 0, 0)).linear_x, 1.0);
}

TEST(GoalArbiter, RejectsZeroUuidInvalidInactiveExpiredAndRegressedLeases)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 1);
  const auto zero_uuid = identity(0, "pilot", 1);
  EXPECT_FALSE(arbiter.requestGoal(zero_uuid).accepted());
  const auto first_identity = identity(1, "pilot", 5);
  const auto first = arbiter.requestGoal(first_identity, leaseFor(first_identity, clock));
  ASSERT_EQ(first.admission, GoalAdmission::Activated);
  auto inactive = leaseFor(first_identity, clock, false);
  EXPECT_FALSE(arbiter.establishLease(first.handle, inactive));
  auto expired = leaseFor(first_identity, clock);
  expired.deadline = clock.now();
  EXPECT_FALSE(arbiter.establishLease(first.handle, expired));
  EXPECT_TRUE(arbiter.cancel(first.handle));
  EXPECT_TRUE(arbiter.recordZeroPublication());
  ASSERT_TRUE(arbiter.reset());
  EXPECT_FALSE(arbiter.requestGoal(identity(2, "pilot", 4)).accepted());
}

TEST(GoalArbiter, ExpiredActiveLeaseLatchesZeroAndTerminalizesAfterBarrier)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 2);
  const auto goal_identity = identity(1, "pilot", 1);
  const auto goal = arbiter.requestGoal(goal_identity, leaseFor(goal_identity, clock));
  ASSERT_TRUE(goal.accepted());
  clock.advance(11s);
  EXPECT_DOUBLE_EQ(
    arbiter.selectOutput(goal.handle, MotionIntent::localEnu(1, 0, 0)).linear_x, 0.0);
  EXPECT_EQ(arbiter.state(), GoalState::ZeroBarrier);
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_TRUE(arbiter.recordZeroPublication());
  EXPECT_EQ(arbiter.state(), GoalState::LeaseLost);
  EXPECT_EQ(arbiter.zeroPublicationCount(), 0U);
}

TEST(GoalArbiter, ActiveFaultAndLeaseLossUseTheSameZeroBarrier)
{
  FakeClock clock;
  for (const auto reason : {StopReason::Fault, StopReason::LeaseLoss}) {
    GoalArbiter arbiter(clock, 2);
    const auto goal_identity = identity(1, "pilot", 1);
    const auto goal = arbiter.requestGoal(goal_identity, leaseFor(goal_identity, clock));
    ASSERT_TRUE(goal.accepted());
    const bool stopped = reason == StopReason::Fault ? arbiter.fault(goal.handle) :
      arbiter.leaseLost(goal.handle);
    EXPECT_TRUE(stopped);
    EXPECT_EQ(arbiter.state(), GoalState::ZeroBarrier);
    EXPECT_TRUE(arbiter.zeroLatched());
    EXPECT_FALSE(arbiter.recordZeroPublication());
    EXPECT_TRUE(arbiter.recordZeroPublication());
    EXPECT_EQ(arbiter.stopReason(), reason);
    EXPECT_TRUE(arbiter.zeroLatched());
  }
}

TEST(GoalArbiter, RequiresConsecutiveZeroPublicationsForHandoffAndTerminalRelease)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 3);
  const auto first_identity = identity(1, "pilot", 1);
  const auto first = arbiter.requestGoal(first_identity, leaseFor(first_identity, clock));
  const auto second_identity = identity(2, "pilot", 2);
  const auto second = arbiter.requestGoal(second_identity, leaseFor(second_identity, clock));
  ASSERT_EQ(second.admission, GoalAdmission::Pending);
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_EQ(arbiter.zeroPublicationCount(), 1U);
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_EQ(arbiter.zeroPublicationCount(), 2U);
  EXPECT_TRUE(arbiter.recordZeroPublication());
  EXPECT_TRUE(arbiter.isCurrent(second.handle));
  EXPECT_FALSE(arbiter.isCurrent(first.handle));
  EXPECT_TRUE(arbiter.complete(second.handle));
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_TRUE(arbiter.recordZeroPublication());
  EXPECT_EQ(arbiter.state(), GoalState::Completed);
}

TEST(GoalArbiter, PendingCancelFaultAndLeaseLossNeverActivate)
{
  FakeClock clock;
  for (const auto reason : {StopReason::Cancelled, StopReason::Fault, StopReason::LeaseLoss}) {
    GoalArbiter arbiter(clock, 2);
    const auto first_identity = identity(1, "pilot", 1);
    ASSERT_TRUE(arbiter.requestGoal(first_identity, leaseFor(first_identity, clock)).accepted());
    const auto pending_identity = identity(2, "pilot", 2);
    const auto pending = arbiter.requestGoal(pending_identity);
    ASSERT_EQ(pending.admission, GoalAdmission::Pending);
    bool stopped = false;
    if (reason == StopReason::Cancelled) {stopped = arbiter.cancel(pending.handle);}
    if (reason == StopReason::Fault) {stopped = arbiter.fault(pending.handle);}
    if (reason == StopReason::LeaseLoss) {stopped = arbiter.leaseLost(pending.handle);}
    EXPECT_TRUE(stopped);
    EXPECT_FALSE(arbiter.pendingGoal().has_value());
    EXPECT_FALSE(arbiter.recordZeroPublication());
    EXPECT_TRUE(arbiter.recordZeroPublication());
    EXPECT_FALSE(arbiter.activeGoal().has_value());
    EXPECT_EQ(arbiter.stopReason(), reason);
  }
}

TEST(GoalArbiter, PreservesDisplacedActiveTerminalRecordAndResetsBarrierOnFault)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 3);
  const auto first_identity = identity(1, "pilot", 1);
  const auto first = arbiter.requestGoal(first_identity, leaseFor(first_identity, clock));
  const auto second = arbiter.requestGoal(identity(2, "pilot", 2));
  ASSERT_TRUE(second.accepted());
  ASSERT_TRUE(arbiter.displacedActive().has_value());
  EXPECT_EQ(arbiter.displacedActive()->goal, first.handle.goal);
  EXPECT_EQ(arbiter.displacedActive()->state, GoalState::Displaced);
  EXPECT_EQ(arbiter.displacedActive()->reason, StopReason::Handoff);
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_EQ(arbiter.zeroPublicationCount(), 1U);
  arbiter.fault();
  EXPECT_EQ(arbiter.zeroPublicationCount(), 0U);
  EXPECT_FALSE(arbiter.pendingGoal().has_value());
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_TRUE(arbiter.recordZeroPublication());
  EXPECT_EQ(arbiter.state(), GoalState::Faulted);
}

class VehicleExecutionNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST(ActionServerMath, NavigationGpsCalculatesOffsetsAndBearing)
{
  double dx = 0.0;
  double dy = 0.0;
  banyu_vehicle_execution::latLonToEnuOffsets(0.0, 0.0, 0.0, 0.001, dx, dy);
  EXPECT_GT(dx, 0.0);
  EXPECT_NEAR(dy, 0.0, 1e-6);

  banyu_vehicle_execution::latLonToEnuOffsets(0.0, 0.0, 0.001, 0.0, dx, dy);
  EXPECT_NEAR(dx, 0.0, 1e-6);
  EXPECT_GT(dy, 0.0);
}

TEST(ActionServerMath, AngleNormalizationWrapsCorrectly)
{
  EXPECT_NEAR(banyu_vehicle_execution::normalizeAngle(0.0), 0.0, 1e-9);
  EXPECT_NEAR(banyu_vehicle_execution::normalizeAngle(M_PI), M_PI, 1e-9);
  EXPECT_NEAR(banyu_vehicle_execution::normalizeAngle(3.0 * M_PI), M_PI, 1e-6);
  EXPECT_NEAR(banyu_vehicle_execution::normalizeAngle(-3.0 * M_PI), -M_PI, 1e-6);
  EXPECT_NEAR(banyu_vehicle_execution::normalizeAngle(0.5), 0.5, 1e-9);
}

TEST(ActionServerMath, QuaternionToYawAccuracy)
{
  EXPECT_NEAR(banyu_vehicle_execution::getYawFromQuaternion(0.0, 0.0, 0.0, 1.0), 0.0, 1e-9);

  const double sin45 = std::sin(M_PI / 4.0);
  const double cos45 = std::cos(M_PI / 4.0);
  EXPECT_NEAR(
    banyu_vehicle_execution::getYawFromQuaternion(
      0.0, 0.0, sin45,
      cos45), M_PI / 2.0, 1e-6);
}

void spinFor(const rclcpp::Node::SharedPtr & node, int cycles = 15)
{
  for (int i = 0; i < cycles; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(VehicleExecutionNodeTest, NodeStartsAndDeclaresPreviewSink)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  EXPECT_EQ(node->arbiter().state(), GoalState::Idle);
  EXPECT_FALSE(node->telemetry().pose_valid);
  EXPECT_FALSE(node->telemetry().status_valid);
  EXPECT_FALSE(node->telemetry().gps_valid);
}

TEST_F(VehicleExecutionNodeTest, TelemetryUpdatesAndLeaseValidation)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", true);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  const auto goal_id = identity(5, "/banyu/actions/align_heading", 1);
  const auto req = node->requestGoal(goal_id);
  ASSERT_EQ(req.admission, GoalAdmission::Activated);
  EXPECT_TRUE(node->arbiter().zeroLatched());

  auto lease_pub =
    node->create_publisher<banyu_msgs::msg::ActionLease>("/banyu/execution/lease", 10);
  spinFor(node, 5);

  auto lease_msg = std::make_shared<banyu_msgs::msg::ActionLease>();
  lease_msg->header.stamp = node->now();
  std::copy(goal_id.uuid.begin(), goal_id.uuid.end(), lease_msg->goal_id.uuid.begin());
  lease_msg->owner = "/banyu/actions/align_heading";
  lease_msg->generation = 1;
  lease_msg->active = true;
  lease_msg->expires_at = node->now() + rclcpp::Duration::from_seconds(2.0);

  for (int i = 0; i < 50 && node->arbiter().zeroLatched(); ++i) {
    lease_pub->publish(*lease_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_FALSE(node->arbiter().zeroLatched());
}

TEST_F(VehicleExecutionNodeTest, OperatorTakeoverFaultsActiveGoals)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  const auto goal_id = identity(7, "/banyu/actions/dive_to_depth", 1);
  const auto req = node->requestGoal(goal_id);
  ASSERT_EQ(req.admission, GoalAdmission::Activated);
  EXPECT_FALSE(node->arbiter().zeroLatched());

  auto status_pub = node->create_publisher<banyu_msgs::msg::VehicleStatus>(
    "/banyu/vehicle_status",
    10);
  spinFor(node, 5);

  banyu_msgs::msg::VehicleStatus status;
  status.header.stamp = node->now();
  status.fcu_connected = true;
  status.operator_takeover = true;
  status_pub->publish(status);

  spinFor(node, 15);

  EXPECT_EQ(node->arbiter().state(), GoalState::ZeroBarrier);
  EXPECT_EQ(node->arbiter().stopReason(), StopReason::Fault);
}

TEST_F(VehicleExecutionNodeTest, FollowPipelineConsumesGuidanceWhenActive)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  auto guidance_pub = node->create_publisher<banyu_guidance::msg::VelocityIntent>(
    "/banyu/guidance/velocity_intent", 10);
  banyu_guidance::msg::VelocityIntent intent;
  intent.header.stamp = node->now();
  intent.status = 4;
  intent.surge_mps = 0.8f;
  intent.sway_mps = -0.3f;
  intent.yaw_rate_rad_s = 0.15f;
  intent.yaw_control_active = true;
  guidance_pub->publish(intent);

  rclcpp::spin_some(node);

  EXPECT_TRUE(node->telemetry().guidance_intent_valid);
  EXPECT_FLOAT_EQ(node->telemetry().guidance_surge_mps, 0.8f);
  EXPECT_FLOAT_EQ(node->telemetry().guidance_sway_mps, -0.3f);
  EXPECT_FLOAT_EQ(node->telemetry().guidance_yaw_rate_rad_s, 0.15f);
}

TEST(GoalArbiter, SequentialGoalAdmittedDuringZeroBarrierAndPromoted)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 2);

  const auto goal1_id = identity(1, "nav", 1);
  const auto goal1 = arbiter.requestGoal(goal1_id, leaseFor(goal1_id, clock));
  ASSERT_EQ(goal1.admission, GoalAdmission::Activated);

  // Goal 1 completes normally
  EXPECT_TRUE(arbiter.complete(goal1.handle));
  EXPECT_EQ(arbiter.state(), GoalState::ZeroBarrier);
  EXPECT_EQ(arbiter.stopReason(), StopReason::Completed);

  // Goal 2 arrives while arbiter is satisfying ZeroBarrier
  const auto goal2_id = identity(2, "align", 1);
  const auto goal2 = arbiter.requestGoal(goal2_id, leaseFor(goal2_id, clock));
  EXPECT_EQ(goal2.admission, GoalAdmission::Pending);

  // Publish required zero setpoints
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_TRUE(arbiter.recordZeroPublication());

  // Goal 2 is promoted to active after zero barrier
  EXPECT_EQ(arbiter.state(), GoalState::Active);
  ASSERT_TRUE(arbiter.activeGoal().has_value());
  EXPECT_EQ(arbiter.activeGoal()->goal.uuid, goal2_id.uuid);
}

TEST(GoalArbiter, SequentialGoalAdmittedAfterCompletedState)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 1);

  const auto goal1_id = identity(1, "dive", 1);
  const auto goal1 = arbiter.requestGoal(goal1_id, leaseFor(goal1_id, clock));
  ASSERT_EQ(goal1.admission, GoalAdmission::Activated);

  EXPECT_TRUE(arbiter.complete(goal1.handle));
  EXPECT_TRUE(arbiter.recordZeroPublication());
  EXPECT_EQ(arbiter.state(), GoalState::Completed);

  // Request goal 2 after completion
  const auto goal2_id = identity(2, "surface", 1);
  const auto goal2 = arbiter.requestGoal(goal2_id, leaseFor(goal2_id, clock));
  EXPECT_EQ(goal2.admission, GoalAdmission::Activated);
  EXPECT_EQ(arbiter.state(), GoalState::Active);
}

TEST(GoalArbiter, PendingGoalCanEstablishLeaseDuringZeroBarrier)
{
  FakeClock clock;
  GoalArbiter arbiter(clock, 2);

  const auto goal1_id = identity(1, "task", 1);
  ASSERT_TRUE(arbiter.requestGoal(goal1_id, leaseFor(goal1_id, clock)).accepted());
  EXPECT_TRUE(arbiter.complete(banyu_vehicle_execution::GoalHandle{goal1_id}));

  // Goal 2 requested without initial lease (zero latched)
  const auto goal2_id = identity(2, "task2", 1);
  const auto goal2 = arbiter.requestGoal(goal2_id);
  ASSERT_EQ(goal2.admission, GoalAdmission::Pending);

  // Establish lease while pending in ZeroBarrier
  EXPECT_TRUE(arbiter.establishLease(goal2.handle, leaseFor(goal2_id, clock)));

  // Complete zero cycles
  EXPECT_FALSE(arbiter.recordZeroPublication());
  EXPECT_TRUE(arbiter.recordZeroPublication());

  EXPECT_EQ(arbiter.state(), GoalState::Active);
  EXPECT_FALSE(arbiter.zeroLatched());
}

TEST_F(VehicleExecutionNodeTest, StaleStatusWatchdogFaultsArbiter)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  const auto goal_id = identity(8, "/banyu/actions/align_heading", 1);
  ASSERT_EQ(node->requestGoal(goal_id).admission, GoalAdmission::Activated);

  auto status_pub = node->create_publisher<banyu_msgs::msg::VehicleStatus>(
    "/banyu/vehicle_status", 10);
  spinFor(node, 5);

  // Send status stamped 2 seconds in the past to simulate stale drop
  banyu_msgs::msg::VehicleStatus status;
  status.header.stamp = node->now() - rclcpp::Duration::from_seconds(2.0);
  status.fcu_connected = true;
  status.fcu_status_fresh = true;
  status.operator_takeover = false;
  status_pub->publish(status);

  spinFor(node, 10);

  // Watchdog should detect stale status (> 1.0s) and fault
  EXPECT_FALSE(node->telemetry().status_valid);
  EXPECT_EQ(node->arbiter().state(), GoalState::ZeroBarrier);
  EXPECT_EQ(node->arbiter().stopReason(), StopReason::Fault);
}

TEST_F(VehicleExecutionNodeTest, HoldProbeContinuousResetOnRingLoss)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  auto server = std::make_shared<banyu_vehicle_execution::HoldProbeOnRingServer>(
    node.get(), "/banyu/actions/hold_probe_on_ring_test", node.get());

  banyu_vehicle_execution::VehicleTelemetry telem{};
  telem.ring_obs_valid = true;
  telem.ring_visible = true;
  telem.ring_offset_x = 0.01;
  telem.ring_offset_y = 0.01;

  // With no active goal, tick returns zeroIntent
  EXPECT_EQ(server->tick(telem, node->now(), 0.1).linear_x, 0.0);
}

TEST_F(VehicleExecutionNodeTest, ActionServersReturnZeroOnInvalidTelemetry)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  auto nav_server = std::make_shared<banyu_vehicle_execution::NavigateSurfaceGpsServer>(
    node.get(), "/banyu/actions/nav_test", node.get());
  auto surface_server = std::make_shared<banyu_vehicle_execution::SurfaceServer>(
    node.get(), "/banyu/actions/surf_test", node.get());

  banyu_vehicle_execution::VehicleTelemetry telem{};
  telem.gps_valid = false;
  telem.depth_valid = false;

  EXPECT_EQ(nav_server->tick(telem, node->now(), 0.1).linear_x, 0.0);
  EXPECT_EQ(surface_server->tick(telem, node->now(), 0.1).linear_z, 0.0);
}
