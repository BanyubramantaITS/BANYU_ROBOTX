#include "banyu_vehicle_execution/command_encoder.hpp"
#include "banyu_vehicle_execution/goal_arbiter.hpp"
#include "banyu_vehicle_execution/motion_intent.hpp"
#include "banyu_vehicle_execution/vehicle_telemetry.hpp"
#include "banyu_vehicle_execution/action_servers.hpp"
#include "banyu_vehicle_execution/vehicle_execution_node.hpp"
#include "banyu_vehicle_execution/por_visual_helpers.hpp"
#include "banyu_vehicle_execution/light_cue.hpp"
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

TEST(ActionServerMath, PorDetectionFreshnessUsesObjectAndAge)
{
  banyu_vehicle_execution::VehicleTelemetry telemetry;
  telemetry.detection_valid = true;
  telemetry.detection.object_id = 1;
  telemetry.detection.offset_x = 0.25F;
  telemetry.detection_stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  const auto now = rclcpp::Time(10, 200000000, RCL_ROS_TIME);
  banyu_msgs::msg::Detection copy;
  EXPECT_TRUE(banyu_vehicle_execution::detectionIsFresh(telemetry, now, 0.5, 1, &copy));
  EXPECT_FLOAT_EQ(copy.offset_x, 0.25F);
  EXPECT_FALSE(banyu_vehicle_execution::detectionIsFresh(telemetry, now, 0.5, 0));
  EXPECT_FALSE(banyu_vehicle_execution::detectionIsFresh(telemetry, now, 0.1, 1));
}

TEST(CommandEncoder, RcOverrideMapsBodyIntentToArduSubSticks)
{
  banyu_vehicle_execution::RcOverrideConfig config;
  config.channel_forward = 3;
  config.channel_lateral = 4;
  config.channel_throttle = 2;
  config.channel_yaw = 1;
  config.gain_xy_pwm_per_mps = 300.0;
  config.gain_z_pwm_per_mps = 300.0;
  config.gain_yaw_pwm_per_rad_s = 300.0;
  // Heading 90 deg (ENU north): body forward 0.5 m/s, left 0.2 m/s, up 0.1, CCW 0.4.
  const double yaw = M_PI / 2.0;
  const auto body = banyu_vehicle_execution::MotionIntent::bodyFlu(0.5, 0.2, 0.1, 0.4);
  const auto local = banyu_vehicle_execution::bodyFluToLocalEnu(body, yaw);
  ASSERT_TRUE(local.has_value());
  const auto msg = banyu_vehicle_execution::encodeRcOverride(*local, yaw, config);
  EXPECT_EQ(msg.channels[2], 1650);   // forward +0.5 -> stick forward
  EXPECT_EQ(msg.channels[3], 1440);   // left +0.2 -> lateral stick LEFT (below neutral)
  EXPECT_EQ(msg.channels[1], 1530);   // up +0.1 -> throttle up
  EXPECT_EQ(msg.channels[0], 1380);   // CCW +0.4 -> yaw stick LEFT
  EXPECT_EQ(msg.channels[4], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
}

TEST(CommandEncoder, RcOverrideCompensatesArduSubDeadZone)
{
  banyu_vehicle_execution::RcOverrideConfig config;   // yaw on channel 4
  config.gain_yaw_pwm_per_rad_s = 143.4;
  config.deadzone_yaw_pwm = 20;
  // Slow orbit yaw (CW 0.217 rad/s) starts outside the 20 PWM dead zone: 20 + 31 = 51.
  auto msg = banyu_vehicle_execution::encodeRcOverride(
    banyu_vehicle_execution::MotionIntent::localEnu(0.0, 0.0, 0.0, -0.217), 0.0, config);
  EXPECT_EQ(msg.channels[3], 1551);
  // Zero command stays exactly neutral (ArduSub then holds heading).
  msg = banyu_vehicle_execution::encodeRcOverride(
    banyu_vehicle_execution::MotionIntent::localEnu(0.0, 0.0, 0.0, 0.0), 0.0, config);
  EXPECT_EQ(msg.channels[3], 1500);
}

TEST(CommandEncoder, RcOverrideClampsAndNeutralReleasesOtherChannels)
{
  banyu_vehicle_execution::RcOverrideConfig config;   // RcOverrideConfig defaults (ArduSub stock RCMAP 5,6,3,4)
  config.max_delta_pwm = 200;
  const auto local = banyu_vehicle_execution::MotionIntent::localEnu(5.0, 0.0, 0.0, 0.0);
  const auto msg = banyu_vehicle_execution::encodeRcOverride(
    local, 0.0, config, banyu_vehicle_execution::IntentLimits(10.0, 1.0));
  EXPECT_EQ(msg.channels[4], 1700);   // clamped to neutral + max_delta_pwm
  const auto neutral = banyu_vehicle_execution::neutralRcOverride(config);
  EXPECT_EQ(neutral.channels[4], 1500);
  EXPECT_EQ(neutral.channels[2], 1500);
  EXPECT_EQ(neutral.channels[0], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  const auto released = banyu_vehicle_execution::releasedRcOverride();
  for (const auto value : released.channels) {
    EXPECT_EQ(value, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }
}

TEST(ActionServerMath, PorDetectionSlotsKeepGateAndMarkerTogether)
{
  banyu_vehicle_execution::VehicleTelemetry telemetry;
  // Marker arrived last in the legacy single slot, gate is in its own slot.
  telemetry.detection_valid = true;
  telemetry.detection.object_id = 1;
  telemetry.detection_stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  telemetry.detection_slot_valid[0] = true;
  telemetry.detection_slot[0].object_id = 0;
  telemetry.detection_slot[0].offset_x = -0.4F;
  telemetry.detection_slot_stamp[0] = rclcpp::Time(10, 0, RCL_ROS_TIME);
  telemetry.detection_slot_valid[1] = true;
  telemetry.detection_slot[1].object_id = 1;
  telemetry.detection_slot_stamp[1] = rclcpp::Time(9, 0, RCL_ROS_TIME);
  const auto now = rclcpp::Time(10, 100000000, RCL_ROS_TIME);
  banyu_msgs::msg::Detection gate;
  EXPECT_TRUE(banyu_vehicle_execution::detectionIsFresh(telemetry, now, 0.5, 0, &gate));
  EXPECT_FLOAT_EQ(gate.offset_x, -0.4F);
  // Marker slot is stale even though the legacy slot holds a fresh marker id.
  EXPECT_FALSE(banyu_vehicle_execution::detectionIsFresh(telemetry, now, 0.5, 1));
  EXPECT_NEAR(banyu_vehicle_execution::detectionAge(telemetry, now, 1), 1.1, 1e-6);
}

TEST(ActionServerMath, PorVisualCorrectionUsesLocalEnuRightAndYawSigns)
{
  const auto right = banyu_vehicle_execution::visualCorrection(
    0.5, 0.0, 1.0, 0.0,
    1.0, 1.0, 1.0, 1.0);
  const auto left = banyu_vehicle_execution::visualCorrection(
    -0.5, 0.0, 1.0, 0.0,
    1.0, 1.0, 1.0, 1.0);
  EXPECT_LT(right.lateral_flu_mps, 0.0);
  EXPECT_LT(right.yaw_rate_rad_s, 0.0);
  EXPECT_GT(left.lateral_flu_mps, 0.0);
  EXPECT_GT(left.yaw_rate_rad_s, 0.0);

  const auto local = banyu_vehicle_execution::bodyFluToLocalEnu(
    banyu_vehicle_execution::MotionIntent::bodyFlu(
      0.0, right.lateral_flu_mps, 0.0, right.yaw_rate_rad_s), 0.0);
  ASSERT_TRUE(local.has_value());
  EXPECT_LT(local->linear_y, 0.0);
  EXPECT_LT(local->yaw_rate, 0.0);
}

TEST_F(VehicleExecutionNodeTest, PorActionServersHaveSafeNoGoalOutput)
{
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>();
  banyu_vehicle_execution::VehicleTelemetry telemetry;
  telemetry.pose_valid = true;
  const auto now = node->now();
  auto check_zero = [&telemetry, &now](const auto & server) {
      const auto intent = server->tick(telemetry, now, 0.05);
      EXPECT_EQ(intent.frame, banyu_vehicle_execution::IntentFrame::LocalEnu);
      EXPECT_DOUBLE_EQ(intent.linear_x, 0.0);
      EXPECT_DOUBLE_EQ(intent.linear_y, 0.0);
      EXPECT_DOUBLE_EQ(intent.linear_z, 0.0);
      EXPECT_DOUBLE_EQ(intent.yaw_rate, 0.0);
    };
  check_zero(
    std::make_shared<banyu_vehicle_execution::AcquireGateServer>(
      node.get(), "/test/por/acquire_gate", node.get()));
  check_zero(
    std::make_shared<banyu_vehicle_execution::AlignMarkerServer>(
      node.get(), "/test/por/align_marker", node.get()));
  check_zero(
    std::make_shared<banyu_vehicle_execution::OrbitMarkerServer>(
      node.get(), "/test/por/orbit_marker", node.get()));
  check_zero(
    std::make_shared<banyu_vehicle_execution::TurnAroundServer>(
      node.get(), "/test/por/turn_around", node.get()));
}

void spinFor(const rclcpp::Node::SharedPtr & node, int cycles = 15)
{
  for (int i = 0; i < cycles; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template<typename FutureT>
bool waitForFuture(const rclcpp::Node::SharedPtr & node, FutureT & future, int cycles = 150)
{
  for (int i = 0; i < cycles; ++i) {
    rclcpp::spin_some(node);
    if (future.wait_for(0ms) == std::future_status::ready) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void publishPose(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher,
  double x, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.pose.position.x = x;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  publisher->publish(pose);
}

void publishDetection(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr & publisher,
  int64_t object_id, double offset_x, double offset_y, double distance)
{
  banyu_msgs::msg::Detection detection;
  detection.header.stamp = node->now();
  detection.object_id = object_id;
  detection.offset_x = static_cast<float>(offset_x);
  detection.offset_y = static_cast<float>(offset_y);
  detection.distance = static_cast<float>(distance);
  publisher->publish(detection);
}

void publishDepth(
  const std::shared_ptr<rclcpp::Node> & node,
  const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher,
  double depth)
{
  (void)node;
  std_msgs::msg::Float64 message;
  message.data = -depth;
  publisher->publish(message);
}

TEST_F(VehicleExecutionNodeTest, PorAlignMarkerHoldsInsideConfiguredStandoff)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::AlignMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/align_marker");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.deadband_x = 0.05; goal.deadband_y = 0.05;
  goal.minimum_distance_m = 1.8; goal.standoff_min_m = 2.2; goal.standoff_max_m = 2.5;
  goal.hold_time_s = 0.05; goal.timeout_s = 1.0;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);
  auto result_future = client->async_get_result(goal_handle);
  for (int i = 0; i < 40 && result_future.wait_for(0ms) != std::future_status::ready; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 1, 0.0, 0.0, 2.35);
    spinFor(node, 1);
  }
  ASSERT_TRUE(result_future.wait_for(0ms) == std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->success);
  EXPECT_GE(result.result->final_distance_m, 2.2);
  EXPECT_LE(result.result->final_distance_m, 2.5);
}

TEST_F(VehicleExecutionNodeTest, PorAlignMarkerFreshRequirementsKeepWatchdogAlive)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::AlignMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/align_marker");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.deadband_x = 0.05; goal.deadband_y = 0.05;
  goal.minimum_distance_m = 1.8; goal.standoff_min_m = 2.2; goal.standoff_max_m = 2.5;
  goal.hold_time_s = 0.5; goal.timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 25; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 1, 0.0, 0.0, 8.0 - 0.15 * i);
    spinFor(node, 2);
  }
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  auto cancel_future = client->async_cancel_goal(handle);
  ASSERT_TRUE(waitForFuture(node, cancel_future));
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(VehicleExecutionNodeTest, PorAlignMarkerMissingDetectionTimesOut)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::AlignMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/align_marker");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.deadband_x = 0.05; goal.deadband_y = 0.05;
  goal.minimum_distance_m = 1.8; goal.standoff_min_m = 2.2; goal.standoff_max_m = 2.5;
  goal.hold_time_s = 0.5; goal.timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 25; ++i) {
    publishPose(node, pose_pub, 0.0);
    spinFor(node, 2);
  }
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(VehicleExecutionNodeTest, CenterOnRingSearchesThenAbortsWhenRingNeverReappears)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("control.ring_search_timeout_s", 0.2);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::CenterOnRing;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/center_on_ring");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.ring_color = "red";
  goal.tolerance = 0.1;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 40; ++i) {
    publishPose(node, pose_pub, 0.0);
    spinFor(node, 2);
  }
  ASSERT_TRUE(waitForFuture(node, result_future));
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(wrapped.result->success);
  EXPECT_NE(wrapped.result->reason.find("not reacquired"), std::string::npos);
}

TEST_F(VehicleExecutionNodeTest, PorAcquireGatePoseLossTimesOutAndRecoveryResetsWatchdog)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::AcquireGate;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/acquire_gate");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto depth_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.target_depth_m = 2.5; goal.depth_tolerance_m = 0.1; goal.timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  publishDepth(node, depth_pub, 2.5);
  publishDetection(node, detection_pub, 0, 0.0, 0.0, 3.0);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);

  send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  result_future = client->async_get_result(handle);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  for (int i = 0; i < 25; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDepth(node, depth_pub, 3.0);
    publishDetection(node, detection_pub, 0, 0.0, 0.0, 3.0);
    spinFor(node, 2);
  }
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  publishPose(node, pose_pub, 0.0);
  publishDepth(node, depth_pub, 2.5);
  publishDetection(node, detection_pub, 0, 0.0, 0.0, 3.0);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(VehicleExecutionNodeTest, PorAlignGatePoseLossTimesOutAndRecoveryResetsWatchdog)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("actions.align_gate", "/test/align_gate_pose_watchdog");
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::AlignGate;
  auto client = rclcpp_action::create_client<Action>(node, "/test/align_gate_pose_watchdog");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.deadband_x = 0.05; goal.deadband_y = 0.05; goal.hold_time_s = 0.05; goal.timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  publishDetection(node, detection_pub, 0, 0.0, 0.0, 3.0);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);

  send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  result_future = client->async_get_result(handle);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  for (int i = 0; i < 25; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 0, 0.0, 0.0, 3.0);
    spinFor(node, 2);
  }
  publishPose(node, pose_pub, 0.0);
  publishDetection(node, detection_pub, 0, 0.0, 0.0, 3.0);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(VehicleExecutionNodeTest, PorAlignMarkerPoseLossTimesOutAndRecoveryResetsWatchdog)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("actions.align_marker", "/test/align_marker_pose_watchdog");
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::AlignMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/test/align_marker_pose_watchdog");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.deadband_x = 0.05; goal.deadband_y = 0.05;
  goal.minimum_distance_m = 1.8; goal.standoff_min_m = 2.2; goal.standoff_max_m = 2.5;
  goal.hold_time_s = 0.05; goal.timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  publishDetection(node, detection_pub, 1, 0.0, 0.0, 2.35);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);

  send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  result_future = client->async_get_result(handle);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  for (int i = 0; i < 25; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 1, 0.0, 0.0, 2.35);
    spinFor(node, 2);
  }
  publishPose(node, pose_pub, 0.0);
  publishDetection(node, detection_pub, 1, 0.0, 0.0, 2.35);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(VehicleExecutionNodeTest, PorPassGateWithoutFreshGateTimesOutBeforeBlindMode)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::PassGate;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/pass_gate");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.return_leg = true; goal.speed_mps = 0.3; goal.blind_duration_s = 0.1; goal.timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 25; ++i) {
    publishPose(node, pose_pub, 0.0);
    spinFor(node, 2);
  }
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(VehicleExecutionNodeTest, PorPassGatePoseLossDuringBlindAborts)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("actions.pass_gate", "/test/pass_gate_pose_watchdog");
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::PassGate;
  auto client = rclcpp_action::create_client<Action>(node, "/test/pass_gate_pose_watchdog");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal;
  goal.return_leg = true; goal.speed_mps = 0.3; goal.blind_duration_s = 2.0; goal.timeout_s = 4.0;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 10; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 0, 0.0, 0.0, 3.0);
    spinFor(node, 2);
  }
  for (int i = 0; i < 35; ++i) {
    publishPose(node, pose_pub, 0.0);
    spinFor(node, 2);
  }
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  for (int i = 0; i < 35; ++i) {
    spinFor(node, 2);
  }
  ASSERT_TRUE(waitForFuture(node, result_future, 20));
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_FALSE(result.result->success);
  EXPECT_EQ(result.result->reason, "Pose lost during blind gate pass");
}

TEST_F(VehicleExecutionNodeTest, PorRejectsUnsafeAndNonorderedGoals)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Orbit = banyu_msgs::action::OrbitMarker;
  auto orbit_client = rclcpp_action::create_client<Orbit>(node, "/banyu/actions/orbit_marker");
  ASSERT_TRUE(orbit_client->wait_for_action_server(1s));
  Orbit::Goal orbit_goal; orbit_goal.radius_m = 2.19;
  auto orbit_future = orbit_client->async_send_goal(orbit_goal);
  ASSERT_TRUE(waitForFuture(node, orbit_future));
  EXPECT_EQ(orbit_future.get(), nullptr);

  using Align = banyu_msgs::action::AlignMarker;
  auto align_client = rclcpp_action::create_client<Align>(node, "/banyu/actions/align_marker");
  ASSERT_TRUE(align_client->wait_for_action_server(1s));
  Align::Goal align_goal; align_goal.standoff_min_m = 2.5; align_goal.standoff_max_m = 2.2;
  auto align_future = align_client->async_send_goal(align_goal);
  ASSERT_TRUE(waitForFuture(node, align_future));
  EXPECT_EQ(align_future.get(), nullptr);

  Align::Goal minimum_goal;
  minimum_goal.minimum_distance_m = 2.3;
  minimum_goal.standoff_min_m = 2.2;
  minimum_goal.standoff_max_m = 2.5;
  align_future = align_client->async_send_goal(minimum_goal);
  ASSERT_TRUE(waitForFuture(node, align_future));
  EXPECT_EQ(align_future.get(), nullptr);

  using Search = banyu_msgs::action::SearchMarker;
  auto search_client = rclcpp_action::create_client<Search>(node, "/banyu/actions/search_marker");
  ASSERT_TRUE(search_client->wait_for_action_server(1s));
  Search::Goal search_goal;
  search_goal.search_speed_mps = -0.1;
  auto search_future = search_client->async_send_goal(search_goal);
  ASSERT_TRUE(waitForFuture(node, search_future));
  EXPECT_EQ(search_future.get(), nullptr);
}

TEST_F(VehicleExecutionNodeTest, PorOrbitBelowSafeRangeNeverSurgesTowardMarker)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::OrbitMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/orbit_marker");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  double maximum_surge = -std::numeric_limits<double>::infinity();
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&maximum_surge](const mavros_msgs::msg::PositionTarget::SharedPtr message) {
      maximum_surge = std::max(maximum_surge, static_cast<double>(message->velocity.x));
    });
  Action::Goal goal;
  goal.orbit_speed_mps = 0.2; goal.radius_m = 2.2; goal.range_gain = 0.5;
  goal.max_range_speed_mps = 0.3; goal.timeout_s = 0.5;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 15; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 1, 0.0, 0.0, 2.0);
    spinFor(node, 1);
  }
  EXPECT_LE(maximum_surge, 0.0);
  auto cancel_future = client->async_cancel_goal(handle);
  ASSERT_TRUE(waitForFuture(node, cancel_future));
  ASSERT_TRUE(waitForFuture(node, result_future));
}

TEST_F(VehicleExecutionNodeTest, PorPassGateUsesTimedBlindForwardPassAfterStaleDetection)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::PassGate;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/pass_gate");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  double latest_forward = 0.0;
  double latest_lateral = 0.0;
  double latest_yaw_rate = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&latest_forward, &latest_lateral, &latest_yaw_rate](
      const mavros_msgs::msg::PositionTarget::SharedPtr message) {
      latest_forward = message->velocity.x;
      latest_lateral = message->velocity.y;
      latest_yaw_rate = message->yaw_rate;
    });
  Action::Goal goal;
  goal.return_leg = true; goal.speed_mps = 0.4; goal.target_distance_m = 3.0;
  goal.minimum_distance_m = 0.8; goal.blind_duration_s = 0.3; goal.timeout_s = 2.0;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);
  auto result_future = client->async_get_result(goal_handle);
  for (int i = 0; i < 10; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 0, 0.0, 0.0, 0.0);
    spinFor(node, 1);
  }
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  for (int i = 0; i < 55; ++i) {
    publishPose(node, pose_pub, 0.0);
    spinFor(node, 1);
  }
  EXPECT_GT(latest_forward, 0.0);
  EXPECT_DOUBLE_EQ(latest_lateral, 0.0);
  EXPECT_DOUBLE_EQ(latest_yaw_rate, 0.0);
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  spinFor(node, 10);
  for (int i = 0; i < 50 && result_future.wait_for(0ms) != std::future_status::ready; ++i) {
    publishPose(node, pose_pub, 0.0);
    spinFor(node, 1);
  }
  ASSERT_TRUE(result_future.wait_for(0ms) == std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->success);
}

TEST_F(VehicleExecutionNodeTest, PorPassGateReturnLegSucceedsOnTargetDistanceReached)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::PassGate;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/pass_gate");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());

  Action::Goal goal;
  goal.return_leg = true;
  goal.speed_mps = 0.8;
  goal.target_distance_m = 10.5;
  goal.minimum_distance_m = 0.8;
  goal.blind_duration_s = 14.0;
  goal.timeout_s = 5.0;

  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);
  auto result_future = client->async_get_result(goal_handle);

  // Initially see the gate at start pose x = 5.0
  publishPose(node, pose_pub, 5.0, 0.0);
  publishDetection(node, detection_pub, 0, 0.0, 0.0, 9.0);
  spinFor(node, 2);

  // Now detection is lost (blind mode), but vehicle keeps pushing through
  double current_x = 5.0;
  for (int i = 0; i < 60 && result_future.wait_for(0ms) != std::future_status::ready; ++i) {
    if (current_x > -6.0) {
      current_x -= 0.5;
    }
    publishPose(node, pose_pub, current_x, 0.0);
    spinFor(node, 1);
  }

  ASSERT_TRUE(waitForFuture(node, result_future));
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->reason, "Gate pass distance reached");
  EXPECT_GE(result.result->travelled_m, 10.5);
}

TEST_F(VehicleExecutionNodeTest, PorSearchMarkerSucceedsOnFreshDetectionWithoutDistance)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::SearchMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/search_marker");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.search_speed_mps = 0.1; goal.timeout_s = 0.5;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr);
  auto result_future = client->async_get_result(goal_handle);
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  publishDetection(node, detection_pub, 1, 0.0, 0.0, 0.0);
  spinFor(node, 4);
  ASSERT_TRUE(waitForFuture(node, result_future));
  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->success);

  goal.timeout_s = 0.5;
  send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto unavailable_handle = send_future.get();
  ASSERT_NE(unavailable_handle, nullptr);
  auto unavailable_result = client->async_get_result(unavailable_handle);
  publishDetection(
    node, detection_pub, 1, 0.0, 0.0,
    std::numeric_limits<float>::quiet_NaN());
  spinFor(node, 4);
  ASSERT_TRUE(waitForFuture(node, unavailable_result));
  result = unavailable_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->success);
}

TEST_F(VehicleExecutionNodeTest, PorSearchMarkerTimesOutWithoutDetection)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::SearchMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/search_marker");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal; goal.search_speed_mps = 0.1; goal.timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(VehicleExecutionNodeTest, TypedPorCancellationEntersBarrierAndAllowsReuse)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::SearchGate;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/search_gate");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal; goal.search_speed_mps = 0.1; goal.timeout_s = 5.0;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto cancel_future = client->async_cancel_goal(handle);
  ASSERT_TRUE(waitForFuture(node, cancel_future));
  auto result_future = client->async_get_result(handle);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  spinFor(node, 8);

  goal.timeout_s = 0.1;
  send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto reused_handle = send_future.get();
  ASSERT_NE(reused_handle, nullptr);
  auto reused_result = client->async_get_result(reused_handle);
  ASSERT_TRUE(waitForFuture(node, reused_result));
  EXPECT_EQ(reused_result.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(VehicleExecutionNodeTest, PorAcquireGateDoesNotCommandWithoutFreshDepth)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::AcquireGate;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/acquire_gate");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  int nonzero_z = 0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&nonzero_z](const mavros_msgs::msg::PositionTarget::SharedPtr message) {
      if (std::abs(message->velocity.z) > 1e-6) {++nonzero_z;}
    });
  Action::Goal goal; goal.target_depth_m = 2.5; goal.depth_tolerance_m = 0.1;
  goal.timeout_s = 0.2;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(nonzero_z, 0);
}

TEST_F(VehicleExecutionNodeTest, PorTurnCapturesGateAndReportsOnlyARealCapture)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::TurnAround;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/turn_around");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal; goal.turn_yaw_rate_rad_s = 0.3; goal.capture_gain = 0.5;
  goal.capture_deadband = 0.1; goal.max_angle_rad = M_PI; goal.timeout_s = 1.0;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 8; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 0, 0.5, 0.0, 3.0);
    spinFor(node, 1);
  }
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  for (int i = 0; i < 20 && result_future.wait_for(0ms) != std::future_status::ready; ++i) {
    publishPose(node, pose_pub, 0.0);
    publishDetection(node, detection_pub, 0, 0.05, 0.0, 3.0);
    spinFor(node, 1);
  }
  ASSERT_TRUE(result_future.wait_for(0ms) == std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_TRUE(result.result->gate_captured);
}

TEST_F(VehicleExecutionNodeTest, PorOrbitCompletesAfterIntegratedYawWrap)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::OrbitMarker;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/orbit_marker");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());
  Action::Goal goal; goal.orbit_speed_mps = 0.2; goal.radius_m = 3.0;
  goal.angle_rad = 0.0; goal.timeout_s = 3.0;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  for (double unwrapped = 0.0; unwrapped <= 6.4 &&
    result_future.wait_for(0ms) != std::future_status::ready; unwrapped += 0.8)
  {
    publishPose(node, pose_pub, 0.0, banyu_vehicle_execution::normalizeAngle(unwrapped));
    publishDetection(node, detection_pub, 1, 0.0, 0.0, 3.0);
    spinFor(node, 8);
  }
  ASSERT_TRUE(result_future.wait_for(0ms) == std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_GE(std::abs(result.result->accumulated_yaw_rad), 2.0 * M_PI);
  EXPECT_DOUBLE_EQ(result.result->final_distance_m, 3.0);
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

TEST_F(VehicleExecutionNodeTest, GoalActivationImmediate)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  const auto goal_id = identity(5, "/banyu/actions/align_heading", 1);
  const auto req = node->requestGoal(goal_id);
  ASSERT_EQ(req.admission, GoalAdmission::Activated);
  EXPECT_FALSE(node->arbiter().zeroLatched());
  ASSERT_TRUE(node->arbiter().activeGoal().has_value());
  EXPECT_EQ(node->arbiter().activeGoal()->goal.owner, "/banyu/actions/align_heading");
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
  spinFor(node, 5);
  banyu_guidance::msg::VelocityIntent intent;
  intent.header.stamp = node->now();
  intent.status = 4;
  intent.surge_mps = 0.8f;
  intent.sway_mps = -0.3f;
  intent.yaw_rate_rad_s = 0.15f;
  intent.yaw_control_active = true;

  for (int i = 0; i < 50 && !node->telemetry().guidance_intent_valid; ++i) {
    guidance_pub->publish(intent);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

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
  status.armed = true;
  status.operator_takeover = false;
  status_pub->publish(status);

  spinFor(node, 20);

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

TEST_F(VehicleExecutionNodeTest, SurfaceWatchdogIsOptInAndResetsAfterDepthRecovery)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::Surface;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/surface");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.target_depth_m = 2.0; goal.depth_tolerance_m = 0.1; goal.watchdog_timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  spinFor(node, 8);
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);

  auto depth_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());
  std_msgs::msg::Float64 depth;
  depth.data = -1.0;
  depth_pub->publish(depth);
  spinFor(node, 3);
  spinFor(node, 8);
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);

  depth.data = -2.0;
  depth_pub->publish(depth);
  spinFor(node, 5);
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  const auto until = std::chrono::steady_clock::now() + 1500ms;
  while (result_future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < until)
  {
    depth_pub->publish(depth);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(VehicleExecutionNodeTest, SurfaceWatchdogTimesOutWhenEnabledAndDepthIsMissing)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::Surface;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/surface");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.target_depth_m = 2.0; goal.depth_tolerance_m = 0.1; goal.watchdog_timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(VehicleExecutionNodeTest, SurfaceWithNoDepthRisesBlindUntilItsTimeLimit)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("control.surface_blind_rise_rate_mps", 0.3);
  options.append_parameter_override("control.surface_blind_timeout_s", 1.0);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  double max_vz = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&max_vz](const mavros_msgs::msg::PositionTarget::SharedPtr msg) {max_vz = std::max(max_vz, (double)msg->velocity.z);});
  using Action = banyu_msgs::action::Surface;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/surface");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.target_depth_m = 0.0; goal.depth_tolerance_m = 0.3; goal.watchdog_timeout_s = 0.15;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  spinFor(node, 5);
  // Not the 0.15 s depth watchdog: the blind rise runs.
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NEAR(max_vz, 0.3, 1e-6);
}

TEST_F(VehicleExecutionNodeTest, SurfaceWatchdogZeroDisablesMissingDepthTimeout)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::Surface;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/surface");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.target_depth_m = 2.0; goal.depth_tolerance_m = 0.1; goal.watchdog_timeout_s = 0.0;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  spinFor(node, 25);
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  auto cancel_future = client->async_cancel_goal(handle);
  ASSERT_TRUE(waitForFuture(node, cancel_future));
  ASSERT_TRUE(waitForFuture(node, result_future));
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(VehicleExecutionNodeTest, MavrosPublicationGatedByStatusAndGuidedMode)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.enable_mavros_publish", true);
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("control.require_guided", true);
  options.append_parameter_override("topics.status", "/test_gated/status");
  options.append_parameter_override("topics.mavros_target", "/test_gated/mavros_target");
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  int mavros_count = 0;
  mavros_msgs::msg::PositionTarget last_target;
  auto mavros_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/test_gated/mavros_target", 10,
    [&mavros_count, &last_target](const mavros_msgs::msg::PositionTarget::SharedPtr msg) {
      ++mavros_count;
      last_target = *msg;
    });

  auto status_pub = node->create_publisher<banyu_msgs::msg::VehicleStatus>(
    "/test_gated/status", 10);

  spinFor(node, 5);
  // Without status, no publication should occur
  EXPECT_EQ(mavros_count, 0);

  // Send status with non-GUIDED mode (e.g. MANUAL)
  banyu_msgs::msg::VehicleStatus status;
  status.header.stamp = node->now();
  status.fcu_connected = true;
  status.fcu_status_fresh = true;
  status.operator_takeover = false;
  status.mode = "MANUAL";
  status.armed = true;
  status.rc_fresh = true;
  status_pub->publish(status);
  spinFor(node, 5);
  EXPECT_EQ(mavros_count, 0);

  // Send valid GUIDED status
  status.header.stamp = node->now();
  status.mode = "GUIDED";
  status.operational_mode = 2;
  status_pub->publish(status);
  spinFor(node, 10);

  EXPECT_GT(mavros_count, 0);
  EXPECT_EQ(
    last_target.coordinate_frame,
    mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(last_target.type_mask, 0x5C7);
  EXPECT_DOUBLE_EQ(last_target.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(last_target.position.x, 0.0);
  EXPECT_DOUBLE_EQ(last_target.position.y, 0.0);
  EXPECT_DOUBLE_EQ(last_target.position.z, 0.0);

  // Simulate operator takeover: publication must cease
  status.header.stamp = node->now();
  status.operator_takeover = true;
  status_pub->publish(status);
  for (int i = 0; i < 30 && !node->telemetry().operator_takeover; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(node->telemetry().operator_takeover);

  // Allow any in-flight message published before takeover to be received
  spinFor(node, 5);
  const int stopped_count = mavros_count;

  // Spin for multiple timer cycles (timer is 50ms, 150ms is 3 cycles)
  spinFor(node, 15);
  EXPECT_EQ(mavros_count, stopped_count);
}

TEST_F(VehicleExecutionNodeTest, PreviewConfigurationCannotBypassMavrosSafety)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.enable_mavros_publish", true);
  options.append_parameter_override("control.require_guided", false);
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("topics.status", "/test_ungated/status");
  options.append_parameter_override("topics.mavros_target", "/test_ungated/mavros_target");
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  int mavros_count = 0;
  auto mavros_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/test_ungated/mavros_target", 10,
    [&mavros_count](const mavros_msgs::msg::PositionTarget::SharedPtr) {
      ++mavros_count;
    });

  auto status_pub = node->create_publisher<banyu_msgs::msg::VehicleStatus>(
    "/test_ungated/status", 10);

  spinFor(node, 10);
  EXPECT_EQ(mavros_count, 0);

  // Preview configuration cannot authorize hardware without GUIDED and RC.
  banyu_msgs::msg::VehicleStatus status;
  status.header.stamp = node->now();
  status.fcu_connected = true;
  status.fcu_status_fresh = true;
  status.rc_fresh = true;
  status.armed = true;
  status.operator_takeover = false;
  status.mode = "MANUAL";
  status_pub->publish(status);
  spinFor(node, 10);
  EXPECT_EQ(mavros_count, 0);
  status.header.stamp = node->now();
  status.mode = "GUIDED";
  status.operational_mode = 2;
  status.rc_fresh = false;
  status_pub->publish(status);
  spinFor(node, 10);
  EXPECT_EQ(mavros_count, 0);
  status.header.stamp = node->now();
  status.rc_fresh = true;
  status_pub->publish(status);
  spinFor(node, 10);
  EXPECT_GT(mavros_count, 0);
  status.header.stamp = node->now();
  status.operator_takeover = true;
  status_pub->publish(status);

  for (int i = 0; i < 30 && !node->telemetry().operator_takeover; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(node->telemetry().operator_takeover);

  spinFor(node, 5);
  const int stopped_count = mavros_count;
  spinFor(node, 15);
  EXPECT_EQ(mavros_count, stopped_count);
}

TEST_F(VehicleExecutionNodeTest, TimedMoveExecutesTimedSegmentAndSupportsZeroDurationSkip)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::TimedMove;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/timed_move");
  ASSERT_TRUE(client->wait_for_action_server(1s));

  // 1. Pure timed move with zero sensors (no pose, no baro depth, no IMU yaw cap)
  Action::Goal goal;
  goal.segment_name = "pass_gate";
  goal.duration_s = 0.15;
  goal.speed_x_mps = 0.5;
  goal.speed_y_mps = 0.0;
  goal.speed_z_mps = -0.1;
  goal.yaw_rate_rad_s = -0.5;
  goal.use_baro_depth = false;
  goal.target_depth_m = 2.0;
  goal.depth_kp = 0.7;
  goal.max_depth_speed_mps = 0.4;
  goal.use_imu_yaw_cap = false;
  goal.target_yaw_delta_rad = 0.0;

  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);

  const auto until = std::chrono::steady_clock::now() + 1500ms;
  while (result_future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < until)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(waitForFuture(node, result_future));
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped.result->success);
  EXPECT_GE(wrapped.result->elapsed_s, 0.14);

  // 2. Zero-duration segment skips immediately
  goal.segment_name = "skip_turn";
  goal.duration_s = 0.0;
  auto zero_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, zero_future));
  auto zero_handle = zero_future.get();
  ASSERT_NE(zero_handle, nullptr);
  auto zero_result_future = client->async_get_result(zero_handle);
  spinFor(node, 5);
  ASSERT_TRUE(waitForFuture(node, zero_result_future));
  EXPECT_EQ(zero_result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(zero_result_future.get().result->success);
}

TEST_F(VehicleExecutionNodeTest, TimedMoveHoldsHeadingAndCapsHorizontalSpeed)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("timed_move.heading_hold.enabled", true);
  options.append_parameter_override("timed_move.heading_hold.kp", 1.0);
  options.append_parameter_override("timed_move.heading_hold.max_yaw_rate_rad_s", 0.3);
  options.append_parameter_override("timed_move.max_horizontal_speed_mps", 0.1);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::TimedMove;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/timed_move");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());

  double max_speed = 0.0;
  double last_yaw_rate = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&](const mavros_msgs::msg::PositionTarget::SharedPtr m) {
      max_speed = std::max(max_speed, std::hypot(m->velocity.x, m->velocity.y));
      last_yaw_rate = m->yaw_rate;
    });

  Action::Goal goal;
  goal.segment_name = "approach_pole";
  goal.duration_s = 5.0;
  goal.speed_x_mps = 0.5;
  goal.yaw_rate_rad_s = 0.0;
  auto send_future = client->async_send_goal(goal);
  for (int i = 0; i < 5; ++i) {
    publishPose(node, pose_pub, 0.0, 0.0);
    spinFor(node, 1);
  }
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);

  for (int i = 0; i < 5; ++i) {
    publishPose(node, pose_pub, 0.0, 0.0);
    spinFor(node, 1);
  }
  // Vehicle yawed right (clockwise) by 0.2 rad: the hold must command a positive (left) rate.
  for (int i = 0; i < 10; ++i) {
    publishPose(node, pose_pub, 0.0, -0.2);
    spinFor(node, 1);
  }
  EXPECT_GT(last_yaw_rate, 0.15);
  EXPECT_LE(last_yaw_rate, 0.3 + 1e-6);
  EXPECT_GT(max_speed, 0.0);
  EXPECT_LE(max_speed, 0.1 + 1e-6);

  auto cancel_future = client->async_cancel_goal(handle);
  ASSERT_TRUE(waitForFuture(node, cancel_future));
  ASSERT_TRUE(waitForFuture(node, result_future));
}

TEST_F(VehicleExecutionNodeTest, TimedMoveCollisionProximityGuardAbortsWhenObstacleDetected)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  auto detection_pub = node->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection", rclcpp::SensorDataQoS());

  using Action = banyu_msgs::action::TimedMove;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/timed_move");
  ASSERT_TRUE(client->wait_for_action_server(1s));

  Action::Goal goal;
  goal.segment_name = "approach_pole";
  goal.duration_s = 2.0;
  goal.speed_x_mps = 0.5;
  goal.enable_collision_guard = true;
  goal.collision_distance_m = 0.8;

  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);

  // Publish obstacle detection closer than collision safety distance (0.4 m < 0.8 m)
  for (int i = 0; i < 20 && result_future.wait_for(0ms) != std::future_status::ready; ++i) {
    publishDetection(node, detection_pub, 1, 0.0, 0.0, 0.4);
    spinFor(node, 2);
  }

  ASSERT_TRUE(waitForFuture(node, result_future));
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(wrapped.result->success);
  EXPECT_EQ(wrapped.result->reason, "Collision guard triggered: object detected within safety distance");
}

TEST_F(VehicleExecutionNodeTest, TimedMoveCompletesWhenImuYawTargetReached)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());

  using Action = banyu_msgs::action::TimedMove;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/timed_move");
  ASSERT_TRUE(client->wait_for_action_server(1s));

  Action::Goal goal;
  goal.segment_name = "turn_90deg";
  goal.duration_s = 5.0;
  goal.yaw_rate_rad_s = 0.6;
  goal.use_imu_yaw_cap = true;
  goal.target_yaw_delta_rad = 1.570796327;

  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);

  // Initialize baseline yaw at 0.0
  for (int i = 0; i < 5; ++i) {
    publishPose(node, pose_pub, 0.0, 0.0);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Turn to 1.58 rad (delta = 1.58 > 1.570796327)
  for (int i = 0; i < 30 && result_future.wait_for(0ms) != std::future_status::ready; ++i) {
    publishPose(node, pose_pub, 0.0, 1.58);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(waitForFuture(node, result_future));
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped.result->success);
  EXPECT_NE(wrapped.result->reason.find("Target yaw delta reached"), std::string::npos);
  EXPECT_LT(wrapped.result->elapsed_s, 4.0);
}

TEST_F(VehicleExecutionNodeTest, TimedMoveCompletesWhenBaroDepthReached)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  auto depth_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());

  using Action = banyu_msgs::action::TimedMove;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/timed_move");
  ASSERT_TRUE(client->wait_for_action_server(1s));

  Action::Goal goal;
  goal.segment_name = "dive_to_gate";
  goal.duration_s = 5.0;
  goal.use_baro_depth = true;
  goal.target_depth_m = 2.0;
  goal.check_depth_reached = true;
  goal.depth_tolerance_m = 0.2;

  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);

  for (int i = 0; i < 35; ++i) {
    publishDepth(node, depth_pub, 2.0);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(waitForFuture(node, result_future));
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped.result->success);
  EXPECT_NE(wrapped.result->reason.find("Target depth reached"), std::string::npos);
  EXPECT_LT(wrapped.result->elapsed_s, 4.0);
}

namespace
{
// Runs SearchPipeline for a while with a fresh pinger dead ahead; returns the min and max ENU x
// velocity (vehicle heading east) that the search commanded.
std::pair<double, double> searchWithPingerAhead(bool use_hydrophone, bool * pinger_seen)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("control.use_hydrophone", use_hydrophone);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  double min_vx = 0.0;
  double max_vx = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&](const mavros_msgs::msg::PositionTarget::SharedPtr msg) {
      min_vx = std::min(min_vx, static_cast<double>(msg->velocity.x));
      max_vx = std::max(max_vx, static_cast<double>(msg->velocity.x));
    });
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto pinger_pub = node->create_publisher<banyu_msgs::msg::PingerObservation>(
    "/banyu/pinger/observation", rclcpp::SensorDataQoS());
  using Action = banyu_msgs::action::SearchPipeline;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/search_pipeline");
  EXPECT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.search_span_rad = 10.0;
  goal.sweep_rate_rad_s = 0.2;
  auto send_future = client->async_send_goal(goal);
  EXPECT_TRUE(waitForFuture(node, send_future));
  for (int i = 0; i < 40; ++i) {
    publishPose(node, pose_pub, 0.0);
    banyu_msgs::msg::PingerObservation obs;
    obs.header.stamp = node->now();
    obs.forward = 1.0F;
    obs.bearing = 0.0F;
    pinger_pub->publish(obs);
    spinFor(node, 2);
  }
  *pinger_seen = node->telemetry().pinger_obs_valid;
  return {min_vx, max_vx};
}
}  // namespace

TEST_F(VehicleExecutionNodeTest, SearchPipelineWithoutHydrophoneIgnoresPingerAndRetraces)
{
  bool pinger_seen = true;
  const auto [min_vx, max_vx] = searchWithPingerAhead(false, &pinger_seen);
  EXPECT_FALSE(pinger_seen);
  // Retrace backward along the entry heading (east), never the pinger's forward homing.
  EXPECT_NEAR(min_vx, -0.2, 1e-3);
  EXPECT_LE(max_vx, 1e-6);
}

TEST_F(VehicleExecutionNodeTest, SearchPipelineHomesOnPingerOnlyWhenOptedIn)
{
  bool pinger_seen = false;
  const auto [min_vx, max_vx] = searchWithPingerAhead(true, &pinger_seen);
  EXPECT_TRUE(pinger_seen);
  EXPECT_NEAR(max_vx, 0.25, 1e-3);
  EXPECT_GE(min_vx, -1e-6);
}

TEST_F(VehicleExecutionNodeTest, BaroCameraDescentSinksUntilThePipeAndStopsAtTheMaximumDepth)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("camera_descent.with_depth", true);
  options.append_parameter_override("camera_descent.rate_mps", 0.15);
  options.append_parameter_override("camera_descent.max_depth_m", 4.0);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  double min_vz = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&min_vz](const mavros_msgs::msg::PositionTarget::SharedPtr msg) {
      min_vz = std::min(min_vz, static_cast<double>(msg->velocity.z));
    });
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto depth_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());
  using Action = banyu_msgs::action::DiveToDepth;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/dive_to_depth");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.target_depth_m = 12.5;
  goal.depth_tolerance_m = 0.3;
  goal.heading_tolerance_rad = 0.2;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);
  std_msgs::msg::Float64 depth;
  depth.data = -1.0;
  for (int i = 0; i < 20; ++i) {
    publishPose(node, pose_pub, 0.0);
    depth_pub->publish(depth);
    spinFor(node, 2);
  }
  // The camera descent rate, not the depth P-loop toward 12.5 m.
  EXPECT_NEAR(min_vz, -0.15, 1e-3);
  EXPECT_TRUE(result_future.wait_for(0ms) != std::future_status::ready);
  depth.data = -4.2;
  const auto until = std::chrono::steady_clock::now() + 1500ms;
  while (result_future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < until)
  {
    publishPose(node, pose_pub, 0.0);
    depth_pub->publish(depth);
    spinFor(node, 1);
  }
  ASSERT_TRUE(waitForFuture(node, result_future));
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(wrapped.result->reason.find("maximum depth"), std::string::npos);
}

TEST_F(VehicleExecutionNodeTest, FollowPipelineReverseBacksUpAlongThePipe)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  double min_vx = 0.0;
  double max_vx = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&](const mavros_msgs::msg::PositionTarget::SharedPtr msg) {
      min_vx = std::min(min_vx, static_cast<double>(msg->velocity.x));
      max_vx = std::max(max_vx, static_cast<double>(msg->velocity.x));
    });
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto guidance_pub = node->create_publisher<banyu_guidance::msg::VelocityIntent>(
    "/banyu/guidance/velocity_intent", 10);
  using Action = banyu_msgs::action::FollowPipeline;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/follow_pipeline");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.enable = true;
  goal.reverse = true;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  for (int i = 0; i < 40; ++i) {
    publishPose(node, pose_pub, 0.0);   // facing east
    banyu_guidance::msg::VelocityIntent intent;
    intent.header.stamp = node->now();
    intent.status = banyu_guidance::msg::VelocityIntent::TRACKING;
    intent.surge_mps = 0.4F;
    guidance_pub->publish(intent);
    spinFor(node, 2);
  }
  EXPECT_NEAR(min_vx, -0.4, 1e-3);   // backing up (west)
  EXPECT_LE(max_vx, 1e-6);
}

namespace
{
// Drives a point at a fixed step toward keepOutAim until it reaches the goal; returns the closest approach to any
// obstacle (world frame) and whether the goal was reached.
std::pair<double, bool> driveAround(
  std::pair<double, double> start, std::pair<double, double> goal,
  const std::vector<std::pair<double, double>> & obstacles, double radius)
{
  double x = start.first;
  double y = start.second;
  double closest = 1e9;
  for (int i = 0; i < 2000; ++i) {
    if (std::hypot(goal.first - x, goal.second - y) < 0.2) {return {closest, true};}
    std::vector<std::pair<double, double>> relative;
    for (const auto & o : obstacles) {
      relative.emplace_back(o.first - x, o.second - y);
    }
    const auto [ax, ay] = banyu_vehicle_execution::keepOutAim(
      {goal.first - x, goal.second - y}, relative, radius);
    const double len = std::hypot(ax, ay);
    x += 0.05 * ax / len;
    y += 0.05 * ay / len;
    for (const auto & o : obstacles) {
      closest = std::min(closest, std::hypot(o.first - x, o.second - y));
    }
  }
  return {closest, false};
}
}  // namespace

TEST(KeepOutAim, ClearLineGoesStraightToTheGoal)
{
  const auto [ax, ay] = banyu_vehicle_execution::keepOutAim({10.0, 0.0}, {{5.0, 3.0}}, 1.2);
  EXPECT_DOUBLE_EQ(ax, 10.0);
  EXPECT_DOUBLE_EQ(ay, 0.0);
}

TEST(KeepOutAim, EndBuoyOnTheLineIsPassedOutsideTheRadius)
{
  // The end buoy sits right on the way to the goal.
  const auto [closest, reached] = driveAround({-10.0, 0.0}, {10.0, 0.0}, {{0.0, 0.0}}, 1.2);
  EXPECT_TRUE(reached);
  EXPECT_GE(closest, 1.2);
}

TEST(KeepOutAim, GoalJustInsideTheStartBuoyIsReachedFromTheFarSide)
{
  // The goal is 1.5 m inside the start buoy (the standoff) and the vehicle comes from beyond the buoy; the end buoy is
  // 8 m further. The vehicle must go around the start buoy.
  const std::vector<std::pair<double, double>> buoys = {{0.0, 0.0}, {9.5, 0.0}};
  const auto [closest, reached] = driveAround({-12.0, 0.3}, {1.5, 0.0}, buoys, 1.2);
  EXPECT_TRUE(reached);
  EXPECT_GE(closest, 1.2);
}

TEST(KeepOutAim, BothBuoysInTheWayAreAvoided)
{
  const std::vector<std::pair<double, double>> buoys = {{-3.0, 0.1}, {3.0, -0.1}};
  const auto [closest, reached] = driveAround({-12.0, 0.0}, {12.0, 0.0}, buoys, 1.2);
  EXPECT_TRUE(reached);
  EXPECT_GE(closest, 1.2);
}

TEST(KeepOutAim, InsideTheRadiusMovesStraightOut)
{
  const auto [ax, ay] = banyu_vehicle_execution::keepOutAim({10.0, 0.0}, {{0.5, 0.0}}, 1.2);
  EXPECT_LT(ax, 0.0);   // away from the buoy just ahead
  EXPECT_NEAR(ay, 0.0, 1e-9);
}

TEST_F(VehicleExecutionNodeTest, AlignHeadingHoldsItsGpsPositionWhileTurning)
{
  // The 180 deg turn at the dive point let the vehicle creep 0.4 m toward the start buoy; it now steers back.
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  double min_vx = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&min_vx](const mavros_msgs::msg::PositionTarget::SharedPtr msg) {
      min_vx = std::min(min_vx, static_cast<double>(msg->velocity.x));
    });
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto gps_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/raw/fix", rclcpp::SensorDataQoS());
  auto publish_fix = [&](double longitude) {
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = node->now();
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.latitude = 1.0;
      fix.longitude = longitude;
      gps_pub->publish(fix);
    };
  using Action = banyu_msgs::action::AlignHeading;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/align_heading");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  for (int i = 0; i < 5; ++i) {publishPose(node, pose_pub, 0.0); publish_fix(2.0); spinFor(node, 2);}
  Action::Goal goal;
  goal.target_heading_rad = M_PI;   // a full turn ahead
  goal.tolerance_rad = 0.05;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  for (int i = 0; i < 10; ++i) {publishPose(node, pose_pub, 0.0); publish_fix(2.0); spinFor(node, 2);}
  EXPECT_GT(min_vx, -1e-3);   // at the start point: no correction
  for (int i = 0; i < 20; ++i) {publishPose(node, pose_pub, 0.0); publish_fix(2.0 + 4.5e-6); spinFor(node, 2);}
  EXPECT_LT(min_vx, -0.1);    // drifted 0.5 m east: steers back west
}

TEST_F(VehicleExecutionNodeTest, ReplayTrackReplaysSticksAndTracksHeadingAndDepth)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("rc_override.gain_xy_pwm_per_mps", 300.0);
  options.append_parameter_override("rc_override.deadzone_xy_pwm", 0);
  options.append_parameter_override("replay.heading_kp", 1.0);
  options.append_parameter_override("replay.max_heading_correction_rad_s", 0.3);
  options.append_parameter_override("replay.depth_pause_m", 0.0);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::ReplayTrack;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/replay_track");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto depth_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());

  mavros_msgs::msg::PositionTarget last;
  double min_vz = 0.0;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&](const mavros_msgs::msg::PositionTarget::SharedPtr m) {
      last = *m;
      min_vz = std::min(min_vz, static_cast<double>(m->velocity.z));
    });

  Action::Goal bad;
  bad.time_s = {0.0, 1.0};
  bad.forward_pwm = {0.0};
  auto bad_future = client->async_send_goal(bad);
  ASSERT_TRUE(waitForFuture(node, bad_future));
  EXPECT_EQ(bad_future.get(), nullptr);

  // 150 PWM forward and 90 PWM right at 300 PWM per m/s, diving to 2 m over 1.5 s.
  Action::Goal goal;
  goal.track_name = "test";
  goal.time_s = {0.0, 0.75, 1.5};
  goal.forward_pwm = {150.0, 150.0, 150.0};
  goal.lateral_pwm = {90.0, 90.0, 90.0};
  goal.yaw_pwm = {0.0, 0.0, 0.0};
  goal.heading_rad = {0.0, 0.0, 0.0};
  goal.depth_m = {0.0, 1.0, 2.0};
  goal.depth_kp = 0.7;
  goal.max_depth_speed_mps = 0.4;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);
  auto result_future = client->async_get_result(handle);

  for (int i = 0; i < 20; ++i) {
    publishPose(node, pose_pub, 0.0, 0.0);
    publishDepth(node, depth_pub, 0.0);
    spinFor(node, 1);
  }
  // Heading 0 is east: forward is +x, right is -y.
  EXPECT_NEAR(last.velocity.x, 0.5, 1e-3);
  EXPECT_NEAR(last.velocity.y, -0.3, 1e-3);
  EXPECT_NEAR(min_vz, -0.4, 1e-3);

  // Vehicle yawed right by 0.2 rad off the recorded heading: correct to the left.
  for (int i = 0; i < 10; ++i) {
    publishPose(node, pose_pub, 0.0, -0.2);
    publishDepth(node, depth_pub, 0.0);
    spinFor(node, 1);
  }
  EXPECT_GT(last.yaw_rate, 0.15);
  EXPECT_LE(last.yaw_rate, 0.3 + 1e-6);

  for (int i = 0; i < 200 && result_future.wait_for(0ms) != std::future_status::ready; ++i) {
    publishPose(node, pose_pub, 0.0, 0.0);
    publishDepth(node, depth_pub, 1.0);
    spinFor(node, 1);
  }
  ASSERT_TRUE(waitForFuture(node, result_future));
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped.result->success);
  EXPECT_GE(wrapped.result->elapsed_s, 1.5);
}

TEST_F(VehicleExecutionNodeTest, ReplayTrackPausesWhileOffTheRecordedDepth)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("rc_override.gain_xy_pwm_per_mps", 300.0);
  options.append_parameter_override("rc_override.deadzone_xy_pwm", 0);
  options.append_parameter_override("replay.depth_pause_m", 0.4);
  options.append_parameter_override("replay.depth_resume_m", 0.2);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  using Action = banyu_msgs::action::ReplayTrack;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/replay_track");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto depth_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());
  mavros_msgs::msg::PositionTarget last;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&](const mavros_msgs::msg::PositionTarget::SharedPtr m) {last = *m;});

  // The teach run was at 2 m almost at once and then drove forward.
  Action::Goal goal;
  goal.track_name = "pause";
  goal.time_s = {0.0, 0.1, 3.0};
  goal.forward_pwm = {150.0, 150.0, 150.0};
  goal.lateral_pwm = {0.0, 0.0, 0.0};
  goal.yaw_pwm = {0.0, 0.0, 0.0};
  goal.heading_rad = {0.0, 0.0, 0.0};
  goal.depth_m = {2.0, 2.0, 2.0};
  goal.depth_kp = 0.7;
  goal.max_depth_speed_mps = 0.4;
  bool paused = false;
  double track_time = -1.0;
  rclcpp_action::Client<Action>::SendGoalOptions send_options;
  send_options.feedback_callback = [&](auto, const std::shared_ptr<const Action::Feedback> fb) {
      paused = fb->paused;
      track_time = fb->track_time_s;
    };
  auto send_future = client->async_send_goal(goal, send_options);
  ASSERT_TRUE(waitForFuture(node, send_future));
  auto handle = send_future.get();
  ASSERT_NE(handle, nullptr);

  // Still at the surface: the clock holds, no forward stick, full dive.
  for (int i = 0; i < 30; ++i) {
    publishPose(node, pose_pub, 0.0, 0.0);
    publishDepth(node, depth_pub, 0.0);
    spinFor(node, 1);
  }
  EXPECT_TRUE(paused);
  EXPECT_LT(track_time, 0.1);
  EXPECT_NEAR(last.velocity.x, 0.0, 1e-6);
  EXPECT_NEAR(last.velocity.z, -0.4, 1e-6);

  // At depth: resume with the recorded forward stick.
  for (int i = 0; i < 30; ++i) {
    publishPose(node, pose_pub, 0.0, 0.0);
    publishDepth(node, depth_pub, 1.95);
    spinFor(node, 1);
  }
  EXPECT_FALSE(paused);
  EXPECT_GT(track_time, 0.0);
  EXPECT_NEAR(last.velocity.x, 0.5, 1e-3);

  auto cancel_future = client->async_cancel_goal(handle);
  ASSERT_TRUE(waitForFuture(node, cancel_future));
}

TEST_F(VehicleExecutionNodeTest, RcOverrideReleasesSticksWhenNoGoalRuns)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.enable_mavros_publish", true);
  options.append_parameter_override("control.require_lease", false);
  options.append_parameter_override("control.output_mode", "rc_override");
  options.append_parameter_override("control.autonomy_mode", "ALT_HOLD");
  options.append_parameter_override("rc_override.idle_release_s", 0.3);
  options.append_parameter_override("topics.status", "/test_rc_idle/status");
  options.append_parameter_override("topics.rc_override", "/test_rc_idle/rc_override");
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);

  std::vector<mavros_msgs::msg::OverrideRCIn> sent;
  auto rc_sub = node->create_subscription<mavros_msgs::msg::OverrideRCIn>(
    "/test_rc_idle/rc_override", 50,
    [&sent](const mavros_msgs::msg::OverrideRCIn::SharedPtr msg) {sent.push_back(*msg);});
  auto status_pub = node->create_publisher<banyu_msgs::msg::VehicleStatus>(
    "/test_rc_idle/status", 10);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  banyu_msgs::msg::VehicleStatus status;
  status.fcu_connected = true;
  status.fcu_status_fresh = true;
  status.armed = true;
  status.rc_fresh = true;
  status.mode = "ALT_HOLD";
  status.operational_mode = 2;
  const auto run_for = [&](int cycles) {
      for (int i = 0; i < cycles; ++i) {
        status.header.stamp = node->now();
        status_pub->publish(status);
        publishPose(node, pose_pub, 0.0, 0.0);
        spinFor(node, 1);
      }
    };
  const auto is_neutral = [](const mavros_msgs::msg::OverrideRCIn & m) {
      return std::any_of(m.channels.begin(), m.channels.end(), [](uint16_t v) {return v == 1500;});
    };
  const auto is_release = [](const mavros_msgs::msg::OverrideRCIn & m) {
      return std::all_of(m.channels.begin(), m.channels.end(), [](uint16_t v) {
               return v == mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;});
    };

  // Armed in ALT_HOLD with autonomy selected but no mission: a pilot teaching a track
  // must not be fought with neutral sticks.
  run_for(40);
  EXPECT_TRUE(std::none_of(sent.begin(), sent.end(), is_neutral));

  // A goal owns the sticks while it runs.
  using Action = banyu_msgs::action::TimedMove;
  auto client = rclcpp_action::create_client<Action>(node, "/banyu/actions/timed_move");
  ASSERT_TRUE(client->wait_for_action_server(1s));
  Action::Goal goal;
  goal.segment_name = "leg";
  goal.duration_s = 0.4;
  goal.speed_x_mps = 0.5;
  auto send_future = client->async_send_goal(goal);
  ASSERT_TRUE(waitForFuture(node, send_future));
  ASSERT_NE(send_future.get(), nullptr);
  sent.clear();
  run_for(20);
  EXPECT_TRUE(std::any_of(sent.begin(), sent.end(), [](const auto & m) {
      return std::any_of(m.channels.begin(), m.channels.end(), [](uint16_t v) {
               return v > 1600 && v != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;});
    }));

  // After the goal: neutral briefly, then released and quiet.
  run_for(80);
  ASSERT_FALSE(sent.empty());
  EXPECT_TRUE(is_release(sent.back()));
  const auto count = sent.size();
  run_for(20);
  EXPECT_EQ(sent.size(), count);
}

TEST(LightCue, SteersTowardALitLightOnlyWhenDeepAndConfident)
{
  banyu_vehicle_execution::VehicleTelemetry t;
  t.light_cue_valid = true;
  t.light_cue_visible = true;
  t.light_cue_confidence = 0.6;
  t.light_cue_color = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
  t.light_cue_offset_x = 0.5;    // right of the image centre
  t.light_cue_offset_y = -0.5;   // above it: ahead for the bottom camera
  t.depth_valid = true;
  t.depth_m = 3.0;
  const banyu_vehicle_execution::LightCueConfig config;
  const auto cue = banyu_vehicle_execution::lightCueVelocity(t, config);
  ASSERT_TRUE(cue.has_value());
  EXPECT_GT(cue->surge, 0.0);
  EXPECT_LT(cue->sway, 0.0);   // FLU: negative sway is to the right

  auto shallow = t;
  shallow.depth_m = 0.3;   // the start buoy's indicator seen from the surface
  EXPECT_FALSE(banyu_vehicle_execution::lightCueVelocity(shallow, config).has_value());
  auto unsure = t;
  unsure.light_cue_confidence = 0.2;
  EXPECT_FALSE(banyu_vehicle_execution::lightCueVelocity(unsure, config).has_value());
  auto red = t;
  red.light_cue_color = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
  EXPECT_TRUE(banyu_vehicle_execution::lightCueVelocity(red, config).has_value());
  for (const auto colour : {banyu_msgs::msg::PipelineRingObservation::COLOR_BLUE,
      banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN})
  {
    auto other = t;
    other.light_cue_color = colour;
    EXPECT_FALSE(banyu_vehicle_execution::lightCueVelocity(other, config).has_value());
  }
  auto unseen = t;
  unseen.light_cue_visible = false;
  EXPECT_FALSE(banyu_vehicle_execution::lightCueVelocity(unseen, config).has_value());
  auto no_depth = t;
  no_depth.depth_valid = false;
  EXPECT_FALSE(banyu_vehicle_execution::lightCueVelocity(no_depth, config).has_value());
  auto disabled = config;
  disabled.enabled = false;
  EXPECT_FALSE(banyu_vehicle_execution::lightCueVelocity(t, disabled).has_value());
}

namespace
{
struct CommandedRange
{
  double min_vx{0.0}, max_vx{0.0}, min_vy{0.0}, max_vy{0.0};
};

// Runs a pipeline action for a while at the given depth (heading east, no pipe in view) with a light right of the image
// centre on the light cue topic; returns the ENU velocities the action commanded.
template<typename Action>
CommandedRange runWithLightCue(const std::string & action_name, const typename Action::Goal & goal, double depth_m)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control.require_lease", false);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>(options);
  CommandedRange range;
  auto preview_sub = node->create_subscription<mavros_msgs::msg::PositionTarget>(
    "/banyu/execution/preview_target", 10,
    [&range](const mavros_msgs::msg::PositionTarget::SharedPtr msg) {
      range.min_vx = std::min(range.min_vx, static_cast<double>(msg->velocity.x));
      range.max_vx = std::max(range.max_vx, static_cast<double>(msg->velocity.x));
      range.min_vy = std::min(range.min_vy, static_cast<double>(msg->velocity.y));
      range.max_vy = std::max(range.max_vy, static_cast<double>(msg->velocity.y));
    });
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS());
  auto depth_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS());
  auto cue_pub = node->create_publisher<banyu_msgs::msg::PipelineRingObservation>(
    "/banyu/pipeline/light_cue", rclcpp::SensorDataQoS());
  auto client = rclcpp_action::create_client<Action>(node, action_name);
  EXPECT_TRUE(client->wait_for_action_server(1s));
  auto send_future = client->async_send_goal(goal);
  EXPECT_TRUE(waitForFuture(node, send_future));
  std_msgs::msg::Float64 depth;
  depth.data = -depth_m;
  for (int i = 0; i < 40; ++i) {
    publishPose(node, pose_pub, 0.0);
    depth_pub->publish(depth);
    banyu_msgs::msg::PipelineRingObservation cue;
    cue.header.stamp = node->now();
    cue.visible = true;
    cue.color = banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN;
    cue.confidence = 0.6F;
    cue.offset_x = 0.5F;
    cue.offset_y = 0.0F;
    cue_pub->publish(cue);
    spinFor(node, 2);
  }
  return range;
}
}  // namespace

TEST_F(VehicleExecutionNodeTest, SearchPipelineSteersOverALightInsteadOfRetracing)
{
  using Action = banyu_msgs::action::SearchPipeline;
  Action::Goal goal;
  goal.search_span_rad = 10.0;
  goal.sweep_rate_rad_s = 0.2;
  const auto deep = runWithLightCue<Action>("/banyu/actions/search_pipeline", goal, 3.0);
  // Heading east: the light right of centre is to the south (negative ENU y); no backward retrace.
  EXPECT_LT(deep.min_vy, -0.05);
  EXPECT_GT(deep.min_vx, -0.05);
  // Near the surface the light is ignored (the start buoy's indicator): the search retraces as before.
  const auto shallow = runWithLightCue<Action>("/banyu/actions/search_pipeline", goal, 0.3);
  EXPECT_NEAR(shallow.min_vx, -0.2, 1e-3);
  EXPECT_GT(shallow.min_vy, -1e-6);
}

TEST_F(VehicleExecutionNodeTest, AcquirePipelineSteersOverALightInsteadOfDrivingOn)
{
  using Action = banyu_msgs::action::AcquirePipeline;
  Action::Goal goal;
  goal.use_heading_prior = false;
  const auto deep = runWithLightCue<Action>("/banyu/actions/acquire_pipeline", goal, 3.0);
  EXPECT_LT(deep.min_vy, -0.05);
  EXPECT_LT(deep.max_vx, 0.05);   // not the 0.3 m/s forward drive
  const auto shallow = runWithLightCue<Action>("/banyu/actions/acquire_pipeline", goal, 0.3);
  EXPECT_NEAR(shallow.max_vx, 0.3, 1e-3);
  EXPECT_GT(shallow.min_vy, -1e-6);
}
