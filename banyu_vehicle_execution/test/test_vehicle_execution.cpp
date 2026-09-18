#include "banyu_vehicle_execution/command_encoder.hpp"
#include "banyu_vehicle_execution/goal_arbiter.hpp"
#include "banyu_vehicle_execution/motion_intent.hpp"
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
