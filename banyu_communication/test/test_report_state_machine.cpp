#include <gtest/gtest.h>

#include "banyu_communication/report_state_machine.hpp"

namespace banyu_communication
{
namespace
{
banyu_msgs::msg::MissionReport report()
{
  banyu_msgs::msg::MissionReport value;
  value.mission_id = "mission-5";
  value.ring_sequence = {2, 7, 4};
  value.crc32 = mission_report_crc32(value.mission_id, value.ring_sequence);
  return value;
}
}  // namespace

TEST(ReportValidationTest, AcceptsMatchingCrc)
{
  const auto value = report();
  const auto validation = validate_report(value.mission_id, value.ring_sequence, value.crc32);
  EXPECT_TRUE(validation.valid);
  EXPECT_EQ(validation.calculated_crc32, value.crc32);
}

TEST(ReportValidationTest, RejectsEmptyMissionAndBadCrc)
{
  const auto value = report();
  EXPECT_FALSE(validate_report("", value.ring_sequence, value.crc32).valid);
  EXPECT_FALSE(validate_report(value.mission_id, value.ring_sequence, value.crc32 + 1).valid);
}

TEST(ReportAckTest, RequiresMissionIdCrcAndAcknowledgedFlag)
{
  ReportRetryStateMachine machine(3);
  const auto value = report();
  machine.start(value, 3);
  machine.record_publication();

  banyu_msgs::msg::MissionReportAck ack;
  ack.mission_id = value.mission_id;
  ack.crc32 = value.crc32;
  EXPECT_EQ(machine.on_ack(ack), ReportRetryStateMachine::AckEvent::Ignored);

  ack.acknowledged = true;
  ack.crc32++;
  EXPECT_EQ(machine.on_ack(ack), ReportRetryStateMachine::AckEvent::Ignored);

  ack.crc32 = value.crc32;
  EXPECT_EQ(machine.on_ack(ack), ReportRetryStateMachine::AckEvent::Accepted);
  EXPECT_EQ(machine.state(), ReportRetryStateMachine::State::Acknowledged);
}

TEST(ReportRetryStateMachineTest, PublishesUntilAttemptLimit)
{
  ReportRetryStateMachine machine(5);
  machine.start(report(), 3);
  EXPECT_EQ(machine.max_attempts(), 4);
  EXPECT_EQ(machine.record_publication(), 1);
  EXPECT_EQ(machine.on_timer(), ReportRetryStateMachine::TimerEvent::Publish);
  EXPECT_EQ(machine.record_publication(), 2);
  EXPECT_EQ(machine.on_timer(), ReportRetryStateMachine::TimerEvent::Publish);
  EXPECT_EQ(machine.record_publication(), 3);
  EXPECT_EQ(machine.on_timer(), ReportRetryStateMachine::TimerEvent::Publish);
  EXPECT_EQ(machine.record_publication(), 4);
  EXPECT_EQ(machine.on_timer(), ReportRetryStateMachine::TimerEvent::Exhausted);
  EXPECT_EQ(machine.state(), ReportRetryStateMachine::State::Exhausted);
}

TEST(ReportRetryStateMachineTest, ConfiguredMaximumCapsGoalRetries)
{
  ReportRetryStateMachine machine(2);
  machine.start(report(), 5);
  EXPECT_EQ(machine.max_attempts(), 2);
  machine.record_publication();
  EXPECT_EQ(machine.on_timer(), ReportRetryStateMachine::TimerEvent::Publish);
  machine.record_publication();
  EXPECT_EQ(machine.on_timer(), ReportRetryStateMachine::TimerEvent::Exhausted);
}

TEST(ReportRetryStateMachineTest, CancellationWinsAgainstAckAndTimer)
{
  ReportRetryStateMachine machine(4);
  const auto value = report();
  machine.start(value, 3);
  machine.record_publication();

  machine.cancel();

  banyu_msgs::msg::MissionReportAck ack;
  ack.mission_id = value.mission_id;
  ack.acknowledged = true;
  ack.crc32 = value.crc32;
  EXPECT_EQ(machine.state(), ReportRetryStateMachine::State::Canceled);
  EXPECT_EQ(machine.on_ack(ack), ReportRetryStateMachine::AckEvent::Ignored);
  EXPECT_EQ(machine.on_timer(), ReportRetryStateMachine::TimerEvent::None);
}

}  // namespace banyu_communication
