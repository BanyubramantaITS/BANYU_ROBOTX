#include "banyu_detection/run_context_gate.hpp"

#include <gtest/gtest.h>

TEST(RunContextGate, RequiresAcceptedContextWhenEnabled)
{
  banyu_detection::RunContextGate gate;
  EXPECT_FALSE(gate.acceptsStamp(100, true));
  EXPECT_TRUE(gate.acceptsStamp(100, false));
  EXPECT_FALSE(gate.accept(0, 1, 100).success);
  EXPECT_FALSE(gate.accept(2, 0, 100).success);
  EXPECT_FALSE(gate.initialized());
}

TEST(RunContextGate, DuplicateAndRejectedContextsPreserveObservationBoundary)
{
  banyu_detection::RunContextGate gate;
  const auto accepted = gate.accept(20, 7, 100);
  ASSERT_TRUE(accepted.success);
  EXPECT_TRUE(accepted.changed);
  const auto duplicate = gate.accept(20, 7, 500);
  EXPECT_TRUE(duplicate.success);
  EXPECT_FALSE(duplicate.changed);
  EXPECT_FALSE(gate.accept(20, 6, 500).success);
  EXPECT_FALSE(gate.accept(21, 7, 500).success);
  EXPECT_FALSE(gate.accept(0, 8, 500).success);
  EXPECT_EQ(gate.runId(), 20U);
  EXPECT_EQ(gate.epoch(), 7U);
  EXPECT_TRUE(gate.acceptsStamp(101, true));
  EXPECT_FALSE(gate.acceptsStamp(99, true));
  EXPECT_FALSE(gate.acceptsStamp(0, true));
}

TEST(RunContextGate, NewEpochRejectsPreviousFramesWithoutOrderingRunIds)
{
  banyu_detection::RunContextGate gate;
  ASSERT_TRUE(gate.accept(20, 7, 100).success);
  const auto next = gate.accept(3, 8, 500);
  ASSERT_TRUE(next.success);
  EXPECT_TRUE(next.changed);
  EXPECT_EQ(gate.runId(), 3U);
  EXPECT_FALSE(gate.acceptsStamp(499, true));
  EXPECT_TRUE(gate.acceptsStamp(500, true));
}
