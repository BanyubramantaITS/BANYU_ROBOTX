#include <gtest/gtest.h>

#include "banyu_detection/dead_reckoner.hpp"

using banyu_detection::DeadReckoner;

TEST(DeadReckoner, IntegratesTheCommandFromTheLastFix)
{
  DeadReckoner r;
  EXPECT_FALSE(r.valid());
  r.reset(2.0, -1.0, 10.0);
  r.setCommand(0.2, 0.0, 10.0);
  for (int i = 1; i <= 50; ++i) {r.setCommand(0.2, 0.0, 10.0 + 0.1 * i);}   // 5 s at 0.2 m/s east, 10 Hz commands
  EXPECT_NEAR(r.x(), 3.0, 1e-9);
  EXPECT_NEAR(r.y(), -1.0, 1e-9);
}

TEST(DeadReckoner, StopsWhenTheCommandIsStale)
{
  DeadReckoner r(1.0, 0.5);
  r.reset(0.0, 0.0, 0.0);
  r.setCommand(0.0, 0.3, 0.0);
  for (int i = 1; i <= 30; ++i) {r.advance(0.1 * i);}
  EXPECT_NEAR(r.y(), 0.15, 0.031);   // only the first 0.5 s count
}

TEST(DeadReckoner, GainScalesTheCommandedSpeed)
{
  DeadReckoner r(0.5);
  r.reset(0.0, 0.0, 0.0);
  r.setCommand(1.0, 0.0, 0.0);
  r.setCommand(1.0, 0.0, 0.4);
  EXPECT_NEAR(r.x(), 0.2, 1e-9);
}
