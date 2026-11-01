#include <gtest/gtest.h>

#include <optional>

#include "banyu_vehicle_execution/depth_health_monitor.hpp"

using banyu_vehicle_execution::DepthHealthMonitor;
using Fault = DepthHealthMonitor::Fault;

namespace
{
// Replays a 10 Hz dive at 0.2 m/s (commanded and real) for `seconds`, with `fault_at` onwards handled by `sample`.
// Returns the time the first fault was flagged, or -1.
template<typename SampleFn>
double replay(DepthHealthMonitor & m, double seconds, SampleFn sample)
{
  for (int i = 0; i <= static_cast<int>(seconds * 10); ++i) {
    const double t = i * 0.1;
    m.setCommandedHeave(-0.2);
    const auto depth = sample(t);
    if (depth) {m.addSample(*depth, t);} else {m.check(t);}
    if (!m.healthy()) {return t;}
  }
  return -1.0;
}
}  // namespace

TEST(DepthHealthMonitor, HealthyDiveIsNotFlagged)
{
  DepthHealthMonitor m;
  EXPECT_LT(replay(m, 30.0, [](double t) {return std::optional<double>(0.2 * t);}), 0.0);
}

TEST(DepthHealthMonitor, StuckWhileDivingIsFlaggedWithinThreeSeconds)
{
  DepthHealthMonitor m;
  const double t = replay(m, 30.0, [](double t) {return std::optional<double>(t < 10.0 ? 0.2 * t : 2.0);});
  EXPECT_EQ(m.fault(), Fault::kStuck);
  EXPECT_GT(t, 10.0);
  EXPECT_LE(t - 10.0, 3.5);
}

TEST(DepthHealthMonitor, SilentSensorIsFlaggedWithinThreeSeconds)
{
  DepthHealthMonitor m;
  const double t = replay(m, 30.0, [](double t) {return t < 10.0 ? std::optional<double>(0.2 * t) : std::nullopt;});
  EXPECT_EQ(m.fault(), Fault::kSilent);
  EXPECT_LE(t - 10.0, 3.5);
}

TEST(DepthHealthMonitor, StepIsFlaggedAtOnce)
{
  DepthHealthMonitor m;
  const double t = replay(m, 30.0, [](double t) {return std::optional<double>(0.2 * t + (t >= 10.0 ? 1.0 : 0.0));});
  EXPECT_EQ(m.fault(), Fault::kStep);
  EXPECT_NEAR(t, 10.0, 0.15);
}

TEST(DepthHealthMonitor, SlowOffsetAgainstTheHeaveIsFlagged)
{
  // A drift of 0.25 m/s on top of the real depth (an offset that grows) is not a step, but disagrees with the heave.
  DepthHealthMonitor m;
  const double t = replay(m, 30.0, [](double t) {return std::optional<double>(0.2 * t + (t >= 10.0 ? 0.25 * (t - 10.0) : 0.0));});
  EXPECT_EQ(m.fault(), Fault::kDisagree);
  EXPECT_LE(t - 10.0, 10.0);
}

TEST(DepthHealthMonitor, StillAtDepthWithNoHeaveIsHealthy)
{
  DepthHealthMonitor m;
  for (int i = 0; i < 300; ++i) {
    m.setCommandedHeave(0.0);
    m.addSample(5.0, i * 0.1);
  }
  EXPECT_TRUE(m.healthy());
}

TEST(DepthHealthMonitor, FaultIsLatchedUntilReset)
{
  DepthHealthMonitor m;
  m.addSample(1.0, 0.0);
  m.addSample(2.0, 0.1);
  EXPECT_EQ(m.fault(), Fault::kStep);
  m.addSample(2.0, 0.2);
  EXPECT_FALSE(m.healthy());
  m.reset();
  EXPECT_TRUE(m.healthy());
}
