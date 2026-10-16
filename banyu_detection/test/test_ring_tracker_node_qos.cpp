#include "banyu_detection/ring_tracker_node.hpp"

#include <gtest/gtest.h>

#include <rmw/qos_profiles.h>

TEST(RingTrackerNodeQos, RingStateRetainsReliableLastValue)
{
  const auto profile = banyu_detection::ringStateQoS().get_rmw_qos_profile();
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  EXPECT_EQ(profile.depth, 1U);
}

TEST(RingTrackerNodeQos, AcceptedRecordsRetainReliableHistory)
{
  const auto profile = banyu_detection::acceptedRingQoS().get_rmw_qos_profile();
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  EXPECT_GE(profile.depth, 4U);
}
