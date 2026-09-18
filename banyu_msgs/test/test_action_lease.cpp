#include <gtest/gtest.h>

#include "banyu_msgs/msg/action_lease.hpp"

TEST(ActionLease, CarriesHeartbeatContract)
{
  banyu_msgs::msg::ActionLease lease;
  lease.header.frame_id = "executor";
  lease.goal_id.uuid[0] = 1;
  lease.owner = "client";
  lease.active = true;
  lease.expires_at.sec = 12;
  lease.expires_at.nanosec = 34;
  lease.generation = 56;

  EXPECT_EQ(lease.header.frame_id, "executor");
  EXPECT_EQ(lease.goal_id.uuid[0], 1);
  EXPECT_EQ(lease.owner, "client");
  EXPECT_TRUE(lease.active);
  EXPECT_EQ(lease.expires_at.sec, 12);
  EXPECT_EQ(lease.expires_at.nanosec, 34u);
  EXPECT_EQ(lease.generation, 56u);
}
