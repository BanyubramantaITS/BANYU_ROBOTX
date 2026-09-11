#ifndef BANYU_MISSION_BT_NODES_COMMON_HPP_
#define BANYU_MISSION_BT_NODES_COMMON_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission::bt_nodes::common {

double norm2(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
double wrapToPi(double angle);

void publishZeroVelocity(const std::shared_ptr<MissionContext>& ctx, const rclcpp::Time& stamp);
void publishZeroBodyVelocity(const std::shared_ptr<MissionContext>& ctx,
                             const rclcpp::Time& stamp);

}  // namespace banyu_mission::bt_nodes::common

#endif  // BANYU_MISSION_BT_NODES_COMMON_HPP_
