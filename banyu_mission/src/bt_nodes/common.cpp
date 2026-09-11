#include "banyu_mission/bt_nodes/common.hpp"

#include <cmath>

namespace banyu_mission::bt_nodes::common {

double norm2(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  // ROS ENU convention: yaw about the z axis.
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapToPi(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void publishZeroVelocity(const std::shared_ptr<MissionContext>& ctx, const rclcpp::Time& stamp) {
  if (ctx->setpoint_pub) {
    ctx->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, stamp));
  }
}

void publishZeroBodyVelocity(const std::shared_ptr<MissionContext>& ctx,
                             const rclcpp::Time& stamp) {
  if (ctx->setpoint_pub) {
    ctx->setpoint_pub->publish(bodyVelocitySetpoint(0.0, 0.0, 0.0, stamp));
  }
}

}  // namespace banyu_mission::bt_nodes::common
