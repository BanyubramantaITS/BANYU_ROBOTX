#ifndef BANYU_MISSION_MISSION_CONTEXT_HPP_
#define BANYU_MISSION_MISSION_CONTEXT_HPP_

#include <atomic>
#include <mutex>
#include <optional>

#include <banyu_msgs/msg/detection.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>

namespace banyu_mission {

// MAVLink frame ids as used by mavros_msgs/PositionTarget.coordinate_frame.
namespace frame {
constexpr uint8_t kLocalNed = 1;
constexpr uint8_t kLocalOffsetNed = 7;
constexpr uint8_t kBodyNed = 8;
constexpr uint8_t kBodyOffsetNed = 9;
}  // namespace frame

// SET_POSITION_TARGET_LOCAL_NED type masks (bits set = field ignored).
namespace type_mask {
// position valid only: ignore vel, acc, force, yaw, yaw_rate
constexpr uint16_t kPositionOnly = 0x0DF8;
// velocity valid only: ignore pos, acc, force, yaw, yaw_rate
constexpr uint16_t kVelocityOnly = 0x0DC7;
}  // namespace type_mask

// Shared state between the ROS node and the BehaviorTree.CPP nodes.
// Guarded by data_mutex where applicable; mission/abort flags are atomic so
// the reactive BT control can observe them on every tick.
struct MissionContext {
  std::atomic<bool> mission_active{false};
  std::atomic<bool> aborted{false};

  std::mutex data_mutex;
  geometry_msgs::msg::PoseStamped current_pose;  // /mavros/local_position/pose (NED)
  mavros_msgs::msg::State state;                 // /mavros/state
  banyu_msgs::msg::Detection latest_detection;   // /banyu/detection
  bool has_detection{false};

  // Captured at mission start (first NavToGate tick). All values NED.
  std::optional<geometry_msgs::msg::Point> mission_origin;
  std::optional<geometry_msgs::msg::Point> gate_target;

  // ROS plumbing injected by the node.
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub;
};

// Build a position-only setpoint (FRAME_LOCAL_NED) toward a target.
inline mavros_msgs::msg::PositionTarget positionSetpoint(const geometry_msgs::msg::Point& target,
                                                         const builtin_interfaces::msg::Time& stamp) {
  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = stamp;
  msg.coordinate_frame = frame::kLocalNed;
  msg.type_mask = type_mask::kPositionOnly;
  msg.position.x = target.x;
  msg.position.y = target.y;
  msg.position.z = target.z;
  return msg;
}

// Build a velocity-only setpoint (FRAME_LOCAL_NED). Velocity in NED m/s.
inline mavros_msgs::msg::PositionTarget velocitySetpoint(double vx, double vy, double vz,
                                                         const builtin_interfaces::msg::Time& stamp) {
  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = stamp;
  msg.coordinate_frame = frame::kLocalNed;
  msg.type_mask = type_mask::kVelocityOnly;
  msg.velocity.x = vx;
  msg.velocity.y = vy;
  msg.velocity.z = vz;
  return msg;
}

}  // namespace banyu_mission

#endif  // BANYU_MISSION_MISSION_CONTEXT_HPP_
