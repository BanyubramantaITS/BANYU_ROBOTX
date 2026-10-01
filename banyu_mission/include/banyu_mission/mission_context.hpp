#ifndef BANYU_MISSION_MISSION_CONTEXT_HPP_
#define BANYU_MISSION_MISSION_CONTEXT_HPP_

#include <atomic>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

#include <banyu_msgs/msg/detection.hpp>
#include <banyu_msgs/msg/pinger_observation.hpp>
#include <banyu_msgs/msg/pipe_observation.hpp>
#include <banyu_msgs/msg/pipeline_survey_report.hpp>
#include <banyu_msgs/msg/resource_delivery_request.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "banyu_mission/task2/led_tracker.hpp"

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

  mutable std::mutex data_mutex;
  geometry_msgs::msg::PoseStamped current_pose;  // /mavros/local_position/pose (ROS ENU)
  mavros_msgs::msg::State state;                 // /mavros/state
  banyu_msgs::msg::Detection latest_detection;   // /banyu/detection
  bool has_detection{false};
  rclcpp::Time last_detection_received{static_cast<int64_t>(0), RCL_ROS_TIME};

  // Captured at mission start (first NavToGate tick). Values are ROS ENU.
  std::optional<geometry_msgs::msg::Point> mission_origin;
  std::optional<geometry_msgs::msg::Point> gate_target;

  // --- RobotX Task 2: pipeline survey (camera, echosounder, hydrophones; no DVL) ------------
  // Depth from the pressure sensor, m, positive down. NOT rel_alt (EKF output, garbage without
  // horizontal aiding) and NOT current_pose.z (relative to the EKF origin).
  double pressure_depth{0.0};
  bool has_pressure_depth{false};
  double yaw{0.0};                       // compass heading, ROS ENU rad (/mavros/imu/data)
  bool has_yaw{false};
  std::optional<double> altitude;        // echosounder first echo, m
  std::optional<sensor_msgs::msg::NavSatFix> fix;
  banyu_msgs::msg::PipeObservation pipe;  // newest bottom-camera observation
  uint64_t pipe_seq{0};                  // bumped per observation, so nodes act once per frame
  std::vector<banyu_msgs::msg::PingerObservation> pings;  // unread, consumed by the nodes

  std::optional<double> seabed_depth;    // m, deepest echo FindPipe saw: the pipe top is always shallower
  double depth_target{0.0};              // m, positive down; set by SetSurveyDepth, tuned by FollowPipe
  double buoy_latitude{std::numeric_limits<double>::quiet_NaN()};   // surface GPS fix at the buoy
  double buoy_longitude{std::numeric_limits<double>::quiet_NaN()};
  task2::LedTracker tracker;             // light nodes seen by the recording FollowPipe
  // ResourceDeliveryRequest COLOR_*, set by ReadFlash (resource: COLOR_ANY below Disruptive).
  std::optional<uint8_t> resource_colour, delivery_circle_colour;

  // ROS plumbing injected by the node.
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub;
  rclcpp::Publisher<banyu_msgs::msg::PipelineSurveyReport>::SharedPtr report_pub;
  rclcpp::Publisher<banyu_msgs::msg::ResourceDeliveryRequest>::SharedPtr resource_pub;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr stick_pub;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
};

inline bool detectionIsFresh(const MissionContext& ctx, const rclcpp::Time& now,
                             double timeout_s, int expected_object_id = -1) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_detection || ctx.last_detection_received.nanoseconds() == 0) {
    return false;
  }
  if (expected_object_id >= 0 && ctx.latest_detection.object_id != expected_object_id) {
    return false;
  }

  const double age_s = (now - ctx.last_detection_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

// Build a position-only setpoint. The MAVLink frame id is LOCAL_NED, but
// mavros_msgs fields are supplied in ROS ENU and MAVROS converts them before
// sending MAVLink.
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

// Build a velocity-only local setpoint. vx/vy/vz are ROS ENU m/s; MAVROS
// converts them to NED before sending MAVLink.
//
// IMPORTANT: the yaw-rate field is always sent VALID and ZERO. ArduSub keeps a
// persistent yaw-rate slew (`yaw_look_at_heading_slew`) set by the last
// commanded yaw rate, and its guided velocity run loop applies that slew in
// the LOOK_AT_HEADING branch too — so a setpoint that merely *ignores* yaw
// rate leaves the vehicle rotating toward heading 0. Explicitly commanding
// zero yaw rate clears the slew and holds heading.
inline mavros_msgs::msg::PositionTarget velocitySetpoint(double vx, double vy, double vz,
                                                         const builtin_interfaces::msg::Time& stamp) {
  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = stamp;
  msg.coordinate_frame = frame::kLocalNed;
  msg.type_mask = static_cast<uint16_t>(type_mask::kVelocityOnly & ~0x0800);
  msg.velocity.x = vx;
  msg.velocity.y = vy;
  msg.velocity.z = vz;
  msg.yaw_rate = 0.0f;
  return msg;
}

// Build a velocity-only body setpoint from physical body-NED values. Callers
// provide x=forward, y=right, z=down; mavros expects ROS base_link FLU
// components, so y and z are negated before transmission.
inline mavros_msgs::msg::PositionTarget bodyVelocitySetpoint(double vx, double vy, double vz,
                                                             const builtin_interfaces::msg::Time& stamp) {
  auto msg = velocitySetpoint(vx, -vy, -vz, stamp);
  msg.coordinate_frame = frame::kBodyNed;
  return msg;
}

// Same as bodyVelocitySetpoint but also commands a yaw rate (rad/s, positive
// = clockwise in body-NED). The ROS message field is negated (ROS base_link
// angular z is counter-clockwise positive; mavros converts to body-NED).
inline mavros_msgs::msg::PositionTarget bodyVelocityYawRateSetpoint(double vx, double vy, double vz,
                                                                    double yaw_rate,
                                                                    const builtin_interfaces::msg::Time& stamp) {
  auto msg = bodyVelocitySetpoint(vx, vy, vz, stamp);
  msg.yaw_rate = static_cast<float>(-yaw_rate);
  return msg;
}

}  // namespace banyu_mission

#endif  // BANYU_MISSION_MISSION_CONTEXT_HPP_
