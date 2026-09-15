#ifndef BANYU_MISSION_BT_NODES_TASK2_CAMERA_COMMON_HPP_
#define BANYU_MISSION_BT_NODES_TASK2_CAMERA_COMMON_HPP_

#include <cmath>
#include <limits>

#include <builtin_interfaces/msg/time.hpp>

#include "banyu_mission/bt_nodes/task2_common.hpp"

namespace banyu_mission::bt_nodes::task2_camera {

struct PipelineFrame {
  banyu_msgs::msg::PipelineObservation observation;
  rclcpp::Time source_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
  rclcpp::Time receipt_time{static_cast<int64_t>(0), RCL_ROS_TIME};
};

inline bool freshPipelineLocked(const MissionContext& ctx, const rclcpp::Time& now,
                                double source_timeout_s, double receipt_timeout_s,
                                double future_tolerance_s, PipelineFrame& frame) {
  if (!ctx.has_pipeline_observation || !ctx.pipeline_observation_contract_valid) {
    return false;
  }
  const rclcpp::Time source_stamp(ctx.latest_pipeline_observation.header.stamp, RCL_ROS_TIME);
  const double source_age = (now - source_stamp).seconds();
  const double receipt_age = (now - ctx.last_pipeline_observation_received).seconds();
  if (source_age < -future_tolerance_s || source_age > source_timeout_s || receipt_age < 0.0 ||
      receipt_age > receipt_timeout_s) {
    return false;
  }
  frame.observation = ctx.latest_pipeline_observation;
  frame.source_stamp = source_stamp;
  frame.receipt_time = ctx.last_pipeline_observation_received;
  return true;
}

inline bool sourceNewerThanBaselineLocked(const MissionContext& ctx, const PipelineFrame& frame) {
  return !ctx.task2_camera_run.baseline_set ||
         (frame.observation.frame_sequence > ctx.task2_camera_run.baseline_sequence &&
          frame.source_stamp.nanoseconds() >
              ctx.task2_camera_run.baseline_source_stamp.nanoseconds());
}

inline bool freshRingHeartbeatLocked(const MissionContext& ctx, const rclcpp::Time& now,
                                     double source_timeout_s, double receipt_timeout_s,
                                     double future_tolerance_s) {
  if (!ctx.has_pipeline_ring_observation || !ctx.pipeline_ring_observation_contract_valid) {
    return false;
  }
  const rclcpp::Time source_stamp(ctx.latest_pipeline_ring_observation.header.stamp,
                                  RCL_ROS_TIME);
  const double source_age = (now - source_stamp).seconds();
  const double receipt_age = (now - ctx.last_pipeline_ring_observation_received).seconds();
  return source_age >= -future_tolerance_s && source_age <= source_timeout_s &&
         receipt_age >= 0.0 && receipt_age <= receipt_timeout_s;
}

inline bool ringHeartbeatNewerThanBaselineLocked(const MissionContext& ctx) {
  return !ctx.task2_camera_run.ring_baseline_set ||
         (ctx.last_pipeline_ring_observation_sequence >
              ctx.task2_camera_run.ring_baseline_sequence &&
          ctx.last_pipeline_ring_observation_source_stamp.nanoseconds() >
              ctx.task2_camera_run.ring_baseline_source_stamp.nanoseconds());
}

inline bool associatePoseLocked(const MissionContext& ctx, const rclcpp::Time& source_stamp,
                                double max_skew_s, geometry_msgs::msg::PoseStamped& pose) {
  double best_skew = std::numeric_limits<double>::max();
  bool found = false;
  for (const auto& sample : ctx.map_pose_history) {
    const double skew = std::abs((sample.pose.header.stamp.sec +
                                  sample.pose.header.stamp.nanosec * 1.0e-9) -
                                 source_stamp.seconds());
    if (skew <= max_skew_s && skew < best_skew && task2_common::finitePose(sample.pose)) {
      best_skew = skew;
      pose = sample.pose;
      found = true;
    }
  }
  if (found) {
    pose.header.stamp = static_cast<builtin_interfaces::msg::Time>(source_stamp);
    pose.header.frame_id = "map";
  }
  return found;
}

inline double poseDistance(const geometry_msgs::msg::PoseStamped& first,
                           const geometry_msgs::msg::PoseStamped& second) {
  const double dx = first.pose.position.x - second.pose.position.x;
  const double dy = first.pose.position.y - second.pose.position.y;
  const double dz = first.pose.position.z - second.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline void publishZeroBody(const std::shared_ptr<MissionContext>& ctx,
                            const rclcpp::Time& stamp) {
  if (ctx->setpoint_pub) {
    ctx->setpoint_pub->publish(bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, 0.0, stamp));
  }
}

}  // namespace banyu_mission::bt_nodes::task2_camera

#endif  // BANYU_MISSION_BT_NODES_TASK2_CAMERA_COMMON_HPP_
