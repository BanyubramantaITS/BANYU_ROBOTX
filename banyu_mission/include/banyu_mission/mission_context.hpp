#ifndef BANYU_MISSION_MISSION_CONTEXT_HPP_
#define BANYU_MISSION_MISSION_CONTEXT_HPP_

#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <banyu_msgs/msg/detection.hpp>
#include <banyu_msgs/msg/hydrophone_heading.hpp>
#include <banyu_msgs/msg/pipeline_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_state.hpp>
#include <banyu_msgs/msg/task2_report.hpp>
#include <banyu_msgs/msg/task2_ring_record.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

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
  bool has_pose{false};
  rclcpp::Time last_pose_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  mavros_msgs::msg::State state;                 // /mavros/state
  bool has_state{false};
  rclcpp::Time last_state_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  banyu_msgs::msg::Detection latest_detection;   // /banyu/detection
  bool has_detection{false};
  rclcpp::Time last_detection_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  banyu_msgs::msg::HydrophoneHeading latest_hydrophone_heading;
  bool has_hydrophone_heading{false};
  rclcpp::Time last_hydrophone_heading_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  banyu_msgs::msg::PipelineRingState latest_pipeline_ring_state;
  bool has_pipeline_ring_state{false};
  rclcpp::Time last_pipeline_ring_state_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  std::map<uint32_t, banyu_msgs::msg::PipelineRingState> pipeline_ring_states;
  std::map<uint32_t, rclcpp::Time> pipeline_ring_state_receipt_times;
  geometry_msgs::msg::PoseStamped active_buoy_pose;
  bool has_active_buoy_pose{false};
  rclcpp::Time last_active_buoy_pose_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  nav_msgs::msg::Path latest_pipeline_route;
  bool has_pipeline_route{false};
  rclcpp::Time last_pipeline_route_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  bool dvl_live{false};
  bool has_dvl_live{false};
  rclcpp::Time last_dvl_live_received{static_cast<int64_t>(0), RCL_ROS_TIME};

  // Latched Task 2 surface anchor and local-map homing target.
  sensor_msgs::msg::NavSatFix latest_task2_buoy_fix;
  sensor_msgs::msg::NavSatFix latest_task2_auv_fix;
  sensor_msgs::msg::NavSatFix task2_anchor_buoy_fix;
  sensor_msgs::msg::NavSatFix task2_anchor_auv_fix;
  geometry_msgs::msg::PoseStamped task2_anchor_map_pose;
  geometry_msgs::msg::Point task2_target_map_pose;
  bool has_task2_surface_anchor{false};
  rclcpp::Time last_task2_buoy_fix_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  rclcpp::Time last_task2_auv_fix_received{static_cast<int64_t>(0), RCL_ROS_TIME};

  struct PoseHistorySample {
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time receipt_time{static_cast<int64_t>(0), RCL_ROS_TIME};
  };

  struct CameraRingFrame {
    banyu_msgs::msg::PipelineRingObservation observation;
    rclcpp::Time receipt_time{static_cast<int64_t>(0), RCL_ROS_TIME};
    bool contract_valid{false};
  };

  struct Task2CameraRunState {
    bool active{false};
    bool acquired{false};
    bool tracking_started{false};
    bool input_failure{false};
    std::string failure_reason;
    bool baseline_set{false};
    uint64_t baseline_sequence{0};
    rclcpp::Time baseline_source_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
    bool ring_baseline_set{false};
    uint64_t ring_baseline_sequence{0};
    rclcpp::Time ring_baseline_source_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
    bool ring_heartbeat_seen{false};
    uint64_t last_heartbeat_sequence{0};
    rclcpp::Time last_heartbeat_source_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
    uint64_t consumed_pipeline_sequence{0};
    rclcpp::Time consumed_pipeline_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
    std::size_t acquire_tracked_frames{0};
    std::size_t no_pipe_frames{0};
    bool loss_candidate{false};
    rclcpp::Time loss_start_source_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
    bool ring_heartbeat_loss_candidate{false};
    rclcpp::Time ring_heartbeat_loss_start{static_cast<int64_t>(0), RCL_ROS_TIME};
    bool pairing_gap_candidate{false};
    rclcpp::Time pairing_gap_start{static_cast<int64_t>(0), RCL_ROS_TIME};
    bool path_started{false};
    double path_length_m{0.0};
    double path_length_at_last_ring_m{0.0};
    geometry_msgs::msg::PoseStamped previous_observed_pose{};
    bool has_previous_observed_pose{false};
    geometry_msgs::msg::PoseStamped last_stored_pose{};
    bool has_last_stored_pose{false};
    std::vector<geometry_msgs::msg::PoseStamped> path_samples;
    std::vector<banyu_msgs::msg::Task2RingRecord> ring_records;
    bool red_ring_recorded{false};
    std::size_t red_ring_count{0};
    std::size_t ring_invisible_frames{0};
    bool ring_armed{true};
    uint8_t pending_ring_color{banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN};
    std::size_t pending_ring_frames{0};
    uint64_t pending_ring_sequence{0};
    geometry_msgs::msg::PoseStamped last_ring_pose{};
    bool has_last_ring_pose{false};
    bool reverse_target_selected{false};
    std::size_t reverse_target_record_index{0};
    geometry_msgs::msg::PoseStamped reverse_target_pose{};
    bool reverse_reacquired{false};
  };

  // Camera detector caches are always updated, but their failures are latched
  // only while a camera Task 2 run is active.
  banyu_msgs::msg::PipelineObservation latest_pipeline_observation;
  bool has_pipeline_observation{false};
  rclcpp::Time last_pipeline_observation_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  bool pipeline_observation_contract_valid{false};
  uint64_t last_pipeline_observation_sequence{0};
  rclcpp::Time last_pipeline_observation_source_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
  bool pipeline_observation_sequence_seen{false};

  std::map<uint64_t, CameraRingFrame> pipeline_ring_observation_cache;
  banyu_msgs::msg::PipelineRingObservation latest_pipeline_ring_observation;
  bool has_pipeline_ring_observation{false};
  bool pipeline_ring_observation_contract_valid{false};
  rclcpp::Time last_pipeline_ring_observation_received{static_cast<int64_t>(0), RCL_ROS_TIME};
  uint64_t last_pipeline_ring_observation_sequence{0};
  rclcpp::Time last_pipeline_ring_observation_source_stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
  bool pipeline_ring_observation_sequence_seen{false};

  std::deque<PoseHistorySample> map_pose_history;
  Task2CameraRunState task2_camera_run;
  bool task2_camera_source_selected{false};

  // Captured at mission start (first NavToGate tick). Values are ROS ENU.
  std::optional<geometry_msgs::msg::Point> mission_origin;
  std::optional<geometry_msgs::msg::Point> gate_target;

  // ROS plumbing injected by the node.
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robocommand_pub;
  rclcpp::Publisher<banyu_msgs::msg::Task2Report>::SharedPtr task2_report_pub;
  unique_identifier_msgs::msg::UUID task2_run_id;
  bool task2_run_id_valid{false};
  std::mutex task2_terminal_mutex;
  std::atomic<uint8_t> task2_terminal_claim{banyu_msgs::msg::Task2Report::STATUS_UNKNOWN};
  bool task2_report_published{false};
  uint8_t task2_repair_status{banyu_msgs::msg::Task2Report::NOT_ATTEMPTED};
  double task2_repair_hold_duration_s{0.0};
  bool task2_repair_started{false};
  geometry_msgs::msg::PoseStamped task2_final_surface_pose{};
  bool has_task2_final_surface_pose{false};
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

inline bool poseIsFresh(const MissionContext& ctx, const rclcpp::Time& now, double timeout_s) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_pose || ctx.last_pose_received.nanoseconds() == 0) {
    return false;
  }

  const double age_s = (now - ctx.last_pose_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

inline bool stateIsFresh(const MissionContext& ctx, const rclcpp::Time& now, double timeout_s) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_state || ctx.last_state_received.nanoseconds() == 0) {
    return false;
  }

  const double age_s = (now - ctx.last_state_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

inline bool hydrophoneHeadingIsFresh(const MissionContext& ctx, const rclcpp::Time& now,
                                     double timeout_s) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_hydrophone_heading || ctx.last_hydrophone_heading_received.nanoseconds() == 0 ||
      !ctx.latest_hydrophone_heading.valid) {
    return false;
  }

  const double age_s = (now - ctx.last_hydrophone_heading_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

inline bool pipelineRingStateIsFresh(const MissionContext& ctx, const rclcpp::Time& now,
                                     double timeout_s) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_pipeline_ring_state || ctx.last_pipeline_ring_state_received.nanoseconds() == 0) {
    return false;
  }

  const double age_s = (now - ctx.last_pipeline_ring_state_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

inline bool pipelineRingStateIsFresh(const MissionContext& ctx, const rclcpp::Time& now,
                                     double timeout_s, uint32_t index) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  const auto state_it = ctx.pipeline_ring_states.find(index);
  const auto receipt_it = ctx.pipeline_ring_state_receipt_times.find(index);
  if (!ctx.has_pipeline_ring_state || state_it == ctx.pipeline_ring_states.end() ||
      receipt_it == ctx.pipeline_ring_state_receipt_times.end() ||
      receipt_it->second.nanoseconds() == 0) {
    return false;
  }

  const double age_s = (now - receipt_it->second).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

inline bool activeBuoyPoseIsFresh(const MissionContext& ctx, const rclcpp::Time& now,
                                  double timeout_s) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_active_buoy_pose || ctx.last_active_buoy_pose_received.nanoseconds() == 0) {
    return false;
  }

  const double age_s = (now - ctx.last_active_buoy_pose_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

inline bool pipelineRouteIsFresh(const MissionContext& ctx, const rclcpp::Time& now,
                                 double timeout_s) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_pipeline_route || ctx.last_pipeline_route_received.nanoseconds() == 0) {
    return false;
  }

  const double age_s = (now - ctx.last_pipeline_route_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

inline bool dvlLiveIsFresh(const MissionContext& ctx, const rclcpp::Time& now,
                           double timeout_s) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (!ctx.has_dvl_live || !ctx.dvl_live || ctx.last_dvl_live_received.nanoseconds() == 0) {
    return false;
  }

  const double age_s = (now - ctx.last_dvl_live_received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

// Caller must already hold ctx.data_mutex.
inline bool dvlLiveIsFreshLocked(const MissionContext& ctx, const rclcpp::Time& now,
                                 double timeout_s) {
  if (!ctx.has_dvl_live || !ctx.dvl_live || ctx.last_dvl_live_received.nanoseconds() == 0) {
    return false;
  }
  const double age_s = (now - ctx.last_dvl_live_received).seconds();
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
