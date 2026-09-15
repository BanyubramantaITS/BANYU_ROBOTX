#include "banyu_mission/bt_nodes/reverse_to_red_ring.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission {

ReverseToRedRing::ReverseToRedRing(const std::string& name, const BT::NodeConfig& config,
                                   const rclcpp::Node::SharedPtr& node,
                                   std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus ReverseToRedRing::onStart() {
  start_time_ = node_->now();
  reacquire_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  pairing_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  reverse_waypoints_.clear();
  waypoint_index_ = 0;
  reacquire_frames_ = 0;
  last_reacquire_sequence_ = 0;
  last_reacquire_source_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_reacquire_ring_sequence_ = 0;
  last_reacquire_ring_source_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  has_reacquire_baseline_ = false;
  pairing_candidate_ = false;
  target_reached_ = false;
  has_previous_pose_ = false;
  bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);

  std::string failure;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->task2_camera_run.reverse_target_selected = false;
    ctx_->task2_camera_run.reverse_reacquired = false;
    if (ctx_->task2_camera_run.input_failure) {
      failure = ctx_->task2_camera_run.failure_reason;
    } else if (!ctx_->task2_camera_source_selected || !ctx_->task2_camera_run.active ||
               !ctx_->task2_camera_run.acquired) {
      failure = "camera acquisition was not completed";
    } else if (!validateAndBuildReversePathLocked()) {
      failure = ctx_->task2_camera_run.failure_reason.empty()
                    ? "unable to select a unique red ring reverse target"
                    : ctx_->task2_camera_run.failure_reason;
    }
  }
  if (!failure.empty()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);
    RCLCPP_ERROR(node_->get_logger(), "ReverseToRedRing: %s", failure.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReverseToRedRing::onRunning() {
  const rclcpp::Time now = node_->now();
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    return BT::NodeStatus::FAILURE;
  }
  if ((now - start_time_).seconds() >=
      node_->get_parameter("task2_camera_reverse_timeout_s").as_double()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "ReverseToRedRing: reverse timeout");
    return BT::NodeStatus::FAILURE;
  }

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  bool command_velocity = false;
  bool command_zero = false;
  bool complete = false;
  std::string failure;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    auto& run = ctx_->task2_camera_run;
    if (run.input_failure) {
      failure = run.failure_reason;
    } else if (!run.reverse_target_selected || reverse_waypoints_.empty()) {
      failure = "reverse red-ring target is unavailable";
    } else {
      geometry_msgs::msg::PoseStamped pose;
      mavros_msgs::msg::State state;
      const bool ready =
          bt_nodes::task2_common::dvlReadyLocked(
              *ctx_, now, node_->get_parameter("task2_use_dvl").as_bool(),
              node_->get_parameter("task2_dvl_timeout_s").as_double()) &&
          bt_nodes::task2_common::vehicleReady(
              *ctx_, now, node_->get_parameter("task2_pose_timeout_s").as_double(),
              node_->get_parameter("task2_state_timeout_s").as_double(), pose, state);
      if (!ready) {
        failure = "vehicle readiness lost during red-ring reverse";
      } else if (pose.header.frame_id != "map" || !bt_nodes::task2_common::finitePose(pose)) {
        failure = "reverse pose is invalid or not in map frame";
      } else {
        if (has_previous_pose_ &&
            bt_nodes::task2_camera::poseDistance(previous_pose_, pose) >
                node_->get_parameter("task2_camera_reverse_max_pose_jump_m").as_double()) {
          failure = "reverse pose jump exceeded the safety limit";
        }
        if (!has_previous_pose_ &&
            bt_nodes::task2_camera::poseDistance(pose, reverse_waypoints_.front()) >
                node_->get_parameter("task2_camera_reverse_max_start_error_m").as_double()) {
          failure = "vehicle is not near the recorded reverse-path endpoint";
        }
        previous_pose_ = pose;
        has_previous_pose_ = true;
      }

      if (failure.empty() && !target_reached_) {
        while (waypoint_index_ < reverse_waypoints_.size() &&
               poseAtWaypoint(pose, reverse_waypoints_[waypoint_index_])) {
          if (waypoint_index_ + 1 == reverse_waypoints_.size()) {
            target_reached_ = true;
            reacquire_start_time_ = now;
            reacquire_frames_ = 0;
            has_reacquire_baseline_ = true;
            last_reacquire_sequence_ =
                ctx_->has_pipeline_observation && ctx_->pipeline_observation_sequence_seen
                    ? ctx_->last_pipeline_observation_sequence
                    : 0;
            last_reacquire_source_stamp_ =
                ctx_->has_pipeline_observation && ctx_->pipeline_observation_sequence_seen
                    ? ctx_->last_pipeline_observation_source_stamp
                    : rclcpp::Time(0, 0, RCL_ROS_TIME);
            last_reacquire_ring_sequence_ =
                ctx_->has_pipeline_ring_observation &&
                        ctx_->pipeline_ring_observation_sequence_seen
                    ? ctx_->last_pipeline_ring_observation_sequence
                    : 0;
            last_reacquire_ring_source_stamp_ =
                ctx_->has_pipeline_ring_observation &&
                        ctx_->pipeline_ring_observation_sequence_seen
                    ? ctx_->last_pipeline_ring_observation_source_stamp
                    : rclcpp::Time(0, 0, RCL_ROS_TIME);
            command_zero = true;
            break;
          }
          ++waypoint_index_;
        }

        if (!target_reached_) {
          const auto& target = reverse_waypoints_[waypoint_index_];
          const double dx = target.pose.position.x - pose.pose.position.x;
          const double dy = target.pose.position.y - pose.pose.position.y;
          const double dz = target.pose.position.z - pose.pose.position.z;
          const double kp_h = node_->get_parameter("task2_camera_reverse_kp_h").as_double();
          const double kp_v = node_->get_parameter("task2_camera_reverse_kp_v").as_double();
          const double max_h =
              node_->get_parameter("task2_camera_reverse_max_speed_h").as_double();
          const double max_v =
              node_->get_parameter("task2_camera_reverse_max_speed_v").as_double();
          vx = kp_h * dx;
          vy = kp_h * dy;
          const double horizontal_speed = std::hypot(vx, vy);
          if (horizontal_speed > max_h && horizontal_speed > 0.0) {
            const double scale = max_h / horizontal_speed;
            vx *= scale;
            vy *= scale;
          }
          vz = std::clamp(kp_v * dz, -max_v, max_v);
          command_velocity = true;
        }
      }

      if (failure.empty() && target_reached_) {
        command_zero = true;
        if ((now - reacquire_start_time_).seconds() >=
            node_->get_parameter("task2_camera_reverse_reacquire_timeout_s").as_double()) {
          failure = "red-ring visual reacquisition timeout";
        } else {
          bt_nodes::task2_camera::PipelineFrame frame;
          const bool pipeline_fresh = bt_nodes::task2_camera::freshPipelineLocked(
              *ctx_, now, node_->get_parameter("task2_camera_source_timeout_s").as_double(),
              node_->get_parameter("task2_camera_receipt_timeout_s").as_double(),
              node_->get_parameter("task2_camera_future_tolerance_s").as_double(), frame);
          if (!pipeline_fresh) {
            if (ctx_->has_pipeline_observation) {
              failure = "red-ring pipeline observation is stale or invalid";
            }
          } else {
            const bool ring_fresh = bt_nodes::task2_camera::freshRingHeartbeatLocked(
                *ctx_, now, node_->get_parameter("task2_camera_source_timeout_s").as_double(),
                node_->get_parameter("task2_camera_ring_heartbeat_timeout_s").as_double(),
                node_->get_parameter("task2_camera_future_tolerance_s").as_double());
            if (!ring_fresh) {
              if (ctx_->has_pipeline_ring_observation) {
                failure = "red-ring observation heartbeat is stale or invalid";
              }
            } else if (!freshMatchingRingLocked(frame, now)) {
              if (!pairing_candidate_) {
                pairing_candidate_ = true;
                pairing_start_time_ = now;
              } else if ((now - pairing_start_time_).seconds() >=
                         node_->get_parameter("task2_camera_reverse_pair_timeout_s").as_double()) {
                failure = "red-ring pipeline and ring observations are unpaired";
              }
            } else {
              pairing_candidate_ = false;
              const auto& ring =
                  ctx_->pipeline_ring_observation_cache.at(frame.observation.frame_sequence).observation;
              const rclcpp::Time ring_source_stamp(ring.header.stamp, RCL_ROS_TIME);
              const bool pipeline_newer =
                  frame.observation.frame_sequence > last_reacquire_sequence_ &&
                  frame.source_stamp.nanoseconds() > last_reacquire_source_stamp_.nanoseconds();
              const bool ring_newer =
                  ring.frame_sequence > last_reacquire_ring_sequence_ &&
                  ring_source_stamp.nanoseconds() >
                      last_reacquire_ring_source_stamp_.nanoseconds();
              if (has_reacquire_baseline_ &&
                  ((frame.observation.frame_sequence > last_reacquire_sequence_ &&
                    frame.source_stamp.nanoseconds() <=
                        last_reacquire_source_stamp_.nanoseconds()) ||
                   (ring.frame_sequence > last_reacquire_ring_sequence_ &&
                    ring_source_stamp.nanoseconds() <=
                        last_reacquire_ring_source_stamp_.nanoseconds()))) {
                failure = "red-ring camera source stamp regressed";
              } else if (pipeline_newer && ring_newer) {
                last_reacquire_sequence_ = frame.observation.frame_sequence;
                last_reacquire_source_stamp_ = frame.source_stamp;
                last_reacquire_ring_sequence_ = ring.frame_sequence;
                last_reacquire_ring_source_stamp_ = ring_source_stamp;
                const double offset_limit = node_->get_parameter(
                    "task2_camera_red_reacquire_offset_deadband").as_double();
                const bool red_candidate =
                    frame.observation.status ==
                        banyu_msgs::msg::PipelineObservation::TRACKED &&
                    frame.observation.confidence >= node_->get_parameter(
                        "task2_camera_red_reacquire_min_pipeline_confidence").as_double() &&
                    ring.visible &&
                    ring.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED &&
                    ring.confidence >= node_->get_parameter(
                        "task2_camera_red_reacquire_min_confidence").as_double() &&
                    std::abs(static_cast<double>(ring.offset_x)) <= offset_limit &&
                    std::abs(static_cast<double>(ring.offset_y)) <= offset_limit;
                if (frame.observation.status ==
                    banyu_msgs::msg::PipelineObservation::INVALID) {
                  failure = "INVALID pipeline observation during red-ring reacquisition";
                } else if (red_candidate) {
                  ++reacquire_frames_;
                } else {
                  reacquire_frames_ = 0;
                }
                if (failure.empty() && reacquire_frames_ >= static_cast<std::size_t>(
                                               node_->get_parameter(
                                                   "task2_camera_red_reacquire_required_frames")
                                                   .as_int())) {
                  run.reverse_reacquired = true;
                  complete = true;
                }
              }
            }
          }
        }
      }
    }
  }

  if (!failure.empty()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "ReverseToRedRing: %s", failure.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (complete) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "ReverseToRedRing: red ring visually reacquired");
    return BT::NodeStatus::SUCCESS;
  }
  if (command_velocity) {
    if (ctx_->setpoint_pub) {
      ctx_->setpoint_pub->publish(velocitySetpoint(vx, vy, vz, now));
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ReverseToRedRing: setpoint publisher unavailable");
      return BT::NodeStatus::FAILURE;
    }
  } else if (command_zero || target_reached_) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
  } else {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
  }
  return BT::NodeStatus::RUNNING;
}

void ReverseToRedRing::onHalted() {
  bt_nodes::task2_camera::publishZeroBody(ctx_, node_->now());
}

bool ReverseToRedRing::validateAndBuildReversePathLocked() {
  auto& run = ctx_->task2_camera_run;
  std::size_t red_index = 0;
  std::size_t red_count = 0;
  for (std::size_t i = 0; i < run.ring_records.size(); ++i) {
    if (run.ring_records[i].color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
      red_index = i;
      ++red_count;
    }
  }
  if (red_count != 1 || run.red_ring_count != 1 || !run.red_ring_recorded) {
    failLocked("camera run does not contain exactly one recorded red ring");
    return false;
  }
  if (run.path_samples.empty()) {
    failLocked("camera run has no recorded path samples for reverse travel");
    return false;
  }

  const auto& red_pose = run.ring_records[red_index].vehicle_pose_map;
  if (red_pose.header.frame_id != "map" || !bt_nodes::task2_common::finitePose(red_pose)) {
    failLocked("recorded red-ring target is invalid or not in map frame");
    return false;
  }
  const double max_jump =
      node_->get_parameter("task2_camera_reverse_max_path_jump_m").as_double();
  for (std::size_t i = 0; i < run.path_samples.size(); ++i) {
    const auto& sample = run.path_samples[i];
    if (sample.header.frame_id != "map" || !bt_nodes::task2_common::finitePose(sample)) {
      failLocked("recorded reverse path contains an invalid or non-map sample");
      return false;
    }
    if (i > 0 && (!std::isfinite(bt_nodes::task2_camera::poseDistance(run.path_samples[i - 1], sample)) ||
                  bt_nodes::task2_camera::poseDistance(run.path_samples[i - 1], sample) > max_jump)) {
      failLocked("recorded reverse path contains an unsafe pose jump");
      return false;
    }
  }

  const rclcpp::Time red_source_stamp(red_pose.header.stamp, RCL_ROS_TIME);
  if (red_source_stamp.nanoseconds() == 0) {
    failLocked("recorded red-ring source timestamp is invalid");
    return false;
  }
  std::size_t matching_sample_count = 0;
  std::size_t target_sample_index = 0;
  for (std::size_t i = 0; i < run.path_samples.size(); ++i) {
    const rclcpp::Time sample_stamp(run.path_samples[i].header.stamp, RCL_ROS_TIME);
    if (sample_stamp.nanoseconds() == red_source_stamp.nanoseconds()) {
      target_sample_index = i;
      ++matching_sample_count;
    }
  }
  if (matching_sample_count != 1) {
    failLocked("recorded red-ring source timestamp does not identify exactly one path sample");
    return false;
  }

  reverse_waypoints_.reserve(run.path_samples.size() - target_sample_index);
  for (std::size_t i = run.path_samples.size(); i-- > target_sample_index;) {
    reverse_waypoints_.push_back(run.path_samples[i]);
  }
  for (std::size_t i = 1; i < reverse_waypoints_.size(); ++i) {
    const double jump =
        bt_nodes::task2_camera::poseDistance(reverse_waypoints_[i - 1], reverse_waypoints_[i]);
    if (!std::isfinite(jump) || jump > max_jump) {
      failLocked("reverse red-ring target path jump exceeded the safety limit");
      reverse_waypoints_.clear();
      return false;
    }
  }

  run.reverse_target_record_index = red_index;
  run.reverse_target_pose = run.path_samples[target_sample_index];
  run.reverse_target_selected = true;
  return true;
}

bool ReverseToRedRing::poseAtWaypoint(const geometry_msgs::msg::PoseStamped& pose,
                                      const geometry_msgs::msg::PoseStamped& waypoint) const {
  return std::hypot(pose.pose.position.x - waypoint.pose.position.x,
                    pose.pose.position.y - waypoint.pose.position.y) <=
             node_->get_parameter("task2_camera_reverse_horizontal_tolerance_m").as_double() &&
         std::abs(pose.pose.position.z - waypoint.pose.position.z) <=
             node_->get_parameter("task2_camera_reverse_vertical_tolerance_m").as_double();
}

bool ReverseToRedRing::freshMatchingRingLocked(
    const bt_nodes::task2_camera::PipelineFrame& frame, const rclcpp::Time& now) const {
  const auto ring_it = ctx_->pipeline_ring_observation_cache.find(frame.observation.frame_sequence);
  if (ring_it == ctx_->pipeline_ring_observation_cache.end()) {
    return false;
  }
  const auto& ring_frame = ring_it->second;
  const rclcpp::Time ring_stamp(ring_frame.observation.header.stamp, RCL_ROS_TIME);
  const double receipt_age = (now - ring_frame.receipt_time).seconds();
  return ring_frame.contract_valid &&
         ring_frame.observation.frame_sequence == frame.observation.frame_sequence &&
         ring_stamp.nanoseconds() == frame.source_stamp.nanoseconds() && receipt_age >= 0.0 &&
         receipt_age <= node_->get_parameter("task2_camera_receipt_timeout_s").as_double();
}

void ReverseToRedRing::failLocked(const std::string& reason) {
  ctx_->task2_camera_run.input_failure = true;
  ctx_->task2_camera_run.failure_reason = reason;
}

}  // namespace banyu_mission
