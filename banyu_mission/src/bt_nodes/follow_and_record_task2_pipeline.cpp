#include "banyu_mission/bt_nodes/follow_and_record_task2_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission {

namespace {

bool freshMatchingRingLocked(const MissionContext& ctx,
                             const bt_nodes::task2_camera::PipelineFrame& frame,
                             const rclcpp::Time& now, double source_timeout_s,
                             double receipt_timeout_s, double future_tolerance_s) {
  const auto ring_it = ctx.pipeline_ring_observation_cache.find(frame.observation.frame_sequence);
  if (ring_it == ctx.pipeline_ring_observation_cache.end()) {
    return false;
  }

  const auto& ring_frame = ring_it->second;
  const rclcpp::Time ring_stamp(ring_frame.observation.header.stamp, RCL_ROS_TIME);
  const double source_age = (now - ring_stamp).seconds();
  const double receipt_age = (now - ring_frame.receipt_time).seconds();
  return ring_frame.contract_valid &&
         ring_frame.observation.frame_sequence == frame.observation.frame_sequence &&
         ring_stamp.nanoseconds() == frame.source_stamp.nanoseconds() &&
         source_age >= -future_tolerance_s && source_age <= source_timeout_s &&
         receipt_age >= 0.0 && receipt_age <= receipt_timeout_s;
}

}  // namespace

FollowAndRecordTask2Pipeline::FollowAndRecordTask2Pipeline(
    const std::string& name, const BT::NodeConfig& config,
    const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus FollowAndRecordTask2Pipeline::onStart() {
  start_time_ = node_->now();
  bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);
  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  if (!ctx_->task2_camera_source_selected || !ctx_->task2_camera_run.active ||
      !ctx_->task2_camera_run.acquired) {
    RCLCPP_ERROR(node_->get_logger(),
                 "FollowAndRecordTask2Pipeline: camera acquisition was not completed");
    return BT::NodeStatus::FAILURE;
  }
  ctx_->task2_camera_run.loss_candidate = false;
  ctx_->task2_camera_run.no_pipe_frames = 0;
  ctx_->task2_camera_run.pending_ring_color =
      banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  ctx_->task2_camera_run.pending_ring_frames = 0;
  ctx_->task2_camera_run.pending_ring_sequence = 0;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowAndRecordTask2Pipeline::onRunning() {
  const rclcpp::Time now = node_->now();
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }
  if ((now - start_time_).seconds() >=
      node_->get_parameter("task2_camera_follow_timeout_s").as_double()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "FollowAndRecordTask2Pipeline: timeout");
    return BT::NodeStatus::FAILURE;
  }

  bt_nodes::task2_camera::PipelineFrame frame;
  banyu_msgs::msg::PipelineObservation tracked_observation;
  bool command_follow = false;
  bool command_zero = false;
  bool complete = false;
  std::string failure;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->task2_camera_run.input_failure) {
      failure = ctx_->task2_camera_run.failure_reason;
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
      const bool fresh = bt_nodes::task2_camera::freshPipelineLocked(
          *ctx_, now, node_->get_parameter("task2_camera_source_timeout_s").as_double(),
          node_->get_parameter("task2_camera_receipt_timeout_s").as_double(),
          node_->get_parameter("task2_camera_future_tolerance_s").as_double(), frame);
      if (!ready) {
        failure = "vehicle readiness lost";
      } else if (!fresh) {
        failure = "camera observation is missing or stale";
      } else {
        const bool ring_heartbeat_fresh = bt_nodes::task2_camera::freshRingHeartbeatLocked(
            *ctx_, now, node_->get_parameter("task2_camera_source_timeout_s").as_double(),
            node_->get_parameter("task2_camera_ring_heartbeat_timeout_s").as_double(),
            node_->get_parameter("task2_camera_future_tolerance_s").as_double());
        if (!ring_heartbeat_fresh) {
          command_zero = true;
          if (!ctx_->task2_camera_run.ring_heartbeat_loss_candidate) {
            ctx_->task2_camera_run.ring_heartbeat_loss_candidate = true;
            ctx_->task2_camera_run.ring_heartbeat_loss_start = now;
          } else if ((now - ctx_->task2_camera_run.ring_heartbeat_loss_start).seconds() >=
                     node_->get_parameter("task2_camera_ring_heartbeat_loss_tolerance_s").as_double()) {
            failure = "ring heartbeat source or receipt freshness lost";
          }
        } else {
          ctx_->task2_camera_run.ring_heartbeat_loss_candidate = false;
        }
        if (frame.observation.status == banyu_msgs::msg::PipelineObservation::INVALID) {
          failure = "INVALID pipeline observation reached the camera follower";
        }
        if (!failure.empty() || command_zero) {
          // Do not consume a pipeline frame while the ring heartbeat is stale.
        } else {
          const bool new_frame =
              frame.observation.frame_sequence > ctx_->task2_camera_run.consumed_pipeline_sequence;
          if (new_frame && frame.source_stamp.nanoseconds() <=
                               ctx_->task2_camera_run.consumed_pipeline_stamp.nanoseconds()) {
            failure = "camera source stamp regressed after sequence advanced";
          } else {
            bool pair_ready = freshMatchingRingLocked(
                *ctx_, frame, now,
                node_->get_parameter("task2_camera_source_timeout_s").as_double(),
                node_->get_parameter("task2_camera_receipt_timeout_s").as_double(),
                node_->get_parameter("task2_camera_future_tolerance_s").as_double());
            if (new_frame && !pair_ready) {
              command_zero = true;
              if (!ctx_->task2_camera_run.pairing_gap_candidate) {
                ctx_->task2_camera_run.pairing_gap_candidate = true;
                ctx_->task2_camera_run.pairing_gap_start = now;
              } else if ((now - ctx_->task2_camera_run.pairing_gap_start).seconds() >=
                         node_->get_parameter(
                             "task2_camera_ring_heartbeat_loss_tolerance_s").as_double()) {
                failure = "matching fresh ring observation was not received";
              }
            } else if (new_frame) {
              ctx_->task2_camera_run.pairing_gap_candidate = false;
            }

            if (pair_ready) {
              if (frame.observation.status ==
                  banyu_msgs::msg::PipelineObservation::TRACKED) {
                tracked_observation = frame.observation;
                command_follow = true;
                if (new_frame) {
                  ctx_->task2_camera_run.tracking_started = true;
                  ctx_->task2_camera_run.loss_candidate = false;
                  ctx_->task2_camera_run.no_pipe_frames = 0;
                  geometry_msgs::msg::PoseStamped associated_pose;
                  if (!bt_nodes::task2_camera::associatePoseLocked(
                          *ctx_, frame.source_stamp,
                          node_->get_parameter("task2_camera_pose_association_skew_s").as_double(),
                          associated_pose) ||
                      !recordPathLocked(frame.source_stamp, associated_pose) ||
                      !processRingLocked(frame, associated_pose, now)) {
                    failure = ctx_->task2_camera_run.failure_reason.empty()
                                  ? "unable to associate or record camera frame"
                                  : ctx_->task2_camera_run.failure_reason;
                  }
                }
              } else if (frame.observation.status ==
                         banyu_msgs::msg::PipelineObservation::NO_PIPE) {
                command_zero = true;
                if (new_frame) {
                  if (!ctx_->task2_camera_run.loss_candidate) {
                    ctx_->task2_camera_run.loss_candidate = true;
                    ctx_->task2_camera_run.loss_start_source_stamp = frame.source_stamp;
                    ctx_->task2_camera_run.no_pipe_frames = 0;
                    if (!storeFinalObservedPathSampleLocked()) {
                      failure = ctx_->task2_camera_run.failure_reason;
                    }
                  }
                  if (failure.empty()) {
                    ++ctx_->task2_camera_run.no_pipe_frames;
                  }
                }
                complete = ctx_->task2_camera_run.tracking_started &&
                           ctx_->task2_camera_run.loss_candidate &&
                           ctx_->task2_camera_run.no_pipe_frames >= static_cast<std::size_t>(
                               node_->get_parameter("task2_camera_loss_required_frames").as_int()) &&
                           (frame.source_stamp -
                            ctx_->task2_camera_run.loss_start_source_stamp).seconds() >=
                               node_->get_parameter("task2_camera_loss_persistence_s").as_double() &&
                           endConditionsMetLocked(frame.source_stamp);
              }

              if (new_frame && failure.empty()) {
                const auto& accepted_ring =
                    ctx_->pipeline_ring_observation_cache.at(frame.observation.frame_sequence);
                ctx_->task2_camera_run.consumed_pipeline_sequence =
                    frame.observation.frame_sequence;
                ctx_->task2_camera_run.consumed_pipeline_stamp = frame.source_stamp;
                ctx_->task2_camera_run.last_heartbeat_sequence =
                    accepted_ring.observation.frame_sequence;
                ctx_->task2_camera_run.last_heartbeat_source_stamp =
                    rclcpp::Time(accepted_ring.observation.header.stamp, RCL_ROS_TIME);
              }
            }
          }
        }
      }
    }
  }

  if (!failure.empty()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "FollowAndRecordTask2Pipeline: %s", failure.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (complete) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "FollowAndRecordTask2Pipeline: guarded pipeline loss complete");
    return BT::NodeStatus::SUCCESS;
  }
  if (command_zero) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    return BT::NodeStatus::RUNNING;
  }
  if (command_follow) {
    publishFollowCommand(tracked_observation, now);
  } else {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
  }
  return BT::NodeStatus::RUNNING;
}

bool FollowAndRecordTask2Pipeline::recordPathLocked(
    const rclcpp::Time& source_stamp, const geometry_msgs::msg::PoseStamped& associated_pose) {
  auto& run = ctx_->task2_camera_run;
  const double max_segment_jump =
      node_->get_parameter("task2_camera_max_segment_jump_m").as_double();
  const double max_path_length =
      node_->get_parameter("task2_camera_max_path_length_m").as_double();
  const double max_duration =
      node_->get_parameter("task2_camera_max_path_duration_s").as_double();
  const double sample_spacing =
      node_->get_parameter("task2_camera_path_sample_spacing_m").as_double();
  const std::size_t max_samples = static_cast<std::size_t>(
      node_->get_parameter("task2_camera_max_path_samples").as_int());

  geometry_msgs::msg::PoseStamped pose = associated_pose;
  pose.header.stamp = static_cast<builtin_interfaces::msg::Time>(source_stamp);
  pose.header.frame_id = "map";
  if (!run.path_started) {
    run.path_started = true;
    run.path_samples.push_back(pose);
    run.last_stored_pose = pose;
    run.has_last_stored_pose = true;
    run.previous_observed_pose = pose;
    run.has_previous_observed_pose = true;
    return true;
  }

  const double segment = bt_nodes::task2_camera::poseDistance(run.previous_observed_pose, pose);
  if (!std::isfinite(segment) || segment > max_segment_jump) {
    failLocked("camera path segment exceeded the hard jump limit");
    return false;
  }
  run.path_length_m += segment;
  if (run.path_length_m > max_path_length) {
    failLocked("camera path exceeded its hard length limit");
    return false;
  }
  if ((source_stamp - rclcpp::Time(run.path_samples.front().header.stamp, RCL_ROS_TIME)).seconds() >
      max_duration) {
    failLocked("camera path exceeded its hard duration limit");
    return false;
  }
  const double storage_distance =
      bt_nodes::task2_camera::poseDistance(run.last_stored_pose, pose);
  if (!std::isfinite(storage_distance)) {
    failLocked("camera path storage distance was not finite");
    return false;
  }
  if (storage_distance >= sample_spacing) {
    if (run.path_samples.size() >= max_samples) {
      failLocked("camera path sample limit reached");
      return false;
    }
    run.path_samples.push_back(pose);
    run.last_stored_pose = pose;
    run.has_last_stored_pose = true;
  }
  run.previous_observed_pose = pose;
  run.has_previous_observed_pose = true;
  return true;
}

bool FollowAndRecordTask2Pipeline::storeRequiredPathSampleLocked(
    const geometry_msgs::msg::PoseStamped& pose) {
  auto& run = ctx_->task2_camera_run;
  const std::size_t max_samples = static_cast<std::size_t>(
      node_->get_parameter("task2_camera_max_path_samples").as_int());
  if (run.has_last_stored_pose && run.last_stored_pose.header.stamp == pose.header.stamp) {
    return true;
  }
  if (run.path_samples.size() >= max_samples) {
    failLocked("camera path sample limit prevented a required sample");
    return false;
  }
  run.path_samples.push_back(pose);
  run.last_stored_pose = pose;
  run.has_last_stored_pose = true;
  return true;
}

bool FollowAndRecordTask2Pipeline::storeFinalObservedPathSampleLocked() {
  auto& run = ctx_->task2_camera_run;
  if (!run.has_previous_observed_pose) {
    return true;
  }
  return storeRequiredPathSampleLocked(run.previous_observed_pose);
}

bool FollowAndRecordTask2Pipeline::processRingLocked(
    const bt_nodes::task2_camera::PipelineFrame& frame,
    const geometry_msgs::msg::PoseStamped& associated_pose, const rclcpp::Time& now) {
  auto& run = ctx_->task2_camera_run;
  const auto ring_it = ctx_->pipeline_ring_observation_cache.find(frame.observation.frame_sequence);
  if (ring_it == ctx_->pipeline_ring_observation_cache.end()) {
    if (run.pending_ring_frames > 0 && !run.pairing_gap_candidate) {
      run.pairing_gap_candidate = true;
      run.pairing_gap_start = now;
    }
    if (run.pairing_gap_candidate &&
        (now - run.pairing_gap_start).seconds() >=
            node_->get_parameter("task2_camera_ring_heartbeat_loss_tolerance_s").as_double()) {
      run.pending_ring_color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
      run.pending_ring_frames = 0;
      run.pending_ring_sequence = 0;
      run.pairing_gap_candidate = false;
    }
    return true;
  }
  run.pairing_gap_candidate = false;
  const auto& ring_frame = ring_it->second;
  const auto& ring = ring_frame.observation;
  const rclcpp::Time ring_stamp(ring.header.stamp, RCL_ROS_TIME);
  if (!ring_frame.contract_valid || ring_stamp.nanoseconds() != frame.source_stamp.nanoseconds() ||
      (now - ring_frame.receipt_time).seconds() < 0.0 ||
      (now - ring_frame.receipt_time).seconds() >
          node_->get_parameter("task2_camera_receipt_timeout_s").as_double()) {
    failLocked("paired ring observation failed freshness or sequence matching");
    return false;
  }

  if (!ring.visible) {
    ++run.ring_invisible_frames;
    run.pending_ring_color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
    run.pending_ring_frames = 0;
    run.pending_ring_sequence = 0;
    run.ring_armed =
        run.ring_invisible_frames >= static_cast<std::size_t>(
            node_->get_parameter("task2_camera_ring_invisible_required_frames").as_int()) &&
        (!run.has_last_ring_pose ||
                     run.path_length_m - run.path_length_at_last_ring_m >=
                         node_->get_parameter("task2_camera_ring_min_travel_spacing_m").as_double());
    return true;
  }

  run.ring_invisible_frames = 0;
  if (!run.ring_armed) {
    return true;
  }
  if (ring.confidence <
      node_->get_parameter("task2_camera_ring_min_confidence").as_double()) {
    run.pending_ring_color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
    run.pending_ring_frames = 0;
    run.pending_ring_sequence = 0;
    return true;
  }
  if (run.pending_ring_color == ring.color && frame.observation.frame_sequence >
                                                   run.pending_ring_sequence) {
    ++run.pending_ring_frames;
  } else {
    run.pending_ring_color = ring.color;
    run.pending_ring_frames = 1;
  }
  run.pending_ring_sequence = frame.observation.frame_sequence;
  const bool enough_confirmation = run.pending_ring_frames >= static_cast<std::size_t>(
      node_->get_parameter("task2_camera_ring_confirmation_frames").as_int());
  const bool enough_spacing =
      !run.has_last_ring_pose ||
      run.path_length_m - run.path_length_at_last_ring_m >=
          node_->get_parameter("task2_camera_ring_min_travel_spacing_m").as_double();
  const std::size_t max_records = static_cast<std::size_t>(
      node_->get_parameter("task2_camera_max_ring_records").as_int());
  const std::size_t required_records = static_cast<std::size_t>(
      node_->get_parameter("task2_camera_required_ring_count").as_int());
  if (!enough_confirmation || !enough_spacing) {
    return true;
  }
  if (run.ring_records.size() >= required_records) {
    failLocked("camera ring record count exceeded the fixed required count");
    return false;
  }
  if (ring.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED &&
      run.red_ring_count >= 1) {
    failLocked("camera ring stream produced a second red record");
    return false;
  }
  if (run.ring_records.size() >= max_records) {
    failLocked("camera ring record limit reached");
    return false;
  }

  banyu_msgs::msg::Task2RingRecord record;
  record.frame_sequence = ring.frame_sequence;
  record.color = ring.color;
  record.confidence = ring.confidence;
  record.vehicle_pose_map = associated_pose;
  if (!storeRequiredPathSampleLocked(associated_pose)) {
    return false;
  }
  run.ring_records.push_back(record);
  run.red_ring_recorded = run.red_ring_recorded ||
                          ring.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
  if (ring.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
    ++run.red_ring_count;
  }
  run.path_length_at_last_ring_m = run.path_length_m;
  run.last_ring_pose = associated_pose;
  run.has_last_ring_pose = true;
  run.ring_armed = false;
  run.ring_invisible_frames = 0;
  run.pending_ring_color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  run.pending_ring_frames = 0;
  run.pending_ring_sequence = 0;
  RCLCPP_INFO(node_->get_logger(), "FollowAndRecordTask2Pipeline: accepted ring color=%u",
              static_cast<unsigned>(ring.color));
  return true;
}

bool FollowAndRecordTask2Pipeline::endConditionsMetLocked(const rclcpp::Time&) const {
  const auto& run = ctx_->task2_camera_run;
  return run.ring_records.size() == static_cast<std::size_t>(
             node_->get_parameter("task2_camera_required_ring_count").as_int()) &&
         run.red_ring_count == 1 &&
         run.path_length_m >= node_->get_parameter("task2_camera_min_path_length_m").as_double() &&
         run.path_length_m - run.path_length_at_last_ring_m >=
             node_->get_parameter("task2_camera_min_travel_after_last_ring_m").as_double();
}

void FollowAndRecordTask2Pipeline::failLocked(const std::string& reason) {
  ctx_->task2_camera_run.input_failure = true;
  ctx_->task2_camera_run.failure_reason = reason;
}

void FollowAndRecordTask2Pipeline::publishFollowCommand(
    const banyu_msgs::msg::PipelineObservation& observation, const rclcpp::Time& stamp) {
  const double lateral_limit = node_->get_parameter("task2_camera_forward_lateral_limit").as_double();
  const double heading_limit = node_->get_parameter("task2_camera_forward_heading_limit").as_double();
  const double lateral_error = std::abs(static_cast<double>(observation.offset_x));
  const double heading_error = std::abs(static_cast<double>(observation.heading_error_rad));
  const double alignment = std::max(lateral_error / std::max(1.0e-6, lateral_limit),
                                    heading_error / std::max(1.0e-6, heading_limit));
  const double forward_scale = std::clamp(1.0 - alignment, 0.0, 1.0);
  const double forward = std::min(
      node_->get_parameter("task2_camera_forward_speed").as_double(),
      node_->get_parameter("task2_camera_max_forward_speed").as_double()) * forward_scale;
  const double right = std::clamp(
      node_->get_parameter("task2_camera_lateral_kp").as_double() * observation.offset_x,
      -node_->get_parameter("task2_camera_max_lateral_speed").as_double(),
      node_->get_parameter("task2_camera_max_lateral_speed").as_double());
  const double yaw_rate = std::clamp(
      node_->get_parameter("task2_camera_heading_kp").as_double() *
          observation.heading_error_rad,
      -node_->get_parameter("task2_camera_max_yaw_rate").as_double(),
      node_->get_parameter("task2_camera_max_yaw_rate").as_double());
  if (ctx_->setpoint_pub) {
    // Image-right and positive clockwise image heading use the same positive
    // body-right and body-NED yaw signs.
    ctx_->setpoint_pub->publish(
        bodyVelocityYawRateSetpoint(forward, right, 0.0, yaw_rate, stamp));
  }
}

void FollowAndRecordTask2Pipeline::onHalted() {
  bt_nodes::task2_camera::publishZeroBody(ctx_, node_->now());
}

}  // namespace banyu_mission
