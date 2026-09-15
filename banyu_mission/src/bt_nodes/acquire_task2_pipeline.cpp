#include "banyu_mission/bt_nodes/acquire_task2_pipeline.hpp"

#include <algorithm>

#include "banyu_mission/bt_nodes/task2_camera_common.hpp"
#include "banyu_mission/bt_nodes/task2_common.hpp"

namespace banyu_mission {

AcquireTask2Pipeline::AcquireTask2Pipeline(const std::string& name,
                                           const BT::NodeConfig& config,
                                           const rclcpp::Node::SharedPtr& node,
                                           std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus AcquireTask2Pipeline::onStart() {
  start_time_ = node_->now();
  depth_ready_ = false;
  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  ctx_->task2_camera_run.acquire_tracked_frames = 0;
  ctx_->task2_camera_run.acquired = false;
  bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AcquireTask2Pipeline::onRunning() {
  const rclcpp::Time now = node_->now();
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }
  if ((now - start_time_).seconds() >=
      node_->get_parameter("task2_camera_acquire_timeout_s").as_double()) {
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "AcquireTask2Pipeline: acquisition timeout");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  mavros_msgs::msg::State state;
  bt_nodes::task2_camera::PipelineFrame frame;
  bool fresh = false;
  bool new_frame = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->task2_camera_run.input_failure) {
      bt_nodes::task2_camera::publishZeroBody(ctx_, now);
      RCLCPP_ERROR(node_->get_logger(), "AcquireTask2Pipeline: %s",
                   ctx_->task2_camera_run.failure_reason.c_str());
      return BT::NodeStatus::FAILURE;
    }
    const bool ready =
        bt_nodes::task2_common::dvlReadyLocked(
            *ctx_, now, node_->get_parameter("task2_use_dvl").as_bool(),
            node_->get_parameter("task2_dvl_timeout_s").as_double()) &&
        bt_nodes::task2_common::vehicleReady(
            *ctx_, now, node_->get_parameter("task2_pose_timeout_s").as_double(),
            node_->get_parameter("task2_state_timeout_s").as_double(), pose, state);
    fresh = bt_nodes::task2_camera::freshPipelineLocked(
        *ctx_, now, node_->get_parameter("task2_camera_source_timeout_s").as_double(),
        node_->get_parameter("task2_camera_receipt_timeout_s").as_double(),
        node_->get_parameter("task2_camera_future_tolerance_s").as_double(), frame);
    if (!ready || !fresh) {
      bt_nodes::task2_camera::publishZeroBody(ctx_, now);
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "AcquireTask2Pipeline: readiness or camera freshness lost");
      return BT::NodeStatus::FAILURE;
    }
    if (frame.observation.status == banyu_msgs::msg::PipelineObservation::INVALID) {
      bt_nodes::task2_camera::publishZeroBody(ctx_, now);
      RCLCPP_ERROR(node_->get_logger(), "AcquireTask2Pipeline: INVALID pipeline observation");
      return BT::NodeStatus::FAILURE;
    }

    if (!depth_ready_) {
      const double depth_error =
          node_->get_parameter("task2_camera_operating_depth").as_double() -
          pose.pose.position.z;
      if (std::abs(depth_error) <=
          node_->get_parameter("task2_camera_depth_tolerance_m").as_double()) {
        depth_ready_ = true;
      } else {
        if ((now - start_time_).seconds() >=
            node_->get_parameter("task2_camera_depth_timeout_s").as_double()) {
          bt_nodes::task2_camera::publishZeroBody(ctx_, now);
          RCLCPP_ERROR(node_->get_logger(), "AcquireTask2Pipeline: operating-depth timeout");
          return BT::NodeStatus::FAILURE;
        }
        const double max_depth_speed =
            node_->get_parameter("task2_camera_max_depth_speed").as_double();
        const double body_down = std::clamp(
            -node_->get_parameter("task2_camera_depth_kp").as_double() * depth_error,
            -max_depth_speed, max_depth_speed);
        if (ctx_->setpoint_pub) {
          ctx_->setpoint_pub->publish(
              bodyVelocityYawRateSetpoint(0.0, 0.0, body_down, 0.0, now));
        }
        return BT::NodeStatus::RUNNING;
      }
    }
    new_frame = frame.observation.frame_sequence >
                    ctx_->task2_camera_run.consumed_pipeline_sequence &&
                frame.source_stamp.nanoseconds() >
                    ctx_->task2_camera_run.consumed_pipeline_stamp.nanoseconds();
    if (new_frame) {
      ctx_->task2_camera_run.consumed_pipeline_sequence = frame.observation.frame_sequence;
      ctx_->task2_camera_run.consumed_pipeline_stamp = frame.source_stamp;
      if (frame.observation.status == banyu_msgs::msg::PipelineObservation::TRACKED &&
          frame.observation.confidence >=
              node_->get_parameter("task2_camera_acquire_min_confidence").as_double()) {
        ++ctx_->task2_camera_run.acquire_tracked_frames;
      } else {
        ctx_->task2_camera_run.acquire_tracked_frames = 0;
      }
      if (ctx_->task2_camera_run.acquire_tracked_frames >= static_cast<std::size_t>(
              node_->get_parameter("task2_camera_acquire_required_frames").as_int())) {
        ctx_->task2_camera_run.acquired = true;
        ctx_->task2_camera_run.tracking_started = true;
        bt_nodes::task2_camera::publishZeroBody(ctx_, now);
        RCLCPP_INFO(node_->get_logger(), "AcquireTask2Pipeline: pipeline acquired");
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  // Acquisition never surges forward. A fresh NO_PIPE or an unchanged fresh
  // frame is handled by the bounded yaw-only search.
  if (ctx_->setpoint_pub) {
    const double max_yaw_rate = node_->get_parameter("task2_camera_max_yaw_rate").as_double();
    const double yaw_rate = std::clamp(
        node_->get_parameter("task2_camera_search_yaw_rate").as_double(),
        -max_yaw_rate, max_yaw_rate);
    ctx_->setpoint_pub->publish(bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, yaw_rate, now));
  }
  (void)new_frame;
  return BT::NodeStatus::RUNNING;
}

void AcquireTask2Pipeline::onHalted() {
  bt_nodes::task2_camera::publishZeroBody(ctx_, node_->now());
}

}  // namespace banyu_mission
