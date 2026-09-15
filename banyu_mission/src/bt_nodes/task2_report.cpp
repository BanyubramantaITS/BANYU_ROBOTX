#include "banyu_mission/bt_nodes/task2_report.hpp"

#include "banyu_mission/bt_nodes/task2_camera_common.hpp"
#include "banyu_mission/bt_nodes/task2_common.hpp"

namespace banyu_mission {

namespace {

bool claimTask2TerminalLocked(const std::shared_ptr<MissionContext>& ctx,
                              uint8_t terminal_status) {
  if (terminal_status != banyu_msgs::msg::Task2Report::COMPLETED &&
      terminal_status != banyu_msgs::msg::Task2Report::FAILED &&
      terminal_status != banyu_msgs::msg::Task2Report::ABORTED) {
    return false;
  }
  if (terminal_status == banyu_msgs::msg::Task2Report::COMPLETED &&
      (!ctx->mission_active.load() || ctx->aborted.load())) {
    return false;
  }
  uint8_t expected = banyu_msgs::msg::Task2Report::STATUS_UNKNOWN;
  return ctx->task2_terminal_claim.compare_exchange_strong(expected, terminal_status);
}

}  // namespace

bool claimTask2Terminal(const std::shared_ptr<MissionContext>& ctx, uint8_t terminal_status) {
  std::lock_guard<std::mutex> terminal_lock(ctx->task2_terminal_mutex);
  return claimTask2TerminalLocked(ctx, terminal_status);
}

bool cancelTask2Terminal(const std::shared_ptr<MissionContext>& ctx) {
  std::lock_guard<std::mutex> terminal_lock(ctx->task2_terminal_mutex);
  ctx->aborted.store(true);
  ctx->mission_active.store(false);
  return claimTask2TerminalLocked(ctx, banyu_msgs::msg::Task2Report::ABORTED);
}

bool publishTask2Report(const std::shared_ptr<MissionContext>& ctx,
                        const rclcpp::Time& stamp, uint8_t terminal_status,
                        const std::string& detail, double source_timeout_s,
                        double future_tolerance_s, bool terminal_claimed) {
  {
    std::lock_guard<std::mutex> lock(ctx->data_mutex);
    if (!ctx->task2_report_pub || !ctx->task2_run_id_valid || ctx->task2_report_published) {
      ctx->task2_camera_run.failure_reason = "Task2Report publisher or run UUID unavailable";
      return false;
    }
    if (!terminal_claimed && terminal_status == banyu_msgs::msg::Task2Report::COMPLETED &&
        (!ctx->task2_camera_source_selected || !ctx->task2_camera_run.active ||
         !ctx->task2_camera_run.reverse_reacquired ||
         ctx->task2_repair_status != banyu_msgs::msg::Task2Report::HOLD_COMPLETED_UNVERIFIED ||
         !ctx->has_task2_final_surface_pose ||
         !bt_nodes::task2_common::poseSourceIsFresh(
             ctx->task2_final_surface_pose, stamp, source_timeout_s, future_tolerance_s))) {
      ctx->task2_camera_run.failure_reason =
          "Task2Report completion invariants were not satisfied";
      return false;
    }
  }
  if (!terminal_claimed && !claimTask2Terminal(ctx, terminal_status)) {
    return false;
  }
  if (terminal_claimed &&
      ctx->task2_terminal_claim.load() != terminal_status) {
    return false;
  }

  std::lock_guard<std::mutex> lock(ctx->data_mutex);
  if (ctx->task2_report_published) {
    return false;
  }

  banyu_msgs::msg::Task2Report report;
  report.header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
  report.header.frame_id = "map";
  report.run_id = ctx->task2_run_id;
  report.terminal_status = terminal_status;
  report.repair_status = ctx->task2_repair_status;
  report.repair_hold_duration_s = static_cast<float>(ctx->task2_repair_hold_duration_s);
  report.detail = detail;
  report.has_enu_anchor = ctx->has_task2_surface_anchor;
  if (report.has_enu_anchor) {
    report.enu_anchor_fix = ctx->task2_anchor_auv_fix;
    report.enu_anchor_pose_map = ctx->task2_anchor_map_pose;
  }

  const auto& run = ctx->task2_camera_run;
  report.rings = run.ring_records;
  report.red_ring_record_index = -1;
  if (run.reverse_target_selected &&
      run.reverse_target_record_index < run.ring_records.size() &&
      run.ring_records[run.reverse_target_record_index].color ==
          banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
    report.red_ring_record_index = static_cast<int32_t>(run.reverse_target_record_index);
  } else {
    for (std::size_t i = 0; i < run.ring_records.size(); ++i) {
      if (run.ring_records[i].color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
        if (report.red_ring_record_index >= 0) {
          report.red_ring_record_index = -1;
          break;
        }
        report.red_ring_record_index = static_cast<int32_t>(i);
      }
    }
  }

  if (!run.path_samples.empty() &&
      run.path_samples.front().header.frame_id == "map" &&
      bt_nodes::task2_common::finitePose(run.path_samples.front())) {
    report.has_first_pass_start_pose = true;
    report.first_pass_start_pose_map = run.path_samples.front();
  }
  if (!run.path_samples.empty() && run.path_samples.back().header.frame_id == "map" &&
      bt_nodes::task2_common::finitePose(run.path_samples.back())) {
    report.has_first_pass_end_pose = true;
    report.first_pass_end_pose_map = run.path_samples.back();
  }
  if (ctx->has_task2_final_surface_pose &&
      bt_nodes::task2_common::validMapPose(ctx->task2_final_surface_pose)) {
    report.has_final_surface_pose = true;
    report.final_surface_pose_map = ctx->task2_final_surface_pose;
  }

  ctx->task2_report_pub->publish(report);
  ctx->task2_report_published = true;
  return true;
}

Task2ReportNode::Task2ReportNode(const std::string& name, const BT::NodeConfig& config,
                                 const rclcpp::Node::SharedPtr& node,
                                 std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus Task2ReportNode::onStart() {
  const rclcpp::Time now = node_->now();
  bt_nodes::task2_camera::publishZeroBody(ctx_, now);
  bool ready = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ready = ctx_->mission_active.load() && !ctx_->aborted.load() &&
            ctx_->task2_camera_source_selected && ctx_->task2_camera_run.active &&
            ctx_->task2_camera_run.reverse_reacquired &&
            ctx_->task2_repair_status == banyu_msgs::msg::Task2Report::HOLD_COMPLETED_UNVERIFIED &&
            ctx_->has_task2_final_surface_pose &&
            bt_nodes::task2_common::poseSourceIsFresh(
                ctx_->task2_final_surface_pose, now,
                node_->get_parameter("task2_camera_source_timeout_s").as_double(),
                node_->get_parameter("task2_camera_future_tolerance_s").as_double());
  }
  if (!ready ||
      !publishTask2Report(ctx_, now, banyu_msgs::msg::Task2Report::COMPLETED,
                          "Task 2 camera mission completed",
                          node_->get_parameter("task2_camera_source_timeout_s").as_double(),
                          node_->get_parameter("task2_camera_future_tolerance_s").as_double(),
                          false)) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->task2_camera_run.failure_reason =
        ready ? "Task2Report publisher or terminal claim failed"
              : "Task2Report active completion invariants were not satisfied";
    bt_nodes::task2_camera::publishZeroBody(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "Task2Report: unable to publish the completed report");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "Task2Report: completed report published");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Task2ReportNode::onRunning() {
  const rclcpp::Time now = node_->now();
  bt_nodes::task2_camera::publishZeroBody(ctx_, now);
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->task2_camera_run.failure_reason = "Task2Report was cancelled before terminal completion";
    return BT::NodeStatus::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->task2_camera_run.failure_reason = "Task2Report entered an invalid running state";
  }
  return BT::NodeStatus::FAILURE;
}

void Task2ReportNode::onHalted() {
  bt_nodes::task2_camera::publishZeroBody(ctx_, node_->now());
}

}  // namespace banyu_mission
