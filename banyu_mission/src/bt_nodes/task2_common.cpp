#include "banyu_mission/bt_nodes/task2_common.hpp"

#include <cmath>

namespace {
double norm2(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}

namespace banyu_mission::bt_nodes::task2_common {
bool finitePose(const geometry_msgs::msg::PoseStamped& pose) {
  return std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y) &&
         std::isfinite(pose.pose.position.z) && std::isfinite(pose.pose.orientation.x) &&
         std::isfinite(pose.pose.orientation.y) && std::isfinite(pose.pose.orientation.z) &&
         std::isfinite(pose.pose.orientation.w);
}

bool receiptIsFresh(const rclcpp::Time& now, const rclcpp::Time& received, double timeout_s) {
  if (received.nanoseconds() == 0) {
    return false;
  }
  const double age_s = (now - received).seconds();
  return age_s >= 0.0 && age_s <= timeout_s;
}

bool task2RingColorKnown(uint8_t color) {
  return color == banyu_msgs::msg::PipelineRingState::COLOR_RED ||
         color == banyu_msgs::msg::PipelineRingState::COLOR_GREEN ||
         color == banyu_msgs::msg::PipelineRingState::COLOR_BLUE;
}

std::string task2RingColorName(uint8_t color) {
  switch (color) {
    case banyu_msgs::msg::PipelineRingState::COLOR_RED:
      return "red";
    case banyu_msgs::msg::PipelineRingState::COLOR_GREEN:
      return "green";
    case banyu_msgs::msg::PipelineRingState::COLOR_BLUE:
      return "blue";
    default:
      return "unknown";
  }
}

bool snapshotInputsLocked(
    const MissionContext& ctx, const rclcpp::Time& now, double route_timeout_s,
    double ring_timeout_s, double ring_confidence_min, double route_min_z, double route_max_z,
    double min_segment_spacing, double max_segment_spacing, double min_path_length,
    double max_path_length, std::array<geometry_msgs::msg::PoseStamped, kWaypointCount>& route,
    std::array<banyu_msgs::msg::PipelineRingState, kRingCount>& rings) {
  if (!ctx.has_pipeline_route || !ctx.has_pipeline_ring_state ||
      !receiptIsFresh(now, ctx.last_pipeline_route_received, route_timeout_s) ||
      ctx.latest_pipeline_route.header.frame_id != "map" ||
      ctx.latest_pipeline_route.poses.size() != kWaypointCount) {
    return false;
  }

  double path_length = 0.0;
  for (std::size_t i = 0; i < kWaypointCount; ++i) {
    const auto& route_pose = ctx.latest_pipeline_route.poses[i];
    if (route_pose.header.frame_id != "map" || !finitePose(route_pose) ||
        route_pose.pose.position.z < route_min_z || route_pose.pose.position.z > route_max_z) {
      return false;
    }
    route[i] = route_pose;
    if (i > 0) {
      const double segment_length = ::norm2(route[i - 1].pose.position, route[i].pose.position);
      if (segment_length < min_segment_spacing || segment_length > max_segment_spacing) {
        return false;
      }
      path_length += segment_length;
    }
  }
  if (path_length < min_path_length || path_length > max_path_length) {
    return false;
  }

  for (std::size_t i = 0; i < kRingCount; ++i) {
    const uint32_t index = static_cast<uint32_t>(i);
    const auto state_it = ctx.pipeline_ring_states.find(index);
    const auto receipt_it = ctx.pipeline_ring_state_receipt_times.find(index);
    if (state_it == ctx.pipeline_ring_states.end() ||
        receipt_it == ctx.pipeline_ring_state_receipt_times.end() ||
        !receiptIsFresh(now, receipt_it->second, ring_timeout_s)) {
      return false;
    }

    const auto& ring = state_it->second;
    if (ring.index != index || !task2RingColorKnown(ring.color) ||
        !std::isfinite(static_cast<double>(ring.confidence)) ||
        ring.confidence <= ring_confidence_min) {
      return false;
    }
    rings[i] = ring;
  }

  return true;
}

bool vehicleReady(const MissionContext& ctx, const rclcpp::Time& now,
                       double pose_timeout_s, double state_timeout_s,
                       geometry_msgs::msg::PoseStamped& pose,
                       mavros_msgs::msg::State& state) {
  if (!ctx.has_pose || !receiptIsFresh(now, ctx.last_pose_received, pose_timeout_s) ||
      !ctx.has_state || !receiptIsFresh(now, ctx.last_state_received, state_timeout_s) ||
      !ctx.state.connected || !ctx.state.armed || ctx.state.mode != "GUIDED" ||
      !finitePose(ctx.current_pose)) {
    return false;
  }
  pose = ctx.current_pose;
  state = ctx.state;
  return true;
}

}  // namespace banyu_mission::bt_nodes::task2_common
