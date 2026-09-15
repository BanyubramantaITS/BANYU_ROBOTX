#include "banyu_mission/bt_nodes/surface_acquire_gps_anchor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "banyu_mission/bt_nodes/common.hpp"
#include "banyu_mission/bt_nodes/task2_common.hpp"

namespace banyu_mission {
namespace {

constexpr double kWgs84SemiMajorM = 6378137.0;
constexpr double kWgs84EccentricitySquared = 6.6943799901413165e-3;

bool validFix(const sensor_msgs::msg::NavSatFix& fix, double max_horizontal_covariance) {
  if (fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) ||
      fix.latitude < -90.0 || fix.latitude > 90.0 ||
      fix.longitude < -180.0 || fix.longitude > 180.0 ||
      fix.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    return false;
  }

  for (std::size_t i = 0; i < fix.position_covariance.size(); ++i) {
    if (!std::isfinite(fix.position_covariance[i]) ||
        ((i == 0 || i == 4 || i == 8) && fix.position_covariance[i] < 0.0)) {
      return false;
    }
  }
  const double xx = fix.position_covariance[0];
  const double xy = fix.position_covariance[1];
  const double yy = fix.position_covariance[4];
  const double horizontal_max_eigenvalue =
      0.5 * (xx + yy + std::sqrt((xx - yy) * (xx - yy) + 4.0 * xy * xy));
  return std::isfinite(horizontal_max_eigenvalue) &&
         horizontal_max_eigenvalue <= max_horizontal_covariance;
}

bool receiptFresh(const rclcpp::Time& now, const rclcpp::Time& received, double timeout_s) {
  return received.nanoseconds() != 0 && (now - received).seconds() >= 0.0 &&
         (now - received).seconds() <= timeout_s;
}

bool stampFresh(const rclcpp::Time& now, const builtin_interfaces::msg::Time& stamp,
                double timeout_s) {
  const rclcpp::Time message_time(stamp);
  return message_time.nanoseconds() != 0 && (now - message_time).seconds() >= 0.0 &&
         (now - message_time).seconds() <= timeout_s;
}

bool sourceStampAvailable(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
         (stamp.sec != 0 || stamp.nanosec != 0);
}

bool sourceOrReceiptFresh(const rclcpp::Time& now,
                          const builtin_interfaces::msg::Time& stamp,
                          const rclcpp::Time& received, double timeout_s,
                          bool allow_zero_source_stamp) {
  if (!receiptFresh(now, received, timeout_s)) {
    return false;
  }
  return sourceStampAvailable(stamp) ? stampFresh(now, stamp, timeout_s)
                                     : allow_zero_source_stamp;
}

double stampSkew(const builtin_interfaces::msg::Time& lhs,
                 const builtin_interfaces::msg::Time& rhs) {
  return std::abs((rclcpp::Time(lhs) - rclcpp::Time(rhs)).seconds());
}

double receiptSkew(const rclcpp::Time& lhs, const rclcpp::Time& rhs) {
  return std::abs((lhs - rhs).seconds());
}

void wgs84DeltaMeters(const sensor_msgs::msg::NavSatFix& from,
                      const sensor_msgs::msg::NavSatFix& to,
                      double& east_m, double& north_m) {
  const double from_lat = from.latitude * M_PI / 180.0;
  const double to_lat = to.latitude * M_PI / 180.0;
  const double mean_lat = (from_lat + to_lat) * 0.5;
  double delta_lon = (to.longitude - from.longitude) * M_PI / 180.0;
  if (delta_lon > M_PI) delta_lon -= 2.0 * M_PI;
  if (delta_lon < -M_PI) delta_lon += 2.0 * M_PI;
  const double sin_lat = std::sin(mean_lat);
  const double denominator =
      std::sqrt(1.0 - kWgs84EccentricitySquared * sin_lat * sin_lat);
  const double radius_n = kWgs84SemiMajorM / denominator;
  const double radius_m = kWgs84SemiMajorM *
                          (1.0 - kWgs84EccentricitySquared) /
                          (denominator * denominator * denominator);
  east_m = radius_n * std::cos(mean_lat) * delta_lon;
  north_m = radius_m * (to_lat - from_lat);
}

}  // namespace

SurfaceAcquireGpsAnchor::SurfaceAcquireGpsAnchor(
    const std::string& name, const BT::NodeConfig& config,
    const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus SurfaceAcquireGpsAnchor::onStart() {
  start_time_ = node_->now();
  readiness_achieved_ = false;
  bt_nodes::common::publishZeroVelocity(ctx_, start_time_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SurfaceAcquireGpsAnchor::onRunning() {
  const rclcpp::Time now = node_->now();
  bt_nodes::common::publishZeroVelocity(ctx_, now);

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->has_task2_surface_anchor) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  if ((now - start_time_).seconds() >= node_->get_parameter("task2_gps_anchor_timeout_s").as_double()) {
    RCLCPP_ERROR(node_->get_logger(), "SurfaceAcquireGpsAnchor: timeout");
    return BT::NodeStatus::FAILURE;
  }

  const double surface_depth = node_->get_parameter("task2_gps_surface_depth").as_double();
  const double surface_tolerance =
      node_->get_parameter("task2_gps_surface_tolerance").as_double();
  const double pose_timeout = node_->get_parameter("task2_pose_timeout_s").as_double();
  const double state_timeout = node_->get_parameter("task2_state_timeout_s").as_double();

  geometry_msgs::msg::PoseStamped pose;
  mavros_msgs::msg::State state;
  sensor_msgs::msg::NavSatFix buoy_fix;
  sensor_msgs::msg::NavSatFix auv_fix;
  rclcpp::Time pose_received;
  rclcpp::Time state_received;
  rclcpp::Time buoy_received;
  rclcpp::Time auv_received;
  bool has_buoy_fix = false;
  bool has_auv_fix = false;
  bool ready = false;
  bool observations_present = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    observations_present = ctx_->has_pose || ctx_->has_state;
    pose = ctx_->current_pose;
    state = ctx_->state;
    pose_received = ctx_->last_pose_received;
    state_received = ctx_->last_state_received;
    buoy_fix = ctx_->latest_task2_buoy_fix;
    auv_fix = ctx_->latest_task2_auv_fix;
    buoy_received = ctx_->last_task2_buoy_fix_received;
    auv_received = ctx_->last_task2_auv_fix_received;
    has_buoy_fix = buoy_received.nanoseconds() != 0;
    has_auv_fix = auv_received.nanoseconds() != 0;
    ready = ctx_->has_pose && receiptFresh(now, pose_received, pose_timeout) &&
            ctx_->has_state && receiptFresh(now, state_received, state_timeout) &&
            state.connected && state.armed && state.mode == "GUIDED" &&
            bt_nodes::task2_common::finitePose(pose);
  }
  if (!ready) {
    if (readiness_achieved_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "SurfaceAcquireGpsAnchor: pose/state readiness lost");
      return BT::NodeStatus::FAILURE;
    }
    if (observations_present) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "SurfaceAcquireGpsAnchor: waiting for GUIDED vehicle readiness");
    }
    return BT::NodeStatus::RUNNING;
  }
  readiness_achieved_ = true;

  const double dz = surface_depth - pose.pose.position.z;
  if (std::abs(dz) > surface_tolerance) {
    const double speed = std::clamp(
        node_->get_parameter("nav_kp_v").as_double() * dz,
        -node_->get_parameter("nav_max_speed_v").as_double(),
        node_->get_parameter("nav_max_speed_v").as_double());
    ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, speed, now));
    return BT::NodeStatus::RUNNING;
  }

  const double fix_timeout = node_->get_parameter("task2_gps_fix_timeout_s").as_double();
  const double max_skew =
      node_->get_parameter("task2_gps_max_fix_pose_skew_s").as_double();
  const double receipt_pair_timeout =
      node_->get_parameter("task2_gps_receipt_pair_timeout_s").as_double();
  const double max_range = node_->get_parameter("task2_gps_max_buoy_range_m").as_double();
  const double max_covariance =
      node_->get_parameter("task2_gps_max_horizontal_covariance_m2").as_double();
  const bool allow_zero_source_stamp =
      node_->get_parameter("task2_fixture_mode").as_bool();
  const bool buoy_stamp_available = sourceStampAvailable(buoy_fix.header.stamp);
  const bool auv_stamp_available = sourceStampAvailable(auv_fix.header.stamp);
  const bool pose_stamp_available = sourceStampAvailable(pose.header.stamp);
  const bool buoy_fresh =
      has_buoy_fix &&
      sourceOrReceiptFresh(now, buoy_fix.header.stamp, buoy_received, fix_timeout,
                           allow_zero_source_stamp) &&
      validFix(buoy_fix, max_covariance);
  const bool auv_fresh =
      has_auv_fix &&
      sourceOrReceiptFresh(now, auv_fix.header.stamp, auv_received, fix_timeout,
                           allow_zero_source_stamp) &&
      validFix(auv_fix, max_covariance);
  const bool pose_fresh =
      receiptFresh(now, pose_received, pose_timeout) &&
      bt_nodes::task2_common::finitePose(pose) && pose.header.frame_id == "map" &&
      (pose_stamp_available ? stampFresh(now, pose.header.stamp, pose_timeout)
                            : allow_zero_source_stamp);

  const double auv_buoy_receipt_limit =
      auv_stamp_available && buoy_stamp_available ? max_skew : receipt_pair_timeout;
  const double auv_pose_receipt_limit =
      auv_stamp_available && pose_stamp_available ? max_skew : receipt_pair_timeout;

  if (!buoy_fresh || !auv_fresh || !pose_fresh ||
      (auv_stamp_available && pose_stamp_available &&
       stampSkew(auv_fix.header.stamp, pose.header.stamp) > max_skew) ||
      (buoy_stamp_available && auv_stamp_available &&
       stampSkew(buoy_fix.header.stamp, auv_fix.header.stamp) > max_skew) ||
      receiptSkew(auv_received, buoy_received) > auv_buoy_receipt_limit ||
      receiptSkew(auv_received, pose_received) > auv_pose_receipt_limit) {
    return BT::NodeStatus::RUNNING;
  }

  double east_m = 0.0;
  double north_m = 0.0;
  wgs84DeltaMeters(auv_fix, buoy_fix, east_m, north_m);
  if (!std::isfinite(east_m) || !std::isfinite(north_m) ||
      std::hypot(east_m, north_m) > max_range) {
    RCLCPP_ERROR(node_->get_logger(), "SurfaceAcquireGpsAnchor: buoy fix is out of range");
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (!ctx_->has_task2_surface_anchor) {
      ctx_->task2_anchor_buoy_fix = buoy_fix;
      ctx_->task2_anchor_auv_fix = auv_fix;
      ctx_->task2_anchor_map_pose = pose;
      ctx_->task2_target_map_pose = pose.pose.position;
      ctx_->task2_target_map_pose.x += east_m;
      ctx_->task2_target_map_pose.y += north_m;
      ctx_->task2_target_map_pose.z = surface_depth;
      ctx_->has_task2_surface_anchor = true;
    }
  }
  bt_nodes::common::publishZeroVelocity(ctx_, now);
  RCLCPP_INFO(node_->get_logger(), "SurfaceAcquireGpsAnchor: latched surface anchor");
  return BT::NodeStatus::SUCCESS;
}

void SurfaceAcquireGpsAnchor::onHalted() {
  bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
