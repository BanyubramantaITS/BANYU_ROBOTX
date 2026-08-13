#include "banyu_mission/bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace banyu_mission {

namespace {
double norm2(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  // ROS ENU convention: yaw about the z axis.
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapToPi(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void publishZeroVelocity(const std::shared_ptr<MissionContext>& ctx,
                         const rclcpp::Time& stamp) {
  if (ctx->setpoint_pub) {
    ctx->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, stamp));
  }
}

void publishZeroBodyVelocity(const std::shared_ptr<MissionContext>& ctx,
                             const rclcpp::Time& stamp) {
  if (ctx->setpoint_pub) {
    ctx->setpoint_pub->publish(bodyVelocitySetpoint(0.0, 0.0, 0.0, stamp));
  }
}
}  // namespace

AbortedOrTeleop::AbortedOrTeleop(const std::string& name, const BT::NodeConfig& config,
                                 std::shared_ptr<MissionContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus AbortedOrTeleop::tick() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

MissionActive::MissionActive(const std::string& name, const BT::NodeConfig& config,
                             std::shared_ptr<MissionContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus MissionActive::tick() {
  return ctx_->mission_active.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

NavToGate::NavToGate(const std::string& name, const BT::NodeConfig& config,
                     const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus NavToGate::onStart() {
  start_time_ = node_->now();

  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  if (!ctx_->mission_origin) {
    ctx_->mission_origin = ctx_->current_pose.pose.position;
  }
  const auto& origin = *ctx_->mission_origin;
  geometry_msgs::msg::Point target;
  // Parameters are expressed as NED mission offsets, while MAVROS local
  // pose/setpoint fields use ROS ENU: east -> x, north -> y, down -> -z.
  target.x = origin.x + node_->get_parameter("gate_offset_east").as_double();
  target.y = origin.y + node_->get_parameter("gate_offset_north").as_double();
  target.z = origin.z - node_->get_parameter("gate_offset_down").as_double();
  ctx_->gate_target = target;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "NavToGate: origin=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
                       origin.x, origin.y, origin.z, target.x, target.y, target.z);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavToGate::onRunning() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroVelocity(ctx_, node_->now());
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }

  if (!ctx_->gate_target) {
    return BT::NodeStatus::RUNNING;
  }

  // Closed-loop navigation with velocity-only setpoints: ArduSub's GUIDED
  // velocity controller is reliable in this sim, while its AC_WPNav position
  // controller crawls and lets depth drift. Velocity components are ROS ENU.
  const auto& target = *ctx_->gate_target;
  const double dx = target.x - pose.pose.position.x;
  const double dy = target.y - pose.pose.position.y;
  const double dz = target.z - pose.pose.position.z;

  if (norm2(pose.pose.position, target) < node_->get_parameter("arrival_tolerance").as_double()) {
    publishZeroVelocity(ctx_, node_->now());
    // Throttled: ReactiveSequence re-ticks this node every cycle while the
    // vehicle stays at the target.
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "NavToGate: arrived at gate");
    return BT::NodeStatus::SUCCESS;
  }

  const double kp_h = node_->get_parameter("nav_kp_h").as_double();
  const double kp_v = node_->get_parameter("nav_kp_v").as_double();
  const double max_h = node_->get_parameter("nav_max_speed_h").as_double();
  const double max_v = node_->get_parameter("nav_max_speed_v").as_double();

  const double vx = std::clamp(kp_h * dx, -max_h, max_h);
  const double vy = std::clamp(kp_h * dy, -max_h, max_h);
  const double vz = std::clamp(kp_v * dz, -max_v, max_v);

  ctx_->setpoint_pub->publish(velocitySetpoint(vx, vy, vz, node_->now()));

  if ((node_->now() - start_time_).seconds() > node_->get_parameter("nav_timeout").as_double()) {
    RCLCPP_ERROR(node_->get_logger(), "NavToGate: timeout reaching gate");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void NavToGate::onHalted() {
  publishZeroVelocity(ctx_, node_->now());
}

VisualAlign::VisualAlign(const std::string& name, const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx,
                         int object_id, const std::string& prefix)
    : BT::StatefulActionNode(name, config),
      node_(node),
      ctx_(std::move(ctx)),
      object_id_(object_id),
      prefix_(prefix) {}

double VisualAlign::param(const std::string& name) const {
  return node_->get_parameter(prefix_ + name).as_double();
}

BT::NodeStatus VisualAlign::onStart() {
  start_time_ = node_->now();
  alignment_started_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VisualAlign::onRunning() {
  const rclcpp::Time now = node_->now();
  const char* label = object_id_ == 0 ? "GateAlign" : "MarkerAlign";

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  if ((now - start_time_).seconds() > param("timeout")) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "%s: timeout", label);
    return BT::NodeStatus::FAILURE;
  }

  if (!detectionIsFresh(*ctx_, now, node_->get_parameter("detection_timeout_s").as_double(), object_id_)) {
    // Never command from an old image. Hold safely while waiting for a fresh
    // detection; the action timeout handles a permanently lost target.
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "%s: waiting for fresh detection", label);
    return BT::NodeStatus::RUNNING;
  }

  banyu_msgs::msg::Detection detection;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    detection = ctx_->latest_detection;
  }

  if (!std::isfinite(detection.offset_x) || !std::isfinite(detection.offset_y)) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::RUNNING;
  }

  const bool aligned = std::abs(detection.offset_x) <= param("deadband_x") &&
                       std::abs(detection.offset_y) <= param("deadband_y");

  if (aligned) {
    publishZeroBodyVelocity(ctx_, now);
    if (!alignment_started_) {
      aligned_since_ = now;
      alignment_started_ = true;
    }

    if ((now - aligned_since_).seconds() >= param("hold_time")) {
      RCLCPP_INFO(node_->get_logger(), "%s: object centered", label);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  alignment_started_ = false;

  const double lateral_speed = std::clamp(
      param("gain_x") * static_cast<double>(detection.offset_x),
      -param("max_speed_y"), param("max_speed_y"));
  const double vertical_speed = std::clamp(
      param("gain_y") * static_cast<double>(detection.offset_y),
      -param("max_speed_z"), param("max_speed_z"));

  // Assumes the camera is aligned with base_link: image right maps to body
  // right, and image down maps to body down. The helper converts these
  // physical body-NED commands to the ROS base_link convention.
  if (ctx_->setpoint_pub) {
    ctx_->setpoint_pub->publish(bodyVelocitySetpoint(0.0, lateral_speed, vertical_speed, now));
  }
  return BT::NodeStatus::RUNNING;
}

void VisualAlign::onHalted() {
  publishZeroBodyVelocity(ctx_, node_->now());
}

GatePass::GatePass(const std::string& name, const BT::NodeConfig& config,
                   const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus GatePass::onStart() {
  start_time_ = node_->now();
  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  start_pose_ = ctx_->current_pose.pose.position;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GatePass::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }

  const double travelled = norm2(pose.pose.position, start_pose_);
  const double target_distance = node_->get_parameter("gate_pass_distance").as_double();
  const double min_distance = node_->get_parameter("gate_pass_min_distance").as_double();

  // Through once the required distance is covered, or once the gate leaves
  // the camera after a minimum distance (the sub is inside/past it).
  const bool gate_lost = travelled >= min_distance &&
                         !detectionIsFresh(*ctx_, now,
                                           node_->get_parameter("detection_timeout_s").as_double(), 0);
  if (travelled >= target_distance || gate_lost) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "GatePass: through the gate (%.2f m)", travelled);
    return BT::NodeStatus::SUCCESS;
  }

  ctx_->setpoint_pub->publish(
      bodyVelocitySetpoint(node_->get_parameter("gate_pass_speed").as_double(), 0.0, 0.0, now));

  if ((now - start_time_).seconds() > node_->get_parameter("gate_pass_timeout").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "GatePass: timeout (travelled %.2f m)", travelled);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void GatePass::onHalted() {
  publishZeroBodyVelocity(ctx_, node_->now());
}

SearchMarker::SearchMarker(const std::string& name, const BT::NodeConfig& config,
                           const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus SearchMarker::onStart() {
  start_time_ = node_->now();
  seen_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchMarker::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  const double detection_timeout = node_->get_parameter("detection_timeout_s").as_double();
  const double approach_distance = node_->get_parameter("search_approach_distance").as_double();
  const double min_distance = node_->get_parameter("search_min_distance").as_double();

  if (detectionIsFresh(*ctx_, now, detection_timeout, 1)) {
    seen_ = true;

    banyu_msgs::msg::Detection detection;
    {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      detection = ctx_->latest_detection;
    }

    const double distance = static_cast<double>(detection.distance);
    if (distance > 0.0f) {
      if (distance < min_distance) {
        publishZeroBodyVelocity(ctx_, now);
        RCLCPP_ERROR(node_->get_logger(), "SearchMarker: too close (%.2f m), aborting", distance);
        return BT::NodeStatus::FAILURE;
      }
      if (distance <= approach_distance) {
        publishZeroBodyVelocity(ctx_, now);
        RCLCPP_INFO(node_->get_logger(), "SearchMarker: approach complete at %.2f m", distance);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // Approach while keeping the buoy centered: forward + lateral/vertical
    // corrections from the image offsets (same convention as MarkerAlign).
    const double lateral = std::clamp(
        node_->get_parameter("marker_align_gain_x").as_double() *
            static_cast<double>(detection.offset_x),
        -node_->get_parameter("marker_align_max_speed_y").as_double(),
        node_->get_parameter("marker_align_max_speed_y").as_double());
    const double vertical = std::clamp(
        node_->get_parameter("marker_align_gain_y").as_double() *
            static_cast<double>(detection.offset_y),
        -node_->get_parameter("marker_align_max_speed_z").as_double(),
        node_->get_parameter("marker_align_max_speed_z").as_double());

    ctx_->setpoint_pub->publish(bodyVelocitySetpoint(
        node_->get_parameter("search_speed").as_double(), lateral, vertical, now));
  } else if (seen_) {
    // Lost the buoy mid-approach: hold position and wait for re-acquisition
    // rather than driving forward without a range estimate.
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "SearchMarker: waiting for re-acquisition");
  } else {
    // Initial sweep: slow forward drive while scanning for the yellow buoy.
    ctx_->setpoint_pub->publish(
        bodyVelocitySetpoint(node_->get_parameter("search_speed").as_double(), 0.0, 0.0, now));
  }

  if ((now - start_time_).seconds() > node_->get_parameter("search_timeout").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "SearchMarker: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SearchMarker::onHalted() {
  publishZeroBodyVelocity(ctx_, node_->now());
}

MarkerOrbit::MarkerOrbit(const std::string& name, const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus MarkerOrbit::onStart() {
  start_time_ = node_->now();
  accumulated_yaw_ = 0.0;
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  start_yaw_ = yawFromQuaternion(pose.pose.orientation);
  last_yaw_ = start_yaw_;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MarkerOrbit::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  // Track the integrated yaw rotation: the orbit completes after a full
  // revolution, regardless of radius or speed.
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  const double yaw = yawFromQuaternion(pose.pose.orientation);
  accumulated_yaw_ += wrapToPi(yaw - last_yaw_);
  last_yaw_ = yaw;

  if (std::fabs(accumulated_yaw_) >= node_->get_parameter("orbit_angle_rad").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "MarkerOrbit: full revolution (%.1f deg)", accumulated_yaw_ * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }

  if ((now - start_time_).seconds() > node_->get_parameter("orbit_timeout").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "MarkerOrbit: timeout (rotated %.1f deg)",
                 accumulated_yaw_ * 180.0 / M_PI);
    return BT::NodeStatus::FAILURE;
  }

  banyu_msgs::msg::Detection detection;
  bool have_detection = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->has_detection && ctx_->latest_detection.object_id == 1) {
      const double age = (now - ctx_->last_detection_received).seconds();
      if (age >= 0.0 && age <= node_->get_parameter("detection_timeout_s").as_double()) {
        detection = ctx_->latest_detection;
        have_detection = true;
      }
    }
  }

  if (!have_detection) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "MarkerOrbit: waiting for re-acquisition");
    return BT::NodeStatus::RUNNING;
  }

  // Strafe orbit: body-y is the circle speed, body-x locks the standoff
  // range, and yaw rate keeps the buoy centered in the camera while circling.
  const double distance = static_cast<double>(detection.distance);
  const double orbit_radius = node_->get_parameter("orbit_radius_m").as_double();
  const double forward = distance > 0.0f
                             ? std::clamp(node_->get_parameter("orbit_kp_range").as_double() *
                                              (distance - orbit_radius),
                                          -node_->get_parameter("orbit_max_range_speed").as_double(),
                                          node_->get_parameter("orbit_max_range_speed").as_double())
                             : 0.0;
  const double lateral = node_->get_parameter("orbit_speed").as_double();
  const double yaw_rate = std::clamp(
      node_->get_parameter("orbit_kp_yaw").as_double() * static_cast<double>(detection.offset_x),
      -node_->get_parameter("orbit_max_yaw_rate").as_double(),
      node_->get_parameter("orbit_max_yaw_rate").as_double());
  const double vertical = std::clamp(
      node_->get_parameter("orbit_kp_vertical").as_double() * static_cast<double>(detection.offset_y),
      -node_->get_parameter("orbit_max_vertical").as_double(),
      node_->get_parameter("orbit_max_vertical").as_double());

  ctx_->setpoint_pub->publish(bodyVelocityYawRateSetpoint(forward, lateral, vertical, yaw_rate, now));
  return BT::NodeStatus::RUNNING;
}

void MarkerOrbit::onHalted() {
  publishZeroBodyVelocity(ctx_, node_->now());
}

TurnAround::TurnAround(const std::string& name, const BT::NodeConfig& config,
                       const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus TurnAround::onStart() {
  start_time_ = node_->now();
  accumulated_yaw_ = 0.0;
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  start_yaw_ = yawFromQuaternion(pose.pose.orientation);
  last_yaw_ = start_yaw_;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnAround::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  const double yaw = yawFromQuaternion(pose.pose.orientation);
  accumulated_yaw_ += wrapToPi(yaw - last_yaw_);
  last_yaw_ = yaw;

  // Phase A: rotate at turn_yaw_rate until the gate enters the camera.
  // Phase B (capture): once sighted, close the loop on offset_x so the
  // rotation stops exactly when the gate is centered — no momentum overshoot
  // that swings the gate out of frame. 180 deg remains only the cap.
  banyu_msgs::msg::Detection detection;
  const bool have_gate =
      detectionIsFresh(*ctx_, now, node_->get_parameter("detection_timeout_s").as_double(), 0);
  if (have_gate) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    detection = ctx_->latest_detection;
  }

  if (have_gate && std::isfinite(detection.offset_x)) {
    const double deadband = node_->get_parameter("turn_capture_deadband").as_double();
    if (std::abs(detection.offset_x) <= deadband) {
      publishZeroBodyVelocity(ctx_, now);
      RCLCPP_INFO(node_->get_logger(), "TurnAround: aligned with gate after %.1f deg",
                  accumulated_yaw_ * 180.0 / M_PI);
      return BT::NodeStatus::SUCCESS;
    }

    // Same sign convention as the MarkerOrbit yaw-follow (which converges in
    // sim): yaw_rate = +kp*offset_x, offset_x>0 means the gate is to the
    // right, so the sub turns right to center it.
    const double max_rate = node_->get_parameter("turn_yaw_rate").as_double();
    const double yaw_cmd = std::clamp(
        node_->get_parameter("turn_capture_gain").as_double() * static_cast<double>(detection.offset_x),
        -max_rate, max_rate);
    ctx_->setpoint_pub->publish(bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, yaw_cmd, now));
  } else {
    // Keep rotating until the gate appears.
    ctx_->setpoint_pub->publish(
        bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, node_->get_parameter("turn_yaw_rate").as_double(), now));
  }

  if (std::fabs(accumulated_yaw_) >= node_->get_parameter("turn_angle_rad").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "TurnAround: turned %.1f deg (cap)", accumulated_yaw_ * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }

  if ((now - start_time_).seconds() > node_->get_parameter("turn_timeout").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "TurnAround: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TurnAround::onHalted() {
  publishZeroBodyVelocity(ctx_, node_->now());
}

SearchGate::SearchGate(const std::string& name, const BT::NodeConfig& config,
                       const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus SearchGate::onStart() {
  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchGate::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  if (detectionIsFresh(*ctx_, now, node_->get_parameter("detection_timeout_s").as_double(), 0)) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "SearchGate: gate found");
    return BT::NodeStatus::SUCCESS;
  }

  // Slow forward sweep while scanning for the gate.
  ctx_->setpoint_pub->publish(
      bodyVelocitySetpoint(node_->get_parameter("search_speed").as_double(), 0.0, 0.0, now));

  if ((now - start_time_).seconds() > node_->get_parameter("search_timeout").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "SearchGate: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SearchGate::onHalted() {
  publishZeroBodyVelocity(ctx_, node_->now());
}

ReturnGatePass::ReturnGatePass(const std::string& name, const BT::NodeConfig& config,
                               const rclcpp::Node::SharedPtr& node,
                               std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus ReturnGatePass::onStart() {
  start_time_ = node_->now();
  min_distance_ = std::numeric_limits<double>::infinity();
  after_loss_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnGatePass::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }

  const double detection_timeout = node_->get_parameter("detection_timeout_s").as_double();
  const double lost_distance = node_->get_parameter("gate_pass_lost_distance").as_double();

  if (after_loss_) {
    // Blind final push to clear the gate plane after the gate left the
    // frame. Push distance is adapted to where detection was lost plus the
    // vehicle half-length/margin, so the rear clears the gate too.
    const double travelled = norm2(pose.pose.position, loss_start_pose_);
    const double push_distance = loss_distance_ + node_->get_parameter("gate_pass_clearance_m").as_double();
    if (travelled >= push_distance) {
      publishZeroBodyVelocity(ctx_, now);
      RCLCPP_INFO(node_->get_logger(), "ReturnGatePass: through the gate (%.2f m past loss)", travelled);
      return BT::NodeStatus::SUCCESS;
    }
    ctx_->setpoint_pub->publish(
        bodyVelocitySetpoint(node_->get_parameter("gate_pass_speed").as_double(), 0.0, 0.0, now));
  } else if (detectionIsFresh(*ctx_, now, detection_timeout, 0)) {
    banyu_msgs::msg::Detection detection;
    {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      detection = ctx_->latest_detection;
    }

    const double distance = static_cast<double>(detection.distance);
    if (distance > 0.0f) {
      min_distance_ = std::min(min_distance_, distance);
      // Reached the gate while still seeing it: done (safety net).
      if (min_distance_ <= node_->get_parameter("gate_pass_min_distance").as_double()) {
        publishZeroBodyVelocity(ctx_, now);
        RCLCPP_INFO(node_->get_logger(), "ReturnGatePass: at gate (%.2f m)", min_distance_);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // Drive forward while keeping the gate centered in the camera.
    const double lateral = std::clamp(
        node_->get_parameter("gate_align_gain_x").as_double() * static_cast<double>(detection.offset_x),
        -node_->get_parameter("gate_align_max_speed_y").as_double(),
        node_->get_parameter("gate_align_max_speed_y").as_double());
    const double vertical = std::clamp(
        node_->get_parameter("gate_align_gain_y").as_double() * static_cast<double>(detection.offset_y),
        -node_->get_parameter("gate_align_max_speed_z").as_double(),
        node_->get_parameter("gate_align_max_speed_z").as_double());

    ctx_->setpoint_pub->publish(bodyVelocitySetpoint(
        node_->get_parameter("gate_pass_speed").as_double(), lateral, vertical, now));
  } else if (min_distance_ <= lost_distance) {
    // Gate was seen close and is now out of frame: start the blind push.
    after_loss_ = true;
    loss_distance_ = min_distance_;
    loss_start_pose_ = pose.pose.position;
    RCLCPP_INFO(node_->get_logger(), "ReturnGatePass: gate lost at %.2f m, pushing through",
                min_distance_);
  } else {
    // Not seen close yet and no detection: hold and wait for re-acquisition.
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "ReturnGatePass: waiting for re-acquisition");
  }

  if ((now - start_time_).seconds() > node_->get_parameter("gate_pass_timeout").as_double()) {
    publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "ReturnGatePass: timeout (min distance %.2f m)", min_distance_);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void ReturnGatePass::onHalted() {
  publishZeroBodyVelocity(ctx_, node_->now());
}

Surface::Surface(const std::string& name, const BT::NodeConfig& config,
                 const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus Surface::onStart() {
  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Surface::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    publishZeroVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }

  const double surface_depth = node_->get_parameter("surface_depth").as_double();
  const double surface_tolerance = node_->get_parameter("surface_tolerance").as_double();

  if (std::abs(pose.pose.position.z - surface_depth) < surface_tolerance) {
    publishZeroVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "Surface: at surface, mission complete");
    return BT::NodeStatus::SUCCESS;
  }

  const double kp_v = node_->get_parameter("nav_kp_v").as_double();
  const double max_v = node_->get_parameter("nav_max_speed_v").as_double();
  const double dz = surface_depth - pose.pose.position.z;
  ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, std::clamp(kp_v * dz, -max_v, max_v), now));

  if ((now - start_time_).seconds() > node_->get_parameter("surface_timeout").as_double()) {
    publishZeroVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "Surface: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void Surface::onHalted() {
  publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
