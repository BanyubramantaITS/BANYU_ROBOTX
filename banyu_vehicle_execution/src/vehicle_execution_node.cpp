#include "banyu_vehicle_execution/vehicle_execution_node.hpp"

namespace banyu_vehicle_execution
{

VehicleExecutionNode::VehicleExecutionNode(const rclcpp::NodeOptions & options)
: Node("banyu_vehicle_execution", options),
  arbiter_(steady_clock_, 2)
{
  initParameters();
  initSubscribers();
  initPublishers();
  initActionServers();

  const double publish_rate_hz = declare_parameter<double>("control.publish_rate_hz", 20.0);
  const double safe_rate_hz = (std::isfinite(publish_rate_hz) && publish_rate_hz > 0.0) ?
    publish_rate_hz : 20.0;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_rate_hz));

  last_tick_time_ = now();
  loop_timer_ = create_wall_timer(
    period, std::bind(&VehicleExecutionNode::timerCallback, this));

  RCLCPP_INFO(get_logger(), "Vehicle execution node initialized at %.1f Hz", safe_rate_hz);
}

void VehicleExecutionNode::initParameters()
{
  required_zero_cycles_ = declare_parameter<int64_t>("control.required_zero_cycles", 2);
  require_lease_ = declare_parameter<bool>("control.require_lease", true);
  lease_timeout_s_ = declare_parameter<double>("control.lease_timeout_s", 1.5);
  enable_mavros_publish_ = declare_parameter<bool>("control.enable_mavros_publish", false);
  require_guided_ = declare_parameter<bool>("control.require_guided", true);

  const double max_linear = declare_parameter<double>("control.max_linear_speed", 2.0);
  const double max_yaw = declare_parameter<double>("control.max_yaw_rate", 1.0);
  limits_ = IntentLimits(max_linear, max_yaw);
}

void VehicleExecutionNode::initSubscribers()
{
  const auto status_topic = declare_parameter<std::string>(
    "topics.status", "/banyu/vehicle_status");
  const auto pose_topic = declare_parameter<std::string>(
    "topics.pose", "/mavros/local_position/pose");
  const auto gps_topic = declare_parameter<std::string>(
    "topics.gps", "/mavros/global_position/raw/fix");
  const auto rel_alt_topic = declare_parameter<std::string>(
    "topics.rel_alt", "/mavros/global_position/rel_alt");
  const auto pipeline_obs_topic = declare_parameter<std::string>(
    "topics.pipeline_obs", "/banyu/pipeline/observation");
  const auto ring_obs_topic = declare_parameter<std::string>(
    "topics.ring_obs", "/banyu/pipeline/ring_observation");
  const auto guidance_intent_topic = declare_parameter<std::string>(
    "topics.guidance_intent", "/banyu/guidance/velocity_intent");
  const auto lease_topic = declare_parameter<std::string>(
    "topics.lease", "/banyu/execution/lease");

  status_sub_ = create_subscription<banyu_msgs::msg::VehicleStatus>(
    status_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleStatus, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handlePose, this, std::placeholders::_1));

  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleGps, this, std::placeholders::_1));

  rel_alt_sub_ = create_subscription<std_msgs::msg::Float64>(
    rel_alt_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleRelAlt, this, std::placeholders::_1));

  pipeline_obs_sub_ = create_subscription<banyu_msgs::msg::PipelineObservation>(
    pipeline_obs_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handlePipelineObs, this, std::placeholders::_1));

  ring_obs_sub_ = create_subscription<banyu_msgs::msg::PipelineRingObservation>(
    ring_obs_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handleRingObs, this, std::placeholders::_1));

  guidance_intent_sub_ = create_subscription<banyu_guidance::msg::VelocityIntent>(
    guidance_intent_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleGuidanceIntent, this, std::placeholders::_1));

  lease_sub_ = create_subscription<banyu_msgs::msg::ActionLease>(
    lease_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleLease, this, std::placeholders::_1));
}

void VehicleExecutionNode::initPublishers()
{
  const auto preview_topic = declare_parameter<std::string>(
    "topics.preview_target", "/banyu/execution/preview_target");
  preview_target_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
    preview_topic, rclcpp::QoS(10));

  if (enable_mavros_publish_) {
    const auto mavros_topic = declare_parameter<std::string>(
      "topics.mavros_target", "/mavros/setpoint_raw/local");
    mavros_target_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      mavros_topic, rclcpp::QoS(10));
  }
}

void VehicleExecutionNode::initActionServers()
{
  const auto nav_action = declare_parameter<std::string>(
    "actions.navigate_surface_gps", "/banyu/actions/navigate_surface_gps");
  const auto align_action = declare_parameter<std::string>(
    "actions.align_heading", "/banyu/actions/align_heading");
  const auto dive_action = declare_parameter<std::string>(
    "actions.dive_to_depth", "/banyu/actions/dive_to_depth");
  const auto acquire_action = declare_parameter<std::string>(
    "actions.acquire_pipeline", "/banyu/actions/acquire_pipeline");
  const auto search_action = declare_parameter<std::string>(
    "actions.search_pipeline", "/banyu/actions/search_pipeline");
  const auto follow_action = declare_parameter<std::string>(
    "actions.follow_pipeline", "/banyu/actions/follow_pipeline");
  const auto center_action = declare_parameter<std::string>(
    "actions.center_on_ring", "/banyu/actions/center_on_ring");
  const auto hold_action = declare_parameter<std::string>(
    "actions.hold_probe_on_ring", "/banyu/actions/hold_probe_on_ring");
  const auto surface_action = declare_parameter<std::string>(
    "actions.surface", "/banyu/actions/surface");

  const double nav_kp = declare_parameter<double>("control.nav_kp", 0.5);
  const double nav_max_speed = declare_parameter<double>("control.nav_max_speed", 1.0);
  const double yaw_kp = declare_parameter<double>("control.yaw_kp", 1.0);
  const double max_yaw_rate = get_parameter("control.max_yaw_rate").as_double();
  const double depth_kp = declare_parameter<double>("control.depth_kp", 0.5);
  const double max_depth_rate = declare_parameter<double>("control.max_depth_rate", 0.5);
  const double ring_kp_sway = declare_parameter<double>("control.ring_kp_sway", 0.8);
  const double ring_kp_z = declare_parameter<double>("control.ring_kp_z", 0.5);

  auto nav_server = std::make_shared<NavigateSurfaceGpsServer>(
    this, nav_action, this, nav_kp, nav_max_speed, yaw_kp, max_yaw_rate);
  auto align_server = std::make_shared<AlignHeadingServer>(
    this, align_action, this, yaw_kp, max_yaw_rate);
  auto dive_server = std::make_shared<DiveToDepthServer>(
    this, dive_action, this, depth_kp, max_depth_rate, yaw_kp, max_yaw_rate);
  auto acquire_server = std::make_shared<AcquirePipelineServer>(
    this, acquire_action, this, yaw_kp, max_yaw_rate);
  auto search_server = std::make_shared<SearchPipelineServer>(
    this, search_action, this);
  auto follow_server = std::make_shared<FollowPipelineServer>(
    this, follow_action, this);
  auto center_server = std::make_shared<CenterOnRingServer>(
    this, center_action, this, ring_kp_sway, ring_kp_z);
  auto hold_server = std::make_shared<HoldProbeOnRingServer>(
    this, hold_action, this);
  auto surface_server = std::make_shared<SurfaceServer>(
    this, surface_action, this, depth_kp, max_depth_rate);

  servers_ = {
    nav_server, align_server, dive_server,
    acquire_server, search_server, follow_server,
    center_server, hold_server, surface_server
  };

  for (const auto & server : servers_) {
    server_map_[server->name()] = server;
  }
}

GoalRequest VehicleExecutionNode::requestGoal(const GoalIdentity & identity)
{
  std::optional<Lease> initial_lease = std::nullopt;
  if (!require_lease_) {
    const auto now = steady_clock_.now();
    initial_lease = Lease{identity, now, now + std::chrono::hours(24), true};
  }

  auto req = arbiter_.requestGoal(identity, initial_lease);
  if (req.admission == GoalAdmission::Pending) {
    const auto & displaced = arbiter_.displacedActive();
    if (displaced.has_value()) {
      auto it = server_map_.find(displaced->goal.owner);
      if (it != server_map_.end()) {
        it->second->abortActive("Goal displaced by " + identity.owner);
      }
    }
  }
  return req;
}

bool VehicleExecutionNode::cancelGoal(const GoalHandle & handle)
{
  return arbiter_.cancel(handle);
}

void VehicleExecutionNode::completeGoal(const GoalHandle & handle)
{
  arbiter_.complete(handle);
}

void VehicleExecutionNode::faultGoal(const GoalHandle & handle)
{
  arbiter_.fault(handle);
}

void VehicleExecutionNode::notifyDisplaced(
  const GoalIdentity & displaced_identity, const std::string & new_owner)
{
  auto it = server_map_.find(displaced_identity.owner);
  if (it != server_map_.end()) {
    it->second->abortActive("Goal displaced by " + new_owner);
  }
}

void VehicleExecutionNode::timerCallback()
{
  const auto current_ros_time = now();
  double dt = 0.05;
  if (last_tick_time_.nanoseconds() > 0) {
    dt = (current_ros_time - last_tick_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.05;
    }
  }
  last_tick_time_ = current_ros_time;
  telemetry_.current_time = current_ros_time;

  // Telemetry freshness watchdog
  constexpr double kPoseTimeoutS = 0.5;
  constexpr double kGpsTimeoutS = 2.0;
  constexpr double kStatusTimeoutS = 1.0;
  constexpr double kGuidanceTimeoutS = 0.5;
  constexpr double kObservationTimeoutS = 0.5;

  if (telemetry_.pose_valid &&
    (current_ros_time - telemetry_.pose_stamp).seconds() > kPoseTimeoutS)
  {
    telemetry_.pose_valid = false;
  }
  if (telemetry_.gps_valid && (current_ros_time - telemetry_.gps_stamp).seconds() > kGpsTimeoutS) {
    telemetry_.gps_valid = false;
  }
  if (telemetry_.status_valid &&
    (current_ros_time - telemetry_.status_stamp).seconds() > kStatusTimeoutS)
  {
    telemetry_.status_valid = false;
  }
  if (telemetry_.guidance_intent_valid &&
    (current_ros_time - telemetry_.guidance_intent_stamp).seconds() > kGuidanceTimeoutS)
  {
    telemetry_.guidance_intent_valid = false;
    telemetry_.guidance_surge_mps = 0.0;
    telemetry_.guidance_sway_mps = 0.0;
    telemetry_.guidance_yaw_rate_rad_s = 0.0;
  }
  if (telemetry_.pipeline_obs_valid &&
    (current_ros_time - telemetry_.pipeline_obs_stamp).seconds() > kObservationTimeoutS)
  {
    telemetry_.pipeline_obs_valid = false;
  }
  if (telemetry_.ring_obs_valid &&
    (current_ros_time - telemetry_.ring_obs_stamp).seconds() > kObservationTimeoutS)
  {
    telemetry_.ring_obs_valid = false;
  }

  // Vehicle safety check
  const bool safety_fault = (telemetry_.status_valid &&
    (telemetry_.operator_takeover || !telemetry_.fcu_connected || !telemetry_.fcu_status_fresh)) ||
    (status_ever_received_ && !telemetry_.status_valid);

  if (safety_fault) {
    arbiter_.fault();
    for (const auto & server : servers_) {
      if (server->hasActiveGoal()) {
        server->abortActive("Vehicle authority or connection fault");
      }
      if (server->hasPendingGoal()) {
        server->abortPending("Vehicle authority or connection fault");
      }
    }
  }

  // Check lease expiration
  const auto active_before_lease_check = arbiter_.activeGoal();
  arbiter_.checkLease();
  if (active_before_lease_check && !arbiter_.activeGoal() &&
    arbiter_.stopReason() == StopReason::LeaseLoss)
  {
    auto it = server_map_.find(active_before_lease_check->goal.owner);
    if (it != server_map_.end()) {
      it->second->abortActive("Action lease expired");
    }
  }

  MotionIntent intent = zeroIntent();

  if (arbiter_.state() == GoalState::ZeroBarrier) {
    const bool barrier_done = arbiter_.recordZeroPublication();
    intent = zeroIntent();
    if (barrier_done) {
      if (arbiter_.activeGoal().has_value()) {
        auto it = server_map_.find(arbiter_.activeGoal()->goal.owner);
        if (it != server_map_.end()) {
          it->second->activatePending();
        }
      } else {
        arbiter_.reset();
      }
    }
  } else if (arbiter_.activeGoal().has_value()) {
    const auto & handle = *arbiter_.activeGoal();
    auto it = server_map_.find(handle.goal.owner);
    if (it != server_map_.end()) {
      const auto requested = it->second->tick(telemetry_, current_ros_time, dt);
      intent = arbiter_.selectOutput(handle, requested);
    } else {
      intent = zeroIntent();
    }
  } else {
    intent = zeroIntent();
  }

  // Encode to PositionTarget and publish to preview sink
  try {
    auto target = encodeVelocityYawRate(intent, limits_);
    target.header.stamp = current_ros_time;
    target.header.frame_id = "map";
    preview_target_pub_->publish(target);

    if (enable_mavros_publish_ && mavros_target_pub_) {
      const bool takeover = telemetry_.status_valid && telemetry_.operator_takeover;
      const bool status_ready = telemetry_.status_valid &&
        telemetry_.fcu_connected &&
        telemetry_.fcu_status_fresh &&
        telemetry_.mode == "GUIDED";

      if (!takeover && (!require_guided_ || status_ready)) {
        mavros_target_pub_->publish(target);
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Failed to encode preview setpoint: %s", ex.what());
  }
}

void VehicleExecutionNode::handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  telemetry_.pose_valid = true;
  telemetry_.pose_stamp = msg->header.stamp;
  telemetry_.pos_x = msg->pose.position.x;
  telemetry_.pos_y = msg->pose.position.y;
  telemetry_.pos_z = msg->pose.position.z;
  telemetry_.yaw = getYawFromQuaternion(
    msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w);

  // If depth is not received from rel_alt, fall back to -pos_z
  if (!telemetry_.depth_valid) {
    telemetry_.depth_m = -telemetry_.pos_z;
  }
}

void VehicleExecutionNode::handleGps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  telemetry_.gps_valid = (msg->status.status >= 0);
  telemetry_.gps_stamp = msg->header.stamp;
  telemetry_.latitude_deg = msg->latitude;
  telemetry_.longitude_deg = msg->longitude;
  telemetry_.altitude_m = msg->altitude;
}

void VehicleExecutionNode::handleRelAlt(const std_msgs::msg::Float64::SharedPtr msg)
{
  telemetry_.depth_valid = true;
  telemetry_.depth_stamp = now();
  // rel_alt is negative below the water surface; depth is positive downward
  telemetry_.depth_m = -msg->data;
}

void VehicleExecutionNode::handleStatus(const banyu_msgs::msg::VehicleStatus::SharedPtr msg)
{
  status_ever_received_ = true;
  telemetry_.status_valid = true;
  telemetry_.status_stamp = msg->header.stamp;
  telemetry_.fcu_connected = msg->fcu_connected;
  telemetry_.fcu_status_fresh = msg->fcu_status_fresh;
  telemetry_.rc_fresh = msg->rc_fresh;
  telemetry_.operator_takeover = msg->operator_takeover;
  telemetry_.mode = msg->mode;
  telemetry_.authority = msg->authority;
}

void VehicleExecutionNode::handlePipelineObs(
  const banyu_msgs::msg::PipelineObservation::SharedPtr msg)
{
  telemetry_.pipeline_obs_valid = true;
  telemetry_.pipeline_obs_stamp = msg->header.stamp;
  telemetry_.pipeline_status = msg->status;
  telemetry_.pipeline_offset_x = msg->offset_x;
  telemetry_.pipeline_offset_y = msg->offset_y;
  telemetry_.pipeline_heading_error_rad = msg->heading_error_rad;
  telemetry_.pipeline_confidence = msg->confidence;
}

void VehicleExecutionNode::handleRingObs(
  const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg)
{
  telemetry_.ring_obs_valid = true;
  telemetry_.ring_obs_stamp = msg->header.stamp;
  telemetry_.ring_visible = msg->visible;
  telemetry_.ring_color = msg->color;
  telemetry_.ring_offset_x = msg->offset_x;
  telemetry_.ring_offset_y = msg->offset_y;
  telemetry_.ring_confidence = msg->confidence;
}

void VehicleExecutionNode::handleGuidanceIntent(
  const banyu_guidance::msg::VelocityIntent::SharedPtr msg)
{
  telemetry_.guidance_intent_valid = true;
  telemetry_.guidance_intent_stamp = msg->header.stamp;
  telemetry_.guidance_status = msg->status;
  telemetry_.guidance_surge_mps = msg->surge_mps;
  telemetry_.guidance_sway_mps = msg->sway_mps;
  telemetry_.guidance_yaw_rate_rad_s = msg->yaw_rate_rad_s;
  telemetry_.guidance_yaw_control_active = msg->yaw_control_active;
}

void VehicleExecutionNode::handleLease(const banyu_msgs::msg::ActionLease::SharedPtr msg)
{
  GoalIdentity identity;
  std::copy(msg->goal_id.uuid.begin(), msg->goal_id.uuid.end(), identity.uuid.begin());
  identity.owner = msg->owner;
  identity.generation = msg->generation;

  const auto now = steady_clock_.now();
  double timeout_s = lease_timeout_s_;
  if (msg->expires_at.sec > 0 || msg->expires_at.nanosec > 0) {
    const auto ros_now = this->now();
    const auto expires = rclcpp::Time(msg->expires_at, ros_now.get_clock_type());
    if (expires > ros_now) {
      timeout_s = (expires - ros_now).seconds();
    }
  }
  const auto deadline = now + std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(std::max(0.01, timeout_s)));

  const auto matches = [&identity](const GoalIdentity & target) {
      return target.uuid == identity.uuid &&
             target.generation == identity.generation &&
             (target.owner == identity.owner ||
             target.owner.find(identity.owner) != std::string::npos ||
             identity.owner.find(target.owner) != std::string::npos);
    };

  const auto & active_opt = arbiter_.activeGoal();
  if (active_opt.has_value() && matches(active_opt->goal)) {
    const auto active_handle = *active_opt;
    Lease lease{active_handle.goal, now, deadline, msg->active};
    if (msg->active) {
      arbiter_.establishLease(active_handle, lease);
    } else {
      arbiter_.leaseLost(active_handle);
      auto it = server_map_.find(active_handle.goal.owner);
      if (it != server_map_.end()) {
        it->second->abortActive("Action lease revoked");
      }
    }
  } else {
    const auto & pending_opt = arbiter_.pendingGoal();
    if (pending_opt.has_value() && matches(pending_opt->goal)) {
      const auto pending_handle = *pending_opt;
      Lease lease{pending_handle.goal, now, deadline, msg->active};
      if (msg->active) {
        arbiter_.establishLease(pending_handle, lease);
      } else {
        arbiter_.leaseLost(pending_handle);
        auto it = server_map_.find(pending_handle.goal.owner);
        if (it != server_map_.end()) {
          it->second->abortPending("Action lease revoked");
        }
      }
    }
  }
}

}  // namespace banyu_vehicle_execution
