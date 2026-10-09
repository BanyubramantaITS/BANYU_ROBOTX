#include "banyu_vehicle_execution/vehicle_execution_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
  require_run_authorization_ = declare_parameter<bool>("control.require_run_authorization", false);
  run_lifecycle_timeout_s_ = declare_parameter<double>("control.run_lifecycle_timeout_s", 1.0);
  if (!std::isfinite(run_lifecycle_timeout_s_) || run_lifecycle_timeout_s_ <= 0.0) {
    throw std::invalid_argument("control.run_lifecycle_timeout_s must be positive and finite");
  }
  require_calibrated_surface_depth_ = declare_parameter<bool>(
    "control.require_calibrated_surface_depth", false);
  surface_depth_timeout_s_ = declare_parameter<double>("control.surface_depth_timeout_s", 0.5);
  surface_depth_sources_ = declare_parameter<std::vector<std::string>>(
    "surface.depth_sources", std::vector<std::string>{"pressure_depth"});
  if (surface_depth_sources_.empty() || std::any_of(surface_depth_sources_.begin(),
    surface_depth_sources_.end(), [](const std::string & source) {return source.empty();}))
  {
    throw std::invalid_argument("surface.depth_sources must list nonempty trusted source IDs");
  }

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
  const auto water_depth_topic = declare_parameter<std::string>(
    "topics.water_depth", "/banyu/vehicle/water_depth");
  const auto pipeline_obs_topic = declare_parameter<std::string>(
    "topics.pipeline_obs", "/banyu/pipeline/observation");
  const auto ring_obs_topic = declare_parameter<std::string>(
    "topics.ring_obs", "/banyu/pipeline/ring_observation");
  const auto detection_topic = declare_parameter<std::string>(
    "topics.detection", "/banyu/detection");
  const auto guidance_intent_topic = declare_parameter<std::string>(
    "topics.guidance_intent", "/banyu/guidance/velocity_intent");
  const auto lease_topic = declare_parameter<std::string>(
    "topics.lease", "/banyu/execution/lease");
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");

  status_sub_ = create_subscription<banyu_msgs::msg::VehicleStatus>(
    status_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleStatus, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handlePose, this, std::placeholders::_1));

  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handleGps, this, std::placeholders::_1));

  rel_alt_sub_ = create_subscription<std_msgs::msg::Float64>(
    rel_alt_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handleRelAlt, this, std::placeholders::_1));
  water_depth_sub_ = create_subscription<banyu_msgs::msg::WaterDepth>(
    water_depth_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handleWaterDepth, this, std::placeholders::_1));

  pipeline_obs_sub_ = create_subscription<banyu_msgs::msg::PipelineObservation>(
    pipeline_obs_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handlePipelineObs, this, std::placeholders::_1));

  ring_obs_sub_ = create_subscription<banyu_msgs::msg::PipelineRingObservation>(
    ring_obs_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handleRingObs, this, std::placeholders::_1));

  detection_sub_ = create_subscription<banyu_msgs::msg::Detection>(
    detection_topic, rclcpp::SensorDataQoS(),
    std::bind(&VehicleExecutionNode::handleDetection, this, std::placeholders::_1));

  guidance_intent_sub_ = create_subscription<banyu_guidance::msg::VelocityIntent>(
    guidance_intent_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleGuidanceIntent, this, std::placeholders::_1));

  lease_sub_ = create_subscription<banyu_msgs::msg::ActionLease>(
    lease_topic, rclcpp::QoS(10),
    std::bind(&VehicleExecutionNode::handleLease, this, std::placeholders::_1));
  run_lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&VehicleExecutionNode::handleRunLifecycle, this, std::placeholders::_1));
}

void VehicleExecutionNode::initPublishers()
{
  const auto readiness_topic = declare_parameter<std::string>(
    "topics.execution_readiness", "/banyu/execution/readiness");
  readiness_pub_ = create_publisher<banyu_msgs::msg::ExecutionReadiness>(
    readiness_topic, rclcpp::QoS(1).reliable().transient_local());
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
  const auto acquire_gate_action = declare_parameter<std::string>(
    "actions.acquire_gate", "/banyu/actions/acquire_gate");
  const auto search_marker_action = declare_parameter<std::string>(
    "actions.search_marker", "/banyu/actions/search_marker");
  const auto search_gate_action = declare_parameter<std::string>(
    "actions.search_gate", "/banyu/actions/search_gate");
  const auto align_gate_action = declare_parameter<std::string>(
    "actions.align_gate", "/banyu/actions/align_gate");
  const auto align_marker_action = declare_parameter<std::string>(
    "actions.align_marker", "/banyu/actions/align_marker");
  const auto pass_gate_action = declare_parameter<std::string>(
    "actions.pass_gate", "/banyu/actions/pass_gate");
  const auto orbit_marker_action = declare_parameter<std::string>(
    "actions.orbit_marker", "/banyu/actions/orbit_marker");
  const auto turn_around_action = declare_parameter<std::string>(
    "actions.turn_around", "/banyu/actions/turn_around");

  const double nav_kp = declare_parameter<double>("control.nav_kp", 0.5);
  hold_kp_ = nav_kp;
  const double nav_max_speed = declare_parameter<double>("control.nav_max_speed", 1.0);
  const double yaw_kp = declare_parameter<double>("control.yaw_kp", 1.0);
  const double max_yaw_rate = get_parameter("control.max_yaw_rate").as_double();
  const double depth_kp = declare_parameter<double>("control.depth_kp", 0.5);
  const double max_depth_rate = declare_parameter<double>("control.max_depth_rate", 0.5);
  const double ring_kp_sway = declare_parameter<double>("control.ring_kp_sway", 0.8);
  const double ring_kp_z = declare_parameter<double>("control.ring_kp_z", 0.5);
  const double acquire_timeout_s = declare_parameter<double>("control.acquire_timeout_s", 20.0);
  const double acquire_surge_speed = declare_parameter<double>("control.acquire_surge_speed", 0.3);
  const int acquire_confirm_frames = declare_parameter<int>("control.acquire_confirm_frames", 5);

  auto nav_server = std::make_shared<NavigateSurfaceGpsServer>(
    this, nav_action, this, nav_kp, nav_max_speed, yaw_kp, max_yaw_rate);
  auto align_server = std::make_shared<AlignHeadingServer>(
    this, align_action, this, yaw_kp, max_yaw_rate);
  auto dive_server = std::make_shared<DiveToDepthServer>(
    this, dive_action, this, depth_kp, max_depth_rate, yaw_kp, max_yaw_rate);
  auto acquire_server = std::make_shared<AcquirePipelineServer>(
    this, acquire_action, this, yaw_kp, max_yaw_rate, 0.3, acquire_timeout_s, acquire_surge_speed,
    acquire_confirm_frames);
  auto search_server = std::make_shared<SearchPipelineServer>(
    this, search_action, this);
  auto follow_server = std::make_shared<FollowPipelineServer>(
    this, follow_action, this);
  auto center_server = std::make_shared<CenterOnRingServer>(
    this, center_action, this, ring_kp_sway, ring_kp_z);
  auto hold_server = std::make_shared<HoldProbeOnRingServer>(
    this, hold_action, this);
  auto surface_server = std::make_shared<SurfaceServer>(
    this, surface_action, this, depth_kp, max_depth_rate, require_calibrated_surface_depth_,
    declare_parameter<double>("control.surface_hold_time_s", 1.0), surface_depth_timeout_s_,
    declare_parameter<double>("control.surface_max_depth_m", 0.2));
  auto acquire_gate_server = std::make_shared<AcquireGateServer>(
    this, acquire_gate_action, this);
  auto search_marker_server = std::make_shared<SearchMarkerServer>(
    this, search_marker_action, this);
  auto search_gate_server = std::make_shared<SearchGateServer>(
    this, search_gate_action, this);
  auto align_gate_server = std::make_shared<AlignGateServer>(
    this, align_gate_action, this);
  auto align_marker_server = std::make_shared<AlignMarkerServer>(
    this, align_marker_action, this);
  auto pass_gate_server = std::make_shared<PassGateServer>(
    this, pass_gate_action, this);
  auto orbit_marker_server = std::make_shared<OrbitMarkerServer>(
    this, orbit_marker_action, this);
  auto turn_around_server = std::make_shared<TurnAroundServer>(
    this, turn_around_action, this);

  servers_ = {
    nav_server, align_server, dive_server,
    acquire_server, search_server, follow_server,
    center_server, hold_server, surface_server,
    acquire_gate_server, search_marker_server, search_gate_server,
    align_gate_server, align_marker_server, pass_gate_server,
    orbit_marker_server, turn_around_server
  };

  for (const auto & server : servers_) {
    server_map_[server->name()] = server;
  }
}

GoalRequest VehicleExecutionNode::requestGoal(const GoalIdentity & identity)
{
  if ((require_run_authorization_ && !runMotionAuthorized()) ||
    (enable_mavros_publish_ && !hardwareStatusReady()))
  {
    return {};
  }
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

  const auto steady_now = steady_clock_.now();
  const auto steady_age = [steady_now](const SteadyTime & receipt) {
      return std::chrono::duration<double>(steady_now - receipt).count();
    };
  if (telemetry_.pose_valid &&
    ((current_ros_time - telemetry_.pose_stamp).seconds() > kPoseTimeoutS ||
    steady_age(pose_receipt_) > kPoseTimeoutS))
  {
    telemetry_.pose_valid = false;
  }
  if (telemetry_.gps_valid && (current_ros_time - telemetry_.gps_stamp).seconds() > kGpsTimeoutS) {
    telemetry_.gps_valid = false;
  }
  if (telemetry_.status_valid &&
    ((current_ros_time - telemetry_.status_stamp).seconds() > kStatusTimeoutS ||
    steady_age(status_receipt_) > kStatusTimeoutS))
  {
    telemetry_.status_valid = false;
  }
  auto & water_depth = telemetry_.water_depth;
  if (water_depth.valid &&
    ((current_ros_time - water_depth.stamp).seconds() > surface_depth_timeout_s_ ||
    (water_depth.receipt_valid &&
    steady_age(water_depth.receipt_time) > surface_depth_timeout_s_)))
  {
    water_depth.valid = false;
    water_depth.calibrated = false;
    ++water_depth.continuity_generation;
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
  if (telemetry_.detection_valid &&
    (current_ros_time - telemetry_.detection_stamp).seconds() > kObservationTimeoutS)
  {
    telemetry_.detection_valid = false;
  }

  // Vehicle safety check
  const bool safety_fault = (telemetry_.status_valid &&
    (telemetry_.operator_takeover || !telemetry_.fcu_connected || !telemetry_.armed ||
    !telemetry_.fcu_status_fresh ||
    !telemetry_.rc_fresh || (require_guided_ && telemetry_.mode != "GUIDED"))) ||
    (status_ever_received_ && !telemetry_.status_valid) ||
    (require_run_authorization_ && !runMotionAuthorized() && !readinessHoldAuthorized());

  if (safety_fault) {
    revokeGoals("Vehicle safety or run authorization fault");
    if (require_run_authorization_ && lifecycle_received_) {
      authority_handshake_ = false;
      revoked_epoch_ = std::max(revoked_epoch_, run_lifecycle_.epoch);
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
  bool readiness_output_selected = false;
  bool readiness_output_published = false;

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
  } else if (readinessHoldAuthorized() && !safety_fault) {
    intent = readinessHoldIntent();
    readiness_output_selected = hold_pose_captured_ && telemetry_.pose_valid;
  }

  if (safety_fault || (require_run_authorization_ &&
    !runMotionAuthorized() && !readinessHoldAuthorized()))
  {
    intent = zeroIntent();
  }

  // Encode to PositionTarget and publish to preview sink
  try {
    auto target = encodeVelocityYawRate(intent, limits_);
    target.header.stamp = current_ros_time;
    target.header.frame_id = "map";
    preview_target_pub_->publish(target);

    if (enable_mavros_publish_ && mavros_target_pub_) {
      if (!hardwareStatusReady()) {
        terminal_zero_cycles_remaining_ = 0;
      }
      const bool terminal_stop = require_run_authorization_ &&
        terminal_zero_cycles_remaining_ > 0;
      const bool run_output_ready = !require_run_authorization_ ||
        runMotionAuthorized() || (readinessHoldAuthorized() && hold_pose_captured_ &&
        telemetry_.pose_valid) || terminal_stop;
      if (hardwareStatusReady() && run_output_ready && (!safety_fault || terminal_stop)) {
        mavros_target_pub_->publish(target);
        readiness_output_published = readiness_output_selected && !terminal_stop;
        if (terminal_stop) {
          --terminal_zero_cycles_remaining_;
        }
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Failed to encode preview setpoint: %s", ex.what());
  }
  publishReadiness(readiness_output_published,
    readiness_output_published ? "Readiness position hold streaming" :
    !enable_mavros_publish_ ? "MAVROS output disabled" :
    !lifecycleFresh() ? "Mission lifecycle unavailable or expired" :
    !hardwareStatusReady() ? "Hardware safety denied" :
    !telemetry_.pose_valid ? "Fresh map pose unavailable" : "Readiness hold not streaming");
}

rclcpp::Time VehicleExecutionNode::resolveStamp(const builtin_interfaces::msg::Time & stamp_msg)
{
  if (stamp_msg.sec == 0 && stamp_msg.nanosec == 0) {
    return now();
  }
  const rclcpp::Time stamp(stamp_msg, get_clock()->get_clock_type());
  if (std::abs((now() - stamp).seconds()) > 60.0) {
    return now();
  }
  return stamp;
}

void VehicleExecutionNode::handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (require_run_authorization_) {
    if (msg->header.stamp.nanosec >= 1000000000U || msg->header.stamp.sec < 0) {return;}
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (msg->header.frame_id != "map" || stamp.nanoseconds() <= 0 || age < 0.0 || age > 0.5 ||
      (telemetry_.pose_stamp.nanoseconds() > 0 && stamp <= telemetry_.pose_stamp))
    {
      return;
    }
  }
  telemetry_.pose_valid = std::isfinite(msg->pose.position.x) &&
    std::isfinite(msg->pose.position.y) && std::isfinite(msg->pose.position.z);
  telemetry_.pose_stamp = resolveStamp(msg->header.stamp);
  pose_receipt_ = steady_clock_.now();
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
  telemetry_.gps_stamp = resolveStamp(msg->header.stamp);
  telemetry_.latitude_deg = msg->latitude;
  telemetry_.longitude_deg = msg->longitude;
  telemetry_.altitude_m = msg->altitude;
}

void VehicleExecutionNode::handleRelAlt(const std_msgs::msg::Float64::SharedPtr msg)
{
  const auto receipt = steady_clock_.now();
  if (!telemetry_.depth_valid || !std::isfinite(msg->data) ||
    (telemetry_.depth_receipt_valid &&
    std::chrono::duration<double>(receipt - telemetry_.depth_receipt_time).count() >
    surface_depth_timeout_s_))
  {
    ++telemetry_.depth_continuity_generation;
  }
  telemetry_.depth_valid = std::isfinite(msg->data);
  telemetry_.depth_stamp = now();
  telemetry_.depth_receipt_time = receipt;
  telemetry_.depth_receipt_valid = true;
  // rel_alt is negative below the water surface; depth is positive downward
  telemetry_.depth_m = -msg->data;
}

void VehicleExecutionNode::handleWaterDepth(const banyu_msgs::msg::WaterDepth::SharedPtr msg)
{
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!require_calibrated_surface_depth_ || !lifecycle_received_ ||
    run_lifecycle_.phase < Lifecycle::INITIALIZING || run_lifecycle_.phase > Lifecycle::SURFACED ||
    msg->run_id == 0 || msg->epoch == 0 || msg->run_id != run_lifecycle_.run_id ||
    msg->epoch != run_lifecycle_.epoch || msg->header.stamp.sec < 0 ||
    msg->header.stamp.nanosec >= 1000000000U ||
    std::find(surface_depth_sources_.begin(), surface_depth_sources_.end(), msg->source_id) ==
    surface_depth_sources_.end())
  {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  if (stamp.nanoseconds() <= 0 || stamp.nanoseconds() < run_context_start_ns_ ||
    stamp.nanoseconds() <= last_water_depth_stamp_ns_ || age < 0.0 ||
    age > surface_depth_timeout_s_)
  {
    return;
  }
  last_water_depth_stamp_ns_ = stamp.nanoseconds();
  const bool valid = msg->valid && msg->calibrated && std::isfinite(msg->depth_m) &&
    msg->datum == "water_surface" && !msg->calibration_id.empty();
  const auto receipt = steady_clock_.now();
  auto & water = telemetry_.water_depth;
  if (!water.valid || !valid ||
    std::chrono::duration<double>(receipt - water.receipt_time).count() >
    surface_depth_timeout_s_ ||
    (stamp - water.stamp).seconds() > surface_depth_timeout_s_ ||
    msg->source_id != water.source_id || msg->calibration_id != water.calibration_id)
  {
    ++water.continuity_generation;
  }
  water.valid = valid;
  water.calibrated = valid;
  water.receipt_valid = valid;
  if (!valid) {return;}
  water.stamp = stamp;
  water.receipt_time = receipt;
  water.depth_m = msg->depth_m;
  water.run_id = msg->run_id;
  water.epoch = msg->epoch;
  water.datum = msg->datum;
  water.source_id = msg->source_id;
  water.calibration_id = msg->calibration_id;
}

void VehicleExecutionNode::handleStatus(const banyu_msgs::msg::VehicleStatus::SharedPtr msg)
{
  status_ever_received_ = true;
  if (msg->header.stamp.nanosec >= 1000000000U || msg->header.stamp.sec < 0) {return;}
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  if (stamp.nanoseconds() <= 0 || age < 0.0 || age > 1.0 ||
    (telemetry_.status_stamp.nanoseconds() > 0 && stamp <= telemetry_.status_stamp))
  {
    return;
  }
  telemetry_.status_valid = true;
  telemetry_.status_stamp = stamp;
  status_receipt_ = steady_clock_.now();
  telemetry_.fcu_connected = msg->fcu_connected;
  telemetry_.armed = msg->armed;
  telemetry_.fcu_status_fresh = msg->fcu_status_fresh;
  telemetry_.rc_fresh = msg->rc_fresh;
  telemetry_.operator_takeover = msg->operator_takeover;
  telemetry_.mode = msg->mode;
  telemetry_.authority = msg->authority;
}

bool VehicleExecutionNode::lifecycleFresh() const
{
  return lifecycle_received_ && authority_handshake_ &&
         std::chrono::duration<double>(steady_clock_.now() - lifecycle_receipt_).count() <=
         run_lifecycle_timeout_s_;
}

bool VehicleExecutionNode::hardwareStatusReady() const
{
  return telemetry_.status_valid && telemetry_.fcu_connected && telemetry_.fcu_status_fresh &&
         telemetry_.armed && telemetry_.rc_fresh && !telemetry_.operator_takeover &&
         telemetry_.mode == "GUIDED" &&
         std::chrono::duration<double>(steady_clock_.now() - status_receipt_).count() <= 1.0;
}

bool VehicleExecutionNode::runMotionAuthorized() const
{
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  const bool phase_authorized = run_lifecycle_.phase == Lifecycle::ACTIVE ||
    run_lifecycle_.phase == Lifecycle::SURVEY_COMPLETE ||
    run_lifecycle_.phase == Lifecycle::SURFACED;
  return lifecycleFresh() && run_lifecycle_.autonomous_ready && run_lifecycle_.run_id != 0 &&
         run_lifecycle_.epoch > revoked_epoch_ && phase_authorized && hardwareStatusReady();
}

bool VehicleExecutionNode::readinessHoldAuthorized() const
{
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  return require_run_authorization_ && lifecycleFresh() &&
         (run_lifecycle_.phase == Lifecycle::READY ||
         (run_lifecycle_.phase == Lifecycle::INITIALIZING &&
         run_lifecycle_.epoch > revoked_epoch_)) && hardwareStatusReady() &&
         telemetry_.pose_valid &&
         std::chrono::duration<double>(steady_clock_.now() - pose_receipt_).count() <= 0.5;
}

void VehicleExecutionNode::publishReadiness(bool holding, const std::string & reason)
{
  banyu_msgs::msg::ExecutionReadiness readiness;
  readiness.header.stamp = now();
  readiness.authority_instance = run_lifecycle_.authority_instance;
  readiness.run_id = run_lifecycle_.run_id;
  readiness.epoch = run_lifecycle_.epoch;
  readiness.lifecycle_revision = run_lifecycle_.revision;
  readiness.holding = holding;
  readiness.reason = reason;
  readiness_pub_->publish(readiness);
}

void VehicleExecutionNode::revokeGoals(const std::string & reason)
{
  arbiter_.fault();
  for (const auto & server : servers_) {
    if (server->hasActiveGoal()) {
      server->abortActive(reason);
    }
    if (server->hasPendingGoal()) {
      server->abortPending(reason);
    }
  }
}

MotionIntent VehicleExecutionNode::readinessHoldIntent()
{
  if (!telemetry_.pose_valid ||
    std::chrono::duration<double>(steady_clock_.now() - pose_receipt_).count() > 0.5)
  {
    return zeroIntent();
  }
  if (!hold_pose_captured_) {
    hold_x_ = telemetry_.pos_x;
    hold_y_ = telemetry_.pos_y;
    hold_z_ = telemetry_.pos_z;
    hold_pose_captured_ = true;
  }
  double x = hold_kp_ * (hold_x_ - telemetry_.pos_x);
  double y = hold_kp_ * (hold_y_ - telemetry_.pos_y);
  double z = hold_kp_ * (hold_z_ - telemetry_.pos_z);
  const double norm = std::sqrt(x * x + y * y + z * z);
  const double limit = std::min({0.3, limits_.max_linear_x, limits_.max_linear_y,
        limits_.max_linear_z});
  if (norm > limit) {
    x *= limit / norm;
    y *= limit / norm;
    z *= limit / norm;
  }
  return MotionIntent::localEnu(x, y, z, 0.0);
}

void VehicleExecutionNode::handleRunLifecycle(
  const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!require_run_authorization_ || msg->authority_instance.empty() || msg->revision == 0 ||
    retired_authorities_.count(msg->authority_instance) != 0 ||
    msg->phase > Lifecycle::INTERRUPTED || msg->header.stamp.nanosec >= 1000000000U ||
    msg->header.stamp.sec < 0 ||
    msg->accepted_at.sec < 0 || msg->accepted_at.nanosec >= 1000000000U ||
    (msg->phase >= Lifecycle::INITIALIZING && (msg->run_id == 0 || msg->epoch == 0)) ||
    (lifecycle_received_ && (msg->epoch < run_lifecycle_.epoch ||
    (msg->epoch == run_lifecycle_.epoch && msg->run_id != run_lifecycle_.run_id))))
  {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time accepted_stamp(msg->accepted_at, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  if (stamp.nanoseconds() <= 0 || age < 0.0 || age > run_lifecycle_timeout_s_ ||
    accepted_stamp > stamp)
  {
    return;
  }
  const bool new_authority = !lifecycle_received_ ||
    msg->authority_instance != run_lifecycle_.authority_instance;
  if (!new_authority && (msg->revision <= run_lifecycle_.revision ||
    msg->epoch < run_lifecycle_.epoch ||
    stamp <= rclcpp::Time(run_lifecycle_.header.stamp, get_clock()->get_clock_type())))
  {
    return;
  }
  if (new_authority) {
    if (lifecycle_received_) {
      retired_authorities_.insert(run_lifecycle_.authority_instance);
      revokeGoals("Mission authority changed");
      revoked_epoch_ = std::max(revoked_epoch_, run_lifecycle_.epoch);
    }
    authority_handshake_ = false;
    hold_pose_captured_ = false;
  }
  const bool readiness_reestablished = !authority_handshake_;
  if (msg->phase == Lifecycle::PREPARING || msg->phase == Lifecycle::READY) {
    authority_handshake_ = true;
  }
  const bool entering_ready = msg->phase == Lifecycle::READY &&
    (new_authority || run_lifecycle_.phase != Lifecycle::READY || readiness_reestablished);
  const bool context_changed = lifecycle_received_ &&
    (msg->run_id != run_lifecycle_.run_id || msg->epoch != run_lifecycle_.epoch);
  const bool phase_changed = new_authority || msg->phase != run_lifecycle_.phase;
  if (context_changed || !lifecycle_received_) {
    run_context_start_ns_ = accepted_stamp.nanoseconds() > 0 ?
      accepted_stamp.nanoseconds() : stamp.nanoseconds();
    last_water_depth_stamp_ns_ = 0;
    if (require_calibrated_surface_depth_) {
      telemetry_.water_depth.valid = false;
      telemetry_.water_depth.calibrated = false;
      telemetry_.water_depth.receipt_valid = false;
      ++telemetry_.water_depth.continuity_generation;
    }
  }
  if (phase_changed && msg->phase >= Lifecycle::SUCCEEDED &&
    (!lifecycle_received_ || run_lifecycle_.phase < Lifecycle::SUCCEEDED) &&
    hardwareStatusReady())
  {
    terminal_zero_cycles_remaining_ = required_zero_cycles_;
  } else if (msg->phase < Lifecycle::SUCCEEDED) {
    terminal_zero_cycles_remaining_ = 0;
  }
  if (entering_ready || msg->phase >= Lifecycle::SUCCEEDED) {
    hold_pose_captured_ = false;
  }
  if (context_changed || msg->phase >= Lifecycle::SUCCEEDED ||
    (phase_changed && msg->phase != Lifecycle::ACTIVE && msg->phase != Lifecycle::SURVEY_COMPLETE &&
    msg->phase != Lifecycle::SURFACED))
  {
    revokeGoals("Mission lifecycle changed");
  }
  run_lifecycle_ = *msg;
  telemetry_.current_run_id = msg->run_id;
  telemetry_.current_run_epoch = msg->epoch;
  lifecycle_received_ = true;
  lifecycle_receipt_ = steady_clock_.now();
  if (msg->phase >= Lifecycle::SUCCEEDED) {
    authority_handshake_ = false;
    revoked_epoch_ = std::max(revoked_epoch_, msg->epoch);
    if (require_calibrated_surface_depth_) {telemetry_.water_depth.valid = false;}
  }
  if (entering_ready && telemetry_.pose_valid) {
    readinessHoldIntent();
  }
}

void VehicleExecutionNode::handlePipelineObs(
  const banyu_msgs::msg::PipelineObservation::SharedPtr msg)
{
  telemetry_.pipeline_obs_valid = true;
  telemetry_.pipeline_obs_stamp = resolveStamp(msg->header.stamp);
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
  telemetry_.ring_obs_stamp = resolveStamp(msg->header.stamp);
  telemetry_.ring_visible = msg->visible;
  telemetry_.ring_color = msg->color;
  telemetry_.ring_offset_x = msg->offset_x;
  telemetry_.ring_offset_y = msg->offset_y;
  telemetry_.ring_confidence = msg->confidence;
}

void VehicleExecutionNode::handleDetection(const banyu_msgs::msg::Detection::SharedPtr msg)
{
  telemetry_.detection = *msg;
  telemetry_.detection_valid = true;
  telemetry_.detection_stamp = resolveStamp(msg->header.stamp);
}

void VehicleExecutionNode::handleGuidanceIntent(
  const banyu_guidance::msg::VelocityIntent::SharedPtr msg)
{
  telemetry_.guidance_intent_valid = true;
  telemetry_.guidance_intent_stamp = resolveStamp(msg->header.stamp);
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
