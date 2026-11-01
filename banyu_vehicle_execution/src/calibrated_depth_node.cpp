#include "banyu_vehicle_execution/calibrated_depth_node.hpp"

#include <cmath>
#include <chrono>
#include <stdexcept>

namespace banyu_vehicle_execution
{

namespace
{
constexpr double kMaxDatumSurfaceDepthM = 0.5;
}  // namespace

CalibratedDepthNode::CalibratedDepthNode(const rclcpp::NodeOptions & options)
: Node("calibrated_depth_node", options)
{
  source_id_ = declare_parameter<std::string>("source_id", "pressure_depth");
  datum_ = declare_parameter<std::string>("datum", "water_surface");
  calibration_id_ = declare_parameter<std::string>(
    "calibration_id", "surface_rel_alt_datum");
  surface_datum_offset_ = declare_parameter<double>("surface_datum_offset", 0.0);
  auto_calibrate_at_start_ = declare_parameter<bool>(
    "auto_calibrate_surface_at_start", true);
  max_rel_alt_age_s_ = declare_parameter<double>("max_rel_alt_age_s", 1.0);
  const bool manually_calibrated = declare_parameter<bool>("manually_calibrated", false);
  calibrated_ = manually_calibrated;

  const auto rel_alt_topic = declare_parameter<std::string>(
    "topics.rel_alt", "/mavros/global_position/rel_alt");
  const auto water_depth_topic = declare_parameter<std::string>(
    "topics.water_depth", "/banyu/vehicle/water_depth");
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");
  const auto fix_topic = declare_parameter<std::string>(
    "topics.gps_fix", "/mavros/global_position/global");
  gps_surface_timeout_s_ = declare_parameter<double>("gps_surface_timeout_s", 2.0);
  // "pressure": datum minus rel_alt (Bar30). "gps_surface": no depth sensor; depth is 0 while a GPS fix proves the
  // surface and unknown under water.
  const auto depth_source = declare_parameter<std::string>("depth_source", "pressure");
  if (depth_source != "pressure" && depth_source != "gps_surface") {
    throw std::invalid_argument("depth_source must be pressure or gps_surface");
  }
  gps_surface_only_ = depth_source == "gps_surface";
  gps_source_id_ = declare_parameter<std::string>("gps_source_id", "gps_surface");
  health_enabled_ = declare_parameter<bool>("health.enabled", true);
  DepthHealthMonitor::Config health_config;
  health_config.silent_s = declare_parameter<double>("health.silent_s", health_config.silent_s);
  health_config.stuck_s = declare_parameter<double>("health.stuck_s", health_config.stuck_s);
  health_config.max_step_m = declare_parameter<double>("health.max_step_m", health_config.max_step_m);
  health_config.max_disagree_m = declare_parameter<double>(
    "health.max_disagree_m", health_config.max_disagree_m);
  health_ = DepthHealthMonitor(health_config);
  const auto command_topic = declare_parameter<std::string>(
    "topics.command", "/banyu/execution/preview_target");
  const auto health_topic = declare_parameter<std::string>(
    "topics.depth_health", "/banyu/vehicle/depth_health");

  depth_pub_ = create_publisher<banyu_msgs::msg::WaterDepth>(
    water_depth_topic, rclcpp::QoS(10).reliable());
  health_pub_ = create_publisher<std_msgs::msg::String>(health_topic, rclcpp::QoS(5).reliable());
  command_sub_ = create_subscription<mavros_msgs::msg::PositionTarget>(
    command_topic, rclcpp::QoS(10),
    std::bind(&CalibratedDepthNode::handleCommand, this, std::placeholders::_1));
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {onTimer();});

  rel_alt_sub_ = create_subscription<std_msgs::msg::Float64>(
    rel_alt_topic, rclcpp::SensorDataQoS(),
    std::bind(&CalibratedDepthNode::handleRelAlt, this, std::placeholders::_1));

  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic, rclcpp::SensorDataQoS(),
    std::bind(&CalibratedDepthNode::handleFix, this, std::placeholders::_1));

  lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CalibratedDepthNode::handleLifecycle, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "CalibratedDepthNode initialized: source='%s', datum='%s', auto_calibrate=%d, depth_source=%s",
    source_id_.c_str(), datum_.c_str(), auto_calibrate_at_start_, depth_source.c_str());
}

bool CalibratedDepthNode::usingGpsSurface() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return gps_surface_only_ || !health_.healthy();
}

std::string CalibratedDepthNode::depthHealth() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return gps_surface_only_ ? "DISABLED" : DepthHealthMonitor::name(health_.fault());
}

void CalibratedDepthNode::handleCommand(const mavros_msgs::msg::PositionTarget::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg) {health_.setCommandedHeave(msg->velocity.z);}
}

void CalibratedDepthNode::publishGpsSurfaceDepth()
{
  banyu_msgs::msg::WaterDepth depth_msg;
  depth_msg.header.stamp = now();
  depth_msg.header.frame_id = "base_link";
  depth_msg.run_id = run_id_;
  depth_msg.epoch = epoch_;
  depth_msg.depth_m = 0.0;
  depth_msg.valid = gpsAtSurface();
  depth_msg.calibrated = depth_msg.valid;
  depth_msg.datum = datum_;
  depth_msg.source_id = gps_source_id_;
  depth_msg.calibration_id = gps_calibration_id_;
  depth_pub_->publish(depth_msg);
}

void CalibratedDepthNode::onTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool active = current_phase_ >= banyu_msgs::msg::RunLifecycle::ACTIVE;
  if (!gps_surface_only_ && health_enabled_ && active && !gpsAtSurface()) {
    health_.check(now().seconds());
  }
  if (health_.fault() != reported_fault_) {
    reported_fault_ = health_.fault();
    RCLCPP_ERROR(
      get_logger(), "Depth sensor fault %s: depth from the GPS surface proof only for the rest of the run",
      DepthHealthMonitor::name(reported_fault_).c_str());
  }
  if (gps_surface_only_ || !health_.healthy()) {
    publishGpsSurfaceDepth();
  }
  if (++health_ticks_ % 10 == 0) {
    std_msgs::msg::String health;
    health.data = gps_surface_only_ ? "DISABLED" : DepthHealthMonitor::name(health_.fault());
    health_pub_->publish(health);
  }
}

bool CalibratedDepthNode::isCalibrated() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return calibrated_;
}

double CalibratedDepthNode::surfaceDatumOffset() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return surface_datum_offset_;
}

uint32_t CalibratedDepthNode::runId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return run_id_;
}

uint64_t CalibratedDepthNode::epoch() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return epoch_;
}

void CalibratedDepthNode::calibrateDatum(double surface_rel_alt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(surface_rel_alt)) {
    return;
  }
  surface_datum_offset_ = surface_rel_alt;
  calibrated_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Calibrated water surface datum offset to %.3f m",
    surface_datum_offset_);
}

void CalibratedDepthNode::handleFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fix_received_ = true;
  fix_valid_ = msg && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
    std::isfinite(msg->latitude) && std::isfinite(msg->longitude);
  fix_time_ = now();
}

bool CalibratedDepthNode::gpsAtSurface() const
{
  return fix_received_ && fix_valid_ && (now() - fix_time_).seconds() <= gps_surface_timeout_s_;
}

bool CalibratedDepthNode::isNearSurface(double alt) const
{
  return std::isfinite(alt) && alt >= -0.3 && alt <= 0.5;
}

void CalibratedDepthNode::handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  const bool new_run = (msg->run_id != run_id_);
  if (new_run) {health_.reset();}
  run_id_ = msg->run_id;
  epoch_ = msg->epoch;
  current_phase_ = msg->phase;
  if (new_run && auto_calibrate_at_start_) {
    // A pre-start datum stays valid while the vehicle is still at the surface relative to it;
    // raw rel_alt alone drifts out of the surface band with barometer drift.
    const bool at_datum_surface = pre_start_calibration_ && calibrated_ &&
      std::abs(surface_datum_offset_ - last_rel_alt_) <= kMaxDatumSurfaceDepthM;
    if (!rel_alt_received_ || !(isNearSurface(last_rel_alt_) || at_datum_surface || gpsAtSurface())) {
      calibrated_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Run %u start: surface datum dropped (rel_alt received %d, rel_alt %.3f m, datum %.3f m, pre-start datum %d)",
        msg->run_id, rel_alt_received_, last_rel_alt_, surface_datum_offset_, pre_start_calibration_);
    }
  }
  if (current_phase_ >= banyu_msgs::msg::RunLifecycle::ACTIVE) {
    pre_start_calibration_ = false;
  }
}

void CalibratedDepthNode::handleRelAlt(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto current_stamp = now();
  if (!msg || !std::isfinite(msg->data)) {
    banyu_msgs::msg::WaterDepth invalid_msg;
    invalid_msg.header.stamp = current_stamp;
    invalid_msg.header.frame_id = "base_link";
    invalid_msg.run_id = run_id_;
    invalid_msg.epoch = epoch_;
    invalid_msg.depth_m = 0.0;
    invalid_msg.valid = false;
    invalid_msg.calibrated = calibrated_;
    invalid_msg.datum = datum_;
    invalid_msg.source_id = source_id_;
    invalid_msg.calibration_id = calibration_id_;
    depth_pub_->publish(invalid_msg);
    return;
  }

  last_rel_alt_ = msg->data;
  last_rel_alt_time_ = current_stamp;
  rel_alt_received_ = true;
  if (gps_surface_only_) {return;}  // logged by MAVROS, never used
  // Watched only under water: at the surface the GPS fix is the proof, and the sim barometer jumps at arming.
  if (health_enabled_ && current_phase_ >= banyu_msgs::msg::RunLifecycle::ACTIVE && !gpsAtSurface()) {
    health_.addSample(-msg->data, current_stamp.seconds());
  } else {
    health_.restartSamples();
  }
  if (!health_.healthy()) {return;}  // the timer publishes the GPS surface depth

  // Refresh the datum at the surface before every run, not only the first one.
  // Before the run starts a valid GPS fix is surface proof stronger than the barometer band: the datum follows the barometer
  // (it jumps after arming) for as long as the fix is there. Never during the run.
  const bool gps_proof = gpsAtSurface() && current_phase_ < banyu_msgs::msg::RunLifecycle::ACTIVE;
  if (auto_calibrate_at_start_ &&
    (gps_proof || ((!calibrated_ || !pre_start_calibration_) && isNearSurface(last_rel_alt_))))
  {
    if (current_phase_ < banyu_msgs::msg::RunLifecycle::ACTIVE) {
      // Log a first calibration and a real change; small steps while the fix is there are tracked quietly.
      const bool quiet = gps_proof && calibrated_ && pre_start_calibration_ &&
        std::abs(surface_datum_offset_ - last_rel_alt_) <= 0.05;
      surface_datum_offset_ = last_rel_alt_;
      calibrated_ = true;
      pre_start_calibration_ = true;
      if (!quiet) {
        RCLCPP_INFO(
          get_logger(), "Surface datum auto-calibrated to %.3f m%s", surface_datum_offset_,
          gps_proof ? " (GPS fix at the surface)" : "");
      }
    }
  }

  const double depth_m = surface_datum_offset_ - last_rel_alt_;

  banyu_msgs::msg::WaterDepth depth_msg;
  depth_msg.header.stamp = current_stamp;
  depth_msg.header.frame_id = "base_link";
  depth_msg.run_id = run_id_;
  depth_msg.epoch = epoch_;
  depth_msg.depth_m = depth_m;
  depth_msg.valid = true;
  depth_msg.calibrated = calibrated_;
  depth_msg.datum = datum_;
  depth_msg.source_id = source_id_;
  depth_msg.calibration_id = calibration_id_;

  depth_pub_->publish(depth_msg);
}

}  // namespace banyu_vehicle_execution

#ifndef BANYU_VEHICLE_EXECUTION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_vehicle_execution::CalibratedDepthNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
