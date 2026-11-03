#include "banyu_communication/pipeline_buoys_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

namespace banyu_communication
{
namespace
{
using Lifecycle = banyu_msgs::msg::RunLifecycle;

PipelineBuoysTracker::Config loadConfig(rclcpp::Node & node)
{
  PipelineBuoysTracker::Config config;
  config.source_id = node.declare_parameter<std::string>("source_id", config.source_id);
  config.endpoint_id = node.declare_parameter<std::string>("endpoint_id", config.endpoint_id);
  config.default_uncertainty_m = node.declare_parameter<double>(
    "default_uncertainty_m", config.default_uncertainty_m);
  config.require_inside_course = node.declare_parameter<bool>(
    "require_inside_course", config.require_inside_course);
  config.minimum_buoy_separation_m = node.declare_parameter<double>(
    "minimum_buoy_separation_m", config.minimum_buoy_separation_m);
  config.fallback_enabled = node.declare_parameter<bool>(
    "fallback.enabled", config.fallback_enabled);
  config.fallback_latitude = node.declare_parameter<double>(
    "fallback.latitude", config.fallback_latitude);
  config.fallback_longitude = node.declare_parameter<double>(
    "fallback.longitude", config.fallback_longitude);
  config.fallback_uncertainty_m = node.declare_parameter<double>(
    "fallback.uncertainty_m", config.fallback_uncertainty_m);
  config.fallback_green_confirmed = node.declare_parameter<bool>(
    "fallback.green_confirmed", config.fallback_green_confirmed);
  return config;
}

bool livePhase(uint8_t phase)
{
  return phase >= Lifecycle::INITIALIZING && phase <= Lifecycle::SURFACED;
}
}  // namespace

PipelineBuoysNode::PipelineBuoysNode(const rclcpp::NodeOptions & options)
: Node("pipeline_buoys_node", options), tracker_(loadConfig(*this))
{
  lock_depth_m_ = declare_parameter<double>("lock_depth_m", 1.0);
  const auto rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);
  if (!std::isfinite(lock_depth_m_) || lock_depth_m_ <= 0.0 || !std::isfinite(rate_hz) ||
    rate_hz <= 0.0)
  {
    throw std::invalid_argument("lock_depth_m and publish_rate_hz must be positive");
  }
  const auto buoys_topic = declare_parameter<std::string>(
    "topics.pipeline_buoys", "/system/vehicle/usv/task2/pipeline_buoys");
  const auto course_topic = declare_parameter<std::string>(
    "topics.course", "/system/mission/course");
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");
  const auto depth_topic = declare_parameter<std::string>(
    "topics.water_depth", "/banyu/vehicle/water_depth");
  const auto target_topic = declare_parameter<std::string>(
    "topics.buoy_target", "/banyu/task2/buoy_target");
  const auto active_buoy_topic = declare_parameter<std::string>(
    "topics.active_buoy_fix", "/banyu/task2/active_buoy_fix");

  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  target_pub_ = create_publisher<banyu_msgs::msg::Task2BuoyTarget>(target_topic, latched);
  active_buoy_pub_ = create_publisher<banyu_msgs::msg::ActiveBuoyFix>(active_buoy_topic, latched);

  using std::placeholders::_1;
  // rx-sos spec: PipelineBuoys and Course use the plain default QoS (reliable, volatile, keep_last 10).
  // A transient_local subscription would be incompatible with those publishers.
  buoys_sub_ = create_subscription<rx_msgs::msg::PipelineBuoys>(
    buoys_topic, rclcpp::QoS(10).reliable(), std::bind(&PipelineBuoysNode::handleBuoys, this, _1));
  course_sub_ = create_subscription<rx_msgs::msg::Course>(
    course_topic, rclcpp::QoS(10).reliable(),
      std::bind(&PipelineBuoysNode::handleCourse, this, _1));
  lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, latched, std::bind(&PipelineBuoysNode::handleLifecycle, this, _1));
  depth_sub_ = create_subscription<banyu_msgs::msg::WaterDepth>(
    depth_topic, rclcpp::QoS(5).reliable(),
    std::bind(&PipelineBuoysNode::handleDepth, this, _1));

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz)),
    std::bind(&PipelineBuoysNode::publishTick, this));
  RCLCPP_INFO(
    get_logger(), "Waiting for USV PipelineBuoys on %s (lock below %.2f m)",
    buoys_topic.c_str(), lock_depth_m_);
}

void PipelineBuoysNode::handleBuoys(const rx_msgs::msg::PipelineBuoys::SharedPtr msg)
{
  if (!msg) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  const auto reason = tracker_.offer(*msg);
  if (!reason.empty()) {
    // The USV repeats each observation at ~1 Hz: log a rejection once, not every repeat.
    if (reason != "DUPLICATE" && reason != last_rejection_) {
      RCLCPP_WARN(
        get_logger(), "PipelineBuoys seq=%u ignored: %s", msg->seq, reason.c_str());
      last_rejection_ = reason;
    }
    return;
  }
  last_rejection_.clear();
  RCLCPP_INFO(
    get_logger(), "PipelineBuoys seq=%u accepted: course=%s active=%u buoy_b=%s",
    msg->seq, msg->course_id.c_str(), msg->active_buoy, msg->buoy_b_valid ? "yes" : "no");
  publishTargetLocked();
}

void PipelineBuoysNode::handleCourse(const rx_msgs::msg::Course::SharedPtr msg)
{
  if (!msg) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.setCourse(*msg);
  pinger_freq_hz_ = msg->pinger_freq_hz;
}

void PipelineBuoysNode::handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  if (!msg || msg->phase > Lifecycle::INTERRUPTED) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = msg->phase;
  if (!livePhase(phase_)) {
    // Between runs the UUV is on the surface again, so new USV estimates apply to the next run.
    if (tracker_.locked()) {
      tracker_.unlock();
      publishTargetLocked();
    }
    return;
  }
  if (msg->run_id == 0 || msg->epoch == 0) {return;}
  if (msg->run_id != run_id_ || msg->epoch != epoch_) {
    run_id_ = msg->run_id;
    epoch_ = msg->epoch;
    tracker_.beginRun(run_id_, epoch_);
    RCLCPP_INFO(
      get_logger(), "Run %u (epoch %lu): active buoy from %s", run_id_, epoch_,
      tracker_.activeKnown() ? "USV PipelineBuoys" : "fallback/none yet");
  }
  if (phase_ >= Lifecycle::SURVEY_COMPLETE) {lockLocked("survey complete");}
}

void PipelineBuoysNode::handleDepth(const banyu_msgs::msg::WaterDepth::SharedPtr msg)
{
  if (!msg || !msg->valid || !std::isfinite(msg->depth_m)) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (!livePhase(phase_) || phase_ < Lifecycle::ACTIVE || msg->run_id != run_id_ ||
    msg->epoch != epoch_)
  {
    return;
  }
  if (msg->depth_m > lock_depth_m_) {lockLocked("submerged");}
}

void PipelineBuoysNode::lockLocked(const std::string & reason)
{
  if (tracker_.locked()) {return;}
  tracker_.lock();
  RCLCPP_INFO(
    get_logger(), "Active buoy locked for run %u (%s); USV updates ignored until the run ends",
    run_id_, reason.c_str());
  publishTargetLocked();
}

void PipelineBuoysNode::publishTargetLocked()
{
  const auto target = tracker_.target(now());
  if (!target) {return;}
  target_pub_->publish(*target);
  published_target_revision_ = tracker_.revision();
}

bool PipelineBuoysNode::liveLocked() const
{
  return livePhase(phase_) && run_id_ != 0 && epoch_ != 0;
}

void PipelineBuoysNode::publishTick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (tracker_.revision() != published_target_revision_) {publishTargetLocked();}
  if (!liveLocked()) {return;}
  // The survey ledger only accepts a recently stamped buoy until its first ring pins it, so the
  // (unchanged) position is re-asserted with fresh stamps while the run is live.
  const auto buoy = tracker_.activeBuoyFix(now(), pinger_freq_hz_);
  if (buoy) {active_buoy_pub_->publish(*buoy);}
}

}  // namespace banyu_communication

#ifndef BANYU_COMMUNICATION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_communication::PipelineBuoysNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
