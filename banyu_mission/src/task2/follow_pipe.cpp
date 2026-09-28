#include "banyu_mission/task2/follow_pipe.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission::task2 {

namespace {
constexpr int kEndFrames = 15;       // pipe must stay ended this many frames
constexpr double kLostS = 3.0;       // no fit this long after having one = its end
constexpr double kNeverFoundS = 15.0;
constexpr double kTimeoutS = 300.0;
constexpr double kSpeed = 0.2;       // m/s along the pipe: frame-to-frame overlap for the tracker
constexpr double kClimbedM = 0.2;
constexpr double kClimbTimeoutS = 30.0;
constexpr double kAtProbeM = 0.25;   // stop_at_red: RED node this close under the probe tip
}  // namespace

BT::NodeStatus FollowPipe::begin() {
  direction_ = getInput<int>("direction").value_or(1) >= 0 ? 1 : -1;
  record_ = getInput<bool>("record").value_or(false);
  stop_at_red_ = getInput<bool>("stop_at_red").value_or(false);
  climbing_ = getInput<bool>("resume").value_or(false);
  had_fit_ = false;
  end_frames_ = 0;
  lost_since_ = -1.0;
  if (record_) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (climbing_)
      ctx_->tracker.resume(node_->now().seconds());
    else
      ctx_->tracker.reset(node_->now().seconds());
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPipe::step(const Snapshot& s) {
  const char* tag = record_ ? "survey" : "seek-start";
  const auto end = stop_at_red_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  const auto& fit = direction_ > 0 ? s.pipe.ahead : s.pipe.behind;
  const double vz = depthRate(s.depth_target, s.depth);

  // Resuming after the repair, 0.4 m over the pipe: that close the pipe is wider than the
  // detector's background window and looks ended or lost, so those checks wait until it is back
  // at survey depth. It climbs on the move: hovering, it drifts back over nodes already counted.
  if (climbing_ && (s.depth - s.depth_target < kClimbedM || elapsed() > kClimbTimeoutS)) climbing_ = false;

  if (newFrame(s)) {
    // Depth from the echosounder while it is on the pipe (vehicle centred on it); the seabed
    // return, pipe_height further down, falls outside the window.
    const double survey_alt = param("task2_survey_altitude");
    if (fit.valid && std::fabs(fit.lateral) < 30 && s.altitude && std::fabs(*s.altitude - survey_alt) < 0.8) {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      ctx_->depth_target += 0.05 * (s.depth + *s.altitude - survey_alt - ctx_->depth_target);
    }
    if (record_) {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      ctx_->tracker.update(s.pipe.lights, node_->now().seconds(), direction_, s.pipe.height);
    }
    if (stop_at_red_) {
      const auto fix = lightUnderProbe(s, kAtProbeM);
      if (fix && fix->light.colour == banyu_msgs::msg::LightNode::RED) {
        RCLCPP_INFO(logger(), "%s: RED node at the probe", tag);
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        ctx_->tracker.confirm(fix->light.x, fix->light.y);
        return BT::NodeStatus::SUCCESS;
      }
    }
    const bool ends = direction_ > 0 ? s.pipe.ends_ahead : s.pipe.ends_behind;
    end_frames_ = had_fit_ && ends && !climbing_ ? end_frames_ + 1 : 0;
    if (end_frames_ >= kEndFrames) {
      RCLCPP_INFO(logger(), "%s: pipe ends under the vehicle", tag);
      return end;
    }
  }
  if (elapsed() > kTimeoutS) return end;

  if (!fit.valid) {
    if (lost_since_ < 0) lost_since_ = elapsed();
    if (had_fit_ && !climbing_ && elapsed() - lost_since_ > kLostS) {
      RCLCPP_INFO(logger(), "%s: pipe lost for %.0f s, treating as its end", tag, kLostS);
      return end;
    }
    if (!had_fit_ && !climbing_ && elapsed() > kNeverFoundS) {
      RCLCPP_WARN(logger(), "%s: never picked the pipe up", tag);
      return end;
    }
    // ALT_HOLD has no position hold and the hull coasts: brake onto whatever pipe is left in
    // view, or, before the first fit, creep the way we mean to go.
    // Climbing after the repair the pipe mask is unreliable and pulling onto it can run the
    // wrong way: just keep going.
    if (const auto pull = climbing_ ? std::nullopt : pullOverPipe(s.pipe)) {
      sendSticks(*ctx_, pull->first, pull->second, vz);
    } else {
      sendSticks(*ctx_, had_fit_ && !climbing_ ? 0.0 : 0.08 * direction_, 0, vz);
    }
    return BT::NodeStatus::RUNNING;
  }
  lost_since_ = -1.0;
  had_fit_ = true;
  const double half_w = s.pipe.width / 2.0;
  sendSticks(*ctx_, direction_ * kSpeed * std::max(0.0, 1 - std::fabs(fit.angle) / 0.7),
             std::clamp(0.5 * fit.lateral / half_w, -0.2, 0.2), vz, std::clamp(1.2 * fit.angle, -0.4, 0.4));
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
