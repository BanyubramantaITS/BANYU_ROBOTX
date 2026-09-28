// Task 2 vehicle I/O shared by the BT nodes: pilot sticks (ALT_HOLD / STABILIZE), flight mode,
// arming, and thread-safe copies of the sensor state in MissionContext.
#ifndef BANYU_MISSION_TASK2_VEHICLE_IO_HPP_
#define BANYU_MISSION_TASK2_VEHICLE_IO_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission::task2 {

// Consistent copy of what the nodes read each tick.
struct Snapshot {
  double depth;                  // m, positive down
  double yaw;                    // rad, ROS ENU
  std::optional<double> altitude;
  banyu_msgs::msg::PipeObservation pipe;
  uint64_t pipe_seq;
  double depth_target;
};
Snapshot snapshot(MissionContext& ctx);

// Newest unread ping (the rest are dropped), or nullopt.
std::optional<banyu_msgs::msg::PingerObservation> takePing(MissionContext& ctx);
// Every unread ping, oldest first; the queue is emptied.
std::vector<banyu_msgs::msg::PingerObservation> drainPings(MissionContext& ctx);

// Body velocity demand (x fwd, y right, z down, m/s; yaw rate clockwise, rad/s) as MANUAL_CONTROL
// sticks. z 500 is neutral: depth hold in ALT_HOLD, zero vertical thrust in STABILIZE.
void sendSticks(MissionContext& ctx, double fwd, double right, double down, double yaw_rate_cw = 0.0);

// Down-positive depth-hold demand. The hull floats and the thrusters still push out of the
// water, so never drive UP near the surface: buoyancy finishes the job.
double depthRate(double target, double depth);

// (fwd, right) m/s pulling the vehicle over the pipe pixels in view; nullopt if none in view.
std::optional<std::pair<double, double>> pullOverPipe(const banyu_msgs::msg::PipeObservation& pipe);

// A light node's magnetic switch relative to the repair probe tip, horizontal metres.
struct ProbeFix {
  banyu_msgs::msg::LightNode light;
  double fwd, right;
  double z;  // camera height above the switch used, m
};

// The light node closest to being under the probe tip, if within gate_m. Pixels become metres by
// z, the camera's height above the switches; tip_fwd is how far ahead of the camera the tip is.
// Straight under the tip, a switch shows below the tip in the image until the tip is on it.
std::optional<ProbeFix> lightUnderProbe(const banyu_msgs::msg::PipeObservation& pipe, double z, double tip_fwd,
                                        double gate_m);

// Blocking service calls (the BT ticks on its own thread; the node spins elsewhere).
bool setMode(MissionContext& ctx, const rclcpp::Node::SharedPtr& node, const std::string& mode);
bool setArmed(MissionContext& ctx, const rclcpp::Node::SharedPtr& node, bool armed);

double wrapAngle(double a);

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_VEHICLE_IO_HPP_
