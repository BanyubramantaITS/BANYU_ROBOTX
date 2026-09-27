#include "banyu_mission/task2/vehicle_io.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace banyu_mission::task2 {

Snapshot snapshot(MissionContext& ctx) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  return {ctx.pressure_depth, ctx.yaw, ctx.altitude, ctx.pipe, ctx.pipe_seq, ctx.depth_target};
}

std::optional<banyu_msgs::msg::PingerObservation> takePing(MissionContext& ctx) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  if (ctx.pings.empty()) return std::nullopt;
  auto latest = ctx.pings.back();
  ctx.pings.clear();
  return latest;
}

std::vector<banyu_msgs::msg::PingerObservation> drainPings(MissionContext& ctx) {
  std::lock_guard<std::mutex> lock(ctx.data_mutex);
  return std::exchange(ctx.pings, {});
}

void sendSticks(MissionContext& ctx, double fwd, double right, double down, double yaw_rate_cw) {
  // Gains MEASURED in the sim at 13 m: x 200 -> 0.28 m/s, y 200 -> 0.26 m/s; yaw has a ~130
  // deadzone then ~0.045 deg/s per unit. ponytail: re-measure on the real vehicle.
  mavros_msgs::msg::ManualControl m;
  m.x = static_cast<float>(std::clamp(700.0 * fwd, -800.0, 800.0));
  m.y = static_cast<float>(std::clamp(800.0 * right, -400.0, 400.0));
  m.z = static_cast<float>(std::clamp(500.0 - 1000.0 * down, 100.0, 900.0));
  m.r = std::fabs(yaw_rate_cw) > 0.01
            ? static_cast<float>(std::clamp(std::copysign(130.0 + 1300.0 * std::fabs(yaw_rate_cw), yaw_rate_cw), -700.0, 700.0))
            : 0.0f;
  if (ctx.stick_pub) ctx.stick_pub->publish(m);
}

double depthRate(double target, double depth) {
  const double down = std::clamp(0.5 * (target - depth), -0.5, 0.5);
  return depth < 0.5 ? std::max(down, 0.0) : down;
}

std::optional<std::pair<double, double>> pullOverPipe(const banyu_msgs::msg::PipeObservation& pipe) {
  if (!pipe.pipe_in_view || pipe.height == 0) return std::nullopt;
  const double half_h = pipe.height / 2.0, half_w = pipe.width / 2.0;
  return std::make_pair(std::clamp(0.4 * (pipe.reference_row - pipe.centroid_y) / half_h, -0.1, 0.1),
                        std::clamp(0.4 * (pipe.centroid_x - half_w) / half_w, -0.1, 0.1));
}

namespace {
// Waits on the future: the BT ticks on its own thread while the node spins on the main one.
template <typename Srv, typename Ok>
bool callBlocking(const typename rclcpp::Client<Srv>::SharedPtr& client, typename Srv::Request::SharedPtr req, Ok ok) {
  if (!client || !client->wait_for_service(std::chrono::seconds(5))) return false;
  auto future = client->async_send_request(req);
  return future.wait_for(std::chrono::seconds(5)) == std::future_status::ready && ok(*future.get());
}
}  // namespace

bool setMode(MissionContext& ctx, const rclcpp::Node::SharedPtr& node, const std::string& mode) {
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = mode;
  const bool ok = callBlocking<mavros_msgs::srv::SetMode>(ctx.set_mode_client, req,
                                                          [](const auto& r) { return static_cast<bool>(r.mode_sent); });
  RCLCPP_INFO(node->get_logger(), "mode %s: %s", mode.c_str(), ok ? "ok" : "FAILED");
  return ok;
}

bool setArmed(MissionContext& ctx, const rclcpp::Node::SharedPtr& node, bool armed) {
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = armed;
  const bool ok = callBlocking<mavros_msgs::srv::CommandBool>(ctx.arming_client, req,
                                                              [](const auto& r) { return static_cast<bool>(r.success); });
  RCLCPP_INFO(node->get_logger(), "%s: %s", armed ? "arm" : "disarm", ok ? "ok" : "FAILED");
  return ok;
}

double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace banyu_mission::task2
