#ifndef BANYU_COMMUNICATION__PIPELINE_BUOYS_NODE_HPP_
#define BANYU_COMMUNICATION__PIPELINE_BUOYS_NODE_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "banyu_communication/pipeline_buoys.hpp"
#include "banyu_msgs/msg/active_buoy_fix.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_msgs/msg/task2_buoy_target.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rx_msgs/msg/pipeline_buoys.hpp"

namespace banyu_communication
{

// Receives the USV's Task 2 buoy feed (forwarded by the GCS) and provides the mission's
// navigation target plus the run-tagged ActiveBuoyFix used by the survey report.
class PipelineBuoysNode : public rclcpp::Node
{
public:
  explicit PipelineBuoysNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handleBuoys(const rx_msgs::msg::PipelineBuoys::SharedPtr msg);
  void handleCourse(const rx_msgs::msg::Course::SharedPtr msg);
  void handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg);
  void handleDepth(const banyu_msgs::msg::WaterDepth::SharedPtr msg);
  void publishTick();
  void publishTargetLocked();
  void lockLocked(const std::string & reason);
  bool liveLocked() const;

  std::mutex mutex_;
  PipelineBuoysTracker tracker_;
  double lock_depth_m_{1.0};
  uint32_t pinger_freq_hz_{0};
  uint32_t run_id_{0};
  uint64_t epoch_{0};
  uint8_t phase_{banyu_msgs::msg::RunLifecycle::IDLE};
  std::string last_rejection_;
  uint64_t published_target_revision_{0};

  rclcpp::Publisher<banyu_msgs::msg::Task2BuoyTarget>::SharedPtr target_pub_;
  rclcpp::Publisher<banyu_msgs::msg::ActiveBuoyFix>::SharedPtr active_buoy_pub_;
  rclcpp::Subscription<rx_msgs::msg::PipelineBuoys>::SharedPtr buoys_sub_;
  rclcpp::Subscription<rx_msgs::msg::Course>::SharedPtr course_sub_;
  rclcpp::Subscription<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_sub_;
  rclcpp::Subscription<banyu_msgs::msg::WaterDepth>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION__PIPELINE_BUOYS_NODE_HPP_
