#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACQUIRE_PIPELINE_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACQUIRE_PIPELINE_SERVER_HPP_

#include <string>

#include "banyu_msgs/action/acquire_pipeline.hpp"
#include "banyu_msgs/msg/pipeline_observation.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class AcquirePipelineServer final
  : public TypedActionServer<banyu_msgs::action::AcquirePipeline>
{
public:
  AcquirePipelineServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator,
    double yaw_kp = 1.0,
    double max_yaw_rate = 0.5,
    double min_confidence = 0.3,
    double timeout_s = 20.0,
    double surge_speed = 0.3,
    int confirm_frames = 5);

  void onGoalStarted() override;

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

private:
  double yaw_kp_{1.0};
  double max_yaw_rate_{0.5};
  double min_confidence_{0.3};
  double timeout_s_{20.0};
  double surge_speed_{0.3};
  int confirm_frames_{5};
  int consecutive_frames_{0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACQUIRE_PIPELINE_SERVER_HPP_
