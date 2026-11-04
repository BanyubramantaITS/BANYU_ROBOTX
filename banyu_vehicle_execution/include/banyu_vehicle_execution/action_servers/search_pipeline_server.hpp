#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SEARCH_PIPELINE_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SEARCH_PIPELINE_SERVER_HPP_

#include <string>

#include "banyu_msgs/action/search_pipeline.hpp"
#include "banyu_msgs/msg/pipeline_observation.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class SearchPipelineServer final
  : public TypedActionServer<banyu_msgs::action::SearchPipeline>
{
public:
  SearchPipelineServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator,
    double default_sweep_rate = 0.2,
    double min_confidence = 0.3,
    double retrace_distance_m = 1.5,
    double retrace_speed = 0.2,
    double pinger_timeout_s = 3.0,
    double pinger_kp = 0.8,
    double pinger_forward_speed = 0.25,
    bool use_pinger = false);

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  void onGoalStarted() override;

private:
  double default_sweep_rate_{0.2};
  double min_confidence_{0.3};
  double sweep_progress_rad_{0.0};
  double retrace_distance_m_{1.5};
  double retrace_speed_{0.2};
  double pinger_timeout_s_{3.0};
  double pinger_kp_{0.8};
  double pinger_forward_speed_{0.25};
  // Off on the competition vehicle (no hydrophone): retrace + sweep only.
  bool use_pinger_{false};
  bool retrace_started_{false};
  bool retrace_done_{false};
  double retrace_origin_x_{0.0};
  double retrace_origin_y_{0.0};
  double retrace_heading_{0.0};
  double retrace_elapsed_s_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__SEARCH_PIPELINE_SERVER_HPP_
