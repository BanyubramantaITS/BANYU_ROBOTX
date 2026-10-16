#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__FOLLOW_PIPELINE_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__FOLLOW_PIPELINE_SERVER_HPP_

#include <string>

#include "banyu_guidance/msg/velocity_intent.hpp"
#include "banyu_msgs/action/follow_pipeline.hpp"
#include "banyu_msgs/msg/pipeline_observation.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class FollowPipelineServer final
  : public TypedActionServer<banyu_msgs::action::FollowPipeline>
{
public:
  FollowPipelineServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator);

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__FOLLOW_PIPELINE_SERVER_HPP_
