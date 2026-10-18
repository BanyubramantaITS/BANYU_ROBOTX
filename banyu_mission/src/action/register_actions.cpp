#include "banyu_mission/action/register_actions.hpp"

#include "banyu_mission/action/acknowledge_red_ring_node.hpp"
#include "banyu_mission/action/acquire_pipeline_node.hpp"
#include "banyu_mission/action/align_heading_node.hpp"
#include "banyu_mission/action/arm_and_set_guided.hpp"
#include "banyu_mission/action/center_on_ring_node.hpp"
#include "banyu_mission/action/dive_to_depth_node.hpp"
#include "banyu_mission/action/follow_pipeline_node.hpp"
#include "banyu_mission/action/handle_abort_or_teleop_node.hpp"
#include "banyu_mission/action/hold_probe_on_ring_node.hpp"
#include "banyu_mission/action/navigate_surface_gps_node.hpp"
#include "banyu_mission/action/por_action_nodes.hpp"
#include "banyu_mission/action/search_pipeline_node.hpp"
#include "banyu_mission/action/publish_pipeline_survey_node.hpp"
#include "banyu_mission/action/surface_node.hpp"

namespace banyu_mission
{

void registerActionNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MissionState> & state,
  std::function<void()> on_abort)
{
  factory.registerNodeType<HandleAbortOrTeleopNode>(
    "HandleAbortOrTeleop", state, std::move(on_abort));

  const auto set_mode_srv = node->has_parameter("services.set_mode") ?
    node->get_parameter("services.set_mode").as_string() :
    node->declare_parameter<std::string>("services.set_mode", "/mavros/set_mode");
  const auto arming_srv = node->has_parameter("services.arming") ?
    node->get_parameter("services.arming").as_string() :
    node->declare_parameter<std::string>("services.arming", "/mavros/cmd/arming");
  factory.registerNodeType<ArmAndSetGuided>(
    "ArmAndSetGuided", node, state, set_mode_srv, arming_srv);

  const auto ack_service = node->declare_parameter<std::string>(
    "services.acknowledge_ring_repair", "/banyu/pipeline/acknowledge_ring_repair");
  factory.registerNodeType<AcknowledgeRedRingNode>(
    "AcknowledgeRedRing", node, state, ack_service);

  const auto action_name = [node](const std::string & suffix) {
      return node->declare_parameter<std::string>("actions." + suffix, "/banyu/actions/" + suffix);
    };

  factory.registerNodeType<NavigateSurfaceGpsNode>(
    "NavigateSurfaceGps", node, action_name("navigate_surface_gps"));
  factory.registerNodeType<AlignHeadingNode>(
    "AlignHeading", node, action_name("align_heading"));
  factory.registerNodeType<DiveToDepthNode>(
    "DiveToDepth", node, action_name("dive_to_depth"));
  factory.registerNodeType<AcquirePipelineNode>(
    "AcquirePipeline", node, action_name("acquire_pipeline"));
  factory.registerNodeType<SearchPipelineNode>(
    "SearchPipeline", node, action_name("search_pipeline"));
  factory.registerNodeType<FollowPipelineNode>(
    "FollowPipeline", node, action_name("follow_pipeline"));
  factory.registerNodeType<CenterOnRingNode>(
    "CenterOnRing", node, action_name("center_on_ring"));
  factory.registerNodeType<HoldProbeOnRingNode>(
    "HoldProbeOnRing", node, action_name("hold_probe_on_ring"));
  factory.registerNodeType<SurfaceNode>(
    "Surface", node, action_name("surface"));
  const std::string survey_service = node->has_parameter("services.trigger_pipeline_survey") ?
    node->get_parameter("services.trigger_pipeline_survey").as_string() :
    node->declare_parameter<std::string>(
      "services.trigger_pipeline_survey", "/banyu/communication/trigger_pipeline_survey");
  factory.registerNodeType<PublishPipelineSurveyNode>(
    "PublishPipelineSurvey", node, state, survey_service);

  factory.registerNodeType<AcquireGateNode>(
    "AcquireGate", node, action_name("acquire_gate"));
  factory.registerNodeType<SearchMarkerNode>(
    "SearchMarker", node, action_name("search_marker"));
  factory.registerNodeType<SearchGateNode>(
    "SearchGate", node, action_name("search_gate"));
  factory.registerNodeType<AlignGateNode>(
    "AlignGate", node, action_name("align_gate"));
  factory.registerNodeType<AlignMarkerNode>(
    "AlignMarker", node, action_name("align_marker"));
  factory.registerNodeType<PassGateNode>(
    "PassGate", node, action_name("pass_gate"));
  factory.registerNodeType<OrbitMarkerNode>(
    "OrbitMarker", node, action_name("orbit_marker"));
  factory.registerNodeType<TurnAroundNode>(
    "TurnAround", node, action_name("turn_around"));
}

}  // namespace banyu_mission
