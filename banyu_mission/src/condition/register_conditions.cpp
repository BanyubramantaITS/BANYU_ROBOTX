#include "banyu_mission/condition/register_conditions.hpp"

#include "banyu_mission/condition/autonomy_interrupted_condition.hpp"
#include "banyu_mission/condition/pipeline_active_condition.hpp"
#include "banyu_mission/condition/pipeline_ended_condition.hpp"
#include "banyu_mission/condition/pipeline_lost_condition.hpp"
#include "banyu_mission/condition/red_ring_pending_condition.hpp"
#include "banyu_mission/condition/require_gps_available_condition.hpp"
#include "banyu_mission/condition/require_mission_started_condition.hpp"
#include "banyu_mission/condition/require_vehicle_ready_condition.hpp"
#include "banyu_mission/condition/survey_leg_conditions.hpp"

namespace banyu_mission
{

void registerConditionNodes(
  BT::BehaviorTreeFactory & factory,
  const std::shared_ptr<MissionState> & state)
{
  factory.registerNodeType<AutonomyInterruptedCondition>("AutonomyInterrupted", state);
  factory.registerNodeType<RequireVehicleReadyCondition>("RequireVehicleReady", state);
  factory.registerNodeType<RequireGpsAvailableCondition>("RequireGpsAvailable", state);
  factory.registerNodeType<RequireMissionStartedCondition>("RequireMissionStarted", state);
  factory.registerNodeType<PipelineLostCondition>("PipelineLost", state);
  factory.registerNodeType<PipelineActiveCondition>("PipelineActive", state);
  factory.registerNodeType<PipelineEndedCondition>("PipelineEnded", state);
  factory.registerNodeType<RedRingPendingCondition>("RedRingPending", state);
  factory.registerNodeType<RingsCompleteCondition>("RingsComplete", state);
  factory.registerNodeType<PipelineLegEndedCondition>("PipelineLegEnded", state);
}

}  // namespace banyu_mission
