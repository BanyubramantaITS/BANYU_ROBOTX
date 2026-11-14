# Source frame identifier for the accepted ring observation.
uint64 frame_sequence
uint32 run_id
uint64 epoch
uint32 node_id
bool ordering_verified
bool physical_identity_verified
float64 pipeline_progress_m

# Ring color using PipelineRingObservation constants.
uint8 color

# Observation confidence in [0, 1].
float32 confidence

# Vehicle pose associated with the observation, expressed in the map frame.
# vehicle_pose_map.header.stamp is the source-image timestamp. The pose is
# interpolated or selected at that timestamp within the configured maximum
# timestamp skew.
geometry_msgs/PoseStamped vehicle_pose_map

# The ring tracker merged this record into another one (one node recorded twice, seen from far or beside the line):
# it is not a node. Its node_id is never reused.
bool superseded
