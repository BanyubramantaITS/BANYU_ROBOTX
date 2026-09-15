# Source frame identifier for the accepted ring observation.
uint64 frame_sequence

# Ring color using PipelineRingObservation constants.
uint8 color

# Observation confidence in [0, 1].
float32 confidence

# Vehicle pose associated with the observation, expressed in the map frame.
# vehicle_pose_map.header.stamp is the source-image timestamp. The pose is
# interpolated or selected at that timestamp within the configured maximum
# timestamp skew.
geometry_msgs/PoseStamped vehicle_pose_map
