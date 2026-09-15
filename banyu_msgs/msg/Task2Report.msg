std_msgs/Header header

# Unique identifier for this Task 2 run; an all-zero UUID is invalid.
unique_identifier_msgs/UUID run_id

# Terminal status: UNKNOWN, COMPLETED, FAILED, or ABORTED.
uint8 STATUS_UNKNOWN=0
uint8 COMPLETED=1
uint8 FAILED=2
uint8 ABORTED=3
uint8 terminal_status

# Repair status: NOT_ATTEMPTED, HOLD_COMPLETED_UNVERIFIED, VERIFIED, or FAILED (REPAIR_FAILED).
uint8 NOT_ATTEMPTED=0
uint8 HOLD_COMPLETED_UNVERIFIED=1
uint8 VERIFIED=2
uint8 REPAIR_FAILED=3
uint8 repair_status

# Duration of the repair hold in seconds.
float32 repair_hold_duration_s

# Human-readable terminal or repair detail.
string detail

# Whether the synchronized surface ENU anchor fix and map pose are present.
bool has_enu_anchor

# Synchronized surface ENU anchor GPS fix and corresponding map pose. The map
# frame is ENU: x=east, y=north, z=up.
sensor_msgs/NavSatFix enu_anchor_fix
geometry_msgs/PoseStamped enu_anchor_pose_map

# Ordered ring records from the first pass.
Task2RingRecord[] rings

# Index into rings for the red ring, or -1 when absent.
int32 red_ring_record_index

# Presence and map-frame pose for the first-pass start.
bool has_first_pass_start_pose
geometry_msgs/PoseStamped first_pass_start_pose_map

# Presence and map-frame pose for the first-pass end.
bool has_first_pass_end_pose
geometry_msgs/PoseStamped first_pass_end_pose_map

# Presence and map-frame pose for the final surface location.
bool has_final_surface_pose
geometry_msgs/PoseStamped final_surface_pose_map
