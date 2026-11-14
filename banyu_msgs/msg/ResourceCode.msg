# Result of reading the post-repair flash code on a Task 2 light node (Advanced and Disruptive tiers).
# Published about once a second while a decode window is open, and once more when it closes.
std_msgs/Header header
uint32 run_id
uint64 epoch
# True when the node showed steady green after the repair (the magnetic switch worked).
bool repair_verified
# True once enough cycles agree. The colours below are only meaningful when complete is true.
bool complete
# Tin colour (PipelineRingObservation COLOR_*, 1..3). 0 at the Advanced tier, where the code is one colour.
uint8 resource_color
# Delivery circle colour (COLOR_*, 1..3).
uint8 delivery_circle_color
# Cycles that agree with the reported code, and cycles read at all.
uint8 cycles
uint8 cycles_seen
float32 confidence
