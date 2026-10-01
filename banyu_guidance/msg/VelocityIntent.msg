# Bounded velocity intent in the vehicle body frame (FLU): x forward, y left.
# This is an intent only; a separate authority is responsible for arbitration
# and conversion to a vehicle setpoint.
std_msgs/Header header

uint8 STOPPED=0
uint8 STALE=1
uint8 INVALID=2
uint8 LOST=3
uint8 TRACKING=4
uint8 status

# Body-frame linear velocities [m/s]. These are bounded by the guidance node.
float32 surge_mps
float32 sway_mps

# Positive is counter-clockwise about body +z [rad/s]. Always zero when
# yaw_control_active is false.
float32 yaw_rate_rad_s
bool yaw_control_active
