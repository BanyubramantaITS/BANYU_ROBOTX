# /banyu/task4/zones — RoboCommand keep-out zones the surfaced UUV must stay out of (Task 4, a surface vehicle is a USV).
# Run-scoped: cleared at prepare_autonomy. Published on every change and at 1 Hz (transient_local), and recorded.
std_msgs/Header header
uint32 run_id
uint64 epoch
# Static zones (KEEP_OUT_ZONE) until ALL_CLEAR; WGS-84 degrees and metres.
float64[] static_latitude_deg
float64[] static_longitude_deg
float64[] static_radius_m
# Latest moving-object alert (MOVING_OBJECT); consumers predict the position from heading and speed since moving_stamp.
bool moving_active
float64 moving_latitude_deg
float64 moving_longitude_deg
# Direction of travel, ENU radians counter-clockwise from east (the alert's compass heading converted).
float64 moving_heading_enu_rad
float64 moving_speed_mps
float64 moving_radius_m
# Receipt time of the alert, on the ROS clock.
builtin_interfaces/Time moving_stamp
# Last change, for the log: "keep_out_zone", "all_clear", "moving_object", "reset".
string last_event
uint32 last_command_seq
