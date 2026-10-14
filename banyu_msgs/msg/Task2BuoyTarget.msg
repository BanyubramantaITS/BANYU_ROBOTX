# /banyu/task2/buoy_target — validated Task 2 active buoy from the USV PipelineBuoys feed.
# Published only once the active (GREEN) buoy is known; the mission keeps its configured
# target until then.
std_msgs/Header header
string course_id
# PipelineBuoys.seq of the source observation
uint32 source_seq
# rx_msgs/VehicleType of the observer
uint8 source_vehicle
# active buoy, WGS-84 degrees
float64 latitude
float64 longitude
# bearing from the active buoy toward the other buoy (pipe direction), ENU radians
bool heading_valid
float64 heading_enu_rad
# horizontal position error, metres (default applied when the USV sent 0 = unknown)
float64 uncertainty_m
# true once the UUV has dived in the current run; later USV updates are ignored
bool locked
