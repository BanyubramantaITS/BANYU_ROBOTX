# /banyu/surface/obstacles — fixed obstacles NavigateSurfaceGps detours round on the surface, e.g. the UAV's Task 1
# buoy field. Each message replaces the previous list of its source. Published on change and at 1 Hz (transient_local).
std_msgs/Header header
# Who mapped them, e.g. "uav_safe_passage".
string source
# WGS-84 degrees; radius = keep-out distance from the centre (obstacle size plus position error).
float64[] latitude_deg
float64[] longitude_deg
float64[] radius_m
