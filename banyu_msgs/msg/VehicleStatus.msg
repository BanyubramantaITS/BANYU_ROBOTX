# MAVROS-neutral vehicle state for mission preflight and safety gating.
std_msgs/Header header
bool fcu_connected
bool fcu_status_fresh
bool rc_fresh
bool operator_takeover
string mode
string authority
bool gps_valid
bool gps_fresh
