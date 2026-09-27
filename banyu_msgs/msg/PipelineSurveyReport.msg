# RobotX 2026 Task 2 Core Tier minimum response (handbook p.65): "a report of
# the pipeline status to the RoboCommand system. The order of reporting begins
# with the part of the pipeline closest to the active buoy."
#
# This mirrors the handbook's description of RoboCommand's PipelineSurveyReport
# ("communicates the active buoy position and the ordered condition of pipeline
# segments", handbook 3.4.12). It is NOT the wire format: RoboCommand speaks
# protobuf over MQTT, and banyu_robocommand currently holds no source at all
# (only stale __pycache__), so nothing can encode that yet. Whatever revives
# that package should translate this message rather than re-derive it.

std_msgs/Header header

# Where the UUV was when it identified the GREEN (active) buoy, in the EKF
# local ENU frame. Segment order below is measured from here.
geometry_msgs/Point active_buoy_position

# The same, as the surface GPS fix taken at the buoy: RoboCommand's
# PipelineSurveyReport.active_buoy_position is a LatLng. NaN when no fix.
float64 active_buoy_latitude
float64 active_buoy_longitude

# Condition of each pipeline segment, ordered from the segment nearest the
# active buoy outward. One entry per light node seen.
uint8 SEGMENT_INTACT=0    # GREEN light
uint8 SEGMENT_DAMAGED=1   # RED light
uint8[] segment_states
