# One ping from the active buoy's pinger, from hydrophone arrival-time differences
# (banyu_detection/pinger_localization). The array only resolves the pinger's azimuth: there is no
# range or elevation.
std_msgs/Header header
float32 forward            # horizontal unit-vector component along the body x axis
float32 left               # ... along the body y axis
float32 bearing            # rad, atan2(left, forward); > 0 = left of the nose
