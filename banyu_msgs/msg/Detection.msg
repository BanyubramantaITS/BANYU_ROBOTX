std_msgs/Header header

# object class id
#   0 = gate                 (PoR)
#   1 = yellow buoy / marker (PoR)
#   2 = GREEN colour blob    (Task 2)
#   3 = RED colour blob      (Task 2)
#
# Task 2 deliberately does NOT distinguish a buoy Color Indicator from a
# pipeline light ring here. Both are just a red or green blob to a camera, and
# separating them would need a classifier that only ever re-derives something
# the mission already knows: during the transit phase a colour blob is an
# indicator, during the survey phase it is a light. Colour is what the detector
# can actually measure, so colour is all it reports.
int64 object_id

# normalized horizontal offset from image center, [-1, 1]
float32 offset_x

# normalized vertical offset from image center, [-1, 1]
float32 offset_y

# apparent size normalized to image dimension, (0, 1]
float32 apparent_size

# estimated distance to object [m]
float32 distance

# detection confidence [0, 1]
float32 confidence
