# header.stamp is the source image acquisition time; header.frame_id is the
# camera optical frame. Paired pipeline and ring outputs from one image share
# header, frame_sequence, and frame_id. frame_sequence advances for every
# processed image, including INVALID and NO_PIPE; regressions are invalid
# during a run.
std_msgs/Header header

# Strictly increasing source frame identifier.
uint64 frame_sequence

# Detector state: INVALID = unusable, NO_PIPE = no pipeline, TRACKED = valid controls.
uint8 INVALID=0
uint8 NO_PIPE=1
uint8 TRACKED=2
uint8 status

# TRACKED requires finite, in-range controls and confidence: offsets are in
# [-1, 1], heading_error_rad is in [-pi/2, pi/2], and confidence is in [0, 1].
# For NO_PIPE or INVALID, control fields are NaN and must not be consumed.
# Positive x is right and positive y is down.
float32 offset_x
float32 offset_y

# Shortest undirected line heading error in [-pi/2, pi/2], zero when aligned
# with the camera-image direction corresponding to vehicle-forward; positive
# is clockwise.
float32 heading_error_rad

# Detector confidence in [0, 1].
float32 confidence

# Apparent pipe width, edge to edge, as a fraction of the image width; NaN when not measured. With the pipe's real
# diameter and the camera's field of view it gives the range to the pipe (no depth sensor needed).
float32 width
