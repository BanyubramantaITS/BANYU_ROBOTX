# header.stamp is the source image acquisition time; header.frame_id is the
# camera optical frame. Paired pipeline and ring outputs from one image share
# header, frame_sequence, and frame_id. frame_sequence advances for every
# processed image, including INVALID and NO_PIPE; regressions are invalid
# during a run.
std_msgs/Header header

# Strictly increasing source frame identifier; no mission sequence index.
uint64 frame_sequence

# Whether a ring is currently visible.
bool visible

# Ring color: UNKNOWN = unset, RED, GREEN, or BLUE.
uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_GREEN=2
uint8 COLOR_BLUE=3
uint8 color

# A visible ring has a known color, finite offsets, and finite confidence;
# offsets are in [-1, 1] and confidence is in [0, 1]. An invisible ring has
# color COLOR_UNKNOWN, NaN offsets, and zero confidence.
float32 confidence

# Normalized image offset in [-1, 1]; positive x is right and positive y is down.
float32 offset_x
float32 offset_y
