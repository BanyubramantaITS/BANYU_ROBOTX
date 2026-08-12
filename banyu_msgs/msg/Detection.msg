std_msgs/Header header

# object class id (0 = gate, 1 = yellow buoy / marker)
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
