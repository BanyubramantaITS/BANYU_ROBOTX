# One bottom-camera frame's view of the Task 2 pipeline (banyu_detection/pipeline_detection).
# Pixels, origin top-left, +x right, +y down. Up the image = ahead of the vehicle.
std_msgs/Header header
uint16 width
uint16 height
uint16 reference_row   # image row directly under the vehicle

bool pipe_in_view      # enough pipe pixels to hold station over
float32 centroid_x     # of all pipe pixels in view
float32 centroid_y

PipeFit ahead          # stretch ahead: for following nose first
PipeFit behind         # stretch behind: for following tail first
bool ends_ahead        # the pipe does not reach past the vehicle going nose first
bool ends_behind       # ... going tail first

LightNode[] lights
