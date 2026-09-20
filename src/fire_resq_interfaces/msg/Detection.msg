# A single camera-agnostic detection.
# Perception publishes these; nothing above perception may branch on how they were produced.

string CLASS_VICTIM=victim
string CLASS_FIRE=fire
string CLASS_OBSTACLE=obstacle

std_msgs/Header header

string class_id
float32 confidence

# Image-space bounding box (in the frame of the source image).
vision_msgs/BoundingBox2D bbox

# Estimated position, expressed in header.frame_id (normally 'map').
geometry_msgs/Point position
bool position_valid

# Which backend produced this. DEBUG/PROVENANCE ONLY.
# Cognition must never branch on this field - see Architecture.md section 4.
string source_backend

# TODO(phase-future): add position covariance for uncertainty propagation.
