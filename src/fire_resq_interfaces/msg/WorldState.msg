# The robot's complete belief about the scenario.
# Positions are in the 'map' frame. Nothing here is hardcoded at startup.

std_msgs/Header header

VictimState[] victims

geometry_msgs/Point fire_position
float32 fire_confidence
bool fire_known

geometry_msgs/Pose safe_zone_pose
geometry_msgs/Pose robot_pose

string current_target_id
