# Reported state of the rescue actuator, identical in simulation and on hardware.

std_msgs/Header header

bool energized
bool attached

# Which victim is physically held, if any. In simulation this is Gazebo's own model name
# (e.g. "victim_2"), NOT a fire_resq_world_model id ("V1", "V2", ...) - the two id spaces are
# unrelated (Phase 6). DEBUG/PROVENANCE ONLY, exactly like Detection.source_backend: the rescue
# FSM must track which victim IT intended to pick up from its own state, and treat `attached`
# alone as the physical confirmation - never match this field against a world-model id.
string victim_id
