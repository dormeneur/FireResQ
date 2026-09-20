# Believed state of one victim, owned exclusively by the world model.

uint8 UNKNOWN=0
uint8 DETECTED=1
uint8 TARGETED=2
uint8 CARRIED=3
uint8 RESCUED=4
uint8 UNREACHABLE=5

string id
geometry_msgs/Point position
float32 confidence
uint8 status

# Derived reasoning factors, filled by cognition.
float32 risk
float32 accessibility
float32 rescue_cost

builtin_interfaces/Time last_seen
