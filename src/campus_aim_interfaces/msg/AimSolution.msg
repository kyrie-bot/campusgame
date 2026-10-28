# One solver evaluation. Angles, identity and permission cannot be mixed across ticks.
std_msgs/Header header
builtin_interfaces/Time observation_stamp
string state_session
uint64 frame_sequence
uint32 track_id
int32 number
int32 slot
float64 yaw
float64 pitch
bool valid
bool fire
string reason
# Atomic mobile-armor execution check. Structures do not use this window.
builtin_interfaces/Time calculation_stamp
bool mobile_window_valid
float64 fire_yaw
float64 fire_pitch
float64 mobile_yaw_limit_deg
float64 mobile_pitch_limit_deg
