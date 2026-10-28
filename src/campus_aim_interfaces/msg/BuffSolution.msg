std_msgs/Header header
builtin_interfaces/Time observation_stamp
string state_session
uint64 frame_sequence
uint64 goal_id
uint64 request_id
bool valid
# Explicitly selects the measured big-rune tracking observer; small defaults false.
bool big_rune
bool fire
# Single-use fire token; repeated solutions never authorize another round.
uint64 shot_id
float64 yaw
float64 pitch
float64 horizon
float64 command_latency
# Big-rune ballistic trajectory at header.stamp (degrees and seconds).
# Small-rune aiming does not use these fields.
bool trajectory_valid
float64 yaw_rate
float64 pitch_rate
float64 yaw_acceleration
float64 pitch_acceleration
string reason
