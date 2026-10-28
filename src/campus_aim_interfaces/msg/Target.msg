uint32 track_id
# CNN identity, not player index: 0 unknown; 1..5 robots; 6 outpost; 7 sentry; 8 base.
int32 number
int32 armor_count
string state
geometry_msgs/Point center
geometry_msgs/Vector3 velocity
float64 yaw
float64 yaw_rate
float64 radius
float64 radius_odd
float64 height_difference
float64 height_difference_second
float64 width_m
float64 height_m
bool[4] observed
# Historical measurements of the current track, kept by the tracker across plate switches.
bool[4] seen
float64 position_variance
float64 last_observation_age
float64 tilt
float64 yaw_variance
bool geometry_observable
# Angular uncertainty at the image timestamp, used to bound flight-time predictions.
float64 yaw_rate_variance
float64 yaw_yaw_rate_covariance
# Row-major covariance of cx,vx,cy,vy,z,vz,phase,omega,r_even,r_odd,dz1,dz2.
# Retain correlations: center/phase alone can be uncertain while the observed plate is precise.
bool state_covariance_valid
float64[144] state_covariance
