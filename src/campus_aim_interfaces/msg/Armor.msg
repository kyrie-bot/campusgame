# Position in map metres; yaw is the inward horizontal normal in map radians.
geometry_msgs/Point position
float64 yaw
float64 yaw_variance
float64 position_variance
# CNN identity: 0 unknown; 1..5 numbered robots; 6 outpost; 7 sentry; 8 base.
int32 number
float32 confidence
bool large
float64 width_m
float64 height_m
float64 reprojection_error
float64 image_center_distance
geometry_msgs/Point alternative_position
float64 alternative_yaw
bool has_alternative
float64 tangential_variance
float64 tilt
geometry_msgs/Point32[4] image_points
