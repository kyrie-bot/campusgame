# An atomic planning snapshot; all positions are in header.frame_id metres.
std_msgs/Header header
uint64 goal_id
bool valid
string status
nav_msgs/Path path
# Static obstacles already inflated for the robot footprint.
nav_msgs/OccupancyGrid static_grid
geometry_msgs/Point goal
builtin_interfaces/Time enemy_stamp
# Radii include own footprint, margin and uncertainty at header.stamp.
geometry_msgs/Point[] enemy_centres
float64[] enemy_radii
# Body + own footprint + margin, excluding broadcast uncertainty. Optional;
# absent metadata forbids recovery from inside an uncertainty margin.
float64[] enemy_hard_radii
bool[] enemy_mobile
float64 enemy_speed_bound_mps
# Current view transform, updated atomically with the route after explicit alignment.
float64[] unity_to_map
