# perseus_landmarks' landmark_localiser state, on /landmarks/localisation.

std_msgs/Header header
# False until the first accepted fix. Until then site -> map is not published.
bool calibrated
string site_frame
string map_frame
# The current site -> map transform (planar): a point p_map is p_site = R(yaw) p_map + (x, y).
float64 x
float64 y
float64 yaw
# RMS distance between the known tag positions and where the last accepted fix puts the
# observed tags, metres. Jumps when a detection is spurious or a tag has moved.
float64 residual_m
# Tags used by the last accepted fix.
int32[] tags_used
# Fixes accepted / rejected since start.
uint32 accepted
uint32 rejected
# Time of the last accepted fix.
builtin_interfaces/Time last_fix
