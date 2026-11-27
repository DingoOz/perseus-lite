# One fiducial (AprilTag) seen by perseus_landmarks' tag_detector.

int32 id
# Tag family, e.g. "36h11".
string family
# Edge length of the tag's black square used for the pose, in metres.
float64 size_m
# Tag pose in the detection header's frame (the camera optical frame).
# Tag frame: origin at the tag centre, x right and y up as seen by a viewer facing
# the tag, z out of the tag towards the viewer.
geometry_msgs/Pose pose
# RMS reprojection error of the four corners with that pose, pixels.
float64 reprojection_error_px
# Reprojection error of the second-best pose over the best (IPPE gives two). A planar
# square seen nearly head-on, or small and far, fits two mirror-tilted poses almost
# equally well: below ~3 the tag's ORIENTATION is unreliable (errors of tens of
# degrees, even flips), while its POSITION is fine either way.
float64 ambiguity_ratio
# Distance from the camera to the tag centre, metres.
float64 distance_m
# Angle between the camera ray to the tag and the tag's normal, radians (0 = head on).
float64 view_angle_rad
# Whether the tag id is in the landmark map (its size came from the map).
bool known
