# Every tag found in one image. Published for every processed image, even when empty,
# so the rate doubles as the detector's liveness.
std_msgs/Header header
perseus_interfaces/TagDetection[] detections
