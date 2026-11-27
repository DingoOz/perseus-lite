# One snapshot of every topic perseus_monitoring's topic_health_monitor watches.
std_msgs/Header header
perseus_interfaces/TopicHealth[] topics
# True when every topic is STATUS_OK.
bool all_ok
