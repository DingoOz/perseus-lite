# Health of one monitored topic, measured by perseus_monitoring's topic_health_monitor.

uint8 STATUS_OK = 0
# Publishing, but below rate_tolerance x expected_rate_hz.
uint8 STATUS_SLOW = 1
# At least one publisher exists, but nothing arrived for stale_timeout_s.
uint8 STATUS_STALE = 2
# Nobody publishes this topic (or its type could not be resolved yet).
uint8 STATUS_NO_PUBLISHER = 3

string topic
# Message type as resolved from the ROS graph, e.g. sensor_msgs/msg/Image. Empty until known.
string type
uint8 status
# Configured expectation; 0 means "any rate is fine, only check staleness".
float64 expected_rate_hz
# Measured over the monitor's sliding window.
float64 rate_hz
float64 bandwidth_bytes_per_s
# Seconds since the last message arrived; negative if none has ever arrived.
float64 age_s
uint32 publisher_count
