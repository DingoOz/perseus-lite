# Published by perseus_monitoring's mobility_monitor on /mobility/status.
std_msgs/Header header
perseus_interfaces/MobilityChannel[] channels
# A drive channel is faulted: the servos are not reaching the commanded speed
# (bogged down, stalled, current-limited or disconnected).
bool stalled
# A slip channel is faulted: the wheels turn but the body does not follow.
bool slipping
