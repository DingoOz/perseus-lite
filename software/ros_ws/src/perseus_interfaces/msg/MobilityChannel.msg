# One commanded-versus-measured motion channel of perseus_monitoring's mobility_monitor.
# Examples: "drive_linear" (commanded vs wheel speed: are the servos keeping up?),
# "yaw_slip" (wheel yaw rate vs IMU gyro: are the wheels turning the body?).

string name
# Signed values from the most recent sample, in m/s or rad/s.
float64 commanded
float64 measured
# Mean of (1 - measured/commanded) over the detection window: 0 = fully achieved,
# 1 = no motion at all. Can go negative when the robot moves faster than asked.
float64 ratio
# False while the command is too small, too new (grace period), or inputs are stale.
bool evaluating
# True while evaluating and ratio >= the channel's threshold.
bool fault
