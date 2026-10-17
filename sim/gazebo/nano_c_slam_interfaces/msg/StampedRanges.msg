# Gazebo LaserScan measurement time in the simulation clock domain.
std_msgs/Header header
# Identity from the subscribed /<drone>/scan topic namespace.
string drone_id
# Monotonic per-drone sequence; resets when the bridge process restarts.
uint64 sequence
# Millimeters in front, back, left, right order; -1 means invalid/out of range.
int32[4] ranges_mm
