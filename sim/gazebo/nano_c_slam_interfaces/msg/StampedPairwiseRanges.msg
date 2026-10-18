# Pairwise ideal distances derived from one synchronized simulator odometry cache.
std_msgs/Header header
# Per-bridge-process sequence; starts at zero and increments on published updates.
uint64 sequence
# Stable order for cf1/cf3/cf4; source_stamps uses the same order.
string[] drone_ids
# Exact source odometry timestamp used for each corresponding drone position.
builtin_interfaces/Time[] source_stamps
# Stable pair order, e.g. cf1:cf3,cf1:cf4,cf3:cf4.
string[] pair_labels
# Ideal 3D center distances computed from the source odometry positions, in meters.
float32[] ranges_m
