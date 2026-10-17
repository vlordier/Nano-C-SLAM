# Nano-C-SLAM Gazebo interfaces

`StampedRanges` is a Gazebo-only transport for the four-ray Crazyswarm2 lidar
adapter. `header.stamp` is copied from the input `sensor_msgs/LaserScan` and
uses the simulator clock. `drone_id` comes from the subscribed topic namespace;
`sequence` starts at zero and increases per drone for each accepted scan; it
restarts with the bridge process. `ranges_mm` has exactly four signed integer millimeter values
in `front, back, left, right` order; `-1` marks an invalid reading.

This message does not represent the physical 8x8 ToF sensor or firmware wire
format. Consumers must reject uninitialized or future timestamps and must pair
ranges with odometry using the same simulation clock and a bounded skew. The
generated Python setter does not enforce fixed array length, so the bridge
validates payloads before publishing; never serialize malformed messages to
test their length.
