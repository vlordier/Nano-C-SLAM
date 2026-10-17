# Gazebo range telemetry contract

The Gazebo mapper needs a range measurement time to decide whether a range
sample and an odometry pose describe approximately the same simulator state.
The existing bridge receives `sensor_msgs/LaserScan.header.stamp` but its
`Float32MultiArray` output discards that time; the collector then pairs the most
recent independent messages. That proves that both topics advance, not that
they are synchronized.

The replacement contract is
`nano_c_slam_interfaces/msg/StampedRanges`. Its fields are:

| Field | Meaning |
| --- | --- |
| `header.stamp` | Unmodified source stamp from the Gazebo `LaserScan`, in the simulator clock domain. |
| `drone_id` | Vehicle name from the subscribed `/<drone>/scan` topic. |
| `sequence` | Per-drone increasing `uint64`, incremented only for a published valid scan; restarts when the bridge process restarts. |
| `ranges_mm[4]` | Signed integer millimeters in front, back, left, right order; `-1` means invalid. |

The message is a Gazebo adapter contract, not the physical ToF format or
firmware wire protocol. ROS's generated Python setter does not enforce the
declared fixed array length; a malformed three-value instance crashed the
local CDR serializer. The bridge must validate that exactly four in-range
integer values are present before assigning or publishing. Consumers must not
serialize malformed generated messages as a validation technique. The mapping consumer must compare range and odometry
stamps only when both use the same simulator clock. It must reject zero,
future, duplicate and out-of-order stamps, and must retain source timestamps
and the selected pair delta in the evidence report. Source age is measured in
simulator time; command timeout is measured separately with a host monotonic
clock. A simulation clock reset starts a new source-time generation and must
clear queued samples without resetting the still-running bridge sequence.

Initial pairing bounds are 500 ms maximum source age and 100 ms maximum pair
skew. These are provisional engineering limits anchored to the existing
`MAX_ODOMETRY_AGE_S` and `MAX_ODOMETRY_SKEW_S` used by the Gazebo bridge, and
the 10 Hz mapping capture cadence. The live capture must report observed age
and skew and pass within these bounds; if normal sensor cadence cannot satisfy
them, revise the limits from measured timing evidence before accepting a
capture. Do not widen a bound just to make a run pass.

Use a single atomic message rather than a second metadata topic. A parallel
topic can lose or reorder metadata independently from the corresponding
values, recreating the pairing ambiguity. The bridge now replaces the old
unstamped message type on the same topic; there is no legacy fallback in the
mapping acceptance path. The current
depth-10 publisher/subscriber profile is retained during migration, and the
per-drone sequence makes dropped samples observable. QoS compatibility must be
checked in the ROS integration smoke.

The package follows ROS 2's separate interface-package pattern: an
`ament_cmake`/`rosidl` package generates the message, and the Python node
package declares it as a runtime dependency. The upstream Crazyswarm2 workspace
must build `nano_c_slam_interfaces` and `nano_c_slam_gazebo` together.

## Current implementation boundary

The generated message package, bridge publisher, pure pairing policy, and
collector subscriber are implemented. Both the interface and node package
build under the local ROS 2 Jazzy installation. The collector records original
source stamps, selected skew, and rejection counts. The report still does not
prove mission completion or landing; that evidence is tracked separately.
