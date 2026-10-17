# Gazebo four-ray to mapping projection

This note records the convention used by the host mapping adapter. It is an
implementation trace, not a calibration of the physical VL53L5CX sensor.

## Source and scan order

The launch file used by this repository starts
`worlds/crazyflie_swarm_world.sdf` from the nested
`~/Work/crazyswarm2_gazebo/src/ros_gz_crazyflie` worktree at commit
`ce34fffdcef5a8cbd8885510daa0b4c9d4ad7cf7`. The containing
`crazyswarm2_gazebo` checkout was at
`5f2e4cc8e9687fb4c05e9e4be40a20370a022f9e`; neither external checkout is
modified by this repository change. The active world defines the lidar inline
on each `cfN/body` link, with zero relative sensor pose. It has four horizontal
samples, `min_angle=-3.14159`, `max_angle=1.5708`, and `resolution=1`. The
evenly spaced scan increment is
`(1.5708 - (-3.14159)) / (4 - 1) = 1.5707966667` radians. Thus the LaserScan
indices are:

| LaserScan index | SDF angle (rad) | Cardinal direction | `DIR_*` | Adapter order |
| ---: | ---: | --- | --- | ---: |
| 0 | -3.14159 | back (approximately -π) | `DIR_BACK` | 1 |
| 1 | -1.5707933 | right (approximately -π/2) | `DIR_RIGHT` | 3 |
| 2 | 0.0000033 | front (+X) | `DIR_FRONT` | 0 |
| 3 | 1.5708 | left (+Y) | `DIR_LEFT` | 2 |

The bridge adapter's `BEAM_ORDER=(2, 0, 3, 1)` consequently emits
`front, back, left, right`, matching `StampedRanges.msg` and the C enum. The
adapter now checks `angle_min` and `angle_increment` against this specific
four-ray layout. Its 0.01 rad tolerance accepts the world's rounded values
while rejecting a changed scan layout. The separate
`models/crazyflie/model.sdf` contains a related but not identical sensor
definition; it is not the world file selected by the launch we traced.

## C point projection

The host replay writes each of the four readings into ToF column 3 of its
corresponding `DIR_*` row. In `cslam-gap9-app/point-cloud/point_cloud.c`, the
eight-column angular step is `-π/32`; column 3 therefore uses
`π/8 - π/64 - 3π/32 = π/64`, or +2.8125 degrees in the local scan plane.
For a range `r` and body yaw `ψ`, C first computes `x_local = r + offset` and
`y_local = r * tan(π/64)`, then rotates by `ψ` plus the direction yaw:

| C direction | Direction yaw | Local origin offset |
| --- | ---: | ---: |
| `DIR_FRONT` | 0 | +0.025 m |
| `DIR_BACK` | +π | -0.025 m |
| `DIR_LEFT` | +π/2 | +0.020 m |
| `DIR_RIGHT` | -π/2 | -0.020 m |

These offsets and the +2.8125 degree column angle are inherited from the
firmware's eight-column model. The Gazebo sensor supplies only four cardinal
rays; mapping each ray to column 3 does not recreate the physical 8x8 ToF
matrix, its pixel geometry, sensor mounting calibration, or measurement noise.
The C replay test checks that the implementation follows this formula for all
four directions at four cardinal body yaws. That validates code convention,
not the physical sensor model or map accuracy.
