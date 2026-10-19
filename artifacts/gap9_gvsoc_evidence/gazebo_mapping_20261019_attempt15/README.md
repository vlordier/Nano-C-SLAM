# Live GAP9 mapping geometry repeat

Run `8a4bf4c4-de73-4b27-bb6c-7d9a7fd73c86` is an independent baseline square
mission using a fresh Gazebo world session and isolated partition/domain. It
completed with 60/60 host samples, 64 SPI frames, 60 finite firmware poses, 189
finite firmware map points, a captured 18-byte optimizer response, and zero
return codes from both mission/collector and both live-supervised children.
The exact server SDF is archived and SHA-256 bound as
`5d2d9cc1a54bcfc7e9782254bdfa26b6c1de9e2f3a99431aadb5174681815a2`.

The geometry measurement reports pose XY RMSE 0.02096 m, yaw RMSE 0.03296 rad,
and map-to-SDF surface distances of median 0.01572 m, p95 0.04564 m, maximum
0.13086 m. These measurements are diagnostic and do not define acceptance
thresholds. The map contained one fewer point than attempt14; repeated runs are
needed to characterize this variation before preregistering any thresholds.

The sensor bridge supplies idealized simulator-derived four-ray ranges. This
run does not validate physical ToF/UWB behavior, real-world mapping, or
Crazyflie STM32 flight firmware in Gazebo.
