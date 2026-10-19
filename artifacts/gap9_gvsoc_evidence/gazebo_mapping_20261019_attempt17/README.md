# Live GAP9 mapping geometry repeat

Run `d40c82db-d5af-46ec-82e9-7a9630dc13ce` is a fourth independent baseline
square mission using a fresh Gazebo world session and isolated partition/domain.
It completed with 60/60 host samples, 64 SPI frames, 60 finite firmware poses,
192 finite firmware map points, a captured 18-byte optimizer response, and zero
return codes from both mission/collector and both live-supervised children. The
exact server SDF is archived and SHA-256 bound as
`5d2d9cc1a54bcfc7e9782254bdfa26b6c1de9e2f3a99431aadb5174681815a2`.

The geometry measurement reports pose XY RMSE 0.02032 m, yaw RMSE 0.03160 rad,
and map-to-SDF surface distances of median 0.01450 m, p95 0.04823 m, maximum
0.13418 m. These measurements are diagnostic and do not define acceptance
thresholds. This run produced 192 finite map points; run-to-run variation still
needs measurement before preregistering any thresholds.

The sensor bridge supplies idealized simulator-derived four-ray ranges. This
run does not validate physical ToF/UWB behavior, real-world mapping, or
Crazyflie STM32 flight firmware in Gazebo.
