# Matched-rate flight instability retest (2026-10-20)

This positive-only retest used a 500-Hz firmware build, a 500-Hz Gazebo IMU,
the CrazySim motor-thrust conversion from commit `aeafa64`, and the patched
odometry publisher from `b256950`. The firmware was started before Gazebo so
the bridge had no pre-handshake sensor backlog. `TAKEOFF_2` to 2.315 m and
`LAND_2` to 2.015 m both received ACK result 0.

The captured IMU stream was 500.03 Hz with 2-ms maximum gaps; odometry was
200 Hz with 5-ms maximum gaps. The vehicle reached 3.497 m, flipped, and fell
off the raised platform to 0.015 m. It stayed within the 2.4-m hover envelope
for only 0.175 s. The firmware logged a 483/500-Hz stabilizer warning, repeated
95–98/100-Hz Kalman warnings, estimator bounds resets, and `SUP: Crashed,
recovery required`. The firmware handled SIGTERM and exited 0. Gazebo was
stopped with Ctrl-C and exited 1. Aggregate RTF was 0.9513, narrowly inside
the registered [0.95, 1.05] range; world-stat coverage was 94.1%, below 95%.

This does not pass flight acceptance. No paired negative run was made, so the
strict paired checker was not invoked. The run confirms that correct sensor
rate and the intended motor mapping alone do not make the reference flight
repeatable. Compare the first attitude divergence against the successful but
non-accepted attempt09 before changing controller gains or the registered
gate. Neither run executes Nano-C-SLAM firmware or validates mapping.

The first 510-Hz experiment is deliberately excluded: Gazebo Sensors in the
local Harmonic checkout converts `1 / update_rate` to integer milliseconds.
Thus 510 Hz became 1 ms and published at 1 kHz. Use only the generator's
supported 500- and 1000-Hz rates.

`manifest.json` records exact revisions, commands, measurements, and SHA-256
hashes. `source/` preserves the exact CrazySim bridge source bytes used for the
motor mapping; those files are reference snapshots from another repository.
