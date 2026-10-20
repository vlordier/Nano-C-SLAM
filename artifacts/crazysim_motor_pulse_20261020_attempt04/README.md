# Isolated CrazySim motor pulse capture

This run series used one Gazebo startup per motor and a fresh `GZ_PARTITION`.
M2, M3 and M4 are the usable runs from this series. M1's odometry subscriber
started after its pulse, so the M1 response is taken from
`../crazysim_motor_pulse_20261020_attempt05/m1` instead.

`motor_pulse_publisher.cc` publishes a bounded 500 rad/s single-channel pulse
and waits for the Gazebo motor command subscriber and world statistics. Each
`mN` directory retains the SDF, command stream, world stats, odometry, process
exit codes and logs. M3's 500 rad/s angular-rate output became invalid; the
lower-speed repeat is archived separately in
`../crazysim_motor_pulse_20261020_attempt11_m3_100`.

The world has zero gravity and no ground contact. These data check attitude
response signs only; they do not validate thrust, hover, or flight. The M1
through M4 results and limitations are summarized in
[`docs/STM32_SITL_REFERENCE.md`](../../docs/STM32_SITL_REFERENCE.md).
