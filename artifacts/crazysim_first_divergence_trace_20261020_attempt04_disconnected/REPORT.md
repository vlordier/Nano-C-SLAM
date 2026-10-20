# CrazySim actuator-disconnected control: run 04

Date: 2026-10-20. This uses the reference CrazySim SITL firmware, not the
Nano-C-SLAM STM32 application. The plugin explicitly reports
`actuator_bridge=disconnected` and records each received motor packet before
suppressing actuator publication.

## Evidence

- 16,301 plugin receive records were retained, with no drops. Each packet had
  the expected 9-byte format and all four decoded ratios matched its bytes.
- 10,733 records carried at least one nonzero firmware ratio (maximum ratio
  65,535; maximum decoded speed 2,774.98 rad/s).
- Gazebo's motor command topic had 32,497 component values and every value was
  zero. All four rotor joint velocities remained at numerical zero (maximum
  magnitude about 1.5e-26 rad/s).
- Odometry stayed at the spawn height: 2.014990–2.014999 m, maximum rise
  9.24e-6 m. The drone remained grounded while the firmware requested thrust.
- Capture includes 6,500 odometry, 16,251 IMU, 16,249 joint-state, 292
  world-stat samples and the firmware console log.

This is a valid actuator-path negative control: firmware output and packet
reception are present, but disconnecting the bridge prevents motor commands,
rotor response and takeoff. It is not a flight-acceptance run. Its aggregate
RTF was 0.811, and it does not provide the >1-degree event or firmware/Gazebo
clock mapping required by the first-divergence bead. The paired full checker output is retained in
`../attempt02_plugin_trace/CHECKER_OUTPUT.json`; this directory has a
negative-control-specific evidence summary in `NEGATIVE_CONTROL_SUMMARY.json`.
