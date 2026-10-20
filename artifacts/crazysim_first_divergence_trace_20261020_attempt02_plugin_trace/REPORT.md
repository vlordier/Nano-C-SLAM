# CrazySim first-divergence trace: positive run 02

Date: 2026-10-20. This is diagnostic evidence from the reference CrazySim SITL
firmware. It does not run the Nano-C-SLAM STM32 application.

## Captured path

The same CrazySim `cf2` firmware was run with a 32,768-record firmware trace
ring and the Gazebo plugin's bounded receive trace enabled. The plugin trace
stores host monotonic arrival time, the exact 9-byte UDP motor packet, four
little-endian decoded ratios and four motor angular-velocity commands. All
21,435 plugin records were retained (`dropped=0`). The plugin's packet header
was `0x90`, and all four decoded ratios matched the bytes in every record.
The capture has 7,611 odometry, 18,134 IMU, 36,269 actuator command, 18,135
joint-state and 287 world-stat samples. The full JSONL streams and firmware
log are gzip-compressed alongside this report; the exact world and plugin
summary are also retained.

## Observed behavior

- Firmware packets reached the plugin: 13,361 receive records carried at
  least one nonzero ratio; the largest decoded ratio was 37,672.
- The actuator command topic carried nonzero speeds (maximum 2,103.94 rad/s),
  and the joint-state stream records the physical rotor response.
- Odometry rose from 2.015 m to 2.402 m. The captured end was 2.313 m, so the
  run did not finish landing at the 2.015 m pad.
- Maximum odometry tilt was only 0.279 degrees; the 1-degree divergence event
  required by this bead did not occur.
- Aggregate RTF was 0.905, outside the registered 0.95–1.05 window (median
  per-sample RTF 0.955; observed range 0.013–4.445).
- The firmware ring and plugin trace are not tied by a verified clock mapping.
  The ring uses RTOS ticks; the plugin uses host monotonic nanoseconds. A
  sequence match can associate repeated motor-ratio windows, but it does not
  establish an exact tick-to-Gazebo timestamp uncertainty bound at divergence.

The trace therefore verifies the packet-to-decoder boundary and observed
actuator activity during climb/hold. It does not establish stable flight,
completed landing, a first-divergence motor mapping, or Nano-C-SLAM execution.
The paired strict checker also fails this run; its exact output is
`CHECKER_OUTPUT.json`.
