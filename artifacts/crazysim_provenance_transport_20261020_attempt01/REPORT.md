# Provenance transport attempt blocked at Gazebo startup

This was an attempted no-command runtime preflight with the matching Crazyflie firmware source at `d64ae14f`, CrazySim plugin at `02429e8`, and the native Gazebo Harmonic installation. No flight command was sent.

The firmware `cf2` target compiled successfully in Lima Ubuntu 24.04 with `CRAZYSIM_IMU_PROVENANCE=ON`, `CRAZYSIM_TRACE_RING=ON`, `CRAZYSIM_TRACE_DIAGNOSTICS=ON`, 500-Hz main loop, trace capacity 8,192 and stride 4. Binary SHA-256: `0d393dbf2eb77071ec14703c9467fca64a9fc5cf9ccaef109f95bbf5ea24dc86`.

Gazebo started loading the world but could not load Gazebo Physics 7.8.0's DART engine. The plugin requested `libdart-collision-bullet.6.19.dylib`; this host has DART 6.20.1 only. Gazebo crashed before publishing topics. The bounded typed sampler timed out with zero publishers, and `cf2` stayed at `Waiting for connection with gazebo`. These are startup-failure observations, not valid sensor or flight measurements.

The exact world, logs, sampler output, and empty topic files are retained in this directory. The next run requires a coherent Gazebo/DART install (or a verified earlier runtime environment). Do not alias a 6.20 library as 6.19. Nano-C-SLAM firmware and mapping were not exercised.
