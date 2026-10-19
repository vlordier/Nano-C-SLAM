# Held-out geometry attempt 11: symmetric ToF center bins

This fresh `heldout-asymmetric` Gazebo mission exercised the GAP9 production
mapping entry point under GVSOC during the simulated Crazyflie mission. The
synthetic adapter encoded each of the four cardinal ranges in ToF columns 3 and
4, whose firmware projection angles are +pi/64 and -pi/64. The two projected
rays are symmetric around the cardinal direction. The 97-byte packet layout
did not change.

The mission completed with 60 host samples, 60 target poses, three registered
scans, three optimizer edges, and 460 finite target map points. Against the
exact attested SDF (`20eb690d09ca1e2b2974a4adaf345bc308b0eba6be12cfeb70e4e27b554eda7b`),
the frozen profile `gap9-nearfield-square-synthetic-v1`
(`1cbbadcf784e9e212507147a09e5e31e7d57f179a2982d176f2d137eed54b4a3`) passes:

- XY RMSE 0.019594 m; max XY error 0.024407 m.
- Yaw RMSE 0.009619 rad; max yaw error 0.019160 rad.
- Surface median 0.014722 m; p95 0.038368 m; max 0.050625 m.
- 99.78% of map points are within 0.05 m of the SDF surface.

The prior attempt09 used the one-bin adapter, also completed a fresh held-out
mission on this same SDF, and had XY RMSE 0.042254 m with 224 map points. The
missions were not replayed from identical captured telemetry, so this is an
unpaired before/after observation. It supports the column-centering hypothesis
but does not isolate it as the sole cause of the improvement. The frozen
profile was not modified.

The world session and SDF were attested while the Gazebo server was active.
`gazebo-run.json`, `host-transfer.json`, `gvsoc-config.json`, GVSOC logs, target
ELF, `target-evidence.json`, and `geometry-metrics.json` retain the run inputs
and hashes. This validates the GAP9 mapping probe on synthetic four-ray ranges
in GVSOC; it does not validate physical ToF fidelity, UWB, STM32 flight
firmware, or hardware flight.
