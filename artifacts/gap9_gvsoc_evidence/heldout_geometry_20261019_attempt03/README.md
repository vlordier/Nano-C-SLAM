# Held-out GAP9/GVSOC geometry run — attempt 03

This is the first valid live run on the independent `heldout-asymmetric` route.
The Gazebo world remained active while its exact SDF/session identity was
attested. The run completed with 60 paired samples and poses, 64 live SPI
frames, three scan anchors (`[0, 20, 40]`), and two accepted constraints.

The frozen profile `gap9-nearfield-square-synthetic-v1` has SHA-256
`1cbbadcf784e9e212507147a09e5e31e7d57f179a2982d176f2d137eed54b4a3`. The
held-out SDF SHA-256 is
`20eb690d09ca1e2b2974a4adaf345bc308b0eba6be12cfeb70e4e27b554eda7b`. The
validator correctly reports **FAIL**: pose XY RMSE is 0.03482 m against the
unchanged 0.03 m maximum. XY max, yaw, and map-surface gates pass; the target
map contains 225 points, all within 5 cm of a modeled surface.

The geometry report also includes diagnostic scan-anchor errors: XY RMSE
0.03505 m and max 0.04871 m. These are not a substitute for the frozen
all-pose gate. See `geometry-metrics.json` and `geometry-validation.json` for
the complete machine-readable results and `world/` for the active-world
attestation and exact SDF.

This validates only the GAP9 mapping firmware path under GVSOC with idealized
simulator-derived four-ray ranges. It does not validate physical ToF/UWB
sensors, STM32 flight firmware, or real-world mapping quality.
