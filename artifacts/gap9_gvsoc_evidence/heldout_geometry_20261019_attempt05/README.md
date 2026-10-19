# Held-out GAP9/GVSOC geometry repeat — attempt 05

This is a fresh repeat of attempt 03 using the exact same independent
`heldout-asymmetric` route and SDF, with a distinct Gazebo session. It completed
with 60 paired samples and poses, 64 live SPI frames, three scan anchors
(`[0, 20, 40]`), and two accepted constraints.

The frozen profile `gap9-nearfield-square-synthetic-v1` has SHA-256
`1cbbadcf784e9e212507147a09e5e31e7d57f179a2982d176f2d137eed54b4a3`. The
held-out SDF SHA-256 is
`20eb690d09ca1e2b2974a4adaf345bc308b0eba6be12cfeb70e4e27b554eda7b`. The
validator correctly reports **FAIL**: pose XY RMSE is 0.03992 m against the
unchanged 0.03 m maximum. XY max, yaw, and map-surface gates pass; the target
map contains 224 points, with 97.77% within 5 cm of a modeled surface.

Diagnostic scan-anchor errors are XY RMSE 0.03949 m and max 0.05434 m. They
confirm the all-pose exceedance is not explained only by repeated poses between
scan anchors. See `geometry-metrics.json`, `geometry-validation.json`, and
`world/` for complete metrics and provenance. Compare with attempt 03, whose
all-pose RMSE was 0.03482 m and anchor RMSE 0.03505 m.

This validates only the GAP9 mapping firmware path under GVSOC with idealized
simulator-derived four-ray ranges. It does not validate physical ToF/UWB
sensors, STM32 flight firmware, or real-world mapping quality. Root-cause
analysis and a fresh passing run are tracked by bead `ncs-fea.4.9.4`.
