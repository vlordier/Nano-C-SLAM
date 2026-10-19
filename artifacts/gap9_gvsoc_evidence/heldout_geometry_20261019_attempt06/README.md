# Heldout GAP9/GVSOC geometry attempt 06

This is an independent live Gazebo-to-GAP9/GVSOC run for the frozen
`gap9-nearfield-square-synthetic-v1` profile. It repeats the same heldout route
and SDF as attempts 03 and 05; the profile threshold was not changed.

## Result

The run completed successfully (`acfe3bf3-897b-41bf-8e4b-a0c3934ba8cc`): 60
Gazebo telemetry samples, 64 live SPI frames, three registered scan anchors,
and zero collector/mission child exit codes. Its exact world session is
`4336e53b-db9b-42ad-8655-bb403ee34878`; the archived SDF SHA-256 is
`20eb690d09ca1e2b2974a4adaf345bc308b0eba6be12cfeb70e4e27b554eda7b`.

The frozen profile **fails** on all-pose XY RMSE: `0.0415351860 m` against
`0.03 m`. Maximum XY error is `0.0585137397 m` against `0.06 m`, and yaw RMSE
is `0.0098582530 rad` against `0.05 rad`. The projected map passes all map
gates: 224 points, surface median `0.0185824 m`, p95 `0.0448394 m`, maximum
`0.0634447 m`, and 96.875% within `0.05 m` of the SDF surfaces. These are
geometric proximity metrics; they do not establish map completeness or
physical sensor fidelity.

## What the target graph proves

The captured target log and evidence report agree on exactly two stored
constraints, from pose-table indices `20 -> 0` and `40 -> 0`. After graph
reduction, the optimizer sees exactly two unique edges, compact nodes `1 -> 0`
and `2 -> 0`. This is a three-node tree with no cycle and no independently
measured closure edge. It establishes the topology actually optimized in this
run; by itself it does not prove that adding a loop closure will fix the pose
error.

The evidence bundle is deliberately complete enough for offline inspection:

- `gazebo-run.json`, `world/`: source telemetry, session process identity,
  copied world SDF, and input attestation.
- `target-evidence.json`, `gvsoc.log`, `nano_c_slam_gvsoc_spi_main_probe`,
  `gvsoc-config.json`, `host-transfer.json`: target frames, endpoint markers,
  target binary and its run inputs.
- `geometry-metrics.json`, `geometry-validation.json`: measured metrics and
  the explicit frozen-profile FAIL.
- `gazebo.stdout.log`, `gazebo.stderr.log`, `gvsoc.stdout.log`,
  `gvsoc.stderr.log`: process output.

The run uses idealized simulator-derived four-ray ranges. It does not validate
physical ToF/UWB behavior or STM32 flight firmware.
