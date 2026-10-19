# Live GAP9 mapping geometry measurement

This is run `8b86395f-d6cb-4b0c-b758-b7fb37e9550e`, a completed near-field Gazebo
mission whose in-flight telemetry was streamed into the Nano-C-SLAM production
GAP9 main under GVSOC. Both supervised children exited 0. The archived SDF is
byte-identical to the SDF argument recorded for the Gazebo server in
`world-session.json`.

The target emitted 60 finite optimized poses and 191 finite XY map points from
the firmware `pcl_extract_scan` path. `geometry-metrics.json` reports direct
same-world-frame pose errors and point distance to the named SDF box boundaries:

- Pose XY RMSE 0.0201 m, maximum 0.0382 m.
- Yaw RMSE 0.0322 rad (1.84°), maximum 0.0430 rad (2.46°).
- Target/source trajectory path-length ratio 0.9642.
- Map-to-SDF boundary distance median 0.0154 m, p95 0.0499 m, maximum 0.1373 m.
- 95.3% of target points are within the diagnostic 0.05 m bucket.

The 0.05 m bucket is not an acceptance threshold. This single run measures
synthetic behavior; it does not establish repeatability or validate physical
ToF/UWB behavior, real-world mapping, or Crazyflie STM32 flight firmware in
Gazebo. The sensor bridge uses idealized simulator-derived four-ray ranges.
`world-input-attestation.json` preserves the SDF byte hash checked against the
live Gazebo server argument, so geometry metrics can be revalidated after Gazebo
removes its temporary world file.

Revalidate the retained target report (with repository source files at the
captured revision) using:

```sh
artifact_dir=artifacts/gap9_gvsoc_evidence/gazebo_mapping_20261019_attempt12
uv run --locked python -m scripts.capture_gap9_gvsoc_spi_main_probe_evidence \
  --config "$artifact_dir/gvsoc-config.json" \
  --elf "$artifact_dir/nano_c_slam_gvsoc_spi_main_probe" \
  --host-transfer "$artifact_dir/host-transfer.json" \
  --log "$artifact_dir/gvsoc.log" \
  --gazebo-run-bundle "$artifact_dir/gazebo-run.json" \
  --source cslam-gap9-app/main.c \
  --source scripts/capture_gap9_gvsoc_spi_main_probe_evidence.py \
  --source scripts/gvsoc_spi_slave_probe_control.py \
  --source scripts/gap9_gvsoc_spi_probe_protocol.py \
  --source sim/gazebo/nano_c_slam_gazebo/nano_c_slam_gazebo/mapping_packets.py \
  --source sim/gazebo/nano_c_slam_gazebo/nano_c_slam_gazebo/mapping_model.py \
  --output /tmp/attempt12-revalidated.json
```

Recompute the geometry metrics from the retained files, including after the
temporary world file has been removed:

```sh
artifact_dir=artifacts/gap9_gvsoc_evidence/gazebo_mapping_20261019_attempt12
uv run --locked python -m scripts.analyze_gap9_target_mapping_geometry \
  --target-report "$artifact_dir/target-evidence.json" \
  --target-log "$artifact_dir/gvsoc.log" \
  --run-bundle "$artifact_dir/gazebo-run.json" \
  --world-session "$artifact_dir/world-session.json" \
  --world-attestation "$artifact_dir/world-input-attestation.json" \
  --world-sdf "$artifact_dir/world.sdf" \
  --output /tmp/attempt12-geometry-revalidated.json
```
