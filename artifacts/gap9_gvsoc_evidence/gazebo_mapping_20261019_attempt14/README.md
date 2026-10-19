# Live GAP9 mapping geometry run

Run `db260b0f-447e-4dcf-a177-a93dac8cec61` completed a near-field square
mapping mission in Gazebo while the production GAP9 Nano-C-SLAM main processed
live telemetry in GVSOC. The mission reported `COMPLETE`, captured 60/60 host
samples, streamed 64 SPI frames, produced 60 finite firmware poses and 190 finite
map points, and captured the expected 18-byte optimizer response. The supervised
GVSOC and Gazebo children both exited 0. `world-input-attestation.json` binds the
retained SDF to the exact SDF argument observed on the live Gazebo server; its
SHA-256 is `5d2d9cc1a54bcfc7e9782254bdfa26b6c1de9e2f3a99431aadb5174681815a2`.

`geometry-metrics.json` measures same-world-frame pose differences and map-point
distance to SDF surfaces. It reports 60 poses, XY RMSE 0.02093 m (maximum
0.03975 m), yaw RMSE 0.03249 rad, and 190 map points with median surface
distance 0.01580 m, p95 0.04522 m, and maximum 0.14147 m. These are measurements
from one synthetic square-route run; they are not acceptance thresholds and do
not establish repeatability. Independent held-out geometry validation remains
open.

This run does not validate physical ToF or UWB behavior, real-world mapping
accuracy, or Crazyflie STM32 flight firmware in Gazebo. The bridge supplies
idealized simulator-derived four-ray ranges. The exact modified Python source
patch present during capture is retained as `source-dirty.patch`; the run bundle
also records the source revision and dirty-patch digest.

Revalidate the target capture:

```sh
artifact_dir=artifacts/gap9_gvsoc_evidence/gazebo_mapping_20261019_attempt14
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
  --output /tmp/attempt14-target-revalidated.json
```

Recompute geometry measurements:

```sh
uv run --locked python -m scripts.analyze_gap9_target_mapping_geometry \
  --target-report "$artifact_dir/target-evidence.json" \
  --target-log "$artifact_dir/gvsoc.log" \
  --target-elf "$artifact_dir/nano_c_slam_gvsoc_spi_main_probe" \
  --gvsoc-config "$artifact_dir/gvsoc-config.json" \
  --host-transfer "$artifact_dir/host-transfer.json" \
  --run-bundle "$artifact_dir/gazebo-run.json" \
  --world-session "$artifact_dir/world-session.json" \
  --world-attestation "$artifact_dir/world-input-attestation.json" \
  --world-sdf "$artifact_dir/world.sdf" \
  --expected-route square \
  --output /tmp/attempt14-geometry-recomputed.json
```
