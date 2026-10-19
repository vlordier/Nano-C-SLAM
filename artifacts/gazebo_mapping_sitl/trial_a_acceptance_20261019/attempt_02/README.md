# Trial A attempt 02 — native Gazebo mapping run

This is the successful retry for trial A. The run is independent of attempt 01, whose outputs polluted the source snapshot and invalidated its bundle. Attempt 02 wrote all generated output under `/private/tmp` until capture finished, then copied the bundle here.

## Identity and isolation

- Native macOS arm64; ROS domain 55; Gazebo partition `nano_c_slam_trial_a_retry_20261019`.
- Gazebo session UUID: `3bd895ec-71ad-4901-a793-31b282d1071e`.
- Mapping run UUID: `2e6f94c2-7bac-4e8a-b76e-2ba04b0b17f1`.
- Run source revision: `f8df7216c037e7e4352fadc263414cf56f578de8`, clean at execution. Prelaunch process/domain audit is from `808168a`; that interval contains evidence-only commits and no code changes.
- Overlay: `build/gazebo-ros-overlay.ZxGDQ3/install`, build receipt SHA-256 `861023e20cc4902697c07ea41a0aa5e0ca3b2c79ecef117684714e19cb51b43b`.
- Prelaunch audit found no Nano-C-SLAM process conflicts and an isolated ROS graph. It saw unrelated Durandal server/gui PIDs 38302 and 39029; those were left running.

## What passed

- Grounded odometry preflight passed with three ordered samples at x/y = (-0.5, -0.5) m and z ≈ -0.00000085 m; all samples were within the configured age/gap/altitude bounds.
- Before mission process creation, `/cf1/cmd_vel` and `/cf1/enable` each had zero publishers.
- Mission events were bound to the run UUID and reached `WAITING → TAKEOFF → HOVER → LAND → COMPLETE`; mapping path completion was true and both mission and collector exited 0.
- Collector captured 60 synchronized samples across 3 windows. The host mapping replay reported 60 finite poses, 199 map points, 3 graph nodes, 3 optimizer input constraints (3 unique edges), 2 accepted ICP insertions, and 9 completed solver iterations.
- Offline provenance verification passed after the bundle was copied to this committed location. Raw mission/collector logs and capture sidecars are preserved beside the report.
- Session-owned PIDs checked after stop: 76824, 76842, 76843, 76844, 77026 were absent. The post-stop process scan is retained in `processes.after-stop.txt`.

## Claim boundary and remaining caveat

This proves one native Gazebo mission and host execution of the owned C mapping/ICP/pose-graph pipeline using CPU PMSIS stubs with synthetic four-ray lidar and simulated odometry. It does not establish map accuracy/completeness, GAP9/GVSOC execution, real ToF/UWB behavior, simultaneous multi-drone SLAM, or flight hardware operation. The coordinator report covers mission/collector exit codes; shutdown of the separately launched Gazebo/ROS process tree is only evidenced by the PID absence scan, not by a full raw teardown transcript in this attempt.

## Reproduction

The exact coordinator argv is in `coordinator-result.json`; its report is `run/mapping-run.json`. To re-verify offline from the repository root:

```sh
uv run --locked python -m scripts.gazebo_run_manifest artifacts/gazebo_mapping_sitl/trial_a_acceptance_20261019/attempt_02/run/mapping-run.json
```
