# Fresh single-drone host mapping acceptance (2026-10-19)

`final-integrated.json` is the combined output from one new native macOS
Gazebo session (ROS domain 57, unique Gazebo partition) coordinated by
`scripts.gazebo_mapping_run`. The independent bundle verifier accepts the
report and its sidecar manifest.

Observed result: mission `COMPLETE`; collector and mission exit codes both 0;
60 poses, 3 lidar scan windows, 197 projected map points, 3 stored pose-graph
constraints, 60 finite poses, and optimizer completion. The world-session hook
recorded the Ruby-wrapped Gazebo server PID and its ROS Launch parent. Passive
`lsof -a -p <PID> -d txt,mem` evidence observed 382 mapped libraries, including
47 files from `~/Work/gz-macOS/ws/install/lib`; before/after process and mapped
library evidence matched.

This proves one synthetic, single-drone host-SITL path: Gazebo ideal odometry
and four-ray lidar were captured and replayed through the owned Nano-C-SLAM C
point-cloud, ICP, and pose-graph modules using CPU PMSIS stubs. It does not prove
firmware execution, GAP9/GVSOC behavior, real 8x8 ToF or UWB fidelity, mapping
accuracy, three-drone integration, physical flight, or hardware operation.

Verify the retained report offline from the repository root:

```sh
uv run --locked python -m scripts.gazebo_run_manifest \
  artifacts/gazebo_mapping_sitl/host_mapping_acceptance_20261019/final-integrated.json
```

The report retains source/build/runtime identities and embedded range/pose/map
records. Its child process stream files and capture stdout are retained beside
it under the hidden run-scoped paths. The captured output path recorded in the
original run points into `/tmp`; all stream contents needed for offline bundle
verification are included here.
