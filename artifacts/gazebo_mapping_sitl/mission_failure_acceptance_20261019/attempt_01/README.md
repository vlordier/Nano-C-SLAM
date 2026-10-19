# Gazebo mission failure behavior: clean-source trial

This retained trial exercises the single-drone `SimMission` failure path in the native macOS Gazebo Sim 8.14.0 nearfield world, using ROS 2 Jazzy and the repository's `Qlswha` overlay.

## Procedure and result

The probe required an advancing simulator clock, three fresh grounded `/cf1/odom` samples, and no existing `/cf1/cmd_vel` or `/cf1/enable` publishers. It launched one mission with run ID `4970184e-2a29-49db-9be0-2c3c3c6eb363`. After the run-bound `HOVER` status, it injected one `/cf1/odom` message whose x position was NaN. The mission rejected it as `odometry position is not finite`, emitted `FAILED`, and exited with code 1.

After `FAILED`, 12 `/cf1/cmd_vel` messages were observed; all 12 had zero linear and angular components. No `enable=True` event followed `FAILED`. The probe then observed odometry for 5.001 seconds after mission exit (249 samples): altitude stayed between 0.585655 and 0.586174 m, and maximum absolute vertical velocity was 0.000214 m/s. The probe classified the result as **held near a steady altitude during the observed window**.

The loaded Gazebo MulticopterVelocityControl plugin was observed while the server was running and hashed: `libgz-sim8-multicopter-control-system.8.14.0.dylib`, SHA-256 `d2791ec20f8cd8150dbffb39c58bfa5a4cd87baeda64236d6b806b18b4133568`. The overlay-installed `sim_mission.py` hash matched the repository source hash. The source checkout was clean at revision `184da3fb97da151990e44f6a5270af4f76b18aa6`; the overlay itself was built from source revision `68222cf3e6369844277123a86b4007fa380c6e32`, with its receipt and snapshot retained under `overlay/`.

## Scope and limitations

This demonstrates the behavior of this failure path with this Gazebo version and loaded controller plugin. The observed hover is only a five-second software-in-the-loop result; it does not establish physical safety or hardware behavior. The injected failure was a malformed non-finite position sample, not stale odometry or a waypoint timeout. Those triggers remain untested. The probe's post-stop process audit found no process for its own partition; unrelated Durandal Gazebo processes (PIDs 38302 and 39029) remained running and were not touched.

The probe report and raw mission logs are retained in `probe.json`, `mission.stdout.log`, and `mission.stderr.log`. `commands.txt`, `world-session.json`, `processes.after-stop.txt`, plugin identity, overlay build receipt, and source snapshot retain the run context. The report SHA-256 is `09301d9922baaccb283f8c17b4cf4d62b70e145e0ce6404ca3503ed7d23d4c76`.
