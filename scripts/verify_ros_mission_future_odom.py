"""Exercise the native ROS mission callback with isolated test publishers."""

from __future__ import annotations

import json
import os
import secrets
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path
from typing import BinaryIO, cast

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "sim/gazebo/nano_c_slam_gazebo"
MISSION_NODE_NAME = "nano_c_slam_sim_mission"
DRONE_NAME = "cf1"


class MissionProbe(Node):
    """Publish simulated time/odometry and observe mission outputs."""

    def __init__(self) -> None:
        super().__init__("nano_c_slam_future_odom_probe")
        self._clock_publisher = self.create_publisher(
            Clock,
            "/clock",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._odometry_publisher = self.create_publisher(
            Odometry, f"/{DRONE_NAME}/odom", 10
        )
        self.status_events: list[tuple[float, dict[str, object]]] = []
        self.velocity_commands: list[tuple[float, float]] = []
        self.create_subscription(
            String,
            f"/{DRONE_NAME}/mission_status",
            self._on_status,
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self.create_subscription(
            Twist,
            f"/{DRONE_NAME}/cmd_vel",
            self._on_command,
            10,
        )

    def _on_status(self, message: String) -> None:
        event = json.loads(message.data)
        if not isinstance(event, dict):
            raise ValueError("mission status event is not a JSON object")
        self.status_events.append((time.monotonic(), cast(dict[str, object], event)))

    def _on_command(self, message: Twist) -> None:
        self.velocity_commands.append((time.monotonic(), float(message.linear.z)))

    def _publish_clock(self, time_s: float) -> None:
        whole_seconds = int(time_s)
        message = Clock()
        message.clock.sec = whole_seconds
        message.clock.nanosec = round((time_s - whole_seconds) * 1_000_000_000)
        self._clock_publisher.publish(message)

    def _publish_odometry(self, *, stamp_s: float, altitude_m: float) -> None:
        whole_seconds = int(stamp_s)
        message = Odometry()
        message.header.stamp.sec = whole_seconds
        message.header.stamp.nanosec = round((stamp_s - whole_seconds) * 1_000_000_000)
        message.pose.pose.position.z = altitude_m
        self._odometry_publisher.publish(message)

    def _spin_for(
        self, duration_s: float, *, clock_s: float, child: subprocess.Popen[bytes]
    ) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._publish_clock(clock_s)
            rclpy.spin_once(self, timeout_sec=0.01)
            if child.poll() is not None:
                raise RuntimeError("mission node exited before the probe completed")

    def _wait_for_phase(
        self,
        phase: str,
        *,
        clock_s: float,
        timeout_s: float,
        run_id: str,
        child: subprocess.Popen[bytes],
    ) -> tuple[float, dict[str, object]]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for observed_at, event in self.status_events:
                if event.get("phase") == phase and event.get("run_id") == run_id:
                    return observed_at, event
            self._publish_clock(clock_s)
            rclpy.spin_once(self, timeout_sec=0.01)
            if child.poll() is not None:
                raise RuntimeError(
                    f"mission node exited before publishing {phase} status"
                )
        raise TimeoutError(f"mission node did not publish {phase} status")


def _mission_command(run_id: str) -> list[str]:
    return [
        sys.executable,
        "-c",
        "import sys; from nano_c_slam_gazebo.sim_mission import main; "
        "raise SystemExit(main(sys.argv[1:]))",
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        f"drone_name:={DRONE_NAME}",
        "-p",
        f"run_id:={run_id}",
    ]


def run_probe(domain_id: int) -> dict[str, object]:
    """Run the real mission node and require exact boundary/failure behavior."""
    run_id = str(uuid.uuid4())
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(PACKAGE_ROOT), environment.get("PYTHONPATH", "")]
    )
    environment["PYTHONUNBUFFERED"] = "1"
    probe = MissionProbe()
    child_log = tempfile.TemporaryFile()  # noqa: SIM115 -- open for child lifetime
    child = subprocess.Popen(  # noqa: S603 -- fixed owned mission module and args
        _mission_command(run_id),
        cwd=REPOSITORY_ROOT,
        env=environment,
        stdout=child_log,
        stderr=subprocess.STDOUT,
    )
    try:
        probe._spin_for(2.0, clock_s=100.0, child=child)
        graph_names = {name for name, _ in probe.get_node_names_and_namespaces()}
        if MISSION_NODE_NAME not in graph_names:
            raise RuntimeError("mission node did not join the isolated ROS graph")

        # The first sample is exactly 100 ms ahead of /clock and must be accepted.
        probe._publish_odometry(stamp_s=100.1, altitude_m=0.0)
        probe._spin_for(0.3, clock_s=100.0, child=child)
        probe._spin_for(0.3, clock_s=100.05, child=child)
        _, takeoff = probe._wait_for_phase(
            "TAKEOFF",
            clock_s=100.05,
            timeout_s=3.0,
            run_id=run_id,
            child=child,
        )
        if takeoff.get("odometry_stamp_s") != 100.1:
            raise RuntimeError("100 ms boundary sample was not retained")
        if not any(command > 0 for _, command in probe.velocity_commands):
            raise RuntimeError("valid boundary sample did not reach takeoff control")

        # 101 ms ahead of simulation time must fail before another timer command.
        probe._spin_for(0.1, clock_s=100.05, child=child)
        invalid_sample_sent_at = time.monotonic()
        probe._publish_odometry(stamp_s=100.151, altitude_m=0.0)
        _, failed = probe._wait_for_phase(
            "FAILED",
            clock_s=100.05,
            timeout_s=3.0,
            run_id=run_id,
            child=child,
        )
        expected_reason = "odometry timestamp is ahead of simulation time"
        if failed.get("failure_reason") != expected_reason:
            raise RuntimeError(
                f"future odometry failure reason was {failed.get('failure_reason')!r}"
            )
        probe._spin_for(0.35, clock_s=100.05, child=child)
        post_failure_commands = [
            command
            for observed_at, command in probe.velocity_commands
            if observed_at >= invalid_sample_sent_at
        ]
        if any(command > 0 for command in post_failure_commands):
            raise RuntimeError("positive vertical command followed FAILED status")
        if not any(command == 0 for command in post_failure_commands):
            raise RuntimeError("mission did not publish zero velocity after failure")

        deadline = time.monotonic() + 2.0
        while child.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.02)
        if child.poll() != 1:
            log = _read_child_log(child_log)
            raise RuntimeError(
                f"failed mission exited with status {child.poll()}; "
                f"statuses={probe.status_events!r}; log={log}"
            )
        return {
            "result": "passed",
            "evidence_kind": "native_ros_node_contract_only",
            "gazebo_started": False,
            "ros_domain_id": domain_id,
            "run_id": run_id,
            "boundary_odometry_stamp_s": takeoff.get("odometry_stamp_s"),
            "future_failure_reason": failed.get("failure_reason"),
            "post_failure_vertical_commands": post_failure_commands,
            "mission_exit_code": child.returncode,
            "node_log": _read_child_log(child_log),
        }
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2)
        probe.destroy_node()
        child_log.close()
        if rclpy.ok():
            rclpy.shutdown()


def _read_child_log(stream: BinaryIO) -> str:
    stream.seek(0)
    return stream.read().decode("utf-8", errors="replace")[-4000:]


def main() -> int:
    domain_id = 100 + secrets.randbelow(100)
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    rclpy.init()
    try:
        print(json.dumps(run_probe(domain_id), indent=2, sort_keys=True))
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(json.dumps({"result": "failed", "reason": str(error)}, indent=2))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
