"""Run a takeoff-hover-land mission using the Gazebo velocity controller."""

from __future__ import annotations

import math
from collections.abc import Sequence

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .mapping_path import MappingPath
from .mission_model import (
    MissionPhase,
    MissionSequence,
    MissionStatusEvent,
    create_mission_config,
    create_run_id,
    mission_exit_code,
    validate_drone_name,
)


class SimMission(Node):
    """Drive one simulated Crazyflie through a bounded vertical mission."""

    def __init__(self) -> None:
        super().__init__("nano_c_slam_sim_mission")
        self.declare_parameter("drone_name", "cf1")
        self.declare_parameter("run_id", "")
        self.declare_parameter("hover_height_m", 0.5)
        self.declare_parameter("hover_duration_s", 3.0)
        self.declare_parameter("mapping_path", False)
        self._drone_name = validate_drone_name(self.get_parameter("drone_name").value)
        configured_run_id = self.get_parameter("run_id").value
        self._run_id = (
            create_run_id() if configured_run_id == "" else str(configured_run_id)
        )
        config = create_mission_config(
            self.get_parameter("hover_height_m").value,
            self.get_parameter("hover_duration_s").value,
        )
        self._mapping_path_enabled = bool(self.get_parameter("mapping_path").value)
        sequence = MissionSequence(config=config)
        self._sequence = sequence
        self._altitude_m: float | None = None
        self._position_xy_m: tuple[float, float] | None = None
        self._odometry_stamp_s: float | None = None
        self._last_phase = MissionPhase.WAITING
        self._mapping_path: MappingPath | None = None
        self._mapping_path_finished = False
        self._status_sequence = 0
        self._publisher = self.create_publisher(
            Twist, f"/{self._drone_name}/cmd_vel", 10
        )
        self._enable_publisher = self.create_publisher(
            Bool, f"/{self._drone_name}/enable", 10
        )
        self._status_publisher = self.create_publisher(
            String, f"/{self._drone_name}/mission_status", 10
        )
        self.create_subscription(
            Odometry,
            f"/{self._drone_name}/odom",
            self._on_odometry,
            10,
        )
        self._timer = self.create_timer(0.05, self._step)
        self.get_logger().info(
            f"Mission run {self._run_id}; waiting for "
            f"/{self._drone_name}/odom before takeoff to "
            f"{sequence.config.hover_height_m:.2f} m"
        )
        self._publish_status(self.get_clock().now().nanoseconds / 1e9)

    def _on_odometry(self, message: Odometry) -> None:
        self._altitude_m = float(message.pose.pose.position.z)
        position = message.pose.pose.position
        self._position_xy_m = (float(position.x), float(position.y))
        stamp = message.header.stamp
        self._odometry_stamp_s = float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _step(self) -> None:
        if self._sequence.phase in (MissionPhase.COMPLETE, MissionPhase.FAILED):
            self._publish_zero_command()
            if self._sequence.phase is MissionPhase.COMPLETE:
                self._timer.cancel()
            elif self._sequence.failure_reason is not None:
                self.get_logger().error(
                    f"{self._drone_name} mission failed: "
                    f"{self._sequence.failure_reason}"
                )
                self._timer.cancel()
            return

        enabled = Bool()
        enabled.data = True
        self._enable_publisher.publish(enabled)
        command = Twist()
        now_s = self.get_clock().now().nanoseconds / 1e9
        command.linear.z = self._sequence.step(
            now_s,
            self._altitude_m,
            self._odometry_stamp_s,
            hold_hover=self._mapping_path_enabled and not self._mapping_path_finished,
        )
        if self._sequence.phase is MissionPhase.HOVER and self._mapping_path_enabled:
            if self._position_xy_m is None:
                self._sequence.fail("odometry position is unavailable")
            elif self._mapping_path is None:
                self._mapping_path = MappingPath(
                    *self._position_xy_m,
                    now_s,
                )
                self.get_logger().info(
                    "Starting bounded square mapping path at measured position"
                )
            else:
                command.linear.x, command.linear.y = self._mapping_path.step(
                    now_s, *self._position_xy_m
                )
                if self._mapping_path.failure_reason is not None:
                    self._sequence.fail(self._mapping_path.failure_reason)
                elif self._mapping_path.complete and not self._mapping_path_finished:
                    self._mapping_path_finished = True
                    self.get_logger().info(
                        "Mapping path complete; beginning configured hover interval"
                    )
        self._publisher.publish(command)
        if self._sequence.phase is self._last_phase:
            return
        self._last_phase = self._sequence.phase
        self._publish_status(now_s)
        altitude = (
            "unknown" if self._altitude_m is None else f"{self._altitude_m:.3f} m"
        )
        if self._sequence.phase is MissionPhase.FAILED:
            self.get_logger().error(
                f"{self._drone_name} mission failed: "
                f"{self._sequence.failure_reason} at altitude {altitude}"
            )
        elif self._sequence.phase is MissionPhase.COMPLETE:
            self.get_logger().info(
                f"{self._drone_name} completed takeoff, hover, and landing "
                f"at altitude {altitude}"
            )
        else:
            self.get_logger().info(
                f"{self._drone_name} mission phase: "
                f"{self._sequence.phase.name.lower()} at altitude {altitude}"
            )

    def _publish_zero_command(self) -> None:
        """Keep the controller commanded to hover after the mission terminates."""
        self._publisher.publish(Twist())

    def _publish_status(self, sim_time_s: float) -> None:
        """Publish structured evidence for each mission phase transition."""
        altitude_m = self._altitude_m
        if altitude_m is not None and not math.isfinite(altitude_m):
            altitude_m = None
        event = MissionStatusEvent(
            run_id=self._run_id,
            drone_name=self._drone_name,
            sequence=self._status_sequence,
            sim_time_s=sim_time_s,
            phase=self._sequence.phase,
            altitude_m=altitude_m,
            mapping_path_requested=self._mapping_path_enabled,
            mapping_path_complete=self._mapping_path_finished,
            landing_height_m=self._sequence.config.landing_height_m,
            failure_reason=self._sequence.failure_reason,
        )
        message = String()
        message.data = event.to_json()
        self._status_publisher.publish(message)
        self._status_sequence += 1
        self.get_logger().info(f"mission_status={message.data}")


def main(args: Sequence[str] | None = None) -> int:
    rclpy.init(args=args)
    node: SimMission | None = None
    exit_code = mission_exit_code(MissionPhase.FAILED, interrupted=True)
    try:
        node = SimMission()
        while rclpy.ok() and node._sequence.phase not in (
            MissionPhase.COMPLETE,
            MissionPhase.FAILED,
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = mission_exit_code(node._sequence.phase)
    except KeyboardInterrupt:
        exit_code = mission_exit_code(MissionPhase.FAILED, interrupted=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    main()
