"""Run a takeoff-hover-land mission using the Gazebo velocity controller."""

from __future__ import annotations

from collections.abc import Sequence

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from .mission_model import MissionConfig, MissionPhase, MissionSequence


class SimMission(Node):
    """Drive one simulated Crazyflie through a bounded vertical mission."""

    def __init__(self) -> None:
        super().__init__("nano_c_slam_sim_mission")
        self.declare_parameter("drone_name", "cf1")
        self.declare_parameter("hover_height_m", 0.5)
        self.declare_parameter("hover_duration_s", 3.0)
        self._drone_name = str(self.get_parameter("drone_name").value)
        if not self._drone_name.startswith("cf"):
            raise ValueError("drone_name must be a Crazyflie name such as cf1")
        sequence = MissionSequence(
            config=MissionConfig(
                hover_height_m=float(self.get_parameter("hover_height_m").value),
                hover_duration_s=float(self.get_parameter("hover_duration_s").value),
            )
        )
        self._sequence = sequence
        self._altitude_m: float | None = None
        self._last_phase = MissionPhase.WAITING
        self._publisher = self.create_publisher(
            Twist, f"/{self._drone_name}/cmd_vel", 10
        )
        self._enable_publisher = self.create_publisher(
            Bool, f"/{self._drone_name}/enable", 10
        )
        self.create_subscription(
            Odometry,
            f"/{self._drone_name}/odom",
            self._on_odometry,
            10,
        )
        self._timer = self.create_timer(0.05, self._step)
        self.get_logger().info(
            f"Waiting for /{self._drone_name}/odom before takeoff to "
            f"{sequence.config.hover_height_m:.2f} m"
        )

    def _on_odometry(self, message: Odometry) -> None:
        self._altitude_m = float(message.pose.pose.position.z)

    def _step(self) -> None:
        if self._sequence.phase in (MissionPhase.COMPLETE, MissionPhase.FAILED):
            self._publish_zero_command()
            if self._sequence.phase is MissionPhase.COMPLETE:
                self._timer.cancel()
            return

        enabled = Bool()
        enabled.data = True
        self._enable_publisher.publish(enabled)
        command = Twist()
        command.linear.z = self._sequence.step(
            self.get_clock().now().nanoseconds / 1e9,
            self._altitude_m,
        )
        self._publisher.publish(command)
        if self._sequence.phase is self._last_phase:
            return
        self._last_phase = self._sequence.phase
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


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimMission()
    try:
        while rclpy.ok() and node._sequence.phase is not MissionPhase.COMPLETE:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
