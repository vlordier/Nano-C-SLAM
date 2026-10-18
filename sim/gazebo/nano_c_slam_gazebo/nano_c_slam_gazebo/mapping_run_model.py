"""Pure configuration and child-command construction for a mapping run."""

from __future__ import annotations

import math
import shutil
from dataclasses import dataclass, field
from pathlib import Path

from .mission_model import (
    MissionPhase,
    MissionStatusEvent,
    create_run_id,
    validate_drone_name,
    validate_run_id,
)

MINIMUM_RUN_SAMPLES = 60
MAXIMUM_RUN_SAMPLES = 170


@dataclass(frozen=True)
class MappingRunCommands:
    """Exact argv vectors for one telemetry collector and one mission process."""

    collector: tuple[str, ...]
    mission: tuple[str, ...]


class MissionCaptureGate:
    """Ignore unrelated mission topics and gate telemetry on this run taking off."""

    def __init__(self, run_id: str, drone_name: str) -> None:
        self.run_id = validate_run_id(run_id)
        self.drone_name = validate_drone_name(drone_name)
        self._next_sequence = 0
        self.started = False
        self.started_at_s: float | None = None
        self.failure_reason: str | None = None
        self.rejected_status_count = 0

    def observe(self, payload: str) -> bool:
        """Accept the expected ordered status stream and report active flight."""
        if self.failure_reason is not None:
            return False
        try:
            event = MissionStatusEvent.from_json(payload)
        except ValueError as error:
            self.failure_reason = f"invalid mission status: {error}"
            return False
        if event.run_id != self.run_id or event.drone_name != self.drone_name:
            self.rejected_status_count += 1
            return False
        if event.sequence != self._next_sequence:
            self.failure_reason = "mission status sequence is missing or reordered"
            return False
        self._next_sequence += 1
        if event.phase is MissionPhase.FAILED:
            self.failure_reason = f"mission failed: {event.failure_reason}"
            return False
        if event.phase is MissionPhase.COMPLETE:
            self.failure_reason = (
                "mission completed before telemetry target was captured"
            )
            return False
        capture_phase = (
            event.phase is MissionPhase.HOVER
            if event.mapping_path_requested
            else event.phase in (MissionPhase.TAKEOFF, MissionPhase.HOVER)
        )
        if capture_phase and not self.started:
            self.started = True
            self.started_at_s = event.sim_time_s
        return self.started


@dataclass(frozen=True)
class MappingRunConfig:
    """Validated immutable inputs shared by every process in one run."""

    output_path: Path
    binary_path: Path
    drone_name: str = "cf1"
    scene: str = "nearfield-mapping"
    sample_count: int = 60
    timeout_s: float = 120.0
    run_id: str = field(default_factory=create_run_id)

    def __post_init__(self) -> None:
        validate_drone_name(self.drone_name)
        if self.scene not in ("default", "nearfield-mapping"):
            raise ValueError("scene must be default or nearfield-mapping")
        if (
            isinstance(self.sample_count, bool)
            or not isinstance(self.sample_count, int)
            or not MINIMUM_RUN_SAMPLES <= self.sample_count <= MAXIMUM_RUN_SAMPLES
        ):
            raise ValueError(
                f"sample_count must be between {MINIMUM_RUN_SAMPLES} and "
                f"{MAXIMUM_RUN_SAMPLES}"
            )
        if isinstance(self.timeout_s, bool) or not isinstance(
            self.timeout_s, (int, float)
        ):
            raise ValueError("timeout_s must be finite and positive")
        try:
            timeout_is_finite = math.isfinite(self.timeout_s)
        except OverflowError as error:
            raise ValueError("timeout_s must be finite and positive") from error
        if not timeout_is_finite or self.timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        validate_run_id(self.run_id)
        if not isinstance(self.output_path, Path) or not isinstance(
            self.binary_path, Path
        ):
            raise ValueError("output_path and binary_path must be pathlib Paths")
        output = self.output_path.resolve()
        capture = self.capture_path.resolve()
        if (
            output.exists()
            or capture.exists()
            or self.readiness_path.resolve().exists()
            or self.log_directory.resolve().exists()
        ):
            raise FileExistsError("run output path already exists")
        if not output.parent.is_dir():
            raise FileNotFoundError(f"output directory does not exist: {output.parent}")

    @property
    def capture_path(self) -> Path:
        """Return the run-scoped intermediate capture report path."""
        return self.output_path.with_name(
            f".{self.output_path.stem}.{self.run_id}.capture.json"
        )

    @property
    def readiness_path(self) -> Path:
        """Return the collector's subscription-ready marker path."""
        return self.capture_path.with_suffix(".ready")

    @property
    def log_directory(self) -> Path:
        """Return a run-scoped directory for retained child logs."""
        return self.output_path.with_name(
            f".{self.output_path.stem}.{self.run_id}.logs"
        )

    def build_commands(
        self,
        *,
        python_executable: str,
        ros2_executable: str,
    ) -> MappingRunCommands:
        """Build argv without a shell and bind child settings to this run."""
        python_path = shutil.which(python_executable) or python_executable
        ros2_path = shutil.which(ros2_executable) or ros2_executable
        collector = (
            python_path,
            "-m",
            "scripts.gazebo_mapping_sitl",
            "--drone",
            self.drone_name,
            "--run-id",
            self.run_id,
            "--mission-gated",
            "--ready-file",
            str(self.readiness_path.resolve()),
            "--samples",
            str(self.sample_count),
            "--timeout",
            str(self.timeout_s),
            "--scene",
            self.scene,
            "--output",
            str(self.capture_path.resolve()),
            "--binary",
            str(self.binary_path.resolve()),
            "--reuse-binary",
        )
        mission = (
            ros2_path,
            "run",
            "nano_c_slam_gazebo",
            "sim_mission",
            "--ros-args",
            "-p",
            "use_sim_time:=True",
            "-p",
            f"drone_name:={self.drone_name}",
            "-p",
            f"run_id:={self.run_id}",
            "-p",
            "mapping_path:=True",
        )
        return MappingRunCommands(collector=collector, mission=mission)

    def validate_runtime(self, *, python_executable: str, ros2_executable: str) -> None:
        """Fail before capture if required executables or replay are missing."""
        if shutil.which(python_executable) is None:
            raise FileNotFoundError(f"Python executable not found: {python_executable}")
        if shutil.which(ros2_executable) is None:
            raise FileNotFoundError(f"ROS 2 executable not found: {ros2_executable}")
        binary = self.binary_path.resolve()
        if not binary.is_file() or not binary.stat().st_mode & 0o111:
            raise FileNotFoundError(f"host mapping executable is unavailable: {binary}")
