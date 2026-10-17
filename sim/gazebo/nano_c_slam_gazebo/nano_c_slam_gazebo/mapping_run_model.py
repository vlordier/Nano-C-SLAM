"""Pure configuration and child-command construction for a mapping run."""

from __future__ import annotations

import math
import shutil
from dataclasses import dataclass, field
from pathlib import Path

from .mission_model import (
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


@dataclass(frozen=True)
class MappingRunConfig:
    """Validated immutable inputs shared by every process in one run."""

    output_path: Path
    binary_path: Path
    drone_name: str = "cf1"
    scene: str = "nearfield-mapping"
    sample_count: int = 60
    timeout_s: float = 60.0
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
        if (
            isinstance(self.timeout_s, bool)
            or not isinstance(self.timeout_s, (int, float))
            or not math.isfinite(self.timeout_s)
            or self.timeout_s <= 0
        ):
            raise ValueError("timeout_s must be finite and positive")
        validate_run_id(self.run_id)
        if not isinstance(self.output_path, Path) or not isinstance(
            self.binary_path, Path
        ):
            raise ValueError("output_path and binary_path must be pathlib Paths")
        output = self.output_path.resolve()
        capture = self.capture_path.resolve()
        if output.exists() or capture.exists():
            raise FileExistsError("run output path already exists")
        if not output.parent.is_dir():
            raise FileNotFoundError(f"output directory does not exist: {output.parent}")

    @property
    def capture_path(self) -> Path:
        """Return the run-scoped intermediate capture report path."""
        return self.output_path.with_name(
            f".{self.output_path.stem}.{self.run_id}.capture.json"
        )

    def build_commands(
        self,
        *,
        python_executable: str,
        ros2_executable: str,
        collector_script: Path,
    ) -> MappingRunCommands:
        """Build argv without a shell and bind child settings to this run."""
        python_path = shutil.which(python_executable) or python_executable
        ros2_path = shutil.which(ros2_executable) or ros2_executable
        collector = (
            python_path,
            str(collector_script.resolve()),
            "--drone",
            self.drone_name,
            "--run-id",
            self.run_id,
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
