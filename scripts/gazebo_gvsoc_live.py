#!/usr/bin/env python3
"""Run one Gazebo mapping mission with live GAP9 firmware under GVSOC."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import cast

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.live_gvsoc_processes import (
    LiveGvsocCommands,
    supervise_live_gvsoc_mapping,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
    create_run_id,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_COUNT = 60
PACKET_COUNT = 64
LIVE_MISSION_HOVER_SECONDS = 120.0


def _parse_args(args: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", choices=SUPPORTED_DRONE_NAMES, default="cf1")
    parser.add_argument(
        "--scene", choices=("default", "nearfield-mapping"), default="nearfield-mapping"
    )
    parser.add_argument(
        "--mapping-route",
        choices=("square", "heldout-asymmetric"),
        default="square",
    )
    parser.add_argument("--samples", type=int, default=SAMPLE_COUNT)
    parser.add_argument("--mission-timeout", type=float, default=600.0)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--stream-timeout", type=float, default=15.0)
    parser.add_argument(
        "--mission-hover-duration",
        type=float,
        default=LIVE_MISSION_HOVER_SECONDS,
        help="simulated seconds to hover after mapping before landing",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(tempfile.gettempdir()) / "nano-c-slam-gazebo-gvsoc-live.json",
    )
    parser.add_argument(
        "--binary",
        type=Path,
        default=REPOSITORY_ROOT / "cslam-gap9-app/BUILD-host/mapping_replay",
    )
    parser.add_argument(
        "--ros-python",
        type=Path,
        default=Path(
            os.environ.get(
                "NANO_C_SLAM_ROS_PYTHON",
                "/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11",
            )
        ),
    )
    options = parser.parse_args(args)
    if options.samples != SAMPLE_COUNT:
        parser.error(
            "live GAP9 probe currently requires exactly 60 samples (64 frames)"
        )
    for value, label in (
        (options.mission_timeout, "mission timeout"),
        (options.timeout, "live run timeout"),
        (options.stream_timeout, "stream timeout"),
        (options.mission_hover_duration, "mission hover duration"),
    ):
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{label} must be finite and positive")
    output = options.output.resolve()
    if output.exists():
        parser.error(f"run output already exists: {output}")
    if not output.parent.is_dir():
        parser.error(f"run output directory does not exist: {output.parent}")
    options.output = output
    return options


def _commands(
    options: argparse.Namespace, run_id: str
) -> tuple[LiveGvsocCommands, Path, Path, Path]:
    output = cast(Path, options.output)
    run_root = output.with_name(f".{output.stem}.{run_id}.live")
    socket_path = Path("/tmp") / f"ncsl-{run_id}.sock"  # noqa: S108 -- keep the Unix socket path below macOS's limit
    ready_path = Path("/tmp") / f"ncsl-{run_id}.ready"  # noqa: S108 -- paired with the short socket endpoint
    if len(str(socket_path).encode()) >= 104:
        raise ValueError("generated path is too long for the local GVSOC Unix socket")
    work_root = run_root / "gvsoc-work"
    gvsoc_workdir_link = Path("/tmp") / f"ncsl-gvsoc-{run_id}"  # noqa: S108 -- GVSOC requires a short absolute workdir link
    environment = {
        "GAP9_BUILD_GVSOC_SPI_MAIN_PROBE": "1",
        "GAP9_BUILD_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT": str(PACKET_COUNT),
        "GAP9_GVSOC_ENABLE_TESTBENCH": "1",
        "GAP9_GVSOC_SPI_SEQUENCE": "gazebo-live",
        "GAP9_GVSOC_SPI_CAPTURE_RESPONSE": "1",
        "GAP9_GVSOC_SPI_STREAM_SOCKET": str(socket_path),
        "GAP9_GVSOC_SPI_STREAM_READY": str(ready_path),
        "GAP9_GVSOC_SPI_STREAM_RUN_ID": run_id,
        "GAP9_GVSOC_SPI_STREAM_DRONE": options.drone,
        "GAP9_GVSOC_SPI_STREAM_TIMEOUT_S": str(options.stream_timeout),
        "GAP9_GVSOC_TIMEOUT_S": str(options.timeout),
        "GAP9_GVSOC_WORK_ROOT": str(work_root),
        "GAP9_GVSOC_WORKDIR_LINK": str(gvsoc_workdir_link),
        "GAP9_GVSOC_ELF": str(
            REPOSITORY_ROOT
            / "cslam-gap9-app/BUILD-gvsoc-spi-main-probe"
            / "nano_c_slam_gvsoc_spi_main_probe"
        ),
        "GAP9_GVSOC_CONTROL_SCRIPT": str(
            REPOSITORY_ROOT / "scripts/gvsoc_spi_slave_probe_control.py"
        ),
        "GAP9_GVSOC_EXPECTED_MARKER": (
            "NANO_C_SLAM_GVSOC_SPI_MAIN:gazebo_capture_ok packets=64"
        ),
        "GAP9_GVSOC_COMPLETION_MARKER": (
            "NANO_C_SLAM_GVSOC_SPI_HOST:live_response_rx_hex="
        ),
    }
    gazebo = (
        shutil.which("uv") or "uv",
        "run",
        "--no-project",
        "--python",
        str(cast(Path, options.ros_python).resolve()),
        "-m",
        "scripts.gazebo_mapping_run",
        "--drone",
        options.drone,
        "--scene",
        options.scene,
        "--mapping-route",
        options.mapping_route,
        "--samples",
        str(options.samples),
        "--timeout",
        str(options.mission_timeout),
        "--run-id",
        run_id,
        "--stream-socket",
        str(socket_path),
        "--stream-timeout",
        str(options.stream_timeout),
        "--mission-hover-duration",
        str(options.mission_hover_duration),
        "--output",
        str(output),
        "--binary",
        str(options.binary.resolve()),
    )
    return (
        LiveGvsocCommands(
            gvsoc=("bash", str(REPOSITORY_ROOT / "scripts/run_gap9_gvsoc_smoke.sh")),
            gazebo=gazebo,
            gvsoc_environment=environment,
            gazebo_environment={
                "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "42"),
                "GZ_PARTITION": os.environ.get(
                    "GZ_PARTITION", f"nano_c_slam_{os.getuid()}"
                ),
            },
            working_directory=REPOSITORY_ROOT,
        ),
        socket_path,
        ready_path,
        run_root,
    )


def main(args: list[str] | None = None) -> int:
    options = _parse_args(args)
    run_id = create_run_id()
    try:
        if shutil.which("ros2") is None:
            raise FileNotFoundError("ROS 2 executable is not on PATH")
        build_environment = os.environ.copy()
        build_environment["NANO_C_SLAM_HOST_SITL"] = str(options.binary.resolve())
        subprocess.run(  # noqa: S603 -- fixed repository build script
            [str(REPOSITORY_ROOT / "scripts/build_host_mapping_sitl.sh")],
            check=True,
            cwd=REPOSITORY_ROOT,
            env=build_environment,
            timeout=120.0,
        )
        commands, socket_path, ready_path, run_root = _commands(options, run_id)
        result = supervise_live_gvsoc_mapping(
            commands,
            run_id=run_id,
            socket_path=socket_path,
            readiness_path=ready_path,
            log_directory=run_root,
            timeout_s=options.timeout,
        )
    except (
        FileExistsError,
        FileNotFoundError,
        OSError,
        RuntimeError,
        subprocess.SubprocessError,
        TimeoutError,
        ValueError,
    ) as error:
        print(
            f"Live Gazebo/GVSOC mapping failed (run_id={run_id}): {error}",
            file=sys.stderr,
        )
        return 1
    print(
        json.dumps(
            {
                "run_id": run_id,
                "gvsoc_returncode": result.gvsoc_returncode,
                "gazebo_returncode": result.gazebo_returncode,
                "gvsoc_stdout": str(result.gvsoc_stdout),
                "gvsoc_stderr": str(result.gvsoc_stderr),
                "gazebo_stdout": str(result.gazebo_stdout),
                "gazebo_stderr": str(result.gazebo_stderr),
                "firmware_host_report": str(
                    run_root / "gvsoc-work/spi_slave_probe_host.json"
                ),
                "mapping_run_report": str(options.output),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
