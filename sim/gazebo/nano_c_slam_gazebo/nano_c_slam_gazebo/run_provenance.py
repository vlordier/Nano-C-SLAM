"""Source and runtime provenance for a passing Gazebo mapping bundle."""

from __future__ import annotations

import hashlib
import importlib.metadata
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import cast

from scripts.runtime_provenance import (
    GAZEBO_ROS_ENVIRONMENT,
    sha256_file_stable,
)

PROVENANCE_SCHEMA_VERSION = 1
_OWNED_INPUT_PATHS = (
    "pyproject.toml",
    "scripts/gazebo_mapping_run.py",
    "scripts/gazebo_mapping_sitl.py",
    "scripts/gazebo_run_manifest.py",
    "scripts/build_host_mapping_sitl.sh",
    "sim/gazebo/nano_c_slam_gazebo/package.xml",
    "sim/gazebo/nano_c_slam_interfaces/package.xml",
    "sim/gazebo/nano_c_slam_interfaces/CMakeLists.txt",
    "sim/gazebo/nano_c_slam_interfaces/msg/StampedRanges.msg",
    "sim/gazebo/nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg",
)
_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
OWNED_INPUT_PATHS = tuple(
    sorted(
        {
            *_OWNED_INPUT_PATHS,
            *(
                path.relative_to(_REPOSITORY_ROOT).as_posix()
                for path in (
                    *_REPOSITORY_ROOT.joinpath(
                        "sim/gazebo/nano_c_slam_gazebo/nano_c_slam_gazebo"
                    ).rglob("*.py"),
                    *_REPOSITORY_ROOT.joinpath("cslam-gap9-app/point-cloud").rglob("*"),
                    *_REPOSITORY_ROOT.joinpath("cslam-gap9-app/icp").rglob("*"),
                    *_REPOSITORY_ROOT.joinpath("cslam-gap9-app/slam").rglob("*"),
                    *_REPOSITORY_ROOT.joinpath("cslam-gap9-app/host-sitl").rglob("*"),
                    *_REPOSITORY_ROOT.joinpath("tests/stubs").rglob("*"),
                )
                if path.is_file()
                and (
                    path.suffix in {".py", ".c", ".h"}
                    or path.name == "mapping_replay.c"
                )
            ),
        }
    )
)
OVERLAY_PACKAGES = (
    "nano_c_slam_gazebo",
    "nano_c_slam_interfaces",
    "ros_gz_crazyflie_gazebo",
)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _linked_library_identities(
    executable: Path, executable_name: str
) -> list[dict[str, object]]:
    """Hash direct Mach-O dependencies; retain unresolved install names as unknown."""
    inspector = shutil.which("otool")
    unknown_record: dict[str, object] = {
        "name": f"linked-libraries:{executable_name}:unresolved",
        "install_name": "<otool unavailable or binary format unsupported>",
        "path": None,
        "version": None,
        "disposition": "unknown",
        "sha256": None,
    }
    if inspector is None:
        return [unknown_record]
    result = subprocess.run(  # noqa: S603 - fixed otool query on a resolved executable
        [inspector, "-L", str(executable)],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        return [unknown_record]
    dependencies: list[dict[str, object]] = []
    for index, line in enumerate(result.stdout.splitlines()[1:]):
        install_name, separator, version_text = line.strip().partition(
            " (compatibility version "
        )
        install_name = install_name.strip()
        if not install_name:
            continue
        dependency_path = Path(install_name)
        observed_path = (
            dependency_path.resolve()
            if dependency_path.is_absolute() and dependency_path.is_file()
            else None
        )
        dependencies.append(
            {
                "name": f"linked-libraries:{executable_name}:{index}",
                "install_name": install_name,
                "path": str(observed_path) if observed_path is not None else None,
                "version": (
                    version_text.removesuffix(")").strip() if separator else None
                ),
                "disposition": "system-unpinned"
                if observed_path is not None
                else "unknown",
                "sha256": (
                    sha256_file_stable(observed_path)
                    if observed_path is not None
                    else None
                ),
            }
        )
    return dependencies or [unknown_record]


def _git_value(repository_root: Path, *arguments: str) -> str:
    git_executable = shutil.which("git")
    if git_executable is None:
        raise FileNotFoundError("Git executable is not on PATH")
    result = subprocess.run(  # noqa: S603 - fixed Git subcommands and repository path
        [git_executable, "-C", str(repository_root), *arguments],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    return result.stdout.strip()


def _package_prefix(ros2_executable: str, package: str) -> Path:
    result = subprocess.run(  # noqa: S603 - fixed package-prefix query and package names
        [ros2_executable, "pkg", "prefix", package],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    prefix = Path(result.stdout.strip()).resolve()
    if not prefix.is_dir():
        raise FileNotFoundError(f"ROS package prefix is unavailable: {package}")
    return prefix


def _hash_tree(root: Path) -> tuple[str, int]:
    """Hash every file and symlink identity under one installed package prefix."""
    digest = hashlib.sha256()
    file_count = 0
    for path in sorted(root.rglob("*")):
        if not path.is_file() and not path.is_symlink():
            continue
        relative = path.relative_to(root).as_posix()
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        if path.is_symlink():
            digest.update(path.readlink().as_posix().encode("utf-8"))
            digest.update(b"\0")
        if path.is_file():
            digest.update(bytes.fromhex(_sha256_file(path)))
        file_count += 1
    if file_count == 0:
        raise ValueError(f"installed ROS package prefix contains no files: {root}")
    return digest.hexdigest(), file_count


def collect_run_provenance(
    repository_root: Path,
    *,
    ros2_executable: str,
    mapping_binary: Path,
) -> dict[str, object]:
    """Collect fail-closed source, executable, and ROS overlay identities."""
    root = repository_root.resolve()
    ros2_path = Path(shutil.which(ros2_executable) or ros2_executable).resolve()
    gazebo_executable = shutil.which("gz")
    if not ros2_path.is_file() or gazebo_executable is None:
        raise FileNotFoundError("ROS 2 and Gazebo executables must be on PATH")
    gazebo_path = Path(gazebo_executable).resolve()
    python_path = Path(sys.executable).resolve()
    binary_path = mapping_binary.resolve()
    if not binary_path.is_file():
        raise FileNotFoundError(f"mapping executable is unavailable: {binary_path}")

    source_hashes: dict[str, str] = {}
    for relative in OWNED_INPUT_PATHS:
        source_path = root / relative
        if not source_path.is_file():
            raise FileNotFoundError(f"owned provenance input is missing: {relative}")
        source_hashes[relative] = _sha256_file(source_path)

    git_status = _git_value(
        root, "status", "--porcelain", "--untracked-files=all", "--", *OWNED_INPUT_PATHS
    )

    overlay: dict[str, object] = {}
    for package in OVERLAY_PACKAGES:
        prefix = _package_prefix(str(ros2_path), package)
        tree_hash, file_count = _hash_tree(prefix)
        overlay[package] = {
            "prefix": str(prefix),
            "sha256": tree_hash,
            "file_count": file_count,
        }

    gazebo_version = subprocess.run(  # noqa: S603 - fixed argv and resolved executable
        [str(gazebo_path), "sim", "--versions"],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    ).stdout.strip()
    if not gazebo_version:
        raise ValueError("Gazebo did not report its simulator version")
    ros2_version = _ros2_cli_version()
    prefix_environment = {
        name: os.environ.get(name, "")
        for name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH")
    }
    interface_identity = {
        "source_sha256": {
            "nano_c_slam_interfaces/msg/StampedRanges.msg": source_hashes[
                "sim/gazebo/nano_c_slam_interfaces/msg/StampedRanges.msg"
            ],
            "nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg": source_hashes[
                "sim/gazebo/nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg"
            ],
        },
        "overlay": overlay["nano_c_slam_interfaces"],
    }

    provenance: dict[str, object] = {
        "schema_version": PROVENANCE_SCHEMA_VERSION,
        "source": {
            "git_revision": _git_value(root, "rev-parse", "HEAD"),
            "git_dirty": bool(git_status),
            "git_status_porcelain": git_status,
            "owned_input_sha256": source_hashes,
        },
        "executables": {
            "python": {
                "path": str(python_path),
                "version": platform.python_version(),
                "sha256": _sha256_file(python_path),
                "linked_libraries": _linked_library_identities(python_path, "python"),
            },
            "ros2": {
                "path": str(ros2_path),
                "ros_distro": os.environ.get("ROS_DISTRO", ""),
                "cli_version": ros2_version,
                "sha256": _sha256_file(ros2_path),
                "linked_libraries": _linked_library_identities(ros2_path, "ros2"),
            },
            "gazebo": {
                "path": str(gazebo_path),
                "sim_version": gazebo_version,
                "sha256": _sha256_file(gazebo_path),
                "linked_libraries": _linked_library_identities(gazebo_path, "gazebo"),
            },
            "mapping_binary": {
                "path": str(binary_path),
                "sha256": _sha256_file(binary_path),
                "linked_libraries": _linked_library_identities(
                    binary_path, "mapping_binary"
                ),
            },
        },
        "ros_overlay_packages": overlay,
        "interface_identity": interface_identity,
        "runtime_environment": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gazebo_partition": os.environ.get("GZ_PARTITION", "default"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "<rmw-default>"),
            "prefix_paths": prefix_environment,
            "allowlisted_environment": {
                name: (
                    os.environ.get(name, "")
                    if name
                    in {"ROS_DOMAIN_ID", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"}
                    else os.environ.get(name)
                    or (
                        "default"
                        if name == "GZ_PARTITION"
                        else "<rmw-default>"
                        if name == "RMW_IMPLEMENTATION"
                        else None
                    )
                )
                for name in GAZEBO_ROS_ENVIRONMENT
            },
        },
    }
    return validate_run_provenance(provenance)


def validate_run_provenance(
    value: object, *, expected_mapping_binary_sha256: str | None = None
) -> dict[str, object]:
    """Reject incomplete, malformed, or inconsistent provenance before publish."""
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise ValueError("run provenance is missing a supported schema version")
    source = value.get("source")
    if not isinstance(source, dict):
        raise ValueError("run provenance is missing source identity")
    if not isinstance(source.get("git_revision"), str) or not source.get(
        "git_revision"
    ):
        raise ValueError("run provenance is missing Git revision")
    if not isinstance(source.get("git_dirty"), bool) or not isinstance(
        source.get("git_status_porcelain"), str
    ):
        raise ValueError("run provenance is missing Git dirty state")
    if source["git_dirty"] != bool(source["git_status_porcelain"]):
        raise ValueError("run provenance Git dirty flag disagrees with status")
    hashes = source.get("owned_input_sha256")
    if not isinstance(hashes, dict) or set(hashes) != set(OWNED_INPUT_PATHS):
        raise ValueError("run provenance has an incomplete owned source hash set")
    if any(not _is_sha256(digest) for digest in hashes.values()):
        raise ValueError("run provenance contains an invalid source hash")

    executables = value.get("executables")
    if not isinstance(executables, dict):
        raise ValueError("run provenance is missing executable identities")
    for name in ("python", "ros2", "gazebo", "mapping_binary"):
        executable = executables.get(name)
        if not isinstance(executable, dict):
            raise ValueError(f"run provenance is missing {name} identity")
        if not isinstance(executable.get("path"), str) or not executable.get("path"):
            raise ValueError(f"run provenance is missing {name} path")
        if not _is_sha256(executable.get("sha256")):
            raise ValueError(f"run provenance has invalid {name} hash")
        linked_libraries = executable.get("linked_libraries")
        if not isinstance(linked_libraries, list) or not linked_libraries:
            raise ValueError(
                f"run provenance is missing {name} linked library identities"
            )
        for index, library_value in enumerate(linked_libraries):
            if not isinstance(library_value, dict) or set(library_value) != {
                "name",
                "install_name",
                "path",
                "version",
                "disposition",
                "sha256",
            }:
                raise ValueError(
                    f"run provenance has invalid {name} linked library {index}"
                )
            disposition = library_value["disposition"]
            if disposition not in {"system-unpinned", "unknown"}:
                raise ValueError(
                    f"run provenance has unsupported {name} library disposition"
                )
            if not isinstance(library_value["name"], str) or not library_value["name"]:
                raise ValueError(f"run provenance has invalid {name} library name")
            if (
                not isinstance(library_value["install_name"], str)
                or not library_value["install_name"]
            ):
                raise ValueError(
                    f"run provenance has invalid {name} library install name"
                )
            if library_value["version"] is not None and not isinstance(
                library_value["version"], str
            ):
                raise ValueError(f"run provenance has invalid {name} library version")
            if library_value["path"] is not None and not isinstance(
                library_value["path"], str
            ):
                raise ValueError(f"run provenance has invalid {name} library path")
            digest = library_value["sha256"]
            if (disposition == "unknown" and digest is not None) or (
                disposition == "system-unpinned" and not _is_sha256(digest)
            ):
                raise ValueError(f"run provenance has inconsistent {name} library hash")
            if disposition == "system-unpinned" and not isinstance(
                library_value["path"], str
            ):
                raise ValueError(
                    f"run provenance has no path for observed {name} library"
                )
            if (
                disposition == "system-unpinned"
                and not Path(str(library_value["path"])).is_absolute()
            ):
                raise ValueError(f"run provenance has non-absolute {name} library path")
    if not isinstance(executables["python"].get("version"), str):
        raise ValueError("run provenance is missing Python version")
    if not isinstance(executables["ros2"].get("ros_distro"), str) or not (
        executables["ros2"].get("ros_distro")
    ):
        raise ValueError("run provenance is missing ROS distribution")
    if not isinstance(executables["ros2"].get("cli_version"), str) or not (
        executables["ros2"].get("cli_version")
    ):
        raise ValueError("run provenance is missing ROS 2 CLI version")
    if not isinstance(executables["gazebo"].get("sim_version"), str) or not (
        executables["gazebo"].get("sim_version")
    ):
        raise ValueError("run provenance is missing Gazebo version")
    if expected_mapping_binary_sha256 is not None and (
        executables["mapping_binary"].get("sha256") != expected_mapping_binary_sha256
    ):
        raise ValueError("run provenance mapping binary does not match capture")

    overlay = value.get("ros_overlay_packages")
    if not isinstance(overlay, dict) or set(overlay) != set(OVERLAY_PACKAGES):
        raise ValueError("run provenance has an incomplete ROS overlay identity")
    for package in OVERLAY_PACKAGES:
        identity = overlay[package]
        if not isinstance(identity, dict):
            raise ValueError(f"run provenance is missing overlay package: {package}")
        if not isinstance(identity.get("prefix"), str) or not identity.get("prefix"):
            raise ValueError(f"run provenance is missing overlay prefix: {package}")
        if not _is_sha256(identity.get("sha256")):
            raise ValueError(f"run provenance has invalid overlay hash: {package}")
        count = identity.get("file_count")
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError(
                f"run provenance has invalid overlay file count: {package}"
            )

    interfaces = value.get("interface_identity")
    if not isinstance(interfaces, dict):
        raise ValueError("run provenance is missing ROS interface identity")
    interface_sources = interfaces.get("source_sha256")
    expected_interface_names = {
        "nano_c_slam_interfaces/msg/StampedRanges.msg",
        "nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg",
    }
    if (
        not isinstance(interface_sources, dict)
        or set(interface_sources) != expected_interface_names
        or any(not _is_sha256(digest) for digest in interface_sources.values())
        or interfaces.get("overlay") != overlay.get("nano_c_slam_interfaces")
    ):
        raise ValueError("run provenance has inconsistent ROS interface identity")

    environment = value.get("runtime_environment")
    if (
        not isinstance(environment, dict)
        or not isinstance(environment.get("ros_domain_id"), str)
        or not isinstance(environment.get("gazebo_partition"), str)
        or not isinstance(environment.get("rmw_implementation"), str)
        or not environment.get("rmw_implementation")
        or not isinstance(environment.get("prefix_paths"), dict)
        or set(environment["prefix_paths"])
        != {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"}
        or any(
            not isinstance(path, str) for path in environment["prefix_paths"].values()
        )
        or not isinstance(environment.get("allowlisted_environment"), dict)
        or set(environment["allowlisted_environment"]) != set(GAZEBO_ROS_ENVIRONMENT)
        or any(
            value is not None and not isinstance(value, str)
            for value in environment["allowlisted_environment"].values()
        )
    ):
        raise ValueError("run provenance is missing ROS/Gazebo runtime identity")
    allowlisted_environment = cast(
        dict[str, object], environment["allowlisted_environment"]
    )
    if (
        allowlisted_environment["ROS_DOMAIN_ID"] != environment["ros_domain_id"]
        or allowlisted_environment["GZ_PARTITION"] != environment["gazebo_partition"]
        or allowlisted_environment["RMW_IMPLEMENTATION"]
        != environment["rmw_implementation"]
        or allowlisted_environment["AMENT_PREFIX_PATH"]
        != environment["prefix_paths"]["AMENT_PREFIX_PATH"]
        or allowlisted_environment["COLCON_PREFIX_PATH"]
        != environment["prefix_paths"]["COLCON_PREFIX_PATH"]
    ):
        raise ValueError("ROS/Gazebo environment fields disagree with allowlist")
    return cast(dict[str, object], value)


def validate_discovered_topic_types(
    discovered: list[tuple[str, list[str]]], drone_name: str
) -> None:
    """Require the active graph to expose each mission topic with one exact type."""
    expected = {
        f"/{drone_name}/odom": "nav_msgs/msg/Odometry",
        f"/nano_c_slam/sim/{drone_name}/ranges_mm": (
            "nano_c_slam_interfaces/msg/StampedPairwiseRanges"
        ),
        f"/{drone_name}/mission_status": "std_msgs/msg/String",
    }
    actual = dict(discovered)
    for topic, expected_type in expected.items():
        types = actual.get(topic, [])
        if types != [expected_type]:
            raise RuntimeError(
                f"ROS topic {topic} has discovered types {types!r}; "
                f"expected exactly [{expected_type!r}]"
            )


def _ros2_cli_version() -> str:
    """Return the installed ros2cli distribution version from active Python paths."""
    try:
        return importlib.metadata.version("ros2cli")
    except importlib.metadata.PackageNotFoundError as error:
        raise ValueError("active Python environment has no ros2cli metadata") from error


def validate_combined_run_report(value: object) -> dict[str, object]:
    """Validate provenance is present and agrees with the replay executable."""
    if not isinstance(value, dict) or value.get("schema_version") != 2:
        raise ValueError("combined Gazebo run report is not provenance schema v2")
    run_id = value.get("run_id")
    capture = value.get("capture_report")
    if not isinstance(run_id, str) or not isinstance(capture, dict):
        raise ValueError("combined Gazebo run report is missing run identity/capture")
    if capture.get("run_id") != run_id:
        raise ValueError("combined Gazebo report and capture run IDs do not match")
    replay_provenance = capture.get("mapping_replay_provenance")
    if not isinstance(replay_provenance, dict):
        raise ValueError("combined Gazebo capture is missing mapping provenance")
    replay_hash = replay_provenance.get("binary_sha256")
    if not _is_sha256(replay_hash):
        raise ValueError("combined Gazebo capture has invalid mapping binary hash")
    commands = value.get("child_commands")
    if not isinstance(commands, dict):
        raise ValueError("combined Gazebo report is missing child command vectors")
    collector = commands.get("collector")
    mission = commands.get("mission")
    if (
        not isinstance(collector, list)
        or len(collector) < 3
        or not all(isinstance(argument, str) for argument in collector)
        or not isinstance(mission, list)
        or not mission
        or not all(isinstance(argument, str) for argument in mission)
    ):
        raise ValueError("combined Gazebo report has invalid child command vectors")
    provenance = validate_run_provenance(
        value.get("run_provenance"),
        expected_mapping_binary_sha256=replay_hash,
    )
    executables = cast(dict[str, object], provenance["executables"])
    python_identity = cast(dict[str, object], executables["python"])
    ros_identity = cast(dict[str, object], executables["ros2"])
    binary_identity = cast(dict[str, object], executables["mapping_binary"])
    if Path(collector[0]).resolve() != Path(str(python_identity["path"])).resolve():
        raise ValueError("collector command does not use the recorded Python")
    if Path(mission[0]).resolve() != Path(str(ros_identity["path"])).resolve():
        raise ValueError("mission command does not use the recorded ROS 2 CLI")
    if collector[1:3] != ["-m", "scripts.gazebo_mapping_sitl"]:
        raise ValueError("collector command does not use the hashed owned script")
    try:
        binary_argument_index = collector.index("--binary") + 1
        command_binary = Path(collector[binary_argument_index]).resolve()
    except (ValueError, IndexError) as error:
        raise ValueError("collector command is missing its replay binary") from error
    if command_binary != Path(str(binary_identity["path"])).resolve():
        raise ValueError("collector command does not use the recorded mapping binary")
    return {**value, "run_provenance": provenance}


def require_provenance_unchanged(before: object, after: object) -> dict[str, object]:
    """Require a complete provenance snapshot to remain stable during a run."""
    first = validate_run_provenance(before)
    second = validate_run_provenance(after)
    if first != second:
        raise ValueError("source, tool, or overlay changed during Gazebo run")
    return second


def _is_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )
