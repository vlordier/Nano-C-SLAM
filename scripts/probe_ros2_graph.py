#!/usr/bin/env python3
"""Run a bounded ROS-only DDS discovery probe on an isolated ROS domain."""

from __future__ import annotations

import argparse
import ctypes.util
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
_SUPPORTED_DOMAIN_MAX = 232
_MESSAGE_TYPE = "std_msgs/msg/String"
_CHILD_OPTION = "--_child-role"


class ProbeFailure(RuntimeError):
    """Expected diagnostic failure that should be retained as evidence."""


def _remaining_time(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise ProbeFailure("probe deadline expired")
    return remaining


def validate_timeout(value: float) -> float:
    if not math.isfinite(value) or value <= 0:
        raise ValueError("timeout must be finite and positive")
    return value


def validate_domain_id(value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("ROS_DOMAIN_ID must be an integer")
    if not 0 <= value <= _SUPPORTED_DOMAIN_MAX:
        raise ValueError("ROS_DOMAIN_ID must be in the supported range 0..232")
    return value


def resolve_executable(name: str) -> Path:
    executable = shutil.which(name)
    if executable is None:
        raise FileNotFoundError(f"required executable is not on PATH: {name}")
    return Path(executable).resolve()


def parse_node_names(stdout: str) -> set[str]:
    return {
        line.strip().lstrip("/")
        for line in stdout.splitlines()
        if line.strip() and not line.strip().startswith("WARNING:")
    }


def require_unused_domain(node_names: set[str], *, requested: bool) -> None:
    if node_names:
        source = "requested" if requested else "candidate"
        raise ProbeFailure(
            f"{source} ROS domain already has participants: {sorted(node_names)}"
        )


def validate_exchange_evidence(
    *,
    subscriber: dict[str, object],
    publisher: dict[str, object],
    listed_nodes: set[str],
    listed_topics: str,
    subscriber_name: str,
    publisher_name: str,
    topic: str,
    nonce: str,
) -> None:
    if subscriber.get("state") != "received" or subscriber.get("message") != nonce:
        raise ProbeFailure("subscriber did not receive the exact nonce message")
    if publisher.get("state") != "published" or publisher.get("message") != nonce:
        raise ProbeFailure("publisher did not publish the exact nonce message")
    if subscriber.get("topic_type") != _MESSAGE_TYPE:
        raise ProbeFailure("subscriber topic type is not std_msgs/msg/String")
    if publisher.get("topic_type") != _MESSAGE_TYPE:
        raise ProbeFailure("publisher topic type is not std_msgs/msg/String")
    if _MESSAGE_TYPE not in publisher.get("matched_topic_types", []):
        raise ProbeFailure("DDS endpoint discovery did not match std_msgs/msg/String")
    if {subscriber_name, publisher_name}.difference(listed_nodes):
        raise ProbeFailure(
            "ros2 node list did not discover both fixture nodes: "
            f"expected={[subscriber_name, publisher_name]}, "
            f"observed={sorted(listed_nodes)}"
        )
    topic_line = re.compile(
        rf"^{re.escape(topic)}\s+\[{re.escape(_MESSAGE_TYPE)}\]$", re.MULTILINE
    )
    if topic_line.search(listed_topics) is None:
        raise ProbeFailure("ros2 topic list did not report the expected topic type")


def _write_json(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def _read_json(path: Path) -> dict[str, object] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _decode_output(value: str | bytes | None) -> str:
    if value is None:
        return ""
    return value.decode(errors="replace") if isinstance(value, bytes) else value


def _run_command(
    argv: Sequence[str],
    *,
    environment: dict[str, str],
    timeout_s: float,
    commands: list[dict[str, object]],
) -> dict[str, object]:
    started = time.monotonic()
    record: dict[str, object] = {
        "argv": list(argv),
        "timeout_s": timeout_s,
        "returncode": None,
        "timed_out": False,
        "stdout": "",
        "stderr": "",
    }
    try:
        result = subprocess.run(  # noqa: S603 -- fixed ROS CLI and owned arguments
            list(argv),
            cwd=ROOT,
            env=environment,
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout_s,
        )
        record.update(
            returncode=result.returncode,
            stdout=result.stdout,
            stderr=result.stderr,
        )
    except subprocess.TimeoutExpired as error:
        record.update(
            timed_out=True,
            stdout=_decode_output(error.stdout),
            stderr=_decode_output(error.stderr),
        )
        record["stderr"] = f"{record['stderr']}\ncommand timed out".strip()
    record["elapsed_s"] = time.monotonic() - started
    commands.append(record)
    return record


def _require_command_success(record: dict[str, object], label: str) -> str:
    if record["timed_out"]:
        raise ProbeFailure(f"{label} command timed out: {record['argv']}")
    if record["returncode"] != 0:
        raise ProbeFailure(
            f"{label} command failed with {record['returncode']}: {record['argv']}"
        )
    return str(record["stdout"])


def _write_child_status(path: Path, **values: object) -> None:
    _write_json(path, dict(values))


def _child_main(
    role: str,
    *,
    node_name: str,
    topic: str,
    nonce: str,
    status_path: Path,
    stop_path: Path,
    timeout_s: float,
) -> int:
    """Run one ROS child; only this child imports rclpy/std_msgs."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        rmw_identifier = rclpy.get_rmw_implementation_identifier()
        rclpy.init(args=[])
        node = Node(node_name)
        deadline = time.monotonic() + timeout_s
        if role == "subscriber":
            received = False

            def receive(message: String) -> None:
                nonlocal received
                if message.data == nonce and not received:
                    received = True
                    _write_child_status(
                        status_path,
                        state="received",
                        node_name=node_name,
                        topic=topic,
                        topic_type=_MESSAGE_TYPE,
                        message=message.data,
                        rmw_identifier=rmw_identifier,
                    )

            node.create_subscription(String, topic, receive, 10)
            _write_child_status(
                status_path,
                state="ready",
                node_name=node_name,
                topic=topic,
                topic_type=_MESSAGE_TYPE,
                rmw_identifier=rmw_identifier,
            )
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if stop_path.exists():
                    return 0
            if not received:
                _write_child_status(
                    status_path,
                    state="timeout",
                    node_name=node_name,
                    topic=topic,
                    topic_type=_MESSAGE_TYPE,
                    rmw_identifier=rmw_identifier,
                )
                return 2
            return 0

        if role == "publisher":
            publisher = node.create_publisher(String, topic, 10)
            published = False
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if stop_path.exists():
                    return 0
                subscriptions = node.get_subscriptions_info_by_topic(topic)
                matched_types = sorted({item.topic_type for item in subscriptions})
                if subscriptions:
                    message = String()
                    message.data = nonce
                    publisher.publish(message)
                    if not published:
                        published = True
                        _write_child_status(
                            status_path,
                            state="published",
                            node_name=node_name,
                            topic=topic,
                            topic_type=_MESSAGE_TYPE,
                            matched_topic_types=matched_types,
                            message=nonce,
                            rmw_identifier=rmw_identifier,
                        )
            if not published:
                _write_child_status(
                    status_path,
                    state="timeout",
                    node_name=node_name,
                    topic=topic,
                    topic_type=_MESSAGE_TYPE,
                    matched_topic_types=[],
                    rmw_identifier=rmw_identifier,
                )
                return 2
            return 0
        raise ValueError(f"unsupported child role: {role}")
    except Exception as error:
        _write_child_status(
            status_path,
            state="error",
            node_name=node_name,
            topic=topic,
            error=f"{type(error).__name__}: {error}",
        )
        print(f"{type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 2
    finally:
        if "node" in locals():
            node.destroy_node()
        if "rclpy" in locals() and rclpy.ok():
            rclpy.shutdown()


def _child_command(
    role: str,
    *,
    node_name: str,
    topic: str,
    nonce: str,
    status_path: Path,
    stop_path: Path,
    timeout_s: float,
) -> list[str]:
    return [
        sys.executable,
        "-m",
        "scripts.probe_ros2_graph",
        _CHILD_OPTION,
        role,
        "--node-name",
        node_name,
        "--topic",
        topic,
        "--nonce",
        nonce,
        "--status-file",
        str(status_path),
        "--stop-file",
        str(stop_path),
        "--timeout",
        str(timeout_s),
    ]


def _stop_child(child: dict[str, Any], *, timeout_s: float = 3.0) -> str:
    process: subprocess.Popen[str] = child["process"]
    stop_path: Path = child["stop_path"]
    if process.poll() is not None:
        return "already-exited"
    stop_path.touch()
    try:
        process.wait(timeout=timeout_s)
        return "stop-file"
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
            return "terminate"
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=3.0)
                return "kill-owned-child"
            except subprocess.TimeoutExpired:
                return "kill-timeout"


def _wait_for_status(
    path: Path,
    process: subprocess.Popen[str],
    *,
    role: str,
    deadline: float,
    expected_state: str,
) -> dict[str, object]:
    while time.monotonic() < deadline:
        status = _read_json(path)
        if status is not None:
            if status.get("state") == "error":
                raise ProbeFailure(f"{role} child failed: {status.get('error')}")
            if status.get("state") == "timeout":
                raise ProbeFailure(f"{role} child timed out: {status}")
            if status.get("state") == expected_state:
                return status
        returncode = process.poll()
        if returncode is not None:
            if status is not None:
                raise ProbeFailure(
                    f"{role} child exited before {expected_state}: {status}"
                )
            raise ProbeFailure(
                f"{role} child exited early with return code {returncode}"
            )
        time.sleep(0.02)
    raise ProbeFailure(f"{role} child produced no status before the deadline")


def _read_log(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""


def _run_exchange(
    *,
    environment: dict[str, str],
    ros2: Path,
    timeout_s: float,
    nonce: str,
    commands: list[dict[str, object]],
) -> dict[str, object]:
    subscriber_name = f"ncs_probe_sub_{nonce[:12]}"
    publisher_name = f"ncs_probe_pub_{nonce[:12]}"
    topic = f"/ncs_discovery_probe_{nonce[:12]}/nonce"
    deadline = time.monotonic() + timeout_s
    children: list[dict[str, Any]] = []
    result: dict[str, object] = {"success": False, "topic": topic}
    with tempfile.TemporaryDirectory(prefix="ncs-ros-graph-") as temporary:
        directory = Path(temporary)

        def start_child(role: str, node_name: str) -> dict[str, Any]:
            status_path = directory / f"{role}.json"
            stop_path = directory / f"{role}.stop"
            stdout_path = directory / f"{role}.stdout.log"
            stderr_path = directory / f"{role}.stderr.log"
            remaining = max(0.05, deadline - time.monotonic())
            argv = _child_command(
                role,
                node_name=node_name,
                topic=topic,
                nonce=nonce,
                status_path=status_path,
                stop_path=stop_path,
                timeout_s=remaining,
            )
            stdout_file = stdout_path.open("w", encoding="utf-8")
            stderr_file = stderr_path.open("w", encoding="utf-8")
            try:
                process = subprocess.Popen(  # noqa: S603 -- owned probe child module
                    argv,
                    cwd=ROOT,
                    env=environment,
                    stdout=stdout_file,
                    stderr=stderr_file,
                    text=True,
                )
            except Exception:
                stdout_file.close()
                stderr_file.close()
                raise
            child = {
                "role": role,
                "node_name": node_name,
                "argv": argv,
                "process": process,
                "status_path": status_path,
                "stop_path": stop_path,
                "stdout_path": stdout_path,
                "stderr_path": stderr_path,
                "stdout_file": stdout_file,
                "stderr_file": stderr_file,
                "cleanup": "pending",
            }
            children.append(child)
            return child

        try:
            subscriber = start_child("subscriber", subscriber_name)
            ready = _wait_for_status(
                subscriber["status_path"],
                subscriber["process"],
                role="subscriber",
                deadline=deadline,
                expected_state="ready",
            )
            if ready.get("state") != "ready":
                raise ProbeFailure(f"subscriber did not become ready: {ready}")

            publisher = start_child("publisher", publisher_name)
            published = _wait_for_status(
                publisher["status_path"],
                publisher["process"],
                role="publisher",
                deadline=deadline,
                expected_state="published",
            )
            if published.get("state") != "published":
                raise ProbeFailure(
                    f"publisher did not discover subscriber: {published}"
                )
            received = _wait_for_status(
                subscriber["status_path"],
                subscriber["process"],
                role="subscriber",
                deadline=deadline,
                expected_state="received",
            )
            if received.get("state") != "received":
                raise ProbeFailure(f"subscriber did not receive nonce: {received}")

            remaining = max(0.05, deadline - time.monotonic())
            node_list = _run_command(
                [str(ros2), "node", "list", "--no-daemon", "--spin-time", "5.0"],
                environment=environment,
                timeout_s=min(remaining, max(8.0, timeout_s / 2)),
                commands=commands,
            )
            node_names = parse_node_names(
                _require_command_success(node_list, "fixture ros2 node list")
            )
            remaining = max(0.05, deadline - time.monotonic())
            topic_list = _run_command(
                [
                    str(ros2),
                    "topic",
                    "list",
                    "--no-daemon",
                    "--show-types",
                    "--spin-time",
                    "5.0",
                ],
                environment=environment,
                timeout_s=min(remaining, max(8.0, timeout_s / 2)),
                commands=commands,
            )
            topic_stdout = _require_command_success(
                topic_list, "fixture ros2 topic list"
            )
            validate_exchange_evidence(
                subscriber=received,
                publisher=published,
                listed_nodes=node_names,
                listed_topics=topic_stdout,
                subscriber_name=subscriber_name,
                publisher_name=publisher_name,
                topic=topic,
                nonce=nonce,
            )
            result.update(
                success=True,
                subscriber=received,
                publisher=published,
                listed_nodes=sorted(node_names),
                listed_topic_types=topic_stdout,
            )
        except ProbeFailure as error:
            result["error"] = str(error)
            result["subscriber"] = _read_json(directory / "subscriber.json")
            result["publisher"] = _read_json(directory / "publisher.json")
        finally:
            for child in children:
                child["cleanup"] = _stop_child(child)
            for child in children:
                child["stdout_file"].close()
                child["stderr_file"].close()
            cleanup_failures = [
                f"{child['role']} cleanup={child['cleanup']} "
                f"returncode={child['process'].returncode}"
                for child in children
                if child["process"].returncode != 0
            ]
            if cleanup_failures:
                result["success"] = False
                result["error"] = "owned probe child cleanup failed: " + "; ".join(
                    cleanup_failures
                )
            result["children"] = [
                {
                    "role": child["role"],
                    "node_name": child["node_name"],
                    "argv": child["argv"],
                    "status": _read_json(child["status_path"]),
                    "returncode": child["process"].returncode,
                    "cleanup": child["cleanup"],
                    "stdout": _read_log(child["stdout_path"]),
                    "stderr": _read_log(child["stderr_path"]),
                }
                for child in children
            ]
    return result


def _find_rmw_library(identifier: str | None, prefixes: Sequence[str]) -> str | None:
    if not identifier:
        return None
    patterns = (f"lib{identifier}.*", f"{identifier}.dylib", f"{identifier}.so")
    for prefix in prefixes:
        library_directory = Path(prefix) / "lib"
        for pattern in patterns:
            matches = sorted(library_directory.glob(pattern))
            if matches:
                return str(matches[0].resolve())
    located = ctypes.util.find_library(identifier)
    return (
        str(Path(located).resolve())
        if located and Path(located).is_absolute()
        else None
    )


def run_probe(
    *,
    timeout_s: float,
    requested_domain: int | None,
    compare_subnet: bool,
) -> dict[str, object]:
    started = time.monotonic()
    deadline = started + timeout_s
    evidence: dict[str, object] = {
        "schema_version": 1,
        "success": False,
        "started_at_utc": datetime.now(UTC).isoformat(),
        "timeout_s": timeout_s,
        "requested_domain_id": requested_domain,
        "discovery_range": "LOCALHOST",
        "compare_subnet": compare_subnet,
        "commands": [],
        "domain_attempts": [],
        "runs": [],
    }
    try:
        ros2 = resolve_executable("ros2")
        python = Path(sys.executable).resolve()
        if not python.is_file():
            raise FileNotFoundError(f"Python executable is unavailable: {python}")
        environment = os.environ.copy()
        environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        evidence["executables"] = {"ros2": str(ros2), "python": str(python)}
        evidence["environment"] = {
            name: environment.get(name)
            for name in (
                "ROS_DISTRO",
                "ROS_DOMAIN_ID",
                "ROS_AUTOMATIC_DISCOVERY_RANGE",
                "RMW_IMPLEMENTATION",
                "AMENT_PREFIX_PATH",
                "COLCON_PREFIX_PATH",
            )
        }
        commands: list[dict[str, object]] = []
        evidence["commands"] = commands
        prefix_paths = [
            value
            for value in environment.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
            if value
        ]
        prefixes: dict[str, str] = {}
        for package in ("rclpy", "std_msgs"):
            record = _run_command(
                [str(ros2), "pkg", "prefix", package],
                environment=environment,
                timeout_s=min(_remaining_time(deadline), 3.0),
                commands=commands,
            )
            prefixes[package] = _require_command_success(
                record, f"ROS package prefix {package}"
            ).strip()
        if not prefixes["rclpy"] or not prefixes["std_msgs"]:
            raise ProbeFailure("ROS package prefixes for rclpy/std_msgs are empty")
        active_prefixes = {str(Path(value).resolve()) for value in prefix_paths}
        ros_prefixes = {str(Path(value).resolve()) for value in prefixes.values()}
        if active_prefixes and active_prefixes.difference(ros_prefixes):
            raise ProbeFailure(
                "source only the ROS underlay; overlay prefixes are active: "
                f"{sorted(active_prefixes.difference(ros_prefixes))}"
            )
        evidence["ros_prefixes"] = prefixes

        candidates = (
            [requested_domain]
            if requested_domain is not None
            else [
                (100 + os.getpid() % 133 + offset) % (_SUPPORTED_DOMAIN_MAX + 1)
                for offset in range(8)
            ]
        )
        selected_domain: int | None = None
        for candidate in candidates:
            domain = validate_domain_id(candidate)
            environment["ROS_DOMAIN_ID"] = str(domain)
            evidence["environment"] = {
                **evidence["environment"],
                "ROS_DOMAIN_ID": str(domain),
                "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            }
            check = _run_command(
                [str(ros2), "node", "list", "--no-daemon", "--spin-time", "0.2"],
                environment=environment,
                timeout_s=min(_remaining_time(deadline), max(8.0, timeout_s / 2)),
                commands=commands,
            )
            check_result = {
                "domain_id": domain,
                "returncode": check["returncode"],
                "timed_out": check["timed_out"],
                "node_names": sorted(parse_node_names(str(check["stdout"]))),
            }
            evidence["domain_attempts"].append(check_result)
            stdout = _require_command_success(check, "domain occupancy check")
            names = parse_node_names(stdout)
            if not names:
                selected_domain = domain
                break
            if requested_domain is not None:
                require_unused_domain(names, requested=True)
        if selected_domain is None:
            raise ProbeFailure("no unused ROS domain found in the candidate set")
        evidence["domain_id"] = selected_domain
        evidence["environment"] = {
            **evidence["environment"],
            "ROS_DOMAIN_ID": str(selected_domain),
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        }

        nonce = uuid.uuid4().hex
        runs: list[dict[str, object]] = []
        discovery_ranges = ["LOCALHOST", "SUBNET"] if compare_subnet else ["LOCALHOST"]
        for discovery_range in discovery_ranges:
            environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = discovery_range
            environment["ROS_DOMAIN_ID"] = str(selected_domain)
            run = _run_exchange(
                environment=environment,
                ros2=ros2,
                timeout_s=_remaining_time(deadline),
                nonce=nonce,
                commands=commands,
            )
            run["discovery_range"] = discovery_range
            runs.append(run)
            evidence["runs"] = runs
            if not run["success"]:
                raise ProbeFailure(str(run.get("error", "DDS exchange failed")))
            if compare_subnet and discovery_range == "LOCALHOST":
                idle = _run_command(
                    [str(ros2), "node", "list", "--no-daemon", "--spin-time", "0.2"],
                    environment=environment,
                    timeout_s=min(_remaining_time(deadline), 2.0),
                    commands=commands,
                )
                idle_names = parse_node_names(
                    _require_command_success(idle, "between-mode occupancy check")
                )
                require_unused_domain(idle_names, requested=True)
        rmw_identifier = next(
            (
                child_status.get("rmw_identifier")
                for run in runs
                for child in run.get("children", [])
                if isinstance(child, dict)
                for child_status in [child.get("status")]
                if isinstance(child_status, dict) and child_status.get("rmw_identifier")
            ),
            None,
        )
        evidence["rmw"] = {
            "requested_implementation": environment.get("RMW_IMPLEMENTATION"),
            "observed_implementation": rmw_identifier,
            "library_path": _find_rmw_library(
                str(rmw_identifier) if rmw_identifier else None, prefix_paths
            ),
        }
        evidence["success"] = True
    except Exception as error:
        evidence["error"] = str(error)
        evidence["error_type"] = type(error).__name__
    finally:
        evidence["elapsed_s"] = time.monotonic() - started
    return evidence


def _default_output_path() -> Path:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return (
        ROOT / "artifacts" / f"ros2_discovery_probe_{stamp}_{uuid.uuid4().hex[:8]}.json"
    )


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--domain-id", type=int)
    parser.add_argument(
        "--compare-subnet",
        action="store_true",
        help="after LOCALHOST passes, repeat on the same domain with SUBNET discovery",
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        _CHILD_OPTION, choices=("subscriber", "publisher"), help=argparse.SUPPRESS
    )
    parser.add_argument("--node-name", help=argparse.SUPPRESS)
    parser.add_argument("--topic", help=argparse.SUPPRESS)
    parser.add_argument("--nonce", help=argparse.SUPPRESS)
    parser.add_argument("--status-file", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--stop-file", type=Path, help=argparse.SUPPRESS)
    options = parser.parse_args(args)
    if options.__dict__.get("_child_role"):
        if not all(
            (
                options.node_name,
                options.topic,
                options.nonce,
                options.status_file,
                options.stop_file,
            )
        ):
            parser.error(
                "child mode requires node, topic, nonce, status, and stop paths"
            )
        return _child_main(
            options._child_role,
            node_name=options.node_name,
            topic=options.topic,
            nonce=options.nonce,
            status_path=options.status_file,
            stop_path=options.stop_file,
            timeout_s=validate_timeout(options.timeout),
        )
    try:
        timeout_s = validate_timeout(options.timeout)
        domain = (
            validate_domain_id(options.domain_id)
            if options.domain_id is not None
            else None
        )
    except ValueError as error:
        parser.error(str(error))
    evidence = run_probe(
        timeout_s=timeout_s,
        requested_domain=domain,
        compare_subnet=options.compare_subnet,
    )
    output_path = options.output or _default_output_path()
    _write_json(output_path, evidence)
    print(json.dumps({"evidence_path": str(output_path), **evidence}, sort_keys=True))
    return 0 if evidence["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
