"""Archive the exact near-field SDF and world-session identity while Gazebo runs."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
import uuid
from datetime import UTC, datetime
from pathlib import Path

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model import (
    session_file_for_partition,
)


def _unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"world session repeats JSON key {key!r}")
        result[key] = value
    return result


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def archive_world_evidence(partition: str, output_dir: Path) -> dict[str, object]:
    """Copy and bind the active world SDF and session manifest atomically."""
    session_path = session_file_for_partition(partition)
    try:
        session_bytes = session_path.read_bytes()
        session = json.loads(session_bytes, object_pairs_hook=_unique_object)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"cannot read active Gazebo world session: {error}") from error
    if not isinstance(session, dict) or session.get("scene") != "nearfield-mapping":
        raise ValueError("active world session is not a near-field mapping scene")
    session_id = session.get("session_id")
    source_path_value = session.get("world_sdf_path")
    source_hash = session.get("world_sdf_sha256")
    source_size = session.get("world_sdf_bytes")
    processes = session.get("processes")
    if (
        not isinstance(session_id, str)
        or not isinstance(source_path_value, str)
        or not isinstance(source_hash, str)
        or isinstance(source_size, bool)
        or not isinstance(source_size, int)
        or not isinstance(processes, list)
    ):
        raise ValueError("active world session has incomplete SDF or process identity")
    server_records = [
        item
        for item in processes
        if isinstance(item, dict) and item.get("role") == "gazebo-server"
    ]
    if len(server_records) != 1:
        raise ValueError("world session must identify exactly one Gazebo server")
    server = server_records[0]
    pid = server.get("pid")
    command = server.get("argv")
    source_path = Path(source_path_value)
    launched_sdf_arguments = (
        [
            argument
            for argument in command
            if isinstance(argument, str) and argument.endswith(".sdf")
        ]
        if isinstance(command, list)
        else []
    )
    if (
        isinstance(pid, bool)
        or not isinstance(pid, int)
        or pid <= 0
        or not isinstance(command, list)
        or not all(isinstance(argument, str) for argument in command)
        or len(launched_sdf_arguments) != 1
        or Path(launched_sdf_arguments[0]).resolve() != source_path.resolve()
    ):
        raise ValueError("world session server command does not bind its SDF path")
    try:
        os.kill(pid, 0)
        source_bytes = source_path.read_bytes()
    except OSError as error:
        raise ValueError(
            "Gazebo server or its launched SDF is no longer available"
        ) from error
    if (
        not source_bytes
        or len(source_bytes) != source_size
        or _sha256(source_bytes) != source_hash
    ):
        raise ValueError("launched SDF differs from its world-session identity")
    if output_dir.exists():
        raise FileExistsError(f"world evidence output already exists: {output_dir}")

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary_dir = Path(
        tempfile.mkdtemp(
            prefix=f".{output_dir.name}.{uuid.uuid4().hex}.", dir=output_dir.parent
        )
    )
    try:
        archived_sdf = temporary_dir / "world.sdf"
        archived_session = temporary_dir / "world-session.json"
        archived_sdf.write_bytes(source_bytes)
        archived_session.write_bytes(session_bytes)
        if source_path.read_bytes() != source_bytes:
            raise ValueError("launched SDF changed while being archived")
        if session_path.read_bytes() != session_bytes:
            raise ValueError("world session changed while being archived; retry")
        attestation: dict[str, object] = {
            "schema_version": 1,
            "session_id": session_id,
            "session_sha256": _sha256(session_bytes),
            "gazebo_server_pid": pid,
            "gazebo_server_sdf_argument": launched_sdf_arguments[0],
            "world_sdf_sha256": source_hash,
            "archived_sdf_sha256": _sha256(archived_sdf.read_bytes()),
            "verified_while_server_active": True,
            "attested_at_utc": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        }
        (temporary_dir / "world-input-attestation.json").write_text(
            json.dumps(attestation, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary_dir.rename(output_dir)
    except BaseException:
        shutil.rmtree(temporary_dir, ignore_errors=True)
        raise
    return attestation


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = archive_world_evidence(args.partition, args.output_dir)
    except (FileExistsError, OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))
    print(f"Archived world evidence: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
