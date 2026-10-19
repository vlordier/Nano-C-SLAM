"""Adversarial parsing and process-scoped mapped-library evidence tests."""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch

from scripts.runtime_loaded_libraries import (
    _hash_mapped_file,
    capture_session_gazebo_libraries,
    parse_lsof_mapped_paths,
)


def _lsof_fields(
    pid: int,
    entries: list[tuple[str, str, str]],
    *,
    parent_pid: int | None = None,
) -> bytes:
    fields = [f"p{pid}"]
    if parent_pid is not None:
        fields.append(f"R{parent_pid}")
    for descriptor, file_type, path in entries:
        fields.extend((f"f{descriptor}", f"t{file_type}", f"n{path}"))
    return ("\0".join(fields) + "\0").encode()


def _write_session(
    path: Path, *, pid: int, executable: Path, argv: list[str], session_id: str
) -> None:
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "session_id": session_id,
                "scene": "nearfield-mapping",
                "launch_pid": os.getpid(),
                "processes": [
                    {
                        "role": "gazebo-server",
                        "pid": pid,
                        "launch_pid": os.getpid(),
                        "argv": argv,
                        "executable_path": str(executable),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )


class RuntimeLoadedLibrariesTests(unittest.TestCase):
    def test_lsof_parser_filters_library_mappings_and_deduplicates_segments(
        self,
    ) -> None:
        paths = parse_lsof_mapped_paths(
            _lsof_fields(
                123,
                [
                    ("txt", "REG", "/opt/ros/lib/librclcpp.dylib"),
                    ("mem", "REG", "/opt/ros/lib/librclcpp.dylib"),
                    (
                        "txt",
                        "REG",
                        "/System/Library/Frameworks/CoreFoundation.framework/Versions/A/CoreFoundation",
                    ),
                    ("txt", "REG", "/opt/ros/share/manifest.xml"),
                ],
            ),
            expected_pid=123,
        )
        self.assertEqual(
            paths,
            (
                Path(
                    "/System/Library/Frameworks/CoreFoundation.framework/Versions/A/CoreFoundation"
                ),
                Path("/opt/ros/lib/librclcpp.dylib"),
            ),
        )

    def test_lsof_parser_preserves_spaces_and_rejects_other_pids(self) -> None:
        output = _lsof_fields(
            44,
            [("txt", "REG", "/opt/ROS Stack/lib/libfixture.dylib")],
        )
        self.assertEqual(
            parse_lsof_mapped_paths(output, expected_pid=44),
            (Path("/opt/ROS Stack/lib/libfixture.dylib"),),
        )
        with self.assertRaisesRegex(ValueError, "launch PID"):
            parse_lsof_mapped_paths(output, expected_pid=44, expected_parent_pid=40)
        with self.assertRaisesRegex(ValueError, "do not match"):
            parse_lsof_mapped_paths(output, expected_pid=45)
        with self.assertRaisesRegex(ValueError, "invalid PID"):
            parse_lsof_mapped_paths(b"pbad\0", expected_pid=44)

    def test_lsof_parser_rejects_malformed_and_missing_process_output(self) -> None:
        for output in (b"", b"fmem\0tREG\0n/opt/lib/libx.dylib\0"):
            with (
                self.subTest(output=output),
                self.assertRaisesRegex(ValueError, "do not match"),
            ):
                parse_lsof_mapped_paths(output, expected_pid=3)

    def test_missing_or_legacy_session_retains_unknown(self) -> None:
        with patch(
            "scripts.runtime_loaded_libraries.session_file_for_partition",
            return_value=Path("/definitely/missing/world.json"),
        ):
            evidence = capture_session_gazebo_libraries(partition="test")
        self.assertEqual(evidence["status"], "unknown")
        self.assertEqual(evidence["reason"], "world-session manifest is unavailable")

    def test_session_rejects_process_record_from_a_different_launcher(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session_path = Path(temporary) / "session.json"
            executable = Path(temporary) / "gz"
            _write_session(
                session_path,
                pid=3456,
                executable=executable,
                argv=[str(executable), "sim", "world.sdf", "-s"],
                session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            )
            session = json.loads(session_path.read_text(encoding="utf-8"))
            session["processes"][0]["launch_pid"] += 1
            session_path.write_text(json.dumps(session), encoding="utf-8")
            with patch(
                "scripts.runtime_loaded_libraries.session_file_for_partition",
                return_value=session_path,
            ):
                evidence = capture_session_gazebo_libraries(partition="test")
        self.assertEqual(evidence["status"], "unknown")
        self.assertIn("identity is incomplete", evidence["reason"])

    def test_session_identity_drives_lsof_hashing_and_records_exact_argv(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            executable = directory / "gz"
            executable.write_bytes(b"gazebo executable")
            library = directory / "lib plugin.dylib"
            library.write_bytes(b"plugin bytes")
            ps = directory / "ps"
            lsof = directory / "lsof"
            ps.write_bytes(b"")
            lsof.write_bytes(b"")
            session_path = directory / "session.json"
            command = [str(executable), "sim", "world.sdf", "-s"]
            _write_session(
                session_path,
                pid=3456,
                executable=executable,
                argv=command,
                session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            )
            lsof_output = _lsof_fields(
                3456,
                [
                    ("txt", "REG", str(executable)),
                    ("txt", "REG", str(library)),
                    ("mem", "REG", str(library)),
                ],
                parent_pid=os.getpid(),
            )
            results = [
                CompletedProcess(
                    [str(ps)], 0, stdout=f"{' '.join(command)}\n".encode()
                ),
                CompletedProcess([str(lsof)], 0, stdout=lsof_output),
            ]
            with (
                patch(
                    "scripts.runtime_loaded_libraries.platform.system",
                    return_value="Darwin",
                ),
                patch(
                    "scripts.runtime_loaded_libraries.session_file_for_partition",
                    return_value=session_path,
                ),
                patch(
                    "scripts.runtime_loaded_libraries.shutil.which",
                    side_effect=[str(ps), str(lsof)],
                ),
                patch(
                    "scripts.runtime_loaded_libraries.subprocess.run",
                    side_effect=results,
                ) as run,
            ):
                evidence = capture_session_gazebo_libraries(partition="test")

            self.assertEqual(evidence["status"], "observed")
            self.assertEqual(
                evidence["session_id"], "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
            )
            self.assertEqual(evidence["pid"], 3456)
            self.assertEqual(evidence["launch_pid"], os.getpid())
            self.assertEqual(len(evidence["libraries"]), 1)
            library_records = evidence["libraries"]
            plugin = next(
                item
                for item in library_records
                if item["path"] == str(library.resolve())
            )
            self.assertEqual(len(plugin["sha256"]), 64)
            self.assertIn("-a", run.call_args_list[1].args[0])

    def test_ruby_gazebo_wrapper_title_is_bound_to_the_recorded_gz_command(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            ruby = directory / "ruby"
            gz = directory / "gz"
            library = directory / "libgz-sim.dylib"
            for path, contents in (
                (ruby, b"ruby executable"),
                (gz, b"gz launcher"),
                (library, b"mapped Gazebo plugin"),
            ):
                path.write_bytes(contents)
            ps = directory / "ps"
            lsof = directory / "lsof"
            ps.write_bytes(b"")
            lsof.write_bytes(b"")
            session_path = directory / "session.json"
            command = [
                "ruby",
                str(gz),
                "sim",
                "world.sdf",
                "-r",
                "-s",
                "--force-version",
                "8",
            ]
            _write_session(
                session_path,
                pid=4567,
                executable=ruby,
                argv=command,
                session_id="dddddddd-dddd-4ddd-8ddd-dddddddddddd",
            )
            lsof_output = _lsof_fields(
                4567,
                [("txt", "REG", str(ruby)), ("txt", "REG", str(library))],
                parent_pid=os.getpid(),
            )
            with (
                patch(
                    "scripts.runtime_loaded_libraries.platform.system",
                    return_value="Darwin",
                ),
                patch(
                    "scripts.runtime_loaded_libraries.session_file_for_partition",
                    return_value=session_path,
                ),
                patch(
                    "scripts.runtime_loaded_libraries.shutil.which",
                    side_effect=[str(ps), str(lsof)],
                ),
                patch(
                    "scripts.runtime_loaded_libraries.subprocess.run",
                    side_effect=[
                        CompletedProcess(
                            [str(ps)], 0, stdout=b"gz sim world.sdf -r -s\n"
                        ),
                        CompletedProcess([str(lsof)], 0, stdout=lsof_output),
                    ],
                ),
            ):
                evidence = capture_session_gazebo_libraries(
                    partition="test", expected_executable_path=gz
                )

            self.assertEqual(evidence["status"], "observed")
            self.assertEqual(evidence["executable_path"], str(ruby.resolve()))
            self.assertEqual(evidence["pid"], 4567)
            self.assertEqual(
                [item["path"] for item in evidence["libraries"]],
                [str(library.resolve())],
            )

    def test_pid_command_mismatch_and_unavailable_mapped_file_are_unknown(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            executable = directory / "gz"
            executable.write_bytes(b"binary")
            ps = directory / "ps"
            lsof = directory / "lsof"
            ps.write_bytes(b"")
            lsof.write_bytes(b"")
            session_path = directory / "session.json"
            command = [str(executable), "sim", "world.sdf", "-s"]
            _write_session(
                session_path,
                pid=77,
                executable=executable,
                argv=command,
                session_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
            )
            with (
                patch(
                    "scripts.runtime_loaded_libraries.platform.system",
                    return_value="Darwin",
                ),
                patch(
                    "scripts.runtime_loaded_libraries.session_file_for_partition",
                    return_value=session_path,
                ),
                patch(
                    "scripts.runtime_loaded_libraries.shutil.which",
                    side_effect=[str(ps), str(lsof)],
                ),
                patch(
                    "scripts.runtime_loaded_libraries.subprocess.run",
                    return_value=CompletedProcess([str(ps)], 0, stdout=b"other sim\n"),
                ) as run,
            ):
                evidence = capture_session_gazebo_libraries(partition="test")
            self.assertEqual(evidence["status"], "unknown")
            self.assertIn("command changed", evidence["reason"])
            run.assert_called_once()

            deleted = parse_lsof_mapped_paths(
                _lsof_fields(
                    77,
                    [
                        (
                            "txt",
                            "REG",
                            f"{directory / 'libgone.dylib'} (deleted)",
                        )
                    ],
                    parent_pid=os.getpid(),
                ),
                expected_pid=77,
            )
            self.assertEqual(deleted[0].name, "libgone.dylib")
            self.assertEqual(
                _hash_mapped_file(deleted[0])["reason"], "file is unavailable"
            )

    def test_exited_process_is_retained_with_command_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            executable = directory / "gz"
            executable.write_bytes(b"binary")
            ps = directory / "ps"
            lsof = directory / "lsof"
            ps.write_bytes(b"")
            lsof.write_bytes(b"")
            session_path = directory / "session.json"
            command = [str(executable), "sim", "world.sdf", "-s"]
            _write_session(
                session_path,
                pid=78,
                executable=executable,
                argv=command,
                session_id="cccccccc-cccc-4ccc-8ccc-cccccccccccc",
            )
            with (
                patch(
                    "scripts.runtime_loaded_libraries.platform.system",
                    return_value="Darwin",
                ),
                patch(
                    "scripts.runtime_loaded_libraries.session_file_for_partition",
                    return_value=session_path,
                ),
                patch(
                    "scripts.runtime_loaded_libraries.shutil.which",
                    side_effect=[str(ps), str(lsof)],
                ),
                patch(
                    "scripts.runtime_loaded_libraries.subprocess.run",
                    side_effect=[
                        CompletedProcess(
                            [str(ps)], 0, stdout=f"{' '.join(command)}\n".encode()
                        ),
                        CompletedProcess([str(lsof)], 1, stdout=b""),
                    ],
                ),
            ):
                evidence = capture_session_gazebo_libraries(partition="test")
            self.assertEqual(evidence["status"], "unknown")
            self.assertEqual(
                evidence["reason"], "lsof could not observe the recorded process"
            )
            self.assertEqual(evidence["commands"][1]["returncode"], 1)


if __name__ == "__main__":
    unittest.main()
