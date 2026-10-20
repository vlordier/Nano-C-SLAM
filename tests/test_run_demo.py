"""Hardware-free tests for the mission CLI and radio packet format."""

import csv
import importlib.util
import struct
import subprocess
import sys
import tempfile
import unittest
from collections.abc import Callable
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "cslam-stm32-app" / "run_demo.py"
SPEC = importlib.util.spec_from_file_location("run_demo", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
demo = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = demo
SPEC.loader.exec_module(demo)


def pose_payload() -> bytes:
    return struct.pack("<hxxlfff32h", 7, 1234, 1.23456, -2.5, 0.25, *range(32))


class PacketTests(unittest.TestCase):
    def test_pose_decode_and_csv_layout(self) -> None:
        pose = demo.decode_pose_payload(pose_payload())
        self.assertEqual(pose.pose_id, 7)
        self.assertEqual(pose.timestamp, 1234)
        self.assertEqual(pose.tof, tuple(range(32)))
        self.assertEqual(pose.csv_row()[:5], [7, 1234, 1.235, -2.5, 0.25])
        self.assertEqual(len(pose.csv_row()), 37)

    def test_rejects_short_pose_and_scan(self) -> None:
        with self.assertRaises(ValueError):
            demo.decode_pose_payload(pose_payload()[:-1])
        with self.assertRaises(ValueError):
            demo.decode_scan_packet(b"\x01\x00\x02")

    def test_assembles_fragments_and_recovers_after_bad_packet(self) -> None:
        payload = pose_payload()
        assembler = demo.PoseAssembler()
        self.assertIsNone(assembler.feed(b"\x00\x01" + payload[:10]))
        self.assertIsNone(assembler.feed(b"\x00\x00" + payload[:28]))
        self.assertIsNone(assembler.feed(b"\x00\x01" + payload[28:56]))
        result = assembler.feed(b"\x80\x02" + payload[56:])
        self.assertIsNotNone(result)
        self.assertEqual(result.pose_id, 7)
        self.assertIsNone(assembler.feed(b"\x80\x01" + payload[56:]))
        self.assertIsNone(assembler.feed(b"\x80\x00" + payload[:-1]))
        self.assertIsNone(assembler.feed(b"\x00\x00" + payload + b"x"))
        self.assertIsNone(assembler.feed(b"\x80\x01"))
        self.assertIsNone(assembler.feed(b"\x00\x00" + payload[:28]))
        self.assertIsNone(assembler.feed(b"\x80\x02" + payload[28:]))
        self.assertEqual(assembler.feed(b"\x80\x00" + payload).pose_id, 7)

    def test_logger_writes_existing_csv_layout(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log_dir = Path(tmp)
            notifier = demo.MissionStartNotifier()
            logger = demo.DroneLogger("A", log_dir, notifier)
            payload = pose_payload()
            with patch("builtins.print") as print_mock:
                logger.handle_packet(b"\x00\x00" + payload[:28])
                logger.handle_packet(b"\x00\x01" + payload[28:56])
                logger.handle_packet(b"\x80\x02" + payload[56:])
                logger.handle_packet(b"\x80\x00" + payload)
            print_mock.assert_called_once()
            logger.handle_packet(b"\x81\x00" + struct.pack("<h", 42))
            logger.handle_packet(b"\x01\x00\x02")  # Short scan is ignored.
            with (log_dir / "A.csv").open(newline="", encoding="utf-8") as file:
                rows = list(csv.reader(file))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0][:5], ["7", "1234", "1.235", "-2.5", "0.25"])
            self.assertEqual(len(rows[0]), 37)
            with (log_dir / "As.csv").open(newline="", encoding="utf-8") as file:
                self.assertEqual(list(csv.reader(file)), [["42"]])


class MissionTests(unittest.TestCase):
    class CallbackList:
        def __init__(self) -> None:
            self.callbacks: list[Callable[..., None]] = []

        def add_callback(self, callback: Callable[..., None]) -> None:
            self.callbacks.append(callback)

        def remove_callback(self, callback: Callable[..., None]) -> None:
            self.callbacks.remove(callback)

        def call(self, *args: object) -> None:
            for callback in tuple(self.callbacks):
                callback(*args)

    class FakeLink:
        def __init__(self, uri: str, *, respond: bool = True) -> None:
            self.link_uri = uri
            self.respond = respond
            self.connected = MissionTests.CallbackList()
            self.connection_failed = MissionTests.CallbackList()
            self.close_count = 0
            self.param = SimpleNamespace(is_updated=True)
            self.console = SimpleNamespace(receivedChar=MissionTests.CallbackList())

        def add_port_callback(
            self, _port: int, _callback: Callable[[demo.PacketLike], None]
        ) -> None:
            pass

        def open_link(self, uri: str) -> None:
            self.link_uri = uri
            if self.respond:
                self.connected.call(uri)

        def close_link(self) -> None:
            self.close_count += 1

    def test_import_has_no_radio_or_filesystem_side_effects(self) -> None:
        code = f"""
import importlib.util
import sys
from pathlib import Path
spec = importlib.util.spec_from_file_location("run_demo", {str(SCRIPT)!r})
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
assert "cflib" not in sys.modules
assert not Path("logs").exists()
"""
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(  # noqa: S603 - fixed interpreter and inline test code
                [sys.executable, "-c", code],
                cwd=tmp,
                capture_output=True,
                text=True,
                check=False,
            )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_log_directories_do_not_overwrite_existing_runs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assertEqual(demo.create_log_dir(root), root / "log0")
            self.assertEqual(demo.create_log_dir(root), root / "log1")

    def test_cli_preserves_radio_defaults_and_selects_drone_count(self) -> None:
        with patch.object(demo, "run_mission") as run_mission:
            self.assertEqual(demo.main(["2"]), 0)
        run_mission.assert_called_once_with(demo.DRONES[:2], log_root=Path("logs"))
        with patch("sys.stderr"), self.assertRaises(SystemExit) as error:
            demo.main(["5"])
        self.assertEqual(error.exception.code, 2)

    def test_cli_accepts_per_drone_udp_uris_and_log_root(self) -> None:
        uris = ["udp://127.0.0.1:21850", "udp://127.0.0.1:21851"]
        log_root = Path("sim-logs")
        with patch.object(demo, "run_mission") as run_mission:
            self.assertEqual(
                demo.main(
                    [
                        "2",
                        "--uri",
                        uris[0],
                        "--uri",
                        uris[1],
                        "--log-root",
                        str(log_root),
                    ]
                ),
                0,
            )
        selected = run_mission.call_args.args[0]
        self.assertEqual(tuple(config.uri for config in selected), tuple(uris))
        self.assertEqual(
            tuple(config.label for config in selected),
            tuple(config.label for config in demo.DRONES[:2]),
        )
        self.assertEqual(run_mission.call_args.kwargs, {"log_root": log_root})

    def test_cli_rejects_uri_count_mismatch_and_duplicate_uris(self) -> None:
        for arguments in (
            ["2", "--uri", "udp://127.0.0.1:21850"],
            ["2", "--uri", "udp://127.0.0.1:21850", "--uri", "udp://127.0.0.1:21850"],
        ):
            with (
                self.subTest(arguments=arguments),
                patch("sys.stderr"),
                self.assertRaises(SystemExit) as error,
            ):
                demo.main(arguments)
            self.assertEqual(error.exception.code, 2)

    def test_cli_reports_bounded_startup_failure(self) -> None:
        for error in (TimeoutError("link timeout"), ConnectionError("link refused")):
            with (
                self.subTest(error=error),
                patch.object(demo, "run_mission", side_effect=error),
                patch("sys.stderr", new_callable=StringIO) as stderr,
            ):
                self.assertEqual(demo.main(["1"]), 1)
            self.assertIn(f"Mission startup failed: {error}", stderr.getvalue())

    def test_gazebo_uri_selection_preserves_configured_initial_poses(self) -> None:
        selected = demo.select_drone_configs(
            2, ("udp://127.0.0.1:21850", "udp://127.0.0.1:21851")
        )
        self.assertEqual(
            tuple(config.initial_pose for config in selected),
            tuple(config.initial_pose for config in demo.DRONES[:2]),
        )

    def test_gazebo_help_includes_udp_endpoint_example(self) -> None:
        with (
            patch("sys.stdout", new_callable=StringIO) as stdout,
            self.assertRaises(SystemExit) as error,
        ):
            demo.main(["--help"])
        self.assertEqual(error.exception.code, 0)
        self.assertIn("udp://127.0.0.1:21850", stdout.getvalue())
        self.assertIn("udp://127.0.0.1:21851", stdout.getvalue())

    def test_param_callback_routes_packets_by_uri(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            logger = demo.DroneLogger("A", Path(tmp), demo.MissionStartNotifier())
            console_callbacks = []
            packet_callbacks = []
            cf = SimpleNamespace(
                link_uri=demo.DRONES[0].uri,
                param=SimpleNamespace(is_updated=True),
                console=SimpleNamespace(
                    receivedChar=SimpleNamespace(add_callback=console_callbacks.append)
                ),
                add_port_callback=lambda port, callback: packet_callbacks.append(
                    (port, callback)
                ),
            )
            with patch("builtins.print"):
                demo.wait_for_param_download(
                    SimpleNamespace(cf=cf), loggers={demo.DRONES[0].uri: logger}
                )
            self.assertEqual(len(console_callbacks), 1)
            self.assertEqual(packet_callbacks[0][0], 1)
            packet_callbacks[0][1](
                SimpleNamespace(data=b"\x81\x00" + struct.pack("<h", 9))
            )
            with (Path(tmp) / "As.csv").open(newline="", encoding="utf-8") as file:
                self.assertEqual(list(csv.reader(file)), [["9"]])

    def test_parameter_download_timeout_fails_instead_of_waiting_forever(self) -> None:
        cf = SimpleNamespace(
            link_uri="udp://127.0.0.1:21850",
            param=SimpleNamespace(is_updated=False),
        )
        with (
            patch.object(demo.time, "monotonic", side_effect=(0.0, 0.6)),
            self.assertRaisesRegex(TimeoutError, "udp://127.0.0.1:21850"),
        ):
            demo.wait_for_param_download(
                SimpleNamespace(cf=cf),
                loggers={},
                timeout_s=0.5,
            )

    def test_parameter_download_timeout_must_be_positive(self) -> None:
        cf = SimpleNamespace(
            link_uri="udp://127.0.0.1:21850",
            param=SimpleNamespace(is_updated=False),
        )
        with self.assertRaisesRegex(ValueError, "must be positive"):
            demo.wait_for_param_download(
                SimpleNamespace(cf=cf), loggers={}, timeout_s=0.0
            )

    def test_bounded_swarm_opens_runs_callbacks_and_closes_links(self) -> None:
        uri = "udp://127.0.0.1:21850"
        link = self.FakeLink(uri)
        called: list[str] = []
        with demo.BoundedSwarm({uri: link}, connect_timeout_s=0.1) as swarm:
            swarm.parallel_safe(lambda scf: called.append(scf.link_uri))
        self.assertEqual(called, [uri])
        self.assertEqual(link.close_count, 1)
        self.assertEqual(link.connected.callbacks, [])
        self.assertEqual(link.connection_failed.callbacks, [])

    def test_bounded_swarm_times_out_and_closes_unresponsive_link(self) -> None:
        uri = "udp://127.0.0.1:21850"
        link = self.FakeLink(uri, respond=False)
        with (
            self.assertRaisesRegex(TimeoutError, "link connection timed out"),
            demo.BoundedSwarm({uri: link}, connect_timeout_s=0.001),
        ):
            self.fail("connection should not have completed")
        self.assertGreaterEqual(link.close_count, 1)
        self.assertEqual(link.connected.callbacks, [])
        self.assertEqual(link.connection_failed.callbacks, [])

    def test_initial_parameters_match_configured_pose(self) -> None:
        writes = []
        cf = SimpleNamespace(
            link_uri=demo.DRONES[1].uri,
            param=SimpleNamespace(
                set_value=lambda name, value: writes.append((name, value))
            ),
        )
        with patch("builtins.print"):
            demo.set_initial_params(
                SimpleNamespace(cf=cf),
                configs={demo.DRONES[1].uri: demo.DRONES[1]},
            )
        self.assertEqual(
            writes,
            [
                ("cmds.land", "0"),
                ("initial.xinit", "-1.950000"),
                ("initial.yinit", "0.000000"),
                ("initial.yawinit", "0.000000"),
                ("cmds.ready", "1"),
            ],
        )

    def test_operator_heartbeat_updates_the_radio_parameter(self) -> None:
        writes = []
        scf = SimpleNamespace(
            cf=SimpleNamespace(
                param=SimpleNamespace(
                    set_value=lambda name, value: writes.append((name, value))
                )
            )
        )
        demo.set_operator_heartbeat(scf, sequence=42)
        self.assertEqual(writes, [("cmds.heartbeat", "42")])

    def test_heartbeat_loop_updates_all_drones_and_wraps_uint32(self) -> None:
        writes = []

        class FakeSwarm:
            def parallel_safe(
                self, function: Callable[[demo.SyncCrazyflieLike], None]
            ) -> None:
                for _ in range(2):
                    scf = SimpleNamespace(
                        cf=SimpleNamespace(
                            param=SimpleNamespace(
                                set_value=lambda name, value: writes.append(
                                    (name, value)
                                )
                            )
                        )
                    )
                    function(scf)

        sleeps = []

        def stop_after_two_updates(delay: float) -> None:
            sleeps.append(delay)
            if len(sleeps) == 2:
                raise KeyboardInterrupt

        max_uint32 = demo.UINT32_MASK
        with self.assertRaises(KeyboardInterrupt):
            demo.run_operator_heartbeat_loop(
                FakeSwarm(),
                sleep=stop_after_two_updates,
                initial_sequence=max_uint32 - 1,
            )
        self.assertEqual(
            writes,
            [
                ("cmds.heartbeat", str(max_uint32)),
                ("cmds.heartbeat", str(max_uint32)),
                ("cmds.heartbeat", "0"),
                ("cmds.heartbeat", "0"),
            ],
        )
        self.assertEqual(sleeps, [0.25, 0.25])


if __name__ == "__main__":
    unittest.main()
