"""CLI construction checks for the live mapping firmware runner."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.gazebo_gvsoc_live import _commands, _parse_args


class GazeboGvsocLiveCliTests(unittest.TestCase):
    def test_builds_uuid_bound_gvsoc_and_gazebo_commands(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            options = _parse_args(["--output", str(root / "live-run.json")])
            commands, socket_path, ready_path, run_root = _commands(
                options, "15a06f08-657e-4012-922e-4ca35f7dda22"
            )

        self.assertIn("run_gap9_gvsoc_smoke.sh", commands.gvsoc[-1])
        self.assertIn("scripts.gazebo_mapping_run", commands.gazebo)
        self.assertIn("--no-project", commands.gazebo)
        self.assertIn("--run-id", commands.gazebo)
        self.assertIn("15a06f08-657e-4012-922e-4ca35f7dda22", commands.gazebo)
        self.assertIn("--stream-socket", commands.gazebo)
        self.assertEqual(
            commands.gvsoc_environment["GAP9_GVSOC_COMPLETION_MARKER"],
            "NANO_C_SLAM_GVSOC_SPI_MAIN:map_dump_done",
        )
        self.assertEqual(options.mission_timeout, 600.0)
        self.assertEqual(options.mission_hover_duration, 120.0)
        hover_index = commands.gazebo.index("--mission-hover-duration")
        self.assertEqual(commands.gazebo[hover_index + 1], "120.0")
        mission_timeout_index = commands.gazebo.index("--timeout")
        self.assertEqual(commands.gazebo[mission_timeout_index + 1], "600.0")
        self.assertEqual(
            commands.gvsoc_environment["GAP9_GVSOC_SPI_STREAM_SOCKET"],
            str(socket_path),
        )
        self.assertEqual(
            commands.gvsoc_environment["GAP9_GVSOC_SPI_STREAM_READY"],
            str(ready_path),
        )
        self.assertEqual(
            commands.gvsoc_environment["GAP9_GVSOC_SPI_SEQUENCE"], "gazebo-live"
        )
        self.assertEqual(run_root.parent, options.output.parent)

    def test_requires_supported_firmware_packet_count_and_finite_deadlines(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = str(Path(temporary) / "live-run.json")
            for arguments in (
                ["--output", output, "--samples", "61"],
                ["--output", output, "--timeout", "nan"],
                ["--output", output, "--mission-timeout", "0"],
                ["--output", output, "--mission-hover-duration", "nan"],
            ):
                with self.subTest(arguments=arguments), self.assertRaises(SystemExit):
                    _parse_args(arguments)

    def test_forwards_heldout_route_to_mapping_runner(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "live.json"
            options = _parse_args(
                ["--output", str(output), "--mapping-route", "heldout-asymmetric"]
            )
            commands, *_ = _commands(options, "15a06f08-657e-4012-922e-4ca35f7dda22")

        route_index = commands.gazebo.index("--mapping-route")
        self.assertEqual(commands.gazebo[route_index + 1], "heldout-asymmetric")

    def test_forwards_requested_mission_hover_duration(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "live.json"
            options = _parse_args(
                ["--output", str(output), "--mission-hover-duration", "5"]
            )
            commands, *_ = _commands(options, "15a06f08-657e-4012-922e-4ca35f7dda22")

        hover_index = commands.gazebo.index("--mission-hover-duration")
        self.assertEqual(commands.gazebo[hover_index + 1], "5.0")


if __name__ == "__main__":
    unittest.main()
