"""Tests for deriving the firmware-range Gazebo mapping scene."""

from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from scripts.prepare_gazebo_nearfield_world import prepare_nearfield_world


def _model(name: str, pose: str, size: str) -> str:
    return (
        f'<model name="{name}"><pose>{pose}</pose><static>true</static><link '
        f'name="link"><collision name="collision"><geometry><box><size>{size}'
        f'</size></box></geometry></collision><visual name="visual"><geometry>'
        f"<box><size>{size}</size></box></geometry></visual></link></model>"
    )


class NearfieldWorldTests(unittest.TestCase):
    def test_moves_arena_walls_within_firmware_range_gate(self) -> None:
        models = "".join(
            _model(name, "3 0 0.5 0 0 0", "0.2 6 1")
            for name in ("wall1", "wall2", "wall3", "wall4")
        )
        models += "".join(
            _model(name, "0 0 0.25 0 0 0", "0.5 0.5 0.5")
            for name in ("obstacle1", "obstacle2", "obstacle3")
        )
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.sdf"
            output = Path(directory) / "nearfield.sdf"
            source.write_text(f"<sdf version='1.8'><world>{models}</world></sdf>")

            prepare_nearfield_world(source, output)

            world = ET.fromstring(output.read_bytes()).find("world")  # noqa: S314
            assert world is not None
            walls = {
                model.get("name"): model
                for model in world.findall("model")
                if model.get("name", "").startswith("wall")
            }
            self.assertEqual(len(walls), 4)
            expected_extent = {
                "wall1": 1.5,
                "wall2": 1.5,
                "wall3": 0.95,
                "wall4": 1.5,
            }
            for name, wall in walls.items():
                pose = wall.findtext("pose")
                assert pose is not None
                x, y = (float(value) for value in pose.split()[:2])
                self.assertAlmostEqual(max(abs(x), abs(y)), expected_extent[name])
                size = wall.findtext("link/collision/geometry/box/size")
                self.assertEqual(size, "0.2 3.0 1")

            obstacles = {
                model.get("name"): model.findtext("pose")
                for model in world.findall("model")
                if model.get("name", "").startswith("obstacle")
            }
            self.assertEqual(len(obstacles), 3)
            self.assertIn("0.65 -0.15", obstacles["obstacle1"] or "")
            self.assertEqual(
                world.findtext(
                    "model[@name='obstacle1']/link/collision/geometry/box/size"
                ),
                "0.3 0.8 0.5",
            )

    def test_rejects_world_missing_required_boundary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.sdf"
            output = Path(directory) / "nearfield.sdf"
            source.write_text("<sdf version='1.8'><world /></sdf>")

            with self.assertRaisesRegex(ValueError, "wall1"):
                prepare_nearfield_world(source, output)


if __name__ == "__main__":
    unittest.main()
