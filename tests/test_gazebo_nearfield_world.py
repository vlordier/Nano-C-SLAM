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


def _source_world() -> str:
    """Build the installed arena's four cardinal walls and three boxes."""
    walls = "".join(
        _model(
            name,
            f"3 0 0.5 0 0 {yaw}",
            "0.2 6 1",
        )
        for name, yaw in (
            ("wall1", 0),
            ("wall2", 0),
            ("wall3", 1.57),
            ("wall4", 1.57),
        )
    )
    obstacles = "".join(
        _model(name, "0 0 0.25 0 0 0", "0.5 0.5 0.5")
        for name in ("obstacle1", "obstacle2", "obstacle3")
    )
    return f"<sdf version='1.8'><world>{walls}{obstacles}</world></sdf>"


class NearfieldWorldTests(unittest.TestCase):
    def test_moves_arena_walls_within_firmware_range_gate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.sdf"
            output = Path(directory) / "nearfield.sdf"
            source.write_text(_source_world())

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

    def test_heldout_variant_changes_geometry_and_is_repeatable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.sdf"
            baseline = root / "baseline.sdf"
            heldout = root / "heldout.sdf"
            heldout_repeat = root / "heldout-repeat.sdf"
            source.write_text(_source_world())

            prepare_nearfield_world(source, baseline)
            prepare_nearfield_world(source, heldout, variant="heldout")
            prepare_nearfield_world(source, heldout_repeat, variant="heldout")

            self.assertNotEqual(baseline.read_bytes(), heldout.read_bytes())
            self.assertEqual(heldout.read_bytes(), heldout_repeat.read_bytes())
            world = ET.fromstring(heldout.read_bytes()).find("world")  # noqa: S314
            assert world is not None
            self.assertEqual(
                world.findtext("model[@name='wall1']/pose"), "0.8 0.1 0.5 0 0 0"
            )
            self.assertEqual(
                world.findtext("model[@name='wall3']/pose"),
                "0.25 0.75 0.5 0 0 1.57",
            )
            # At the known mission spawn (-0.5, -0.5), all four arena walls
            # remain within the firmware's 1.4 m range gate. This keeps each
            # 20-pose scan window geometrically observable on the held-out map.
            wall1_surface_distance = 0.8 - 0.1 - (-0.5)
            wall2_surface_distance = -0.5 - (-1.7 + 0.1)
            wall3_surface_distance = 0.75 - 0.1 - (-0.5)
            wall4_surface_distance = -0.5 - (-1.7 + 0.1)
            self.assertLessEqual(
                max(
                    wall1_surface_distance,
                    wall2_surface_distance,
                    wall3_surface_distance,
                    wall4_surface_distance,
                ),
                1.4,
            )
            self.assertEqual(
                world.findtext("model[@name='obstacle3']/pose"),
                "-1.25 -0.25 0.2 0 0 0",
            )

    def test_rejects_unknown_world_variant(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.sdf"
            source.write_text("<sdf version='1.8'><world /></sdf>")
            with self.assertRaisesRegex(ValueError, "variant"):
                prepare_nearfield_world(source, root / "output.sdf", variant="other")


if __name__ == "__main__":
    unittest.main()
