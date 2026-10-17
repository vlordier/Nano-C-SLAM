#!/usr/bin/env python3
"""Create a close-wall variant of the installed Crazyswarm2 world for mapping."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

WALL_POSITIONS = {
    "wall1": (1.5, 0.0),
    "wall2": (-1.5, 0.0),
    "wall3": (0.0, 0.95),
    "wall4": (0.0, -1.5),
}
OBSTACLE_POSES = {
    "obstacle1": (0.65, -0.15, 0.25),
    "obstacle2": (-0.55, -1.0, 0.2),
    "obstacle3": (-1.0, -0.55, 0.2),
}


def _required_model(world: ET.Element, name: str) -> ET.Element:
    for model in world.findall("model"):
        if model.get("name") == name:
            return model
    raise ValueError(f"source world is missing model {name!r}")


def _set_pose_xy(model: ET.Element, x: float, y: float) -> None:
    pose = model.find("pose")
    if pose is None or pose.text is None:
        raise ValueError(f"model {model.get('name')!r} has no pose")
    values = pose.text.split()
    if len(values) != 6:
        raise ValueError(f"model {model.get('name')!r} pose must have six values")
    values[0:2] = [str(x), str(y)]
    pose.text = " ".join(values)


def _set_box_size(model: ET.Element, size: str) -> None:
    boxes = model.findall("link/collision/geometry/box/size")
    boxes += model.findall("link/visual/geometry/box/size")
    if len(boxes) < 2:
        raise ValueError(
            f"model {model.get('name')!r} must have collision and visual boxes"
        )
    for box_size in boxes:
        box_size.text = size


def prepare_nearfield_world(source: Path, destination: Path) -> None:
    """Move the arena boundaries and mapping obstacles within the 1.4 m gate."""
    source_data = source.read_bytes()
    if len(source_data) > 5_000_000 or b"<!DOCTYPE" in source_data.upper():
        raise ValueError("source SDF is oversized or contains a document type")
    # Only Gazebo's installed local SDF is used; DTD declarations are rejected.
    root = ET.fromstring(source_data)  # noqa: S314
    tree = ET.ElementTree(root)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("source SDF has no world")

    for name, (x, y) in WALL_POSITIONS.items():
        model = _required_model(world, name)
        _set_pose_xy(model, x, y)
        _set_box_size(model, "0.2 3.0 1")

    for name, (x, y, z) in OBSTACLE_POSES.items():
        model = _required_model(world, name)
        pose = model.find("pose")
        if pose is None:
            raise ValueError(f"model {name!r} has no pose")
        pose.text = f"{x} {y} {z} 0 0 0"
    _set_box_size(_required_model(world, "obstacle1"), "0.3 0.8 0.5")

    ET.indent(tree, space="  ")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare_nearfield_world(args.source, args.output)
    print(f"Prepared 1.4 m mapping scene: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
