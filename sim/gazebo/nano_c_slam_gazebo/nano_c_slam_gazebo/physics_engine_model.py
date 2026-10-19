"""Build the Gazebo server's physics-engine command-line arguments."""

from __future__ import annotations

import re

_ENGINE_NAME = re.compile(r"[A-Za-z0-9_.-]+\Z")
_BASE_SERVER_ARGUMENTS = (
    " -r -s --headless-rendering --render-engine-server=ogre2 "
    "--render-engine-server-api-backend=metal"
)


def gazebo_server_arguments(physics_engine: str | None = None) -> str:
    """Return safe launch arguments, optionally selecting an engine by name."""
    if physics_engine is None or not physics_engine.strip():
        return _BASE_SERVER_ARGUMENTS

    engine_name = physics_engine.strip()
    if engine_name.startswith("-") or _ENGINE_NAME.fullmatch(engine_name) is None:
        raise ValueError("Gazebo physics engine must be a plugin name")
    return f"{_BASE_SERVER_ARGUMENTS} --physics-engine {engine_name}"
