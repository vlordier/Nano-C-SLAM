"""Validate that a generated GAP9 GVSOC config exposes the SPI testbench."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def validate_config(config_path: Path) -> list[str]:
    """Return configuration defects that would make the SPI testbench unusable."""
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return [f"cannot read GVSOC config {config_path}: {error}"]

    if not isinstance(config, dict):
        return ["GVSOC config root must be a JSON object"]
    target = config.get("target")
    if not isinstance(target, dict):
        return ["GVSOC config is missing the target object"]

    problems: list[str] = []
    if target.get("addon_testbench_enabled") is not True:
        problems.append("target.addon_testbench_enabled is not true")

    components = target.get("components")
    if not isinstance(components, list) or "testbench" not in components:
        problems.append("target.components does not contain testbench")

    bindings = target.get("bindings")
    if not isinstance(bindings, list):
        problems.append("target.bindings is not a list")
    else:
        required_bindings = {
            ("chip->spim2_cs0_data", "testbench->spi2_cs0_data"),
            ("chip->spim2_cs0", "testbench->spi2_cs0"),
        }
        observed = {
            tuple(binding)
            for binding in bindings
            if isinstance(binding, list)
            and all(isinstance(item, str) for item in binding)
        }
        problems.extend(
            f"missing SPI2 testbench binding: {source} -> {destination}"
            for source, destination in sorted(required_bindings - observed)
        )

    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="generated gvsoc_config.json path")
    args = parser.parse_args(argv)

    problems = validate_config(args.config)
    if problems:
        for problem in problems:
            print(f"GVSOC SPI testbench config error: {problem}", file=sys.stderr)
        return 1
    print(f"GVSOC SPI testbench enabled and bound in {args.config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
