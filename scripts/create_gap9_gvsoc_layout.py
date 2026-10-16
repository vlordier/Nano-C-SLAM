"""Create the minimal ROM-boot layout for the GAP9 ICP smoke application."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    parser.add_argument("layout", type=Path)
    args = parser.parse_args()
    elf = args.elf.resolve(strict=True)
    args.layout.parent.mkdir(parents=True, exist_ok=True)
    layout = {
        "sections": [
            {
                "name": "rom",
                "template": "rom",
                "properties": {"binary": str(elf), "boot": True},
            }
        ]
    }
    args.layout.write_text(json.dumps(layout, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
