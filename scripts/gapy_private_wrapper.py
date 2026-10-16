"""Load GAP9 GVSOC target modules before running the SDK's Gapy entry point."""

from __future__ import annotations

import runpy
import sys


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: gapy_private_wrapper.py /path/to/gapy [args ...]")

    gapy_path, *gapy_args = sys.argv[1:]
    # GVSOC has two packages named `devices`; loading the target models first
    # keeps the SDK's flash-device registration visible to Gapy.
    import devices.flash  # noqa: F401
    import gap9.gap9_v2  # noqa: F401

    sys.argv = [gapy_path, *gapy_args]
    runpy.run_path(gapy_path, run_name="__main__")


if __name__ == "__main__":
    main()
