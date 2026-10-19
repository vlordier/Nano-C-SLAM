"""Keep the STM32 SITL inventory bound to app tasks and Kbuild sources."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
APP_ROOT = REPOSITORY_ROOT / "cslam-stm32-app"
INVENTORY = REPOSITORY_ROOT / "docs/STM32_SITL_INVENTORY.md"


def _listed_task_symbols(document: str) -> set[str]:
    task_table = document.split("## Runtime tasks created by appMain\n", 1)[1]
    task_table = task_table.split("\n## ", 1)[0]
    return set(re.findall(r"^\| ([a-zA-Z0-9_]+) \|", task_table, re.MULTILINE))


def _created_task_symbols() -> set[str]:
    source = (APP_ROOT / "app_main.c").read_text(encoding="utf-8")
    static = re.findall(r"STATIC_MEM_TASK_CREATE\(\s*(\w+)\s*,", source)
    dynamic = re.findall(r"\bxTaskCreate\(\s*(\w+)\s*,", source)
    return set(static + dynamic)


def _kbuild_source_paths() -> set[str]:
    sources: set[str] = set()
    for kbuild in APP_ROOT.rglob("Kbuild"):
        for line in kbuild.read_text(encoding="utf-8").splitlines():
            match = re.fullmatch(r"\s*obj-y\s*\+=\s*([^\s#]+)\s*", line)
            if match is None:
                continue
            target = match.group(1)
            if not target.endswith(".o"):
                continue
            source = (kbuild.parent / target).with_suffix(".c")
            if not source.is_file():
                raise AssertionError(f"Kbuild source does not exist: {source}")
            sources.add(source.relative_to(APP_ROOT).as_posix())
    return sources


def _listed_source_paths(document: str) -> set[str]:
    module_list = document.split(
        "## Owned modules selected by the app Kbuild files\n", 1
    )[1]
    module_list = module_list.split("\n## ", 1)[0]
    return set(re.findall(r"^- ([^ ]+\.c)$", module_list, re.MULTILINE))


class Stm32SitlInventoryTests(unittest.TestCase):
    def test_inventory_matches_app_task_creation_sites(self) -> None:
        document = INVENTORY.read_text(encoding="utf-8")

        self.assertEqual(
            _listed_task_symbols(document),
            {"mission_task", "tof_task", "flight_task", "uwb_ranging_task"},
        )
        self.assertEqual(_created_task_symbols(), _listed_task_symbols(document))

    def test_inventory_matches_owned_kbuild_translation_units(self) -> None:
        document = INVENTORY.read_text(encoding="utf-8")

        self.assertEqual(_listed_source_paths(document), _kbuild_source_paths())


if __name__ == "__main__":
    unittest.main()
