"""Keep the STM32 SITL inventory bound to app tasks and Kbuild sources."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
APP_ROOT = REPOSITORY_ROOT / "cslam-stm32-app"
INVENTORY = REPOSITORY_ROOT / "docs/STM32_SITL_INVENTORY.md"
DECISION = REPOSITORY_ROOT / "docs/decisions/ADR-0001-nano-stm32-gazebo-sitl.md"


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
    pending = [APP_ROOT]
    visited: set[Path] = set()
    while pending:
        directory = pending.pop()
        if directory in visited:
            continue
        visited.add(directory)
        kbuild = directory / "Kbuild"
        if not kbuild.is_file():
            continue
        for line in kbuild.read_text(encoding="utf-8").splitlines():
            match = re.fullmatch(r"\s*obj-y\s*\+=\s*([^\s#]+)\s*", line)
            if match is None:
                continue
            target = match.group(1)
            if target.startswith("/") and target.endswith("/"):
                child = (directory / target.strip("/")).resolve()
                if APP_ROOT.resolve() not in child.parents:
                    continue
                pending.append(child)
                continue
            if not target.endswith(".o"):
                continue
            source = (kbuild.parent / target).with_suffix(".c")
            if not source.is_file():
                raise AssertionError(f"Kbuild source does not exist: {source}")
            sources.add(source.relative_to(APP_ROOT).as_posix())
    return sources


def _sitl_manifest_source_paths() -> set[str]:
    manifest = (APP_ROOT / "sitl/nano_sources.cmake").read_text(encoding="utf-8")
    return set(re.findall(r"^\s+([^\s#]+\.c)\s*$", manifest, re.MULTILINE))


def _listed_source_paths(document: str) -> set[str]:
    module_list = document.split(
        "## Owned modules selected by the app Kbuild files\n", 1
    )[1]
    module_list = module_list.split("\n## ", 1)[0]
    return set(re.findall(r"^- ([^ ]+\.c)$", module_list, re.MULTILINE))


def _matrix_source_paths(document: str) -> set[str]:
    matrix = document.split(
        "### Source readiness matrix for the Linux `cf2` target\n", 1
    )[1]
    matrix = matrix.split("\nThe classifier is guarded by", 1)[0]
    return set(re.findall(r"^\| `([^`]+\.c)` \|", matrix, re.MULTILINE))


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
        self.assertEqual(_matrix_source_paths(document), _kbuild_source_paths())

    def test_sitl_manifest_is_exactly_kbuild_sources_without_physical_tof(
        self,
    ) -> None:
        kbuild_sources = _kbuild_source_paths()
        physical_tof = {
            "src/tof-matrix/drivers/i2c_expander.c",
            "src/tof-matrix/drivers/vl53l5cx_api.c",
            "src/tof-matrix/i2c_platform.c",
            "src/tof-matrix/tof_matrix.c",
            "app_main.c",
        }
        self.assertEqual(kbuild_sources - physical_tof, _sitl_manifest_source_paths())

    def test_architecture_decision_names_the_pinned_runtime_and_claim_limit(
        self,
    ) -> None:
        decision = DECISION.read_text(encoding="utf-8")
        for evidence in (
            "662a7916bbd4454bad117a39aa324000bdb0c004",
            "30a6177ff5c14f0782a72a50c27a90a955d1b7b8",
            "Gazebo Sim 8.14.0",
            "Cortex-M4",
            "power_distribution_sitl.c::motorsSetRatio",
            "GVSOC GAP9 testbench",
            "does not emulate the Cortex-M4 instruction set",
        ):
            with self.subTest(evidence=evidence):
                self.assertIn(evidence, decision)

    def test_inventory_distinguishes_stub_compilation_from_sitl_execution(self) -> None:
        document = INVENTORY.read_text(encoding="utf-8")

        self.assertIn(
            "The suite uses purpose-built test stubs, not\nthe FreeRTOS POSIX port.",
            document,
        )
        for marker in (
            "NCS_SITL_TASK_READY",
            "NCS_SITL_SENSOR",
            "NCS_SITL_MOTOR",
            "NCS_SITL_MISSION",
        ):
            with self.subTest(marker=marker):
                self.assertIn(marker, document)


if __name__ == "__main__":
    unittest.main()
