"""Compile and fault-inject the owned STM32 ToF initialization path."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class Stm32TofMatrixInitTests(unittest.TestCase):
    def test_mission_task_waits_for_tof_and_estimator_before_commander_enable(
        self,
    ) -> None:
        source = (REPOSITORY_ROOT / "cslam-stm32-app/app_main.c").read_text(
            encoding="utf-8"
        )
        mission_task = source.split("void mission_task(void *parameters) {", 1)[1]
        tof_gate = mission_task.index("while (!tof_is_init())")
        command_gate = mission_task.index("while (!ready || !tof_is_fresh())")
        estimator_init = mission_task.index("estimatorKalmanInitToPose")
        flight_task_create = mission_task.index('"fly"')
        commander_enable = mission_task.index("enable_commander()")
        self.assertLess(tof_gate, command_gate)
        self.assertLess(command_gate, estimator_init)
        self.assertLess(estimator_init, flight_task_create)
        self.assertLess(flight_task_create, commander_enable)

    def test_mission_deadline_is_polled_and_stops_mapping_work(self) -> None:
        source = (REPOSITORY_ROOT / "cslam-stm32-app/app_main.c").read_text(
            encoding="utf-8"
        )
        mission_task = source.split("void mission_task(void *parameters) {", 1)[1]
        deadline_check = mission_task.index("mission_timeout_update(")
        semaphore_wait = mission_task.index("xSemaphoreTake(tofSemaphore0")
        mapping_work = mission_task.index("send_burst_data(0")
        self.assertLess(deadline_check, semaphore_wait)
        self.assertLess(semaphore_wait, mapping_work)
        self.assertIn("M2T(MISSION_SEMAPHORE_POLL_MS)", mission_task)
        self.assertIn("if (mission_timeout.landing_requested)", mission_task)
        self.assertNotIn("portMAX_DELAY", mission_task)

    def test_failed_uwb_token_sweep_requests_landing(self) -> None:
        source = (REPOSITORY_ROOT / "cslam-stm32-app/src/uwb_ranging.c").read_text(
            encoding="utf-8"
        )
        handoff = source.split("if (!uwb_token_try_handoff(", 1)[1].split(
            "agent_has_token = 0", 1
        )[0]
        self.assertIn("land = 1", handoff)
        self.assertIn("vTaskDelete(NULL)", handoff)
        self.assertNotIn("while (token_pass_ok != UWB_SUCCESS)", source)

    def _compile(self, harness: Path, sources: list[Path], executable: Path) -> None:
        compiler = shutil.which("cc") or shutil.which("clang")
        if compiler is None:
            self.fail("host C compiler required for STM32 ToF fault tests")
        result = subprocess.run(  # noqa: S603 -- fixed compiler and repository-owned sources
            [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(REPOSITORY_ROOT / "tests/stubs/stm32"),
                "-I",
                str(REPOSITORY_ROOT / "cslam-stm32-app/src"),
                "-I",
                str(REPOSITORY_ROOT / "cslam-stm32-app/src/tof-matrix"),
                "-I",
                str(REPOSITORY_ROOT / "cslam-stm32-app/src/tof-matrix/drivers"),
                str(harness),
                *map(str, sources),
                "-o",
                str(executable),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_daq_readiness_stays_false_until_recovery_and_backoff_is_capped(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "stm32_tof_daq_test"
            self._compile(
                REPOSITORY_ROOT / "tests/stm32_tof_daq_test.c",
                [
                    REPOSITORY_ROOT / "cslam-stm32-app/src/tof_daq.c",
                    REPOSITORY_ROOT / "cslam-stm32-app/src/tof_freshness.c",
                ],
                executable,
            )
            for scenario in ("persistent", "recovery", "unavailable-data"):
                with self.subTest(scenario=scenario):
                    result = subprocess.run(  # noqa: S603 -- temp binary built above
                        [str(executable), scenario],
                        check=False,
                        capture_output=True,
                        text=True,
                        timeout=10,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn("retry contract passed", result.stdout)

    def test_every_sensor_configuration_failure_is_reported_and_retryable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "stm32_tof_matrix_init_test"
            source = REPOSITORY_ROOT / "cslam-stm32-app/src/tof-matrix"
            compiler = shutil.which("cc") or shutil.which("clang")
            if compiler is None:
                self.fail("host C compiler required for STM32 ToF fault tests")
            self._compile(
                REPOSITORY_ROOT / "tests/stm32_tof_matrix_init_test.c",
                [
                    source / "tof_matrix.c",
                    source / "drivers/i2c_expander.c",
                ],
                executable,
            )
            execution = subprocess.run(  # noqa: S603 -- temp binary built above
                [str(executable)],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )

        self.assertEqual(execution.returncode, 0, execution.stderr)
        self.assertIn("fault injection passed", execution.stdout)


if __name__ == "__main__":
    unittest.main()
