"""Compile and fault-inject the owned STM32 ToF initialization path."""

from __future__ import annotations

import binascii
import shutil
import socket
import struct
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class Stm32TofMatrixInitTests(unittest.TestCase):
    def test_mission_task_waits_for_tof_heartbeat_and_estimator_before_arm(
        self,
    ) -> None:
        source = (REPOSITORY_ROOT / "cslam-stm32-app/app_main.c").read_text(
            encoding="utf-8"
        )
        mission_task = source.split("void mission_task(void *parameters) {", 1)[1]
        tof_gate = mission_task.index("while (!tof_is_init())")
        command_gate = mission_task.index(
            "if (ready && tof_is_fresh() && operator_alive)"
        )
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

    def test_failed_uwb_token_sweep_stops_before_responder_restarts(self) -> None:
        source = (REPOSITORY_ROOT / "cslam-stm32-app/src/uwb_ranging.c").read_text(
            encoding="utf-8"
        )
        handoff = source.split("if (!uwb_token_lease_finish_handoff(", 1)[1].split(
            "send_token_with_opt = 0", 1
        )[0]
        self.assertIn("&land", handoff)
        self.assertIn("vTaskDelete(NULL)", handoff)
        self.assertIn("return;", handoff)
        self.assertNotIn("uwb_responder_on()", handoff)
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

    def test_sitl_udp_frame_decoder_rejects_malformed_and_unsafe_ranges(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "stm32_tof_sitl_frame_test"
            self._compile(
                REPOSITORY_ROOT / "tests/stm32_tof_sitl_frame_test.c",
                [REPOSITORY_ROOT / "cslam-stm32-app/sitl/tof_matrix_sitl.c"],
                executable,
            )
            result = subprocess.run(  # noqa: S603 -- temp binary built above
                [str(executable)],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("frame decoder passed", result.stdout)

    def test_sitl_c_provider_receives_host_encoded_matrix_frame(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "stm32_tof_sitl_listener"
            self._compile(
                REPOSITORY_ROOT / "tests/stm32_tof_sitl_frame_test.c",
                [REPOSITORY_ROOT / "cslam-stm32-app/sitl/tof_matrix_sitl.c"],
                executable,
            )
            receiver = subprocess.Popen(  # noqa: S603 -- temp binary built above
                [str(executable), "--listen"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                header = struct.pack(
                    "!4sBBBBIQI", b"NCTF", 1, 4, 8, 8, 17, 1_000_000_000, 512
                )
                payload = struct.pack("!256h", *([1234] * 256))
                body = header + payload
                packet = body + struct.pack("!I", binascii.crc32(body) & 0xFFFFFFFF)
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    sender.bind(("127.0.0.1", 39950))
                    sender.settimeout(0.01)
                    send_deadline = time.monotonic() + 3.0
                    registration = b""
                    while receiver.poll() is None and time.monotonic() < send_deadline:
                        sender.sendto(packet, ("127.0.0.1", 39951))
                        try:
                            response, _ = sender.recvfrom(64)
                        except TimeoutError:
                            pass
                        else:
                            if response.startswith(b"NCTR\x01\x00"):
                                registration = response
                        # Keep transmitting after registration. The receiver's
                        # initial registration can arrive after the first data
                        # packet was sent, before its task polls the socket.
                        time.sleep(0.01)
                stdout, stderr = receiver.communicate(timeout=10)
            finally:
                if receiver.poll() is None:
                    receiver.kill()
                    receiver.communicate()

        self.assertEqual(receiver.returncode, 0, stderr)
        self.assertEqual(registration[:6], b"NCTR\x01\x00")
        self.assertIn("received fresh sensor 0 matrix (64 valid cells)", stdout)


if __name__ == "__main__":
    unittest.main()
