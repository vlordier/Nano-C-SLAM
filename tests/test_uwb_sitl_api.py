"""Exercise the C UWB provider against the Python broker over localhost UDP."""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

from scripts.uwb_sitl_broker import UwbBrokerCore

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class LocalUwbBroker:
    def __init__(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.005)
        self.port = self.socket.getsockname()[1]
        self.core = UwbBrokerCore()
        self.stop_event = threading.Event()
        self.sim_time_ns = 1_000_000_000
        now_ns = time.monotonic_ns()
        self.core.observe_clock(self.sim_time_ns, now_ns)
        self.core.observe_pose(0, self.sim_time_ns, (2.0, 0.5, 0.0), now_ns)
        self.core.observe_pose(1, self.sim_time_ns, (3.0, 0.5, 0.0), now_ns)
        self.thread = threading.Thread(target=self._run, daemon=True)

    def __enter__(self) -> LocalUwbBroker:
        self.thread.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2)
        self.socket.close()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            now_ns = time.monotonic_ns()
            self.sim_time_ns += 5_000_000
            self.core.observe_clock(self.sim_time_ns, now_ns)
            self.core.observe_pose(0, self.sim_time_ns, (2.0, 0.5, 0.0), now_ns)
            self.core.observe_pose(1, self.sim_time_ns, (3.0, 0.5, 0.0), now_ns)
            try:
                payload, remote = self.socket.recvfrom(2048)
            except TimeoutError:
                payload = b""
                remote = ("127.0.0.1", 0)
            for address, response in self.core.handle(
                payload, remote, time.monotonic_ns()
            ):
                self.socket.sendto(response, address)
            for address, response in self.core.tick(time.monotonic_ns()):
                self.socket.sendto(response, address)


def _reserve_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class UwbSitlApiTests(unittest.TestCase):
    def test_two_c_nodes_exchange_ack_and_timestamped_range(self) -> None:
        compiler = shutil.which("cc") or shutil.which("clang")
        if compiler is None:
            self.fail("host C compiler required for UWB provider integration test")
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "uwb_sitl_api_harness"
            compile_result = subprocess.run(  # noqa: S603 -- fixed compiler and owned sources
                [
                    compiler,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-pedantic",
                    "-Wno-strict-prototypes",
                    "-pthread",
                    "-I",
                    str(REPOSITORY_ROOT / "tests/stubs/uwb_sitl"),
                    "-I",
                    str(REPOSITORY_ROOT / "uwb-software-library/inc"),
                    "-I",
                    str(REPOSITORY_ROOT / "cslam-stm32-app/sitl"),
                    str(REPOSITORY_ROOT / "tests/uwb_sitl_api_harness.c"),
                    str(REPOSITORY_ROOT / "cslam-stm32-app/sitl/uwb_api_sitl.c"),
                    str(REPOSITORY_ROOT / "cslam-stm32-app/sitl/uwb_sitl_protocol.c"),
                    "-o",
                    str(executable),
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
            with LocalUwbBroker() as broker:
                common_environment = os.environ.copy()
                common_environment["NCS_SITL_UWB_BROKER_HOST"] = "127.0.0.1"
                common_environment["NCS_SITL_UWB_BROKER_PORT"] = str(broker.port)
                common_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(_reserve_udp_port())
                node1_environment = common_environment.copy()
                node1_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(_reserve_udp_port())
                node1 = subprocess.Popen(  # noqa: S603 -- executes compiled owned harness
                    [str(executable), "1"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=node1_environment,
                )
                try:
                    time.sleep(0.2)
                    node0 = subprocess.run(  # noqa: S603 -- executes compiled owned harness
                        [str(executable), "0"],
                        check=False,
                        capture_output=True,
                        text=True,
                        timeout=8,
                        env=common_environment,
                    )
                    node1_stdout, node1_stderr = node1.communicate(timeout=8)
                finally:
                    if node1.poll() is None:
                        node1.kill()
                        node1.communicate()
            self.assertEqual(node0.returncode, 0, node0.stderr)
            self.assertIn("ACK2 and 1000 mm range passed", node0.stdout)
            self.assertEqual(node1.returncode, 0, node1_stderr)
            self.assertIn("message and ranging measurement passed", node1_stdout)
            with LocalUwbBroker() as missing_broker:
                missing_environment = os.environ.copy()
                missing_environment["NCS_SITL_UWB_BROKER_HOST"] = "127.0.0.1"
                missing_environment["NCS_SITL_UWB_BROKER_PORT"] = str(
                    missing_broker.port
                )
                missing_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(
                    _reserve_udp_port()
                )
                missing = subprocess.run(  # noqa: S603 -- executes owned harness
                    [str(executable), "missing"],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=8,
                    env=missing_environment,
                )
            self.assertEqual(missing.returncode, 0, missing.stderr)
            self.assertIn("missing peer returned UWB_RX_TIMEOUT", missing.stdout)


if __name__ == "__main__":
    unittest.main()
