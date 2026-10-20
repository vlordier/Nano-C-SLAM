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
from dataclasses import replace
from pathlib import Path

from scripts.uwb_sitl_broker import UwbBrokerCore
from scripts.uwb_sitl_protocol import decode_datagram, encode_datagram

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class LocalUwbBroker:
    def __init__(
        self,
        *,
        stale_peer: bool = False,
        corrupt_errors: bool = False,
        wrong_destination: bool = False,
    ) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.005)
        self.port = self.socket.getsockname()[1]
        self.core = UwbBrokerCore()
        self.stop_event = threading.Event()
        self.sim_time_ns = 1_000_000_000
        self.stale_peer = stale_peer
        self.corrupt_errors = corrupt_errors
        self.wrong_destination = wrong_destination
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
            if not self.stale_peer:
                self.core.observe_pose(1, self.sim_time_ns, (3.0, 0.5, 0.0), now_ns)
            try:
                payload, remote = self.socket.recvfrom(2048)
            except TimeoutError:
                payload = b""
                remote = ("127.0.0.1", 0)
            for address, response in self.core.handle(
                payload, remote, time.monotonic_ns()
            ):
                if self.corrupt_errors and response[5] == 255:
                    response = response[:-1] + bytes((response[-1] ^ 0xFF,))
                if self.wrong_destination and response[5] == 4:
                    packet = decode_datagram(response)
                    response = encode_datagram(replace(packet, destination_id=2))
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
                    if node1.stdout is not None:
                        node1.stdout.close()
                    if node1.stderr is not None:
                        node1.stderr.close()
            self.assertEqual(node0.returncode, 0, node0.stderr)
            self.assertIn("ACK2 and 1000 mm range passed", node0.stdout)
            self.assertEqual(node1.returncode, 0, node1_stderr)
            self.assertIn("message and ranging measurement passed", node1_stdout)
            with LocalUwbBroker(wrong_destination=True) as wrong_broker:
                wrong_environment = os.environ.copy()
                wrong_environment["NCS_SITL_UWB_BROKER_HOST"] = "127.0.0.1"
                wrong_environment["NCS_SITL_UWB_BROKER_PORT"] = str(wrong_broker.port)
                wrong_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(_reserve_udp_port())
                wrong_peer_environment = wrong_environment.copy()
                wrong_peer_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(
                    _reserve_udp_port()
                )
                wrong_peer = subprocess.Popen(  # noqa: S603 -- executes owned harness
                    [str(executable), "1"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=wrong_peer_environment,
                )
                try:
                    time.sleep(0.2)
                    wrong = subprocess.run(  # noqa: S603 -- executes owned harness
                        [str(executable), "wrong-dest"],
                        check=False,
                        capture_output=True,
                        text=True,
                        timeout=8,
                        env=wrong_environment,
                    )
                    wrong_peer_stdout, wrong_peer_stderr = wrong_peer.communicate(
                        timeout=8
                    )
                finally:
                    if wrong_peer.poll() is None:
                        wrong_peer.kill()
                        wrong_peer.communicate()
                    if wrong_peer.stdout is not None:
                        wrong_peer.stdout.close()
                    if wrong_peer.stderr is not None:
                        wrong_peer.stderr.close()
            self.assertEqual(wrong.returncode, 0, wrong.stderr)
            self.assertIn("timeout and zero", wrong.stdout)
            self.assertEqual(wrong_peer.returncode, 0, wrong_peer_stderr)
            self.assertIn("measurement passed", wrong_peer_stdout)
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
            with LocalUwbBroker(corrupt_errors=True) as corrupt_broker:
                corrupt_environment = os.environ.copy()
                corrupt_environment["NCS_SITL_UWB_BROKER_HOST"] = "127.0.0.1"
                corrupt_environment["NCS_SITL_UWB_BROKER_PORT"] = str(
                    corrupt_broker.port
                )
                corrupt_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(
                    _reserve_udp_port()
                )
                corrupt = subprocess.run(  # noqa: S603 -- executes owned harness
                    [str(executable), "range-failure"],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=8,
                    env=corrupt_environment,
                )
            self.assertEqual(corrupt.returncode, 0, corrupt.stderr)
            self.assertIn("zero range", corrupt.stdout)
            with LocalUwbBroker(stale_peer=True) as stale_broker:
                stale_environment = os.environ.copy()
                stale_environment["NCS_SITL_UWB_BROKER_HOST"] = "127.0.0.1"
                stale_environment["NCS_SITL_UWB_BROKER_PORT"] = str(stale_broker.port)
                stale_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(_reserve_udp_port())
                peer_environment = stale_environment.copy()
                peer_environment["NCS_SITL_UWB_LOCAL_PORT"] = str(_reserve_udp_port())
                peer = subprocess.Popen(  # noqa: S603 -- executes owned harness
                    [str(executable), "idle-peer"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=peer_environment,
                )
                try:
                    time.sleep(0.2)
                    stale = subprocess.run(  # noqa: S603 -- executes owned harness
                        [str(executable), "stale-range"],
                        check=False,
                        capture_output=True,
                        text=True,
                        timeout=8,
                        env=stale_environment,
                    )
                finally:
                    if peer.poll() is None:
                        peer.terminate()
                        try:
                            peer.communicate(timeout=2)
                        except subprocess.TimeoutExpired:
                            peer.kill()
                            peer.communicate()
                    else:
                        peer.communicate()
                    if peer.stdout is not None:
                        peer.stdout.close()
                    if peer.stderr is not None:
                        peer.stderr.close()
            self.assertEqual(stale.returncode, 0, stale.stderr)
            self.assertIn("zero range", stale.stdout)


if __name__ == "__main__":
    unittest.main()
