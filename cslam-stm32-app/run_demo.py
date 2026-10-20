"""Run and log a Nano-C-SLAM Crazyflie mission."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from threading import Event, Lock
from typing import NoReturn, Protocol, cast

POSE_STRUCT = struct.Struct("<hxxlfff32h")
SCAN_STRUCT = struct.Struct("<h")
POSE_PACKET = 0
SCAN_PACKET = 1
FINAL_FRAGMENT = 0x80
OPERATOR_HEARTBEAT_INTERVAL_SECONDS = 0.25
UINT32_MASK = 0xFFFFFFFF
PARAM_DOWNLOAD_TIMEOUT_SECONDS = 30.0
LINK_CONNECT_TIMEOUT_SECONDS = 10.0


class PacketLike(Protocol):
    @property
    def data(self) -> bytearray: ...


class ParamLike(Protocol):
    is_updated: bool

    def set_value(self, name: str, value: str) -> None: ...


class CallbackListLike(Protocol):
    def add_callback(self, callback: Callable[[str], None]) -> None: ...


class LinkCallbackListLike(Protocol):
    def add_callback(self, callback: Callable[..., None], /) -> None: ...

    def remove_callback(self, callback: Callable[..., None], /) -> None: ...


class ConsoleLike(Protocol):
    receivedChar: CallbackListLike


class CrazyflieLike(Protocol):
    link_uri: str
    param: ParamLike
    console: ConsoleLike

    def add_port_callback(
        self, port: int, callback: Callable[[PacketLike], None], /
    ) -> None: ...


class LinkCrazyflieLike(CrazyflieLike, Protocol):
    connected: LinkCallbackListLike
    connection_failed: LinkCallbackListLike

    def open_link(self, uri: str) -> None: ...

    def close_link(self) -> None: ...


class SyncCrazyflieLike(Protocol):
    @property
    def cf(self) -> LinkCrazyflieLike: ...

    @property
    def link_uri(self) -> str: ...


class SwarmLike(Protocol):
    def parallel_safe(self, function: Callable[[SyncCrazyflieLike], None]) -> None: ...


class BoundedSwarm:
    """Open cflib links with a timeout and close all links on exit."""

    def __init__(
        self,
        links: Mapping[str, LinkCrazyflieLike],
        *,
        connect_timeout_s: float = LINK_CONNECT_TIMEOUT_SECONDS,
    ) -> None:
        if connect_timeout_s <= 0:
            raise ValueError("link connection timeout must be positive")
        self._links = links
        self._connect_timeout_s = connect_timeout_s
        self._sync_cfs = {
            uri: SyncCrazyflieLikeAdapter(cf, uri) for uri, cf in links.items()
        }

    def __enter__(self) -> BoundedSwarm:
        try:
            self.parallel_safe(self._open_link)
        except Exception:
            self.close_links()
            raise
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.close_links()

    def parallel_safe(self, function: Callable[[SyncCrazyflieLike], None]) -> None:
        with ThreadPoolExecutor(max_workers=max(1, len(self._sync_cfs))) as executor:
            futures = [
                executor.submit(function, scf) for scf in self._sync_cfs.values()
            ]
            for future in futures:
                future.result()

    def close_links(self) -> None:
        for cf in self._links.values():
            cf.close_link()

    def _open_link(self, scf: SyncCrazyflieLike) -> None:
        open_crazyflie_link(
            scf.cf,
            scf.link_uri,
            connect_timeout_s=self._connect_timeout_s,
        )


@dataclass(frozen=True)
class SyncCrazyflieLikeAdapter:
    cf: LinkCrazyflieLike
    link_uri: str


def open_crazyflie_link(
    cf: LinkCrazyflieLike,
    uri: str,
    *,
    connect_timeout_s: float = LINK_CONNECT_TIMEOUT_SECONDS,
) -> None:
    if connect_timeout_s <= 0:
        raise ValueError("link connection timeout must be positive")
    connected = Event()
    failure_messages: list[str] = []

    def on_connected(_link_uri: str) -> None:
        connected.set()

    def on_connection_failed(_link_uri: str, message: str) -> None:
        failure_messages.append(message)
        connected.set()

    cf.connected.add_callback(on_connected)
    cf.connection_failed.add_callback(on_connection_failed)
    try:
        cf.open_link(uri)
        if not connected.wait(connect_timeout_s):
            cf.close_link()
            raise TimeoutError(
                f"link connection timed out for {uri} after {connect_timeout_s:g}s"
            )
        if failure_messages:
            raise ConnectionError(
                f"link connection failed for {uri}: {failure_messages[0]}"
            )
    finally:
        cf.connected.remove_callback(on_connected)
        cf.connection_failed.remove_callback(on_connection_failed)


@dataclass(frozen=True)
class DroneConfig:
    uri: str
    label: str
    initial_pose: tuple[float, float, float]


# Change these positions to match the launch layout before a flight.
DRONES = (
    DroneConfig("radio://0/100/2M/E7E7E7E7EA", "A", (0.0, 0.0, 0.0)),
    DroneConfig("radio://0/100/2M/E7E7E7E7EB", "B", (-1.95, 0.0, 0.0)),
    DroneConfig("radio://0/100/2M/E7E7E7E7EC", "C", (-3.90, 0.0, 0.0)),
    DroneConfig("radio://0/100/2M/E7E7E7E7ED", "D", (-11.5, 0.0, 3.14)),
)


@dataclass(frozen=True)
class PoseRecord:
    pose_id: int
    timestamp: int
    x: float
    y: float
    yaw: float
    tof: tuple[int, ...]

    def csv_row(self) -> list[int | float]:
        return [
            self.pose_id,
            self.timestamp,
            round(self.x, 3),
            round(self.y, 3),
            round(self.yaw, 3),
            *self.tof,
        ]


def decode_pose_payload(payload: bytes) -> PoseRecord:
    if len(payload) != POSE_STRUCT.size:
        raise ValueError(f"pose payload must be {POSE_STRUCT.size} bytes")
    pose_id, timestamp, x, y, yaw, *tof = POSE_STRUCT.unpack(payload)
    return PoseRecord(pose_id, timestamp, x, y, yaw, tuple(tof))


def decode_scan_packet(packet: bytes) -> int:
    if len(packet) < 2 + SCAN_STRUCT.size:
        raise ValueError("scan packet is too short")
    return int(SCAN_STRUCT.unpack_from(packet, 2)[0])


class PoseAssembler:
    def __init__(self) -> None:
        self._buffer: bytearray | None = None
        self._next_fragment = 0

    def feed(self, packet: bytes) -> PoseRecord | None:
        if len(packet) < 2 or packet[0] not in (POSE_PACKET, FINAL_FRAGMENT):
            return None
        if packet[1] == 0:
            self._buffer = bytearray()
            self._next_fragment = 0
        if self._buffer is None or packet[1] != self._next_fragment:
            self._buffer = None
            return None
        self._buffer.extend(packet[2:])
        if len(self._buffer) > POSE_STRUCT.size:
            self._buffer = None
            return None
        if packet[0] != FINAL_FRAGMENT:
            self._next_fragment += 1
            return None
        payload = bytes(self._buffer)
        self._buffer = None
        if len(payload) != POSE_STRUCT.size:
            return None
        return decode_pose_payload(payload)


class MissionStartNotifier:
    def __init__(self) -> None:
        self._lock = Lock()
        self._started = False

    def notify(self) -> None:
        with self._lock:
            if self._started:
                return
            self._started = True
        print("Time ", round(1000 * time.time(), 1))


class DroneLogger:
    def __init__(
        self, label: str, log_dir: Path, start_notifier: MissionStartNotifier
    ) -> None:
        self.label = label
        self.log_dir = log_dir
        self.start_notifier = start_notifier
        self.pose_assembler = PoseAssembler()

    def handle_packet(self, data: bytes | bytearray | list[int]) -> None:
        packet = bytes(data)
        if len(packet) < 2:
            return
        if packet[0] in (POSE_PACKET, POSE_PACKET | FINAL_FRAGMENT):
            pose = self.pose_assembler.feed(packet)
            if pose is None:
                return
            self.start_notifier.notify()
            with (self.log_dir / f"{self.label}.csv").open(
                "a", newline="", encoding="utf-8"
            ) as csv_file:
                csv.writer(csv_file).writerow(pose.csv_row())
        elif packet[0] == SCAN_PACKET | FINAL_FRAGMENT and packet[1] == 0:
            try:
                scan_id = decode_scan_packet(packet)
            except ValueError:
                return
            with (self.log_dir / f"{self.label}s.csv").open(
                "a", newline="", encoding="utf-8"
            ) as csv_file:
                csv.writer(csv_file).writerow([scan_id])


def create_log_dir(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    index = 0
    while True:
        log_dir = root / f"log{index}"
        try:
            log_dir.mkdir()
        except FileExistsError:
            index += 1
        else:
            return log_dir


def wait_for_param_download(
    scf: SyncCrazyflieLike,
    *,
    loggers: Mapping[str, DroneLogger],
    timeout_s: float = PARAM_DOWNLOAD_TIMEOUT_SECONDS,
) -> None:
    if timeout_s <= 0:
        raise ValueError("parameter download timeout must be positive")
    deadline = time.monotonic() + timeout_s
    while not scf.cf.param.is_updated:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                f"parameter download timed out for {scf.cf.link_uri} "
                f"after {timeout_s:g}s"
            )
        time.sleep(min(1.0, remaining))
    uri: str = scf.cf.link_uri
    logger = loggers[uri]
    print("Parameters downloaded for", uri)

    def on_console(text: str) -> None:
        print(f"{logger.label}: {text}", end="")

    def on_packet(packet: PacketLike) -> None:
        logger.handle_packet(packet.data)

    scf.cf.console.receivedChar.add_callback(on_console)
    scf.cf.add_port_callback(1, on_packet)


def set_initial_params(
    scf: SyncCrazyflieLike, *, configs: Mapping[str, DroneConfig]
) -> None:
    config = configs[scf.cf.link_uri]
    x, y, yaw = config.initial_pose
    print("Setting ", x, y, yaw)
    # A previous mission can leave the persistent SITL firmware instance in
    # its landing state. Clear that latch before arming this new mission.
    scf.cf.param.set_value("cmds.land", "0")
    scf.cf.param.set_value("initial.xinit", f"{x:f}")
    scf.cf.param.set_value("initial.yinit", f"{y:f}")
    scf.cf.param.set_value("initial.yawinit", f"{yaw:f}")
    scf.cf.param.set_value("cmds.ready", "1")


def set_operator_heartbeat(scf: SyncCrazyflieLike, *, sequence: int) -> None:
    scf.cf.param.set_value("cmds.heartbeat", str(sequence))


def run_operator_heartbeat_loop(
    swarm: SwarmLike,
    *,
    sleep: Callable[[float], None] = time.sleep,
    initial_sequence: int = 0,
) -> NoReturn:
    sequence = initial_sequence & UINT32_MASK
    while True:
        sequence = (sequence + 1) & UINT32_MASK
        swarm.parallel_safe(partial(set_operator_heartbeat, sequence=sequence))
        sleep(OPERATOR_HEARTBEAT_INTERVAL_SECONDS)


def run_mission(configs: Sequence[DroneConfig], log_root: Path = Path("logs")) -> None:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie

    log_dir = create_log_dir(log_root)
    notifier = MissionStartNotifier()
    loggers = {
        config.uri: DroneLogger(config.label, log_dir, notifier) for config in configs
    }
    by_uri = {config.uri: config for config in configs}

    cflib.crtp.init_drivers(enable_debug_driver=False)
    links = cast(
        dict[str, LinkCrazyflieLike],
        {uri: Crazyflie(rw_cache="./cache") for uri in by_uri},
    )
    with BoundedSwarm(links) as swarm:
        swarm.parallel_safe(partial(wait_for_param_download, loggers=loggers))
        swarm.parallel_safe(partial(set_initial_params, configs=by_uri))
        run_operator_heartbeat_loop(swarm)


def select_drone_configs(
    drone_count: int, uris: Sequence[str] = ()
) -> tuple[DroneConfig, ...]:
    if not 1 <= drone_count <= len(DRONES):
        raise ValueError(f"drone_count must be between 1 and {len(DRONES)}")
    if uris and len(uris) != drone_count:
        raise ValueError("provide exactly one --uri for each selected drone")
    if any(not uri.strip() for uri in uris):
        raise ValueError("drone URIs must not be empty")
    if len(set(uris)) != len(uris):
        raise ValueError("drone URIs must be unique")

    defaults = DRONES[:drone_count]
    if not uris:
        return defaults
    return tuple(
        DroneConfig(uri, default.label, default.initial_pose)
        for default, uri in zip(defaults, uris, strict=True)
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a Nano-C-SLAM drone mission",
        epilog=(
            "Gazebo example: uv run --locked python cslam-stm32-app/run_demo.py 2 "
            "--uri udp://127.0.0.1:21850 "
            "--uri udp://127.0.0.1:21851"
        ),
    )
    parser.add_argument("drone_count", type=int, help="number of drones (1 to 4)")
    parser.add_argument(
        "--uri",
        action="append",
        dest="uris",
        help="override a drone link URI (repeat once per selected drone)",
    )
    parser.add_argument(
        "--log-root",
        type=Path,
        default=Path("logs"),
        help="directory under which a unique mission log folder is created",
    )
    args = parser.parse_args(argv)
    try:
        configs = select_drone_configs(args.drone_count, args.uris or ())
    except ValueError as error:
        parser.error(str(error))
    try:
        run_mission(configs, log_root=args.log_root)
    except KeyboardInterrupt:
        return 130
    except (ConnectionError, TimeoutError) as error:
        print(f"Mission startup failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
