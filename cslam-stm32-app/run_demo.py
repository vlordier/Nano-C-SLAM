"""Run and log a Nano-C-SLAM Crazyflie mission."""

import argparse
import csv
import struct
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from threading import Lock
from typing import NoReturn, Protocol

POSE_STRUCT = struct.Struct("<hxxlfff32h")
SCAN_STRUCT = struct.Struct("<h")
POSE_PACKET = 0
SCAN_PACKET = 1
FINAL_FRAGMENT = 0x80
OPERATOR_HEARTBEAT_INTERVAL_SECONDS = 0.25
UINT32_MASK = 0xFFFFFFFF


class PacketLike(Protocol):
    @property
    def data(self) -> bytearray: ...


class ParamLike(Protocol):
    is_updated: bool

    def set_value(self, name: str, value: str) -> None: ...


class CallbackListLike(Protocol):
    def add_callback(self, callback: Callable[[str], None]) -> None: ...


class ConsoleLike(Protocol):
    receivedChar: CallbackListLike


class CrazyflieLike(Protocol):
    link_uri: str
    param: ParamLike
    console: ConsoleLike

    def add_port_callback(
        self, port: int, callback: Callable[[PacketLike], None]
    ) -> None: ...


class SyncCrazyflieLike(Protocol):
    cf: CrazyflieLike


class SwarmLike(Protocol):
    def parallel_safe(self, function: Callable[[SyncCrazyflieLike], None]) -> None: ...


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
    scf: SyncCrazyflieLike, *, loggers: Mapping[str, DroneLogger]
) -> None:
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
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
    from cflib.crazyflie.swarm import CachedCfFactory, Swarm

    log_dir = create_log_dir(log_root)
    notifier = MissionStartNotifier()
    loggers = {
        config.uri: DroneLogger(config.label, log_dir, notifier) for config in configs
    }
    by_uri = {config.uri: config for config in configs}

    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache="./cache")
    with Swarm(set(by_uri), factory=factory) as swarm:
        swarm.parallel_safe(partial(wait_for_param_download, loggers=loggers))
        swarm.parallel_safe(partial(set_initial_params, configs=by_uri))
        run_operator_heartbeat_loop(swarm)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a Nano-C-SLAM drone mission")
    parser.add_argument("drone_count", type=int, help="number of drones (1 to 4)")
    args = parser.parse_args(argv)
    if not 1 <= args.drone_count <= len(DRONES):
        parser.error(f"drone_count must be between 1 and {len(DRONES)}")
    try:
        run_mission(DRONES[: args.drone_count])
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
