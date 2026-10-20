"""Forward four CrazySim 8x8 GPU lidar scans to the Nano firmware SITL."""

from __future__ import annotations

import argparse
import asyncio
import binascii
import json
import logging
import math
import socket
import struct
from dataclasses import dataclass
from typing import Any

SENSOR_COUNT = 4
SIDE = 8
MAX_RANGE_MM = 4000
FRAME_HEADER = struct.Struct("!4sBBBBIQI")
FRAME_CRC = struct.Struct("!I")
MAX_SCAN_SKEW_NS = 100_000_000
LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class ScanSample:
    sim_time_ns: int
    millimeters: tuple[int, ...]


def _mapping(value: object) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def parse_scan_message(payload: str | bytes) -> ScanSample:
    """Validate a gz.msgs.LaserScan JSON message and convert its 8x8 ranges."""
    try:
        decoded = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise ValueError("invalid LaserScan JSON") from error
    if not isinstance(decoded, dict):
        raise ValueError("LaserScan message must be a JSON object")
    scan = decoded
    if scan.get("count") != SIDE or scan.get("verticalCount") != SIDE:
        raise ValueError("LaserScan must contain 8 horizontal by 8 vertical rays")
    ranges = scan.get("ranges")
    if not isinstance(ranges, list) or len(ranges) != SIDE * SIDE:
        raise ValueError("LaserScan must contain exactly 64 ranges")
    header = _mapping(scan.get("header"))
    stamp = _mapping(header.get("stamp"))
    try:
        seconds = int(stamp["sec"])
        nanoseconds = int(stamp["nsec"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "LaserScan is missing an integer simulation timestamp"
        ) from error
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("LaserScan simulation timestamp is out of range")
    sim_time_ns = seconds * 1_000_000_000 + nanoseconds
    if sim_time_ns == 0:
        raise ValueError("LaserScan simulation timestamp must be nonzero")
    try:
        range_min = float(scan["rangeMin"])
        range_max = float(scan["rangeMax"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("LaserScan has invalid range bounds") from error
    if not math.isfinite(range_min) or not math.isfinite(range_max):
        raise ValueError("LaserScan range bounds must be finite")
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError("LaserScan range bounds are inconsistent")

    millimeters: list[int] = []
    for distance in ranges:
        try:
            meters = float(distance)
        except (TypeError, ValueError):
            millimeters.append(-1)
            continue
        if not math.isfinite(meters) or meters < range_min or meters > range_max:
            millimeters.append(-1)
            continue
        measured_mm = round(meters * 1000)
        millimeters.append(measured_mm if measured_mm <= MAX_RANGE_MM else -1)
    return ScanSample(sim_time_ns, tuple(millimeters))


def encode_matrix_frame(
    sequence: int, sim_time_ns: int, matrices: tuple[tuple[int, ...], ...]
) -> bytes:
    """Encode one frame in the firmware's NCTF v1 network protocol."""
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit uint32")
    if not 0 < sim_time_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("simulation timestamp must fit nonzero uint64")
    if len(matrices) != SENSOR_COUNT or any(
        len(row) != SIDE * SIDE for row in matrices
    ):
        raise ValueError("exactly four 8x8 sensor matrices are required")
    cells = tuple(value for matrix in matrices for value in matrix)
    if any(value < -1 or value > MAX_RANGE_MM for value in cells):
        raise ValueError("matrix values must be -1 or millimeters in 0..4000")
    header = FRAME_HEADER.pack(
        b"NCTF", 1, SENSOR_COUNT, SIDE, SIDE, sequence, sim_time_ns, 512
    )
    payload = struct.pack("!256h", *cells)
    body = header + payload
    return body + FRAME_CRC.pack(binascii.crc32(body) & 0xFFFFFFFF)


class TofUdpBridge:
    def __init__(
        self,
        *,
        gz_command: str,
        topic_prefix: str,
        udp_host: str,
        udp_port: int,
        max_skew_ns: int = MAX_SCAN_SKEW_NS,
    ) -> None:
        if not topic_prefix.startswith("/") or topic_prefix.endswith("/"):
            raise ValueError("topic prefix must be absolute and have no trailing slash")
        if not 1 <= udp_port <= 65535:
            raise ValueError("UDP port must be in 1..65535")
        if max_skew_ns < 0:
            raise ValueError("maximum scan skew cannot be negative")
        self.gz_command = gz_command
        self.topic_prefix = topic_prefix
        self.destination = (udp_host, udp_port)
        self.max_skew_ns = max_skew_ns
        self.samples: list[ScanSample | None] = [None] * SENSOR_COUNT
        self.generations = [0] * SENSOR_COUNT
        self.last_sent_generations = [0] * SENSOR_COUNT
        self.last_sent_sim_time_ns = 0
        self.sequence = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def accept_scan(self, sensor_id: int, payload: str | bytes) -> bytes | None:
        if not 0 <= sensor_id < SENSOR_COUNT:
            raise ValueError("sensor ID must be in 0..3")
        sample = parse_scan_message(payload)
        previous = self.samples[sensor_id]
        if previous is not None and sample.sim_time_ns <= previous.sim_time_ns:
            return None
        self.samples[sensor_id] = sample
        self.generations[sensor_id] += 1
        if any(item is None for item in self.samples):
            return None
        if any(
            self.generations[index] <= self.last_sent_generations[index]
            for index in range(SENSOR_COUNT)
        ):
            return None

        complete = tuple(item for item in self.samples if item is not None)
        timestamps = tuple(item.sim_time_ns for item in complete)
        newest_time = max(timestamps)
        if newest_time - min(timestamps) > self.max_skew_ns:
            return None
        if newest_time <= self.last_sent_sim_time_ns:
            return None
        frame = encode_matrix_frame(
            self.sequence,
            newest_time,
            tuple(item.millimeters for item in complete),
        )
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        self.last_sent_sim_time_ns = newest_time
        self.last_sent_generations = self.generations.copy()
        return frame

    async def _consume_topic(self, sensor_id: int) -> None:
        topic = f"{self.topic_prefix}/{sensor_id}"
        process = await asyncio.create_subprocess_exec(
            self.gz_command,
            "topic",
            "-e",
            "--json-output",
            "-t",
            topic,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        if process.stdout is None:
            raise RuntimeError(f"gz topic has no output stream for {topic}")
        try:
            async for line in process.stdout:
                try:
                    frame = self.accept_scan(sensor_id, line)
                except ValueError as error:
                    LOGGER.warning("discarding malformed scan on %s: %s", topic, error)
                    continue
                if frame is not None:
                    self.socket.sendto(frame, self.destination)
                    LOGGER.info(
                        "sent ToF frame seq=%d sim_time_ns=%d destination=%s:%d",
                        (self.sequence - 1) & 0xFFFFFFFF,
                        self.last_sent_sim_time_ns,
                        *self.destination,
                    )
            return_code = await process.wait()
            raise RuntimeError(f"gz topic exited for {topic} ({return_code})")
        finally:
            if process.returncode is None:
                process.terminate()
                try:
                    await asyncio.wait_for(process.wait(), timeout=2.0)
                except TimeoutError:
                    process.kill()
                    await process.wait()

    async def run(self) -> None:
        await asyncio.gather(
            *(self._consume_topic(sensor_id) for sensor_id in range(SENSOR_COUNT))
        )


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gz-command", default="gz")
    parser.add_argument("--topic-prefix", default="/cf_0/tof")
    parser.add_argument("--udp-host", default="192.168.5.2")
    parser.add_argument("--udp-port", type=int, default=19951)
    parser.add_argument("--max-skew-ms", type=float, default=100.0)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> None:
    arguments = _arguments()
    logging.basicConfig(
        level=logging.INFO if arguments.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    bridge = TofUdpBridge(
        gz_command=arguments.gz_command,
        topic_prefix=arguments.topic_prefix,
        udp_host=arguments.udp_host,
        udp_port=arguments.udp_port,
        max_skew_ns=round(arguments.max_skew_ms * 1_000_000),
    )
    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        LOGGER.info("ToF bridge stopped")
    finally:
        bridge.socket.close()


if __name__ == "__main__":
    main()
