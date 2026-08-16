"""
Wire-format parsing and dispatch for collar TCP lines.

Pipeline step: decoded UTF-8 line → unpack → fusion stream buffer.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Protocol

from device_protocol import unpack_collar_wire_line
from sensor_stream import SENSOR_FLOW, SENSOR_QUAT, SENSOR_RADAR

from server_config import IMU_ONLY_MODE

LOG = logging.getLogger("fusion_server.wire")


class WireSession(Protocol):
    """Session surface required for wire line handling."""

    addr: tuple[str, int]
    packets_received: int
    _logged_first_batch: bool
    live_display: bool

    def _dispatch_stream_samples(
        self, samples: list[tuple[int, int, dict[str, Any]]],
    ) -> None: ...
    def handle_sensor(self, msg: dict[str, Any]) -> None: ...


def process_collar_line(session: WireSession, line: str) -> None:
    """Parse one decoded TCP line and route to fusion or legacy JSON handlers."""
    samples = unpack_collar_wire_line(line)
    if samples:
        _log_batch(session, line, samples)
        session._dispatch_stream_samples(
            [(s.sensor, s.ts_us, s.data) for s in samples],
        )
        return

    if line.strip().startswith("["):
        LOG.warning(
            "Failed to unpack collar wire line from %s (%d bytes): %s",
            session.addr,
            len(line),
            line[:160],
        )
        return

    try:
        msg = json.loads(line)
    except json.JSONDecodeError:
        LOG.warning("Invalid JSON from %s: %s", session.addr, line[:120])
        return

    msg_type = msg.get("type")
    if msg_type == "sensor":
        session.handle_sensor(msg)
    else:
        LOG.debug("Ignoring legacy control message %r from collar", msg_type)


def _log_batch(
    session: WireSession,
    line: str,
    samples: list[Any],
) -> None:
    quat_count = sum(1 for s in samples if s.sensor == SENSOR_QUAT)
    if not session._logged_first_batch:
        session._logged_first_batch = True
        LOG.info(
            "First collar batch from %s — %d samples (%d quat, "
            "%d bytes, ts %d..%d us)",
            session.addr,
            len(samples),
            quat_count,
            len(line),
            samples[0].ts_us,
            samples[-1].ts_us,
        )
    elif len(samples) > 1:
        LOG.debug(
            "Collar batch from %s — %d samples (%d quat, ts %d..%d us)",
            session.addr,
            len(samples),
            quat_count,
            samples[0].ts_us,
            samples[-1].ts_us,
        )
    elif quat_count == 0 and line.strip().startswith("[["):
        LOG.warning(
            "Collar batch from %s unpacked to %d samples but 0 quaternions "
            "(expected wire type 0 with [x,y,z,w])",
            session.addr,
            len(samples),
        )


def filter_imu_only_samples(
    samples: list[tuple[int, int, dict[str, Any]]],
) -> list[tuple[int, int, dict[str, Any]]]:
    if not IMU_ONLY_MODE:
        return samples
    return [
        item for item in samples
        if item[0] not in (SENSOR_FLOW, SENSOR_RADAR)
    ]
