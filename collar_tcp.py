"""
Newline-delimited TCP read loop for collar streams.

Yields complete line payloads (without the trailing newline). Handles recv
timeouts and idle disconnect so zombie sessions do not block reconnects.
"""

from __future__ import annotations

import socket
import time
from collections.abc import Iterator
from dataclasses import dataclass


@dataclass
class TcpReadState:
    """Tracks TCP activity for idle detection."""

    last_recv_at: float | None = None


class TcpIdleTimeout(Exception):
    """Raised when no TCP bytes arrive within the idle limit."""


def read_collar_tcp_lines(
    conn: socket.socket,
    *,
    recv_timeout_s: float,
    idle_disconnect_s: float,
    state: TcpReadState | None = None,
) -> Iterator[bytes]:
    """
    Read from ``conn`` until disconnect or idle timeout.

    Yields each complete newline-delimited payload. Updates ``state.last_recv_at``
    on every received chunk.
    """
    if state is None:
        state = TcpReadState()

    conn.settimeout(recv_timeout_s)
    buffer = b""

    while True:
        try:
            chunk = conn.recv(65536)
        except socket.timeout:
            if (
                state.last_recv_at is not None
                and time.monotonic() - state.last_recv_at > idle_disconnect_s
            ):
                raise TcpIdleTimeout(
                    f"TCP idle timeout ({idle_disconnect_s:.0f}s without data)",
                ) from None
            continue

        if not chunk:
            break

        state.last_recv_at = time.monotonic()
        buffer += chunk
        while b"\n" in buffer:
            line_bytes, buffer = buffer.split(b"\n", 1)
            yield line_bytes

    if buffer.strip():
        yield buffer
