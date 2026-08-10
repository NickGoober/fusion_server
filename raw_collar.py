"""
Minimal collar TCP handler — accept, read, log raw packets only.

No unpack, fusion, auto-cal, webhooks, or STREAM_START.
"""

from __future__ import annotations

import logging
import socket
import sys
import time

from fusion_settings import get_int_setting

LOG = logging.getLogger(__name__)

PACKET_DEBUG_INTERVAL = get_int_setting("PACKET_DEBUG_INTERVAL", 50)
MAX_LINE_BYTES = get_int_setting("MAX_LINE_BYTES", 2_097_152)


def debug_print(msg: str) -> None:
    print(f"[collar debug] {msg}", file=sys.stderr, flush=True)


class RawCollarSession:
    """One collar TCP connection: log newline-delimited raw payloads."""

    def __init__(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        self.conn = conn
        self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.addr = addr
        self.connected_at = time.monotonic()
        self._raw_packets_received = 0

    def _log_raw_packet(self, line_bytes: bytes) -> None:
        if PACKET_DEBUG_INTERVAL <= 0:
            return
        self._raw_packets_received += 1
        count = self._raw_packets_received
        preview = line_bytes[:160].decode("utf-8", errors="replace")
        if count == 1:
            debug_print(
                f"{self.addr} raw packet #1 ({len(line_bytes)} bytes): {preview}"
            )
        elif count % PACKET_DEBUG_INTERVAL == 0:
            debug_print(
                f"{self.addr} raw packet #{count} "
                f"({len(line_bytes)} bytes): {preview}"
            )

    def _ingest_raw_line(self, line_bytes: bytes) -> None:
        if not line_bytes.strip():
            return
        if len(line_bytes) > MAX_LINE_BYTES:
            LOG.warning("Line too large from %s (%d bytes)", self.addr, len(line_bytes))
            return
        self._log_raw_packet(line_bytes)

    def run(self) -> None:
        LOG.info("Collar connected (raw log only) from %s", self.addr)
        if PACKET_DEBUG_INTERVAL > 0:
            debug_print(
                f"{self.addr} connected, raw packet debug every "
                f"{PACKET_DEBUG_INTERVAL}"
            )

        buffer = b""
        try:
            while True:
                chunk = self.conn.recv(65536)
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line_bytes, buffer = buffer.split(b"\n", 1)
                    self._ingest_raw_line(line_bytes)
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            LOG.info("Collar disconnected from %s — %s", self.addr, exc)
        finally:
            if buffer.strip():
                self._ingest_raw_line(buffer)
            duration_s = time.monotonic() - self.connected_at
            LOG.info(
                "Collar session ended %s after %.1fs — %d raw packets logged",
                self.addr,
                duration_s,
                self._raw_packets_received,
            )
            try:
                self.conn.close()
            except OSError:
                pass
