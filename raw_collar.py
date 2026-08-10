"""
Minimal collar TCP handler — accept, read, log raw payloads only.

No unpack, fusion, auto-cal, or webhooks. Sends STREAM_START so the collar
will actually transmit.
"""

from __future__ import annotations

import logging
import socket
import sys
import time

from fusion_settings import get_int_setting

LOG = logging.getLogger(__name__)

MAX_LINE_BYTES = get_int_setting("MAX_LINE_BYTES", 2_097_152)
IDLE_WARN_S = 5.0
RECV_TIMEOUT_S = 1.0


def debug_print(msg: str) -> None:
    print(f"[collar debug] {msg}", file=sys.stderr, flush=True)


class RawCollarSession:
    """One collar TCP connection: log raw recv data before any processing."""

    def __init__(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        self.conn = conn
        self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.addr = addr
        self.connected_at = time.monotonic()
        self._raw_packets_received = 0
        self._raw_bytes_received = 0
        self._recv_calls = 0
        self._last_idle_warn = 0.0
        self._debug_interval = get_int_setting("PACKET_DEBUG_INTERVAL", 50)

    def _send_stream_start(self) -> None:
        try:
            self.conn.sendall(b"STREAM_START\n")
            debug_print(f"{self.addr} sent STREAM_START")
            LOG.info("Sent STREAM_START to %s (raw log mode)", self.addr)
        except OSError as exc:
            debug_print(f"{self.addr} STREAM_START failed: {exc}")
            LOG.warning("Could not send STREAM_START to %s: %s", self.addr, exc)

    def _log_raw_packet(self, line_bytes: bytes) -> None:
        if self._debug_interval <= 0:
            return
        self._raw_packets_received += 1
        count = self._raw_packets_received
        preview = line_bytes[:160].decode("utf-8", errors="replace")
        if count == 1:
            debug_print(
                f"{self.addr} raw line #1 ({len(line_bytes)} bytes): {preview}"
            )
        elif count % self._debug_interval == 0:
            debug_print(
                f"{self.addr} raw line #{count} "
                f"({len(line_bytes)} bytes, {self._raw_bytes_received} B total): "
                f"{preview}"
            )

    def _log_recv_chunk(self, chunk: bytes) -> None:
        self._recv_calls += 1
        self._raw_bytes_received += len(chunk)
        if self._debug_interval <= 0:
            return
        preview = chunk[:120].decode("utf-8", errors="replace")
        if self._recv_calls == 1:
            debug_print(
                f"{self.addr} first TCP recv {len(chunk)} bytes "
                f"(no newline required): {preview!r}"
            )
        elif self._recv_calls % self._debug_interval == 0:
            debug_print(
                f"{self.addr} recv #{self._recv_calls} "
                f"{len(chunk)} bytes ({self._raw_bytes_received} B total): "
                f"{preview!r}"
            )

    def _maybe_warn_idle(self) -> None:
        if self._raw_bytes_received > 0:
            return
        elapsed = time.monotonic() - self.connected_at
        if elapsed < IDLE_WARN_S:
            return
        if time.monotonic() - self._last_idle_warn < IDLE_WARN_S:
            return
        self._last_idle_warn = time.monotonic()
        debug_print(
            f"{self.addr} connected {elapsed:.0f}s — still no TCP data "
            f"(collar may be waiting for STREAM_START or wrong port)"
        )

    def _ingest_raw_line(self, line_bytes: bytes) -> None:
        if not line_bytes.strip():
            return
        if len(line_bytes) > MAX_LINE_BYTES:
            LOG.warning("Line too large from %s (%d bytes)", self.addr, len(line_bytes))
            debug_print(
                f"{self.addr} dropped oversized line ({len(line_bytes)} bytes)"
            )
            return
        self._log_raw_packet(line_bytes)

    def run(self) -> None:
        LOG.info("Collar connected (raw log only) from %s", self.addr)
        debug_print(
            f"{self.addr} connected (raw log only), "
            f"debug every {self._debug_interval} lines/recvs"
        )

        self._send_stream_start()
        self.conn.settimeout(RECV_TIMEOUT_S)

        buffer = b""
        try:
            while True:
                try:
                    chunk = self.conn.recv(65536)
                except socket.timeout:
                    self._maybe_warn_idle()
                    continue
                if not chunk:
                    debug_print(f"{self.addr} peer closed connection (EOF)")
                    break
                self._log_recv_chunk(chunk)
                buffer += chunk
                while b"\n" in buffer:
                    line_bytes, buffer = buffer.split(b"\n", 1)
                    self._ingest_raw_line(line_bytes)
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            debug_print(f"{self.addr} disconnected: {exc}")
            LOG.info("Collar disconnected from %s — %s", self.addr, exc)
        finally:
            if buffer.strip():
                debug_print(
                    f"{self.addr} trailing buffer without newline "
                    f"({len(buffer)} bytes) — logging as one packet"
                )
                self._ingest_raw_line(buffer)
            duration_s = time.monotonic() - self.connected_at
            summary = (
                f"{self.addr} session ended after {duration_s:.1f}s — "
                f"{self._recv_calls} recvs, {self._raw_bytes_received} bytes, "
                f"{self._raw_packets_received} newline lines"
            )
            debug_print(summary)
            LOG.info(summary)
            try:
                self.conn.close()
            except OSError:
                pass
