#!/usr/bin/env python3
"""
Oracle Ubuntu TCP fusion server.

Protocol: newline-delimited JSON (one object per line).

  {"type":"start"}     — reset fusion, begin a streaming session
  {"type":"sensor", ...} — IMU / flow / range sample (see client_example.py)
  {"type":"end"}       — end session; final webhook marks streaming=false

Each fused pose is POSTed to the Vercel webhook (Bearer WEBHOOK_SECRET).
"""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
import uuid
from typing import Any
from urllib import error, request

from fusion_lib import FusionEngine

LOG = logging.getLogger("fusion_server")

SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "9000"))
VERCEL_WEBHOOK_URL = os.environ.get("VERCEL_WEBHOOK_URL", "")
WEBHOOK_SECRET = os.environ.get("WEBHOOK_SECRET", "")
STREAM_IDLE_TIMEOUT_S = float(os.environ.get("STREAM_IDLE_TIMEOUT_S", "3.0"))
MAX_LINE_BYTES = int(os.environ.get("MAX_LINE_BYTES", "65536"))


def now_us() -> int:
    return int(time.time() * 1_000_000)


def post_pose_webhook(payload: dict[str, Any]) -> None:
    if not VERCEL_WEBHOOK_URL:
        LOG.warning("VERCEL_WEBHOOK_URL not set — skipping webhook POST")
        return

    body = json.dumps(payload).encode("utf-8")
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {WEBHOOK_SECRET}",
    }
    req = request.Request(
        VERCEL_WEBHOOK_URL,
        data=body,
        headers=headers,
        method="POST",
    )
    try:
        with request.urlopen(req, timeout=10) as resp:
            LOG.debug("Webhook OK %s", resp.status)
    except error.HTTPError as exc:
        LOG.error("Webhook HTTP error %s: %s", exc.code, exc.read())
    except error.URLError as exc:
        LOG.error("Webhook URL error: %s", exc.reason)


_engine: FusionEngine | None = None
_engine_lock = threading.Lock()


def get_fusion_engine() -> FusionEngine:
    global _engine
    with _engine_lock:
        if _engine is None:
            _engine = FusionEngine()
        return _engine


class ClientSession:
    """Handles one TCP client connection and its fusion stream."""

    def __init__(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        self.conn = conn
        self.addr = addr
        self.session_id = str(uuid.uuid4())
        self.streaming = False
        self.last_pose_step = 0
        self.last_activity = time.monotonic()
        self.engine = get_fusion_engine()

    def _with_engine(self, fn) -> Any:
        with _engine_lock:
            return fn()

    def close(self) -> None:
        try:
            self.conn.close()
        except OSError:
            pass

    def send_ack(self, msg_type: str, extra: dict[str, Any] | None = None) -> None:
        payload: dict[str, Any] = {"type": "ack", "of": msg_type, "session_id": self.session_id}
        if extra:
            payload.update(extra)
        line = (json.dumps(payload) + "\n").encode("utf-8")
        self.conn.sendall(line)

    def push_pose(self, streaming: bool) -> None:
        pose = self._with_engine(self.engine.get_pose)
        if pose is None:
            return

        payload = {
            "session_id": self.session_id,
            "streaming": streaming,
            "updated_at_ms": int(time.time() * 1000),
            "pose": pose,
        }
        post_pose_webhook(payload)

    def handle_start(self) -> None:
        self.session_id = str(uuid.uuid4())
        self.streaming = True
        self.last_pose_step = 0
        self.last_activity = time.monotonic()
        self._with_engine(self.engine.reset)
        LOG.info("Session %s started (%s)", self.session_id, self.addr)
        self.send_ack("start")

    def handle_end(self) -> None:
        if self.streaming:
            self.streaming = False
            self.push_pose(streaming=False)
            LOG.info("Session %s ended (%s)", self.session_id, self.addr)
        self.send_ack("end")

    def handle_sensor(self, msg: dict[str, Any]) -> None:
        if not self.streaming:
            self.send_ack("sensor", {"error": "call start before sensor data"})
            return

        ts_us = int(msg.get("ts_us", now_us()))

        def ingest() -> None:
            quat = msg.get("quat")
            if quat:
                self.engine.submit_quat(
                    float(quat["w"]), float(quat["x"]),
                    float(quat["y"]), float(quat["z"]),
                    ts_us,
                )

            gyro = msg.get("gyro")
            if gyro:
                self.engine.submit_gyro(
                    float(gyro["x"]), float(gyro["y"]), float(gyro["z"]),
                    ts_us,
                )

            accel = msg.get("accel")
            if accel:
                self.engine.submit_accel(
                    float(accel["x"]), float(accel["y"]), float(accel["z"]),
                    ts_us,
                )

            flow = msg.get("flow")
            if flow:
                self.engine.submit_flow(
                    int(flow["dx"]), int(flow["dy"]),
                    int(flow.get("quality", 255)),
                    ts_us,
                )

            range_data = msg.get("range")
            if range_data:
                self.engine.submit_range(
                    int(range_data["mm"]),
                    int(range_data.get("strength", 100)),
                    ts_us,
                )

        self._with_engine(ingest)
        self.last_activity = time.monotonic()

        pose = self._with_engine(self.engine.get_pose)
        if pose and pose["step_count"] > self.last_pose_step:
            self.last_pose_step = pose["step_count"]
            self.push_pose(streaming=True)

    def handle_line(self, line: str) -> None:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            LOG.warning("Invalid JSON from %s: %s", self.addr, line[:120])
            self.send_ack("error", {"error": "invalid json"})
            return

        msg_type = msg.get("type")
        if msg_type == "start":
            self.handle_start()
        elif msg_type == "sensor":
            self.handle_sensor(msg)
        elif msg_type == "end":
            self.handle_end()
        else:
            self.send_ack("error", {"error": f"unknown type: {msg_type}"})

    def idle_watchdog(self) -> None:
        while self.streaming:
            if time.monotonic() - self.last_activity > STREAM_IDLE_TIMEOUT_S:
                LOG.info("Session %s idle timeout", self.session_id)
                self.handle_end()
                break
            time.sleep(0.25)

    def run(self) -> None:
        watchdog = threading.Thread(target=self.idle_watchdog, daemon=True)
        watchdog.start()

        buffer = b""
        try:
            while True:
                chunk = self.conn.recv(4096)
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line_bytes, buffer = buffer.split(b"\n", 1)
                    if not line_bytes.strip():
                        continue
                    if len(line_bytes) > MAX_LINE_BYTES:
                        LOG.warning("Line too large from %s", self.addr)
                        continue
                    self.handle_line(line_bytes.decode("utf-8"))
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            LOG.info("Client %s disconnected: %s", self.addr, exc)
        finally:
            if self.streaming:
                self.handle_end()
            self.close()


def serve() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((SERVER_HOST, SERVER_PORT))
    sock.listen(8)
    LOG.info("Fusion server listening on %s:%d", SERVER_HOST, SERVER_PORT)

    while True:
        conn, addr = sock.accept()
        LOG.info("Connection from %s", addr)
        session = ClientSession(conn, addr)
        thread = threading.Thread(target=session.run, daemon=True)
        thread.start()


if __name__ == "__main__":
    serve()
