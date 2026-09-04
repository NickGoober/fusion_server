"""Low-latency fused-pose fanout for apps.

TCP, newline-delimited JSON (one pose object per line). No HTTP buffering.
Clients connect to POSE_STREAM_PORT and read; optional first-line auth.
"""

from __future__ import annotations

import json
import logging
import socket
import threading
import time
from typing import Any

from pose_stream_format import compact_app_frame, hello_payload
from server_config import (
    POSE_STREAM_ENABLE,
    POSE_STREAM_HOST,
    POSE_STREAM_PORT,
    POSE_STREAM_SECRET,
)

LOG = logging.getLogger("fusion_server.pose_stream")

_lock = threading.Lock()
_clients: list[socket.socket] = []
_server_sock: socket.socket | None = None
_started = False
_last_payload: dict[str, Any] | None = None


def client_count() -> int:
    with _lock:
        return len(_clients)


def listen_addr() -> str:
    return f"{POSE_STREAM_HOST}:{POSE_STREAM_PORT}"


def start_pose_stream_thread() -> None:
    global _started
    if not POSE_STREAM_ENABLE:
        LOG.info("Pose stream disabled (POSE_STREAM_ENABLE=false)")
        return
    if _started:
        return
    _started = True
    thread = threading.Thread(target=_accept_loop, name="pose-stream", daemon=True)
    thread.start()


def broadcast_pose(payload: dict[str, Any]) -> None:
    """Send one compact pose object to every connected app. Drops slow clients."""
    global _last_payload
    if not POSE_STREAM_ENABLE:
        return
    frame = compact_app_frame(payload)
    body = (json.dumps(frame, separators=(",", ":")) + "\n").encode("utf-8")
    with _lock:
        _last_payload = frame
        clients = list(_clients)
    dead: list[socket.socket] = []
    for sock in clients:
        try:
            sock.sendall(body)
        except OSError:
            dead.append(sock)
    if dead:
        with _lock:
            for sock in dead:
                if sock in _clients:
                    _clients.remove(sock)
                try:
                    sock.close()
                except OSError:
                    pass


def _accept_loop() -> None:
    global _server_sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((POSE_STREAM_HOST, POSE_STREAM_PORT))
    sock.listen(16)
    sock.settimeout(1.0)
    _server_sock = sock
    LOG.info("Pose stream listening on %s:%d (NDJSON)", POSE_STREAM_HOST, POSE_STREAM_PORT)
    while True:
        try:
            conn, addr = sock.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        thread = threading.Thread(
            target=_serve_client,
            args=(conn, addr),
            name=f"pose-client-{addr[0]}:{addr[1]}",
            daemon=True,
        )
        thread.start()


def _serve_client(conn: socket.socket, addr: tuple[str, int]) -> None:
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(8.0)
        if POSE_STREAM_SECRET:
            if not _auth_client(conn):
                conn.sendall(b'{"type":"error","error":"auth_required"}\n')
                conn.close()
                LOG.info("Pose stream rejected %s:%d (auth)", addr[0], addr[1])
                return
        hello = hello_payload(int(time.time() * 1000))
        conn.sendall((json.dumps(hello, separators=(",", ":")) + "\n").encode("utf-8"))
        with _lock:
            snapshot = _last_payload
            _clients.append(conn)
        if snapshot is not None:
            try:
                conn.sendall(
                    (json.dumps(snapshot, separators=(",", ":")) + "\n").encode("utf-8")
                )
            except OSError:
                with _lock:
                    if conn in _clients:
                        _clients.remove(conn)
                conn.close()
                return
        LOG.info("Pose stream client %s:%d connected (%d total)", addr[0], addr[1], client_count())
        # Stay open until the app disconnects; pose frames are pushed from broadcast_pose.
        while True:
            try:
                data = conn.recv(256)
            except socket.timeout:
                continue
            if not data:
                break
    except OSError:
        pass
    finally:
        with _lock:
            if conn in _clients:
                _clients.remove(conn)
        try:
            conn.close()
        except OSError:
            pass
        LOG.info("Pose stream client %s:%d disconnected (%d total)", addr[0], addr[1], client_count())


def _auth_client(conn: socket.socket) -> bool:
    buf = b""
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and b"\n" not in buf:
        try:
            chunk = conn.recv(1024)
        except socket.timeout:
            continue
        if not chunk:
            return False
        buf += chunk
        if len(buf) > 4096:
            return False
    if b"\n" not in buf:
        return False
    line, _ = buf.split(b"\n", 1)
    try:
        msg = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return False
    token = ""
    if isinstance(msg, dict):
        token = str(msg.get("token") or msg.get("secret") or "")
        if msg.get("type") == "auth":
            token = str(msg.get("token") or token)
    return token == POSE_STREAM_SECRET
