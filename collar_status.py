"""
Collar-facing calibration / connection status codes over TCP.

Clients connect to CAL_STATUS_PORT and receive a line of ASCII digits every
250 ms, e.g. ``0\\n``, ``1\\n``, ``3\\n``, ``2\\n``, ``6\\n``.
"""

from __future__ import annotations

import logging
import socket
import threading
import time

LOG = logging.getLogger("collar_status")

# Status codes exposed to the collar firmware.
STATUS_IDLE = 9          # no collar connected / waiting for connection
STATUS_POINT_UP = 0      # mount on barbell, top facing up, hold still
STATUS_FRAME_SPIN = 1    # rotate about bar long axis — mount / frame calibration
STATUS_DONE = 2          # calibration complete
STATUS_LEVER_SPIN = 3    # rotate about bar long axis — IMU lever-arm calibration
STATUS_ERROR = 6         # transient error — server returns to failed step

# Backward-compatible alias
STATUS_SPIN = STATUS_FRAME_SPIN

_STATUS_LABELS = {
    STATUS_IDLE: "idle (waiting for collar)",
    STATUS_POINT_UP: "point device up (top facing up), hold still",
    STATUS_FRAME_SPIN: "spin about bar long axis (frame calibration)",
    STATUS_LEVER_SPIN: "spin about bar long axis (lever-arm calibration)",
    STATUS_DONE: "calibration finished",
    STATUS_ERROR: "error — retry current step",
}

_current_code = STATUS_IDLE
_lock = threading.Lock()
_server_thread: threading.Thread | None = None


def get_collar_status() -> int:
    with _lock:
        return _current_code


def get_collar_status_label(code: int | None = None) -> str:
    if code is None:
        code = get_collar_status()
    return _STATUS_LABELS.get(code, f"unknown ({code})")


def set_collar_status(code: int) -> None:
    global _current_code
    with _lock:
        if _current_code == code:
            return
        _current_code = code
    label = get_collar_status_label(code)
    LOG.info("Collar status -> %d (%s)", code, label)
    print(f"[collar status] {code} — {label}", flush=True)


def _handle_status_client(conn: socket.socket, addr: tuple[str, int]) -> None:
    LOG.debug("Status client connected from %s", addr)
    try:
        while True:
            code = get_collar_status()
            conn.sendall(f"{code}\n".encode("ascii"))
            time.sleep(0.25)
    except OSError:
        pass
    finally:
        try:
            conn.close()
        except OSError:
            pass
        LOG.debug("Status client disconnected from %s", addr)


def start_collar_status_server(host: str, port: int) -> None:
    global _server_thread

    def _loop() -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.listen(16)
        LOG.info("Collar status server on %s:%d", host, port)
        while True:
            try:
                conn, addr = sock.accept()
            except OSError as exc:
                LOG.warning("Status accept error: %s", exc)
                continue
            thread = threading.Thread(
                target=_handle_status_client,
                args=(conn, addr),
                name=f"collar-status-{addr[0]}",
                daemon=True,
            )
            thread.start()

    if _server_thread is not None and _server_thread.is_alive():
        return
    _server_thread = threading.Thread(target=_loop, name="collar-status-server", daemon=True)
    _server_thread.start()
