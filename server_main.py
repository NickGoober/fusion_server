"""
Fusion server bootstrap: listen on the collar TCP port and start admin services.

Workflow:
  TCP accept → ClientSession (see client_session.py)
    → collar_tcp.read_collar_tcp_lines
    → line queue → collar_wire_handler.process_collar_line
    → fusion stream buffer → webhooks
"""

from __future__ import annotations

import logging
import socket
import sys
import threading

from admin_socket import start_admin_socket_thread
from client_session import ClientSession
from collar_status import STATUS_IDLE, set_collar_status, start_collar_status_server
from fusion_admin_dispatch import start_admin_console_thread
from fusion_settings import active_settings_path, get_bool_setting, get_int_setting
from raw_collar import RawCollarSession
from server_config import (
    CAL_STATUS_HOST,
    CAL_STATUS_PORT,
    COLLAR_RAW_LOG_ONLY,
    IMU_ONLY_MODE,
    PACKET_DEBUG_INTERVAL,
    SERVER_HOST,
    SERVER_PORT,
    STREAM_FIXED_LATENCY_US,
)

LOG = logging.getLogger("fusion_server")


def serve(*, force_raw_log_only: bool = False) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    settings_path = active_settings_path()
    if settings_path.is_file():
        LOG.info("Loaded settings from %s", settings_path)
    else:
        LOG.warning(
            "No settings file at %s — copy fusion_server.json.example and edit, "
            "or set FUSION_SERVER_CONFIG",
            settings_path,
        )

    raw_log_only = force_raw_log_only or get_bool_setting(
        "COLLAR_RAW_LOG_ONLY", False,
    )
    debug_interval = get_int_setting("PACKET_DEBUG_INTERVAL", 0)
    if raw_log_only or debug_interval > 0:
        banner = (
            f"settings={settings_path} "
            f"COLLAR_RAW_LOG_ONLY={raw_log_only} "
            f"PACKET_DEBUG_INTERVAL={debug_interval} "
            f"port={SERVER_PORT}"
        )
        print(f"[collar debug] {banner}", file=sys.stderr, flush=True)
        if force_raw_log_only:
            print(
                "[collar debug] --raw-log-only CLI flag active",
                file=sys.stderr,
                flush=True,
            )

    if raw_log_only:
        LOG.info(
            "COLLAR_RAW_LOG_ONLY enabled — collar port is connect + raw log only "
            "(no unpack, cal, or webhooks)"
        )
    elif IMU_ONLY_MODE:
        LOG.info("IMU-only barbell mode — optical flow and radar disabled")
    else:
        LOG.info("Full fusion mode — optical flow and radar required for EKF steps")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((SERVER_HOST, SERVER_PORT))
    sock.listen(8)
    listen_addr = sock.getsockname()
    if raw_log_only or debug_interval > 0:
        print(
            f"[collar debug] listening on {listen_addr[0]}:{listen_addr[1]} "
            f"({'raw log only' if raw_log_only else 'full fusion'})",
            file=sys.stderr,
            flush=True,
        )
        sock.settimeout(30.0)
    else:
        sock.settimeout(None)

    if STREAM_FIXED_LATENCY_US is None:
        LOG.info(
            "Fusion server listening on %s:%d (adaptive stream latency)",
            SERVER_HOST, SERVER_PORT,
        )
    else:
        LOG.info(
            "Fusion server listening on %s:%d (fixed stream latency %.0f ms)",
            SERVER_HOST, SERVER_PORT, STREAM_FIXED_LATENCY_US / 1000.0,
        )

    if sys.stdin.isatty():
        start_admin_console_thread()
        LOG.info("Interactive admin console — type 'help' at the fusion> prompt")
    else:
        LOG.info("No TTY — use: python3 fusion_admin.py")

    start_admin_socket_thread()
    start_collar_status_server(CAL_STATUS_HOST, CAL_STATUS_PORT)
    set_collar_status(STATUS_IDLE)

    while True:
        try:
            conn, addr = sock.accept()
        except socket.timeout:
            if raw_log_only or debug_interval > 0:
                print(
                    f"[collar debug] listening on {listen_addr[0]}:{listen_addr[1]} "
                    f"— no TCP connection yet",
                    file=sys.stderr,
                    flush=True,
                )
            continue
        LOG.info("Connection from %s", addr)
        if debug_interval > 0 or raw_log_only:
            print(f"[collar debug] TCP accept from {addr}", file=sys.stderr, flush=True)
        if raw_log_only:
            session = RawCollarSession(conn, addr)
        else:
            session = ClientSession(conn, addr)
        thread = threading.Thread(target=session.run, daemon=True)
        thread.start()
