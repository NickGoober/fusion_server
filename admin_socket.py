"""TCP admin command port (fusion_admin.py)."""

from __future__ import annotations

import logging
import socket
import threading

from fusion_admin_dispatch import dispatch_admin_command
from server_config import ADMIN_HOST, ADMIN_PORT

LOG = logging.getLogger("fusion_server.admin")


def _handle_admin_client(conn: socket.socket) -> None:
    try:
        data = conn.recv(8192)
        if not data:
            return
        text = data.decode("utf-8", errors="replace")
        outputs: list[str] = []
        for line in text.splitlines():
            if not line.strip():
                continue
            _, output = dispatch_admin_command(line)
            if output:
                outputs.append(output.rstrip("\n"))
        response = "\n".join(outputs) + ("\n" if outputs else "OK\n")
        conn.sendall(response.encode("utf-8"))
    except OSError as exc:
        LOG.debug("Admin client error: %s", exc)
    finally:
        try:
            conn.close()
        except OSError:
            pass


def start_admin_socket_thread() -> None:
    def _loop() -> None:
        admin_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        admin_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        admin_sock.bind((ADMIN_HOST, ADMIN_PORT))
        admin_sock.listen(8)
        LOG.info("Admin commands on %s:%d (fusion_admin.py)", ADMIN_HOST, ADMIN_PORT)
        while True:
            conn, _addr = admin_sock.accept()
            thread = threading.Thread(
                target=_handle_admin_client,
                args=(conn,),
                daemon=True,
            )
            thread.start()

    thread = threading.Thread(target=_loop, name="fusion-admin-socket", daemon=True)
    thread.start()
