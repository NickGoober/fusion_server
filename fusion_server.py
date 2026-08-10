#!/usr/bin/env python3
"""
Oracle Ubuntu TCP fusion server.

The collar connects once and streams sensor packets continuously.
See server_main.py for the accept loop and client_session.py for per-connection
handling.

  python3 fusion_server.py          # interactive fusion> prompt
  python3 fusion_admin.py         # if server runs under systemd without TTY
"""

from __future__ import annotations

from client_session import ClientSession
from server_main import serve
from webhook_client import now_us, post_pose_webhook

__all__ = [
    "ClientSession",
    "now_us",
    "post_pose_webhook",
    "serve",
]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Fusion collar TCP server")
    parser.add_argument(
        "--raw-log-only",
        action="store_true",
        help="Bypass fusion/cal — only accept collar TCP and log raw payloads",
    )
    args = parser.parse_args()
    serve(force_raw_log_only=args.raw_log_only)
