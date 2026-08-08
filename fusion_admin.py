#!/usr/bin/env python3
"""
Admin client for fusion_server.

Connects to the local admin port (default 127.0.0.1:9001) and sends commands.
Use when the server runs under systemd without an interactive TTY.

  python3 fusion_admin.py status
  python3 fusion_admin.py cal start
  python3 fusion_admin.py cal finish
  python3 fusion_admin.py display start
  python3 fusion_admin.py display stop

Or interactive REPL:
  python3 fusion_admin.py
"""

from __future__ import annotations

import argparse
import socket
import sys

from fusion_settings import get_int_setting, get_setting


def _admin_endpoint() -> tuple[str, int]:
    host = get_setting("ADMIN_HOST", "127.0.0.1")
    port = get_int_setting("ADMIN_PORT", 9001)
    return host, port


def send_admin_command(command: str) -> str:
    host, port = _admin_endpoint()
    payload = command.strip() + "\n"
    with socket.create_connection((host, port), timeout=5.0) as sock:
        sock.sendall(payload.encode("utf-8"))
        sock.shutdown(socket.SHUT_WR)
        chunks: list[bytes] = []
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
    return b"".join(chunks).decode("utf-8", errors="replace")


def repl() -> int:
    print(f"Fusion admin — connected to {_admin_endpoint()[0]}:{_admin_endpoint()[1]}")
    print("Type 'help' for commands, 'quit' to exit.")
    while True:
        try:
            line = input("fusion> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line.strip():
            continue
        try:
            out = send_admin_command(line)
        except OSError as exc:
            print(f"Admin connection failed: {exc}")
            print("Is fusion_server.py running?")
            return 1
        if out:
            print(out, end="" if out.endswith("\n") else "\n")
        if line.strip().lower() in ("quit", "exit"):
            return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion server admin client")
    parser.add_argument(
        "command",
        nargs="*",
        help="Command to run (e.g. cal start). Omit for interactive REPL.",
    )
    args = parser.parse_args()

    if not args.command:
        return repl()

    command = " ".join(args.command)
    try:
        out = send_admin_command(command)
    except OSError as exc:
        print(f"Admin connection failed: {exc}", file=sys.stderr)
        return 1
    if out:
        print(out, end="" if out.endswith("\n") else "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
