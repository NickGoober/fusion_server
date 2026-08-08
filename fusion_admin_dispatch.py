"""
Shared admin command dispatch for fusion_server (stdin + TCP admin port).
"""

from __future__ import annotations

import io
import json
from contextlib import redirect_stdout
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from fusion_server import ClientSession

_HELP = """
Collar streams packets continuously. Control calibration and live display here:

  status              Show collar connection and server state
  cal start           Begin lever-arm calibration (auto rotation axis)
  cal finish          Finish calibration and save fusion_calib.json
  cal cancel          Abort calibration
  cal status          Show calibration progress
  display start       Start fusion + POST poses to Vercel
  display stop        Stop Vercel updates (collar keeps streaming)
  help                Show this message
  quit                Exit the admin console (server keeps running)
"""


def _get_active_session() -> ClientSession | None:
    from fusion_server import get_active_collar_session
    return get_active_collar_session()


def dispatch_admin_command(line: str) -> tuple[bool, str]:
    """
    Execute one admin command.
    Returns (continue_loop, output_text).
    """
    out = io.StringIO()
    line = line.strip()
    if not line:
        return True, ""

    parts = line.split()
    cmd = parts[0].lower()

    if cmd in ("quit", "exit"):
        with redirect_stdout(out):
            print("Admin console closed. Server still running.")
        return False, out.getvalue()

    if cmd == "help":
        with redirect_stdout(out):
            print(_HELP)
        return True, out.getvalue()

    if cmd == "status":
        with redirect_stdout(out):
            _cmd_status()
        return True, out.getvalue()

    session = _get_active_session()
    if session is None:
        with redirect_stdout(out):
            print("No collar connected — waiting for TCP stream on port 9000.")
        return True, out.getvalue()

    with redirect_stdout(out):
        if cmd == "cal":
            _cmd_cal(session, parts[1:])
        elif cmd in ("display", "stream"):
            _cmd_display(session, parts[1:])
        else:
            print(f"Unknown command: {cmd!r}. Type 'help'.")

    return True, out.getvalue()


def _cmd_status() -> None:
    session = _get_active_session()
    if session is None:
        print("Collar: not connected")
        print("Live display: off")
        print("Calibration: inactive")
        return
    print(session.console_status_text())


def _cmd_cal(session: ClientSession, args: list[str]) -> None:
    if not args:
        print("Usage: cal start | cal finish | cal cancel | cal status")
        return

    action = args[0].lower()
    if action == "start":
        axis = "auto"
        if len(args) >= 3 and args[1] == "--axis":
            axis = args[2]
        session.console_cal_start(axis=axis)
    elif action == "finish":
        session.console_cal_finish()
    elif action == "cancel":
        session.console_cal_cancel()
    elif action == "status":
        status = session.console_cal_status()
        print(json.dumps(status, indent=2))
    else:
        print(f"Unknown cal subcommand: {action!r}")


def _cmd_display(session: ClientSession, args: list[str]) -> None:
    if not args:
        print("Usage: display start | display stop")
        return

    action = args[0].lower()
    if action == "start":
        session.console_display_start()
    elif action == "stop":
        session.console_display_stop()
    else:
        print(f"Unknown display subcommand: {action!r}")


def admin_console_loop() -> None:
    print("Fusion admin console — type 'help' for commands.")
    while True:
        try:
            line = input("fusion> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        cont, output = dispatch_admin_command(line)
        if output:
            print(output, end="" if output.endswith("\n") else "\n")
        if not cont:
            break


def start_admin_console_thread() -> None:
    import threading
    thread = threading.Thread(target=admin_console_loop, name="fusion-admin-stdin", daemon=True)
    thread.start()
