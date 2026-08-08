"""
Shared admin command dispatch for fusion_server (stdin + TCP admin port).
"""

from __future__ import annotations

import io
import json
import time
from contextlib import redirect_stdout
from typing import TYPE_CHECKING

from collar_registry import (
    get_active_collar_session,
    get_collar_events,
    get_last_collar_disconnect,
)
from fusion_settings import get_int_setting, get_setting
from sensor_recorder import default_record_dir, get_sensor_recorder

if TYPE_CHECKING:
    from fusion_server import ClientSession

_HELP = """
Collar streams packets continuously. Control calibration and live display here:

  status              Show collar connection and server state
  log                 Show recent connect/disconnect events
  cal start           Begin IMU lever-arm calibration (auto rotation axis)
  cal finish          Finish calibration and save fusion_calib.json
  cal cancel          Abort calibration
  cal status          Show calibration progress
  (IMU-only barbell mode — rotate about the bar long axis; no optical flow)
  display start       Start fusion + POST poses to Vercel
  display stop        Stop Vercel updates (collar keeps streaming)
  trace rotation start   Log quat packets collar→server and server→web every 1s
  trace rotation stop    Stop rotation trace
  trace rotation         Show rotation trace status
  record start [file]    Record collar sensor stream to recordings/ (JSONL)
  record stop            Stop recording and close the file
  record status          Show recording state
  replay start <file>    Replay a capture to localhost (optional: --speed 2.0, --fast)
  replay stop            Stop an in-progress replay
  replay status          Show replay progress
  help                Show this message
  quit                Exit the admin console (server keeps running)
"""


def _get_active_session() -> ClientSession | None:
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

    if cmd in ("status", "log"):
        with redirect_stdout(out):
            if cmd == "status":
                _cmd_status()
            else:
                _cmd_log()
        return True, out.getvalue()

    if cmd == "trace":
        with redirect_stdout(out):
            _cmd_trace(parts[1:])
        return True, out.getvalue()

    if cmd == "record":
        with redirect_stdout(out):
            _cmd_record(parts[1:])
        return True, out.getvalue()

    if cmd == "replay":
        with redirect_stdout(out):
            _cmd_replay(parts[1:])
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
        last = get_last_collar_disconnect()
        if last:
            print(
                f"Last disconnect: {last['addr']} after {last.get('duration_s', '?')}s "
                f"at {last['t_utc']}"
            )
            print(f"  reason: {last.get('reason', 'unknown')}")
            print(f"  packets received: {last.get('packets_received', 0)}")
        print("Live display: off")
        print("Calibration: inactive")
        print("Tip: run 'log' for full connection history")
        return
    print(session.console_status_text())


def _cmd_log() -> None:
    session = get_active_collar_session()
    if session is not None:
        uptime_s = time.monotonic() - session.connected_at
        print(
            f"Collar connected: {session.addr[0]}:{session.addr[1]} "
            f"({uptime_s:.1f}s, {session.packets_received} packets)"
        )
    events = get_collar_events(20)
    if not events:
        print("No collar connection events yet.")
        return
    print("Recent collar events:")
    for ev in events:
        if ev["kind"] == "connect":
            print(f"  {ev['t_utc']}  CONNECT     {ev['addr']}")
        elif ev["kind"] == "disconnect":
            print(
                f"  {ev['t_utc']}  DISCONNECT  {ev['addr']}  "
                f"after {ev.get('duration_s', '?')}s  "
                f"— {ev.get('reason', '?')}  "
                f"({ev.get('packets_received', 0)} packets)"
            )


def _cmd_trace(args: list[str]) -> None:
    if not args:
        print("Usage: trace rotation start | trace rotation stop | trace rotation")
        return

    target = args[0].lower()
    if target != "rotation":
        print(f"Unknown trace target: {target!r}. Use: trace rotation ...")
        return

    session = get_active_collar_session()
    if len(args) == 1:
        if session is None:
            print("No collar connected.")
            return
        session.console_trace_rotation_status()
        return

    action = args[1].lower()
    if session is None:
        print("No collar connected.")
        return

    if action == "start":
        session.console_trace_rotation_start()
    elif action == "stop":
        session.console_trace_rotation_stop()
    else:
        print(f"Unknown trace action: {action!r}")


def _cmd_record(args: list[str]) -> None:
    rec = get_sensor_recorder()
    if not args:
        _print_record_status(rec)
        return

    action = args[0].lower()
    if action == "status":
        _print_record_status(rec)
    elif action == "start":
        path = args[1] if len(args) > 1 else None
        session = get_active_collar_session()
        sid = session.session_id if session else None
        addr = f"{session.addr[0]}:{session.addr[1]}" if session else None
        try:
            out = rec.start(path, session_id=sid, remote_addr=addr)
        except (RuntimeError, OSError) as exc:
            print(exc)
            return
        print(f"Recording to {out}")
        if session is None:
            print("No collar connected yet — samples record when packets arrive.")
        else:
            print(f"Collar session {sid}")
    elif action == "stop":
        path = rec.stop()
        if path is None:
            print("Not recording.")
        else:
            st = rec.status()
            print(f"Stopped. Saved {st['samples']} samples to:")
            print(f"  {path}")
    else:
        print(f"Unknown record action: {action!r}")


def _print_record_status(rec) -> None:
    st = rec.status()
    if st["recording"]:
        print(f"Recording: ON — {st['samples']} samples")
        print(f"  file: {st['path']}")
        if st["remote_addr"]:
            print(f"  collar: {st['remote_addr']}")
    else:
        print("Recording: off")
        print(f"  directory: {default_record_dir()}")


def _cmd_replay(args: list[str]) -> None:
    rec = get_sensor_recorder()
    if not args:
        _print_replay_status(rec)
        return

    action = args[0].lower()
    if action == "status":
        _print_replay_status(rec)
    elif action == "stop":
        rec.stop_replay()
        print("Replay stopped.")
    elif action == "start":
        if len(args) < 2:
            print("Usage: replay start <capture.jsonl> [--speed 1.0] [--fast]")
            return
        path = args[1]
        speed = 1.0
        realtime = True
        if "--speed" in args:
            idx = args.index("--speed")
            if idx + 1 < len(args):
                speed = float(args[idx + 1])
        if "--fast" in args:
            realtime = False
        host = get_setting("SERVER_HOST", "0.0.0.0") or "0.0.0.0"
        if host == "0.0.0.0":
            host = "127.0.0.1"
        port = get_int_setting("SERVER_PORT", 9000)
        try:
            rec.start_replay(path, host=host, port=port, speed=speed, realtime=realtime)
        except (FileNotFoundError, RuntimeError) as exc:
            print(exc)
            return
        mode = f"at {speed}x speed" if realtime else "as fast as possible"
        print(f"Replaying {path} → {host}:{port} {mode}")
        st = rec.status().get("replay") or {}
        if st.get("estimated_duration_s") is not None:
            print(f"Estimated duration: {st['estimated_duration_s']}s")
        print("Run 'replay status' for progress, 'replay stop' to cancel.")
    else:
        print(f"Unknown replay action: {action!r}")


def _print_replay_status(rec) -> None:
    st = rec.status()
    rep = st.get("replay") or {}
    if st.get("replay_active"):
        print(f"Replay: running — {rep.get('sent', 0)}/{rep.get('total', '?')} lines")
        print(f"  file: {rep.get('path')}")
        if rep.get("error"):
            print(f"  error: {rep['error']}")
    elif rep.get("done"):
        print(f"Replay: finished — {rep.get('sent', 0)}/{rep.get('total', 0)} lines")
        if rep.get("error"):
            print(f"  error: {rep['error']}")
    else:
        print("Replay: idle")


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
