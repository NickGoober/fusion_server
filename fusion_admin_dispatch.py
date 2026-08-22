"""
Shared admin command dispatch for fusion_server (stdin + TCP admin port).
"""

from __future__ import annotations

import io
import time
from contextlib import redirect_stdout
from typing import TYPE_CHECKING

from collar_registry import (
    get_active_collar_session,
    get_collar_events,
    get_last_collar_disconnect,
)
from sensor_recorder import default_record_dir, get_sensor_recorder

if TYPE_CHECKING:
    from client_session import ClientSession

_HELP = """
Collar streams packets continuously. Control live display here:

  status              Show collar connection and server state
  log                 Show recent connect/disconnect events
  display start       Start fusion + POST poses to Vercel
  display stop        Stop Vercel updates (collar keeps streaming)
  trace rotation start   Log quat packets collar→server and server→web every 1s
  trace rotation stop    Stop rotation trace
  trace rotation         Show rotation trace status
  record start [file]    Record full collar stream to recordings/ (.jsonl)
  record imu [file]      Record IMU only (quat + accel)
  record stop            Stop recording and close the file
  record status          Show recording state
  replay start <file>    Replay a capture to localhost (optional: --speed 2.0, --fast, --batched)
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
        if cmd in ("display", "stream"):
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
    if action == "imu":
        _cmd_record_imu(rec, args[1:])
        return
    if action == "status":
        _print_record_status(rec)
    elif action == "start":
        path = args[1] if len(args) > 1 else None
        _start_recording(rec, path, filter_mode="all")
    elif action == "stop":
        _stop_recording(rec)
    else:
        print(f"Unknown record action: {action!r}")


def _cmd_record_imu(rec, args: list[str]) -> None:
    if not args:
        _print_record_status(rec)
        print("  tip: record imu [file.jsonl]  — quat + accel only")
        return

    action = args[0].lower()
    if action == "status":
        _print_record_status(rec)
        return
    if action == "stop":
        _stop_recording(rec)
        return
    if action == "start":
        path = args[1] if len(args) > 1 else None
        _start_recording(rec, path, filter_mode="imu")
        return

    _start_recording(rec, args[0], filter_mode="imu")


def _start_recording(rec, path: str | None, *, filter_mode: str) -> None:
    session = get_active_collar_session()
    sid = session.session_id if session else None
    addr = f"{session.addr[0]}:{session.addr[1]}" if session else None
    try:
        out = rec.start(path, session_id=sid, remote_addr=addr, filter_mode=filter_mode)
    except (RuntimeError, OSError) as exc:
        print(exc)
        return
    if filter_mode == "imu":
        print("Recording IMU only (wire types 0=quat, 1=linear accel).")
        print("Save as .jsonl (one JSON value per line) — not a single .json array.")
    else:
        print(f"Recording full sensor stream to {out}")
    print(f"File: {out}")
    if session is None:
        print("No collar connected yet — samples record when packets arrive.")
    else:
        print(f"Collar session {sid}")
    print("Run 'record stop' when finished.")


def _stop_recording(rec) -> None:
    path = rec.stop()
    if path is None:
        print("Not recording.")
    else:
        st = rec.status()
        print(f"Stopped. Saved {st['samples']} lines to:")
        print(f"  {path}")


def _print_record_status(rec) -> None:
    st = rec.status()
    if st["recording"]:
        mode = st.get("filter") or "all"
        label = "IMU only" if mode == "imu" else "full stream"
        print(f"Recording: ON ({label}) — {st['samples']} lines")
        print(f"  file: {st['path']}")
        if st["remote_addr"]:
            print(f"  collar: {st['remote_addr']}")
    else:
        print("Recording: off")
        print(f"  directory: {default_record_dir()}")


def _cmd_replay(args: list[str]) -> None:
    from fusion_settings import get_int_setting, get_setting

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
            print("Usage: replay start <capture.jsonl> [--speed 1.0] [--fast] [--batched]")
            return
        path = args[1]
        speed = 1.0
        realtime = True
        expand_batches = True
        if "--speed" in args:
            idx = args.index("--speed")
            if idx + 1 < len(args):
                speed = float(args[idx + 1])
        if "--fast" in args:
            realtime = False
        if "--batched" in args:
            expand_batches = False
        host = get_setting("SERVER_HOST", "0.0.0.0") or "0.0.0.0"
        if host == "0.0.0.0":
            host = "127.0.0.1"
        port = get_int_setting("SERVER_PORT", 9000)
        try:
            rec.start_replay(
                path,
                host=host,
                port=port,
                speed=speed,
                realtime=realtime,
                expand_batches=expand_batches,
            )
        except (FileNotFoundError, RuntimeError) as exc:
            print(exc)
            return
        mode = f"at {speed}x speed" if realtime else "as fast as possible"
        batch_mode = "expanded samples" if expand_batches else "raw 1s batches"
        print(f"Replaying {path} -> {host}:{port} {mode} ({batch_mode})")
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
