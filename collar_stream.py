#!/usr/bin/env python3
"""
Live collar → fusion server bridge.

Reads async sensor JSONL from the collar (USB serial or stdin), converts to the
fusion sensor-stream wire format, and streams to the Oracle fusion server. The
server fuses poses and POSTs them to your Vercel webhook.

Typical setup (collar USB on your laptop, server on Oracle):

  # Terminal 1 — server already running on Oracle with VERCEL_WEBHOOK_URL set

  # Terminal 2 — stream from collar serial (Linux/macOS)
  python3 collar_stream.py --serial /dev/ttyACM0 --host <ORACLE_IP>

  # Windows (find COM port in Device Manager)
  py collar_stream.py --serial COM3 --host <ORACLE_IP>

  # Or pipe from collar firmware that prints JSONL to stdout:
  ./collar_firmware --jsonl | python3 collar_stream.py --stdin --host <ORACLE_IP>

Press Ctrl+C to end the session cleanly (sends {"type":"end"}).
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import time
from pathlib import Path
from typing import Iterator

from client_example import read_ack, send_line
from device_protocol import collar_line_to_stream_samples
from sensor_stream import format_sample

_running = True


def _handle_sigint(_signum: int, _frame: object) -> None:
    global _running
    _running = False


def iter_serial_lines(port: str, baud: int) -> Iterator[str]:
    try:
        import serial  # type: ignore[import-untyped]
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required for --serial. Install with: pip install pyserial"
        ) from exc

    with serial.Serial(port, baudrate=baud, timeout=1.0) as ser:
        while _running:
            raw = ser.readline()
            if not raw:
                continue
            yield raw.decode("utf-8", errors="replace")


def iter_stdin_lines() -> Iterator[str]:
    for line in sys.stdin:
        if not _running:
            break
        yield line


def iter_file_lines(path: Path, *, follow: bool) -> Iterator[str]:
    with path.open(encoding="utf-8") as handle:
        if not follow:
            yield from handle
            return

        while _running:
            line = handle.readline()
            if line:
                yield line
            else:
                time.sleep(0.05)


def stream_collar_samples(
    sock: socket.socket,
    lines: Iterator[str],
    *,
    use_host_time: bool,
    status_interval_s: float = 0.0,
) -> int:
    """Forward collar JSONL as fusion sensor-stream samples."""
    sent = 0
    last_status = time.monotonic()
    for line in lines:
        if not _running:
            break
        if status_interval_s > 0 and time.monotonic() - last_status >= status_interval_s:
            send_line(sock, {"type": "cal_lever_arm_status"})
            ack = read_ack(sock)
            print("Cal status:", json.dumps(ack), file=sys.stderr)
            last_status = time.monotonic()
        host_ts_us = int(time.time() * 1_000_000) if use_host_time else None
        samples = collar_line_to_stream_samples(line, host_ts_us=host_ts_us)
        for sensor, ts_us, payload in samples:
            sock.sendall((format_sample(sensor, ts_us, payload) + "\n").encode("utf-8"))
            sent += 1
        if sent and sent % 500 == 0:
            print(f"streamed {sent} samples", file=sys.stderr)
    return sent


def build_line_iterator(args: argparse.Namespace) -> Iterator[str]:
    if args.serial:
        return iter_serial_lines(args.serial, args.baud)
    if args.file:
        return iter_file_lines(args.file, follow=args.follow)
    return iter_stdin_lines()


def stream_to_server(
    sock: socket.socket,
    lines: Iterator[str],
    *,
    use_host_time: bool,
) -> int:
    return stream_collar_samples(sock, lines, use_host_time=use_host_time)


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream live collar data to fusion server")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--serial", metavar="PORT", help="USB serial port (e.g. /dev/ttyACM0, COM3)")
    source.add_argument("--stdin", action="store_true", help="Read JSONL lines from stdin")
    source.add_argument("--file", type=Path, help="Replay a capture file (for testing)")
    parser.add_argument("--follow", action="store_true", help="With --file, tail new lines")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument(
        "--time",
        choices=("device", "host"),
        default="device",
        help="Use device timestamps from JSONL (default) or host wall clock",
    )
    args = parser.parse_args()

    if args.serial:
        line_iter = build_line_iterator(args)
    elif args.file:
        line_iter = build_line_iterator(args)
    else:
        line_iter = build_line_iterator(args)

    signal.signal(signal.SIGINT, _handle_sigint)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _handle_sigint)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {args.host}:{args.port} ...", file=sys.stderr)
    sock.connect((args.host, args.port))

    send_line(sock, {"type": "start"})
    ack = read_ack(sock)
    print("Start:", json.dumps(ack), file=sys.stderr)

    try:
        count = stream_to_server(
            sock,
            line_iter,
            use_host_time=(args.time == "host"),
        )
        print(f"Streamed {count} samples", file=sys.stderr)
    finally:
        send_line(sock, {"type": "end"})
        print("End:", read_ack(sock), file=sys.stderr)
        sock.close()


if __name__ == "__main__":
    main()
