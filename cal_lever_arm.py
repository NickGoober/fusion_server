#!/usr/bin/env python3
"""
Run lever-arm calibration against the fusion server.

Supports:
  - New async sensor stream lines: [sensor_index, timestamp, payload]
  - Legacy bundled {"type":"sensor", ...} messages (via replay_capture resampling)

Variable-rate calibration (recommended): pass --omega 0 so any steady spin about the
axis is accepted. With correct lever arms, centripetal acceleration trends to zero.

Example with synthetic test data:

  py generate_cal_test_data.py -o cal_test.jsonl --duration 35 --variable-rate
  py cal_lever_arm.py cal_test.jsonl --host 127.0.0.1 --omega 0

Allow a short drain after streaming ends (server flushes on finish; adaptive
latency is typically 50–500 ms).
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path

from client_example import read_ack, send_line
from replay_capture import load_capture, resample_capture, to_server_message
from sensor_stream import is_control_message, parse_sample_line

DEFAULT_DRAIN_S = 0.5


def stream_file(sock: socket.socket, path: Path, *, speed: float) -> int:
    """Stream raw lines from a capture file. Returns line count."""
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines:
        return 0

    is_stream = False
    for line in lines[:20]:
        if line.strip() and not is_control_message(line) and parse_sample_line(line):
            is_stream = True
            break

    if is_stream:
        return _stream_sensor_lines(sock, lines, speed=speed)

    series = load_capture(path)
    ticks = resample_capture(series, 100.0)
    return _stream_bundled_ticks(sock, ticks, speed=speed)


def _stream_sensor_lines(sock: socket.socket, lines: list[str], *, speed: float) -> int:
    sent = 0
    replay_start = time.monotonic()
    first_ts_us: int | None = None

    for line in lines:
        line = line.strip()
        if not line:
            continue
        parsed = parse_sample_line(line)
        if parsed is None:
            continue
        _, ts_us, _ = parsed
        if first_ts_us is None:
            first_ts_us = ts_us
        if speed > 0 and first_ts_us is not None:
            target_s = (ts_us - first_ts_us) / 1_000_000.0 / speed
            delay = target_s - (time.monotonic() - replay_start)
            if delay > 0:
                time.sleep(delay)
        sock.sendall((line + "\n").encode("utf-8"))
        sent += 1
        if sent % 200 == 0:
            print(f"sent {sent} stream samples")
    return sent


def _stream_bundled_ticks(sock: socket.socket, ticks: list[dict], *, speed: float) -> int:
    base_us = int(time.time() * 1_000_000)
    replay_start = time.monotonic()
    first_t_ms = ticks[0]["t_ms"]

    for i, tick in enumerate(ticks):
        if speed > 0:
            target_s = (tick["t_ms"] - first_t_ms) / 1000.0 / speed
            delay = target_s - (time.monotonic() - replay_start)
            if delay > 0:
                time.sleep(delay)
        send_line(sock, to_server_message(tick, base_us))
        if i % 50 == 0:
            print(f"sent {i + 1}/{len(ticks)}")
    return len(ticks)


def _drain_seconds(sock: socket.socket, fallback_s: float) -> float:
    send_line(sock, {"type": "stream_status"})
    ack = read_ack(sock)
    if ack.get("of") == "stream_status":
        latency_ms = float(ack.get("latency_ms", fallback_s * 1000))
        return max(DEFAULT_DRAIN_S, latency_ms / 1000.0 * 1.5)
    return fallback_s


def run_calibration(
    sock: socket.socket,
    capture_file: Path,
    *,
    axis: str,
    omega: float,
    omega_tol: float,
    speed: float,
    drain_s: float,
) -> dict:
    send_line(sock, {
        "type": "cal_lever_arm_start",
        "axis": axis,
        "omega_rad_s": omega,
        "omega_tol_rad_s": omega_tol,
    })
    ack = read_ack(sock)
    print("Cal start:", json.dumps(ack, indent=2))
    if ack.get("error"):
        raise RuntimeError(ack["error"])

    count = stream_file(sock, capture_file, speed=speed)
    wait_s = _drain_seconds(sock, drain_s)
    print(f"Streamed {count} samples; waiting {wait_s:.1f}s for buffer drain ...")
    time.sleep(wait_s)

    send_line(sock, {"type": "cal_lever_arm_finish"})
    ack = read_ack(sock)
    print("Cal finish:", json.dumps(ack, indent=2))
    if ack.get("error"):
        raise RuntimeError(ack["error"])
    return ack


def main() -> None:
    parser = argparse.ArgumentParser(description="Lever-arm calibration client")
    parser.add_argument("capture_file", help="JSONL capture or sensor stream file")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--axis", default="auto", choices=("auto", "x", "y", "z"),
                        help="Body axis to rotate about; auto detects from gyro")
    parser.add_argument("--omega", type=float, default=0.0,
                        help="Expected rotation rate [rad/s]; 0 = variable rate")
    parser.add_argument("--omega-tol", type=float, default=0.0,
                        help="Allowed deviation from --omega (0 = auto; ignored when omega=0)")
    parser.add_argument("--drain-s", type=float, default=DEFAULT_DRAIN_S,
                        help="Extra buffer drain time before finish if stream_status unavailable")
    parser.add_argument("--speed", type=float, default=10.0,
                        help="Replay speed multiplier (0 = as fast as possible)")
    args = parser.parse_args()

    capture_path = Path(args.capture_file)
    if not capture_path.is_file():
        raise SystemExit(f"File not found: {capture_path}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {args.host}:{args.port} ...")
    sock.connect((args.host, args.port))

    try:
        run_calibration(
            sock,
            capture_path,
            axis=args.axis,
            omega=args.omega,
            omega_tol=args.omega_tol,
            speed=args.speed,
            drain_s=args.drain_s,
        )
    finally:
        sock.close()


if __name__ == "__main__":
    main()
