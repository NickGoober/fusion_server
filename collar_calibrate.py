#!/usr/bin/env python3
"""
[Optional dev tool] Live lever-arm calibration client.

Production collars should send CAL_START / sensor data / CAL_FINISH directly
to the server (see README). This script is only needed when the collar has
USB serial but no TCP — it bridges serial and sends JSON control messages.
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import time

from cal_lever_arm import DEFAULT_DRAIN_S, _drain_seconds
from client_example import read_ack, send_line
from collar_stream import _running, build_line_iterator, stream_collar_samples
import collar_stream as collar_stream_mod


def _handle_sigint(_signum: int, _frame: object) -> None:
    collar_stream_mod._running = False


def run_live_calibration(
    sock: socket.socket,
    *,
    duration_s: float,
    omega: float,
    omega_tol: float,
    use_host_time: bool,
    line_iter,
    drain_s: float,
) -> dict:
    send_line(sock, {
        "type": "cal_lever_arm_start",
        "axis": "auto",
        "omega_rad_s": omega,
        "omega_tol_rad_s": omega_tol,
    })
    ack = read_ack(sock)
    print("Cal start:", json.dumps(ack, indent=2))
    if ack.get("error"):
        raise RuntimeError(ack["error"])

    print(
        "\n>>> Rotate the collar steadily about ONE axis for at least 5 seconds.\n"
        ">>> Keep it flat on the table; spin about the collar center.\n",
        file=sys.stderr,
    )

    deadline = time.monotonic() + duration_s if duration_s > 0 else None

    def limited_iter():
        for line in line_iter:
            if not _running:
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
            yield line

    count = stream_collar_samples(
        sock,
        limited_iter(),
        use_host_time=use_host_time,
        status_interval_s=2.0,
    )
    print(f"\nStreamed {count} sensor samples", file=sys.stderr)

    wait_s = _drain_seconds(sock, drain_s)
    print(f"Waiting {wait_s:.1f}s for buffer drain ...", file=sys.stderr)
    time.sleep(wait_s)

    send_line(sock, {"type": "cal_lever_arm_finish"})
    ack = read_ack(sock)
    print("Cal finish:", json.dumps(ack, indent=2))
    if ack.get("error"):
        raise RuntimeError(ack["error"])
    return ack


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live collar lever-arm calibration (auto rotation axis)",
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--serial", metavar="PORT", help="USB serial port")
    source.add_argument("--stdin", action="store_true", help="Read JSONL from stdin")
    source.add_argument("--file", type=str, help="Replay capture file (testing)")
    parser.add_argument("--follow", action="store_true")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Seconds to collect data (0 = until Ctrl+C)",
    )
    parser.add_argument("--omega", type=float, default=0.0,
                        help="Expected spin rate [rad/s]; 0 = variable rate")
    parser.add_argument("--omega-tol", type=float, default=0.0)
    parser.add_argument("--drain-s", type=float, default=DEFAULT_DRAIN_S)
    parser.add_argument(
        "--time",
        choices=("device", "host"),
        default="device",
        help="Timestamp source for collar samples",
    )
    args = parser.parse_args()

    if args.file:
        from pathlib import Path
        args.file = Path(args.file)

    signal.signal(signal.SIGINT, _handle_sigint)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _handle_sigint)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {args.host}:{args.port} ...", file=sys.stderr)
    sock.connect((args.host, args.port))

    try:
        result = run_live_calibration(
            sock,
            duration_s=args.duration,
            omega=args.omega,
            omega_tol=args.omega_tol,
            use_host_time=(args.time == "host"),
            line_iter=build_line_iterator(args),
            drain_s=args.drain_s,
        )
        axis = result.get("axis")
        axis_names = ("x", "y", "z")
        if isinstance(axis, int) and 0 <= axis < 3:
            print(f"\nDetected rotation axis: {axis_names[axis]}", file=sys.stderr)
        print("\nCalibration saved on server (fusion_calib.json).", file=sys.stderr)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
