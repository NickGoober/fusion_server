#!/usr/bin/env python3
"""Subscribe to live fused pose from fusion_server (TCP NDJSON).

  python pose_stream_client.py
  python pose_stream_client.py --host 127.0.0.1 --port 9002
"""

from __future__ import annotations

import argparse
import json
import socket
import sys


def _fmt_pos(vals: list) -> str:
    if not vals or len(vals) < 6:
        return "—"
    fx, fy, fz, rx, ry, rz = vals[:6]
    return f"filt=({fx:.4f},{fy:.4f},{fz:.4f}) raw=({rx:.4f},{ry:.4f},{rz:.4f})"


def _fmt_rot(vals: list) -> str:
    if not vals or len(vals) < 8:
        return "—"
    qw, qx, qy, qz, rw, rx, ry, rz = vals[:8]
    return (
        f"filt=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f}) "
        f"raw=({rw:.3f},{rx:.3f},{ry:.3f},{rz:.3f})"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Raedir live pose stream client")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--secret", default="", help="POSE_STREAM_SECRET if the server requires auth")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connecting to pose stream {args.host}:{args.port} ...", file=sys.stderr)
    sock.connect((args.host, args.port))
    if args.secret:
        sock.sendall((json.dumps({"type": "auth", "token": args.secret}) + "\n").encode("utf-8"))

    buf = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            print("stream closed", file=sys.stderr)
            break
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line.strip():
                continue
            msg = json.loads(line.decode("utf-8"))
            if msg.get("type") == "hello":
                print("hello", msg.get("protocol"), msg.get("axes"), file=sys.stderr)
                continue
            print(
                f"n={msg.get('n')} s={msg.get('s')} t={msg.get('t')} f={msg.get('f')} "
                f"p {_fmt_pos(msg.get('p') or [])} "
                f"r {_fmt_rot(msg.get('r') or [])}"
            )


if __name__ == "__main__":
    main()
