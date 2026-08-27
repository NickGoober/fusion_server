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
                print("hello", msg.get("protocol"), file=sys.stderr)
                continue
            pose = msg.get("pose") or {}
            raw = msg.get("pose_raw") or {}
            filt_pos = pose.get("position_m") or {}
            raw_pos = raw.get("position_m") or {}
            rot = pose.get("rotation") or {}
            print(
                f"seq={msg.get('frame_seq')} streaming={msg.get('streaming')} "
                f"filtered=({filt_pos.get('x', 0):.4f},{filt_pos.get('y', 0):.4f},{filt_pos.get('z', 0):.4f}) "
                f"raw=({raw_pos.get('x', 0):.4f},{raw_pos.get('y', 0):.4f},{raw_pos.get('z', 0):.4f}) "
                f"qw={rot.get('w', 0):.3f}"
            )


if __name__ == "__main__":
    main()
