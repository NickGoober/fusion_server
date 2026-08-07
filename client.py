#!/usr/bin/env python3
"""
Hardware / edge client — streams sensor JSON to the Oracle fusion server.

Set SERVER_IP to your Oracle instance (e.g. 79.72.87.48).
Replace the synthetic loop with real sensor reads from your collar.
"""

import json
import os
import socket
import time

SERVER_IP = os.environ.get("SERVER_IP", "79.72.87.48")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "9000"))

from client_example import send_line, read_ack, FLOW_HW_COUNT_SCALE, DT_S, FLOW_NPIX, FLOW_THETAPIX


def stream_synthetic_demo(sock: socket.socket) -> None:
    """Placeholder motion — swap for real IMU / flow / range reads."""
    start_us = int(time.time() * 1_000_000)
    flow_residue_x = 0.0
    z_m = 0.55
    vx = 0.55

    for step in range(200):
        sim_time = step * DT_S
        ts_us = start_us + int(sim_time * 1_000_000)
        vel_x = vx if sim_time >= 0.5 else 0.0
        scale = (DT_S * FLOW_NPIX) / FLOW_THETAPIX
        dpix_x = scale * (vel_x / z_m)
        flow_residue_x += dpix_x
        raw_y = round(-FLOW_HW_COUNT_SCALE * flow_residue_x)
        flow_residue_x -= (-raw_y) / FLOW_HW_COUNT_SCALE

        send_line(sock, {
            "type": "sensor",
            "ts_us": ts_us,
            "quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
            "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
            "flow": {"dx": 0, "dy": raw_y, "quality": 255},
            "range": {"mm": int(z_m * 1000), "strength": 100},
        })
        time.sleep(DT_S)


def main() -> None:
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {SERVER_IP}:{SERVER_PORT} ...")
    client.connect((SERVER_IP, SERVER_PORT))
    print("Connected")

    send_line(client, {"type": "start"})
    print("Start:", read_ack(client))

    stream_synthetic_demo(client)

    send_line(client, {"type": "end"})
    print("End:", read_ack(client))
    client.close()


if __name__ == "__main__":
    main()
