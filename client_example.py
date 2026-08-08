#!/usr/bin/env python3
"""
Example client: streams synthetic sensor data to the fusion server.

Matches the pure-X motion pattern from simulate_barbell.c for local testing.
"""

import json
import math
import socket
import time

SERVER_IP = "127.0.0.1"
SERVER_PORT = 9000

SIM_HZ = 100
DT_S = 1.0 / SIM_HZ
FLOW_HW_COUNT_SCALE = 10.0
FLOW_NPIX = 35.0
FLOW_THETAPIX = 0.71674


def send_line(sock: socket.socket, payload: dict) -> None:
    sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))


def read_ack(sock: socket.socket) -> dict:
    buffer = b""
    while b"\n" not in buffer:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("server closed connection")
        buffer += chunk
    line, _ = buffer.split(b"\n", 1)
    return json.loads(line.decode("utf-8"))


def main() -> None:
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {SERVER_IP}:{SERVER_PORT} ...")
    client.connect((SERVER_IP, SERVER_PORT))
    print("Connected")

    send_line(client, {"type": "start"})
    ack = read_ack(client)
    print("Start ack:", ack)

    start_us = int(time.time() * 1_000_000)
    flow_residue_x = 0.0
    flow_residue_y = 0.0
    z_m = 0.55
    vx = 0.55

    for step in range(250):
        sim_time = step * DT_S
        ts_us = start_us + int(sim_time * 1_000_000)

        x_m = vx * max(0.0, sim_time - 0.5)
        vel_x = vx if sim_time >= 0.5 else 0.0

        tof_m = z_m
        scale = (DT_S * FLOW_NPIX) / FLOW_THETAPIX
        dpix_x = scale * (vel_x / tof_m)
        dpix_y = 0.0

        flow_residue_x += dpix_x
        raw_y = round(-FLOW_HW_COUNT_SCALE * flow_residue_x)
        flow_residue_x -= (-raw_y) / FLOW_HW_COUNT_SCALE
        raw_x = 0

        payload = {
            "type": "sensor",
            "ts_us": ts_us,
            "quat": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
            "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
            "flow": {"dx": raw_x, "dy": raw_y, "quality": 255},
            "range": {"mm": int(tof_m * 1000)},
        }
        send_line(client, payload)
        time.sleep(DT_S)

        if step % 50 == 0:
            print(f"t={sim_time:.2f}s sent x≈{x_m:.2f}m")

    send_line(client, {"type": "end"})
    ack = read_ack(client)
    print("End ack:", ack)
    client.close()
    print("Done")


if __name__ == "__main__":
    main()
