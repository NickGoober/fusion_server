#!/usr/bin/env python3
"""
Replay captured device JSONL to the fusion TCP server.

Real hardware logs arrive asynchronously (quat/gyro/flow/range at different
rates). This script resamples to a fixed tick rate (default 100 Hz, matching
the working simulated stream) and emits one bundled sensor message per tick:

  {"type":"sensor","ts_us":...,"quat":{...},"gyro":{...},"accel":{...},
   "flow":{"dx":...,"dy":...,"quality":...},"range":{"mm":...}}

Flow uses per-interval delta summation (not interpolation) because dx/dy are
discrete pixel counts per optical frame. Quat uses SLERP; gyro/accel/range
use linear interpolation between bracketing samples.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from dataclasses import dataclass, field
from typing import Any

DEFAULT_QUAT = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
DEFAULT_VEC3 = {"x": 0.0, "y": 0.0, "z": 0.0}
DEFAULT_FLOW = {"dx": 0, "dy": 0, "quality": 0}
DEFAULT_RANGE_MM = 550


def send_line(sock: socket.socket, payload: dict) -> None:
    sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))


def read_ack(sock: socket.socket) -> dict:
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("server closed connection")
        buf += chunk
    line, _ = buf.split(b"\n", 1)
    return json.loads(line.decode("utf-8"))


@dataclass
class Sample:
    t_ms: float
    value: Any


@dataclass
class CaptureSeries:
    quat: list[Sample] = field(default_factory=list)
    gyro: list[Sample] = field(default_factory=list)
    accel: list[Sample] = field(default_factory=list)
    range_mm: list[Sample] = field(default_factory=list)
    flow: list[Sample] = field(default_factory=list)  # value: dict dx, dy, quality


def _vec3_from_row(row: dict, *keys: str) -> dict[str, float] | None:
    for key in keys:
        raw = row.get(key)
        if raw is None:
            continue
        return {"x": float(raw["x"]), "y": float(raw["y"]), "z": float(raw["z"])}
    return None


def _quat_from_row(row: dict) -> dict[str, float] | None:
    raw = row.get("quat")
    if raw is None:
        return None
    return {
        "w": float(raw["w"]),
        "x": float(raw["x"]),
        "y": float(raw["y"]),
        "z": float(raw["z"]),
    }


def _time_ms_from_row(row: dict) -> float:
    if "t_ms" in row:
        return float(row["t_ms"])
    if "sim_time_s" in row:
        return float(row["sim_time_s"]) * 1000.0
    raise KeyError("row missing t_ms or sim_time_s")


def parse_device_row(row: dict, series: CaptureSeries) -> None:
    """Parse one hardware JSONL row into time series."""
    t_ms = _time_ms_from_row(row)
    kind = row.get("kind")

    if kind == "quat":
        quat = _quat_from_row(row)
        if quat:
            series.quat.append(Sample(t_ms, quat))
        return

    if kind == "gyro":
        vec = _vec3_from_row(row, "gyro_rad_s")
        if vec:
            series.gyro.append(Sample(t_ms, vec))
        return

    if kind == "accel":
        vec = _vec3_from_row(row, "accel_mps2", "accel_ms2", "accel")
        if vec:
            series.accel.append(Sample(t_ms, vec))
        return

    if kind == "flow":
        flow = row.get("flow")
        if flow is None:
            return
        series.flow.append(
            Sample(
                t_ms,
                {
                    "dx": int(flow.get("dx", flow.get("delta_x", 0))),
                    "dy": int(flow.get("dy", flow.get("delta_y", 0))),
                    "quality": int(flow.get("quality", 255)),
                },
            )
        )
        return

    if kind == "range":
        filtered = row.get("filtered")
        if filtered is None:
            rng = row.get("range")
            if rng is None:
                return
            if not rng.get("valid", True):
                return
            series.range_mm.append(Sample(t_ms, int(rng["mm"])))
            return

        if not filtered.get("valid", True):
            return
        series.range_mm.append(Sample(t_ms, int(filtered["distance_mm"])))
        return

    # Already-simulated bundled rows (optional direct replay format).
    if "quat" in row and kind is None:
        quat = _quat_from_row(row)
        if quat:
            series.quat.append(Sample(t_ms, quat))
        gyro = _vec3_from_row(row, "gyro_rad_s", "gyro")
        if gyro:
            series.gyro.append(Sample(t_ms, gyro))
        accel = _vec3_from_row(row, "accel_mps2", "accel")
        if accel:
            series.accel.append(Sample(t_ms, accel))
        rng = row.get("range")
        if rng:
            series.range_mm.append(Sample(t_ms, int(rng["mm"])))
        flow = row.get("flow")
        if flow:
            series.flow.append(
                Sample(
                    t_ms,
                    {
                        "dx": int(flow["dx"]),
                        "dy": int(flow["dy"]),
                        "quality": int(flow.get("quality", 255)),
                    },
                )
            )


def load_capture(path: str) -> CaptureSeries:
    series = CaptureSeries()
    with open(path, encoding="utf-8") as f:
        text = f.read().strip()

    if not text:
        return series

    # Support JSON array captures as well as JSONL.
    if text.startswith("["):
        rows = json.loads(text)
        for row in rows:
            parse_device_row(row, series)
    else:
        for line in text.splitlines():
            line = line.strip().rstrip(",")
            if line and line not in ("[", "]"):
                parse_device_row(json.loads(line), series)

    for channel in (
        series.quat,
        series.gyro,
        series.accel,
        series.range_mm,
        series.flow,
    ):
        channel.sort(key=lambda s: s.t_ms)

    return series


def _bracket(samples: list[Sample], t_ms: float) -> tuple[Sample | None, Sample | None]:
    if not samples:
        return None, None

    if t_ms <= samples[0].t_ms:
        return samples[0], samples[0]
    if t_ms >= samples[-1].t_ms:
        return samples[-1], samples[-1]

    lo = samples[0]
    hi = samples[-1]
    for i in range(len(samples) - 1):
        if samples[i].t_ms <= t_ms <= samples[i + 1].t_ms:
            lo = samples[i]
            hi = samples[i + 1]
            break
    return lo, hi


def _lerp(a: float, b: float, alpha: float) -> float:
    return a + (b - a) * alpha


def _lerp_vec3(
    lo: dict[str, float], hi: dict[str, float], alpha: float,
) -> dict[str, float]:
    return {
        "x": _lerp(lo["x"], hi["x"], alpha),
        "y": _lerp(lo["y"], hi["y"], alpha),
        "z": _lerp(lo["z"], hi["z"], alpha),
    }


def _quat_dot(a: dict[str, float], b: dict[str, float]) -> float:
    return (
        a["w"] * b["w"]
        + a["x"] * b["x"]
        + a["y"] * b["y"]
        + a["z"] * b["z"]
    )


def _quat_normalize(q: dict[str, float]) -> dict[str, float]:
    n = math.sqrt(_quat_dot(q, q))
    if n < 1e-12:
        return dict(DEFAULT_QUAT)
    return {k: q[k] / n for k in q}


def _slerp_quat(
    lo: dict[str, float], hi: dict[str, float], alpha: float,
) -> dict[str, float]:
    q0 = _quat_normalize(lo)
    q1 = _quat_normalize(hi)
    dot = _quat_dot(q0, q1)
    if dot < 0.0:
        q1 = {k: -v for k, v in q1.items()}
        dot = -dot

    if dot > 0.9995:
        out = {
            "w": _lerp(q0["w"], q1["w"], alpha),
            "x": _lerp(q0["x"], q1["x"], alpha),
            "y": _lerp(q0["y"], q1["y"], alpha),
            "z": _lerp(q0["z"], q1["z"], alpha),
        }
        return _quat_normalize(out)

    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return _quat_normalize(
        {
            "w": s0 * q0["w"] + s1 * q1["w"],
            "x": s0 * q0["x"] + s1 * q1["x"],
            "y": s0 * q0["y"] + s1 * q1["y"],
            "z": s0 * q0["z"] + s1 * q1["z"],
        }
    )


def _interp_vec3_channel(
    samples: list[Sample], t_ms: float, default: dict[str, float],
) -> dict[str, float]:
    lo, hi = _bracket(samples, t_ms)
    if lo is None:
        return dict(default)
    if lo is hi or math.isclose(lo.t_ms, hi.t_ms):
        return dict(lo.value)
    alpha = (t_ms - lo.t_ms) / (hi.t_ms - lo.t_ms)
    return _lerp_vec3(lo.value, hi.value, alpha)


def _interp_quat_channel(
    samples: list[Sample], t_ms: float, default: dict[str, float],
) -> dict[str, float]:
    lo, hi = _bracket(samples, t_ms)
    if lo is None:
        return dict(default)
    if lo is hi or math.isclose(lo.t_ms, hi.t_ms):
        return dict(lo.value)
    alpha = (t_ms - lo.t_ms) / (hi.t_ms - lo.t_ms)
    return _slerp_quat(lo.value, hi.value, alpha)


def _interp_scalar_channel(
    samples: list[Sample], t_ms: float, default: float,
) -> float:
    lo, hi = _bracket(samples, t_ms)
    if lo is None:
        return default
    if lo is hi or math.isclose(lo.t_ms, hi.t_ms):
        return float(lo.value)
    alpha = (t_ms - lo.t_ms) / (hi.t_ms - lo.t_ms)
    return _lerp(float(lo.value), float(hi.value), alpha)


def _flow_in_interval(
    samples: list[Sample], t_lo_ms: float, t_hi_ms: float,
) -> dict[str, int]:
    """Sum optical-flow pixel deltas that arrived in (t_lo, t_hi]."""
    dx = 0
    dy = 0
    quality = 0
    found = False
    for sample in samples:
        if t_lo_ms < sample.t_ms <= t_hi_ms:
            dx += int(sample.value["dx"])
            dy += int(sample.value["dy"])
            quality = max(quality, int(sample.value["quality"]))
            found = True
    if not found:
        return dict(DEFAULT_FLOW)
    return {"dx": dx, "dy": dy, "quality": quality}


def _capture_bounds(series: CaptureSeries) -> tuple[float, float]:
    times: list[float] = []
    for channel in (
        series.quat,
        series.gyro,
        series.accel,
        series.range_mm,
        series.flow,
    ):
        if channel:
            times.append(channel[0].t_ms)
            times.append(channel[-1].t_ms)
    if not times:
        raise ValueError("capture has no usable sensor rows")
    return min(times), max(times)


def resample_capture(
    series: CaptureSeries,
    hz: float,
) -> list[dict[str, Any]]:
    """Build bundled per-tick sensor states matching the simulated stream."""
    t_start_ms, t_end_ms = _capture_bounds(series)
    dt_ms = 1000.0 / hz
    ticks: list[dict[str, Any]] = []

    t_ms = t_start_ms
    prev_t_ms = t_start_ms - dt_ms
    while t_ms <= t_end_ms + 1e-9:
        tick = {
            "t_ms": t_ms,
            "quat": _interp_quat_channel(series.quat, t_ms, DEFAULT_QUAT),
            "gyro": _interp_vec3_channel(series.gyro, t_ms, DEFAULT_VEC3),
            "accel": _interp_vec3_channel(series.accel, t_ms, DEFAULT_VEC3),
            "range": {
                "mm": int(round(_interp_scalar_channel(series.range_mm, t_ms, float(DEFAULT_RANGE_MM)))),
            },
            "flow": _flow_in_interval(series.flow, prev_t_ms, t_ms),
        }
        ticks.append(tick)
        prev_t_ms = t_ms
        t_ms += dt_ms

    return ticks


def to_server_message(tick: dict[str, Any], base_us: int) -> dict[str, Any]:
    """Map resampled tick to fusion_server newline-delimited JSON."""
    return {
        "type": "sensor",
        "ts_us": base_us + int(round(tick["t_ms"] * 1000.0)),
        "quat": tick["quat"],
        "gyro": tick["gyro"],
        "accel": tick["accel"],
        "flow": tick["flow"],
        "range": tick["range"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Resample device JSONL and replay to the fusion TCP server.",
    )
    parser.add_argument("capture_file", help="JSONL or JSON array capture file")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="Output tick rate (default 100 Hz, matching simulated stream)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="1.0 = real time, 2.0 = 2x faster, 0 = as fast as possible",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resample and print stats without connecting to the server",
    )
    args = parser.parse_args()

    series = load_capture(args.capture_file)
    ticks = resample_capture(series, args.hz)
    if not ticks:
        print("No ticks generated")
        return

    print(
        f"Loaded capture: quat={len(series.quat)} gyro={len(series.gyro)} "
        f"accel={len(series.accel)} flow={len(series.flow)} range={len(series.range_mm)}"
    )
    print(
        f"Resampled {len(ticks)} ticks at {args.hz:.1f} Hz "
        f"({ticks[0]['t_ms']:.1f}–{ticks[-1]['t_ms']:.1f} ms)"
    )

    if args.dry_run:
        sample = to_server_message(ticks[0], 0)
        print("First server message:", json.dumps(sample, indent=2))
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {args.host}:{args.port} ...")
    sock.connect((args.host, args.port))

    send_line(sock, {"type": "start"})
    print("Start ack:", read_ack(sock))

    base_us = int(time.time() * 1_000_000)
    dt_s = 1.0 / args.hz
    replay_start = time.monotonic()
    first_t_ms = ticks[0]["t_ms"]

    for i, tick in enumerate(ticks):
        if args.speed > 0:
            target_s = (tick["t_ms"] - first_t_ms) / 1000.0 / args.speed
            delay = target_s - (time.monotonic() - replay_start)
            if delay > 0:
                time.sleep(delay)

        msg = to_server_message(tick, base_us)
        send_line(sock, msg)

        if i == 0 or (i + 1) % 50 == 0 or i + 1 == len(ticks):
            print(
                f"sent {i + 1}/{len(ticks)}  t={tick['t_ms']:.0f}ms  "
                f"range={msg['range']['mm']}mm  "
                f"flow=({msg['flow']['dx']},{msg['flow']['dy']})"
            )

    send_line(sock, {"type": "end"})
    print("End ack:", read_ack(sock))
    sock.close()
    print("Done")


if __name__ == "__main__":
    main()
