"""Derive gravitySpin_compensated.jsonl from gravitySpin.jsonl.

Extends each wire type 1 sample from [ax, ay, az] to
[ax, ay, az, lx, ly, lz] where linear = specific - gravity(quat).

Gravity in world frame is +X (collar / BNO convention for this capture).
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from bisect import bisect_right
from pathlib import Path

GRAVITY_MAG = 9.81
WORLD_GRAVITY = (GRAVITY_MAG, 0.0, 0.0)
QUANT = 1.0 / 256.0


def quantize(v: float) -> float:
    return round(v / QUANT) * QUANT


def expand_batch(batch: list) -> list:
    if not batch:
        return []
    if isinstance(batch[0], list):
        return batch
    if len(batch) % 3 == 0:
        return [batch[i : i + 3] for i in range(0, len(batch), 3)]
    return [batch]


def rotate_vec(qx: float, qy: float, qz: float, qw: float, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz
    return (
        ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy),
        iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz),
        iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx),
    )


def gravity_body_from_quat(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    return rotate_vec(q[0], q[1], q[2], q[3], *WORLD_GRAVITY)


def nearest_quat(quat_ts: list[int], quat_data: list[tuple[float, float, float, float]], ts: int) -> tuple[float, float, float, float]:
    idx = bisect_right(quat_ts, ts) - 1
    return quat_data[max(0, idx)]


def compensate_file(src: Path, dst: Path) -> int:
    lines = src.read_text(encoding="utf-8").splitlines()
    out_lines: list[str] = []
    quat_ts: list[int] = []
    quat_data: list[tuple[float, float, float, float]] = []
    type1_count = 0

    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("{"):
            meta = json.loads(stripped)
            meta["source"] = src.name
            meta["derived_linear"] = (
                "type1 payload [ax,ay,az,lx,ly,lz]; linear = specific - gravity(quat, world +X)"
            )
            out_lines.append(json.dumps(meta, separators=(",", ":")))
            continue

        batch = json.loads(stripped)
        rows = expand_batch(batch)
        new_rows: list = []
        for row in rows:
            if not isinstance(row, list) or len(row) < 3:
                new_rows.append(row)
                continue
            try:
                sensor = int(row[0])
                ts = int(row[1])
                payload = row[2]
            except (TypeError, ValueError):
                new_rows.append(row)
                continue
            if not isinstance(payload, list):
                new_rows.append(row)
                continue

            if sensor == 0 and len(payload) >= 4:
                quat_ts.append(ts)
                quat_data.append(tuple(float(payload[i]) for i in range(4)))

            if sensor == 1 and len(payload) >= 3:
                specific = [float(payload[0]), float(payload[1]), float(payload[2])]
                g = gravity_body_from_quat(nearest_quat(quat_ts, quat_data, ts))
                linear = [
                    quantize(specific[0] - g[0]),
                    quantize(specific[1] - g[1]),
                    quantize(specific[2] - g[2]),
                ]
                accel = [quantize(v) for v in specific]
                new_rows.append([1, ts, accel + linear])
                type1_count += 1
                continue

            new_rows.append(row)

        out_lines.append(json.dumps(new_rows, separators=(",", ":")))

    dst.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
    return type1_count


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    src = root / "gravitySpin.jsonl"
    dst = root / "gravitySpin_compensated.jsonl"
    if len(sys.argv) >= 2:
        src = Path(sys.argv[1])
    if len(sys.argv) >= 3:
        dst = Path(sys.argv[2])

    count = compensate_file(src, dst)
    print(f"Wrote {dst} ({count} type-1 samples extended to 6 floats)")

    mags: list[float] = []
    rest: list[float] = []
    for line in dst.read_text(encoding="utf-8").splitlines()[1:]:
        if line.startswith("{"):
            continue
        for row in json.loads(line):
            if int(row[0]) == 1 and len(row[2]) >= 6:
                lin = row[2][3:6]
                mag = math.sqrt(sum(v * v for v in lin))
                mags.append(mag)
                if int(row[1]) < 155_090_000:
                    rest.append(mag)
    if mags:
        print(
            f"linear |a| median={statistics.median(mags):.3f} m/s², "
            f"rest mean={sum(rest) / len(rest):.3f} m/s²"
        )


if __name__ == "__main__":
    main()
