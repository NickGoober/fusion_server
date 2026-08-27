#!/usr/bin/env python3
"""Find radar spikes and gate behavior in a capture replay."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from position_fusion import PositionFusionEngine
from sensor_stream import SENSOR_FLOW, SENSOR_QUAT, SENSOR_RADAR, payload_array_to_dict

MOUNT = {"w": 0.7071067811865476, "x": -0.7071067811865476, "y": 0.0, "z": 0.0}


def iter_capture(path: Path):
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if isinstance(obj, dict):
                continue
            rows = obj if obj and isinstance(obj[0], list) else [obj]
            for row in rows:
                if not isinstance(row, list) or len(row) < 3:
                    continue
                parsed = payload_array_to_dict(int(row[0]), row[2])
                if parsed is None:
                    continue
                yield int(row[1]), parsed[0], parsed[1]


def replay(path: Path, *, streak: int = 3, target_mm: int | None = None):
    e = PositionFusionEngine(kalman_enable=True, radar_max_reject_streak=streak)
    last_q = None
    last_r: int | None = None
    t0: int | None = None
    prev_fy = None
    radar_rows: list[tuple] = []

    for ts, sensor, data in iter_capture(path):
        if t0 is None:
            t0 = ts
        if sensor == SENSOR_QUAT:
            last_q = {
                "w": float(data["w"]),
                "x": float(data["x"]),
                "y": float(data["y"]),
                "z": float(data["z"]),
            }
        elif sensor == SENSOR_RADAR:
            last_r = int(data["mm"])

        flow = data if sensor == SENSOR_FLOW else None
        pf_before = e.filtered_position()["y"]
        e.update(
            range_mm=last_r,
            flow=flow,
            imu_quat=last_q,
            imu_to_body=MOUNT,
            ts_us=ts,
            radar_update=(sensor == SENSOR_RADAR),
        )
        if sensor == SENSOR_RADAR and last_r is not None:
            sec = (ts - t0) / 1e6
            pf = e.filtered_position()["y"]
            pr = e.raw_position()["y"]
            df = abs(pf - pf_before) if pf_before is not None else 0.0
            radar_rows.append(
                (
                    sec,
                    last_r,
                    pr,
                    pf,
                    df,
                    e.radar_gate.last_reject,
                    e.radar_gate._reject_streak,
                )
            )
            if target_mm is not None and last_r == target_mm:
                print(
                    f"*** target mm={target_mm} t={sec:.3f}s "
                    f"raw_y={pr:.4f} filt_y={pf:.4f} "
                    f"reject={e.radar_gate.last_reject} streak={e.radar_gate._reject_streak}"
                )

    # large filtered jumps
    jumps = [(r[0], r[1], r[4], r[5], r[6]) for r in radar_rows if r[4] > 0.02]
    print(f"\n{path.name} streak={streak}: radar ticks={len(radar_rows)} jumps>20mm={len(jumps)}")
    for sec, mm, df, rej, streak_n in sorted(jumps, key=lambda x: -x[2])[:12]:
        print(f"  t={sec:.3f}s mm={mm:4d} |dfilt|={df*1000:.0f}mm reject={rej} streak={streak_n}")

    # find 151mm occurrences
    mm151 = [r for r in radar_rows if r[1] == 151]
    if mm151:
        print(f"\n151mm occurrences ({len(mm151)}):")
        for r in mm151:
            print(
                f"  t={r[0]:.3f}s raw_y={r[2]:.4f} filt_y={r[3]:.4f} "
                f"|df|={r[4]*1000:.0f}mm reject={r[5]} streak={r[6]}"
            )


if __name__ == "__main__":
    cap = ROOT / "captures" / (sys.argv[1] if len(sys.argv) > 1 else "freeMoveLR_flow_fixed.jsonl")
    streak = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    replay(cap, streak=streak, target_mm=151)
