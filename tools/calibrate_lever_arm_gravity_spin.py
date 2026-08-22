#!/usr/bin/env python3
"""Estimate IMU lever arm from gravitySpin.jsonl centripetal linear acceleration."""

from __future__ import annotations

import json
import statistics as stats
import sys
from bisect import bisect_right
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lever_arm_config import IMU_LEVER_ARM_M, OMEGA_DOT_MAX_RAD_S2
from sensor_stream import (
    QUAT_DEDUP_DOT_THRESHOLD,
    QUAT_GYRO_WINDOW_S,
    _quat_dot,
    _quat_normalize,
    gyro_from_quat_pair,
    gyro_from_quat_window,
)
from tools.compensate_gravity_spin import expand_batch, gravity_body_from_quat
from tools.lever_arm_calib_arzberger import (
    CalibSample,
    GRAVITY_MAG,
    estimate_lever_arm,
    gravity_magnitude_residuals,
    motion_accel,
)

MEASURED = np.array([IMU_LEVER_ARM_M["x"], IMU_LEVER_ARM_M["y"], IMU_LEVER_ARM_M["z"]])
MIN_OMEGA = 0.5
SPIN_BURST_TS = 155_095_000  # default for gravitySpin.jsonl
BURST_MIN_OMEGA_RAD_S = 3.0


def detect_spin_burst_ts(
    quats: list,
    *,
    use_window: bool = True,
    min_omega: float = BURST_MIN_OMEGA_RAD_S,
) -> int:
    """First timestamp where windowed |omega| exceeds min_omega, else 0."""
    omega_series = build_omega_series(quats, use_window=use_window)
    for ts, omega in omega_series:
        if float(np.linalg.norm(omega)) >= min_omega:
            return ts
    return 0


def skew(w: np.ndarray) -> np.ndarray:
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]])


def kinematic_matrix(omega: np.ndarray, omega_dot: np.ndarray) -> np.ndarray:
    return skew(omega) @ skew(omega) + skew(omega_dot)


def load_samples(path: Path) -> tuple[list, list]:
    quats: list[tuple[int, dict[str, float]]] = []
    accels: list[tuple[int, np.ndarray]] = []

    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("{"):
            continue
        for row in expand_batch(json.loads(line)):
            if not isinstance(row, list) or len(row) < 3:
                continue
            sensor = int(row[0])
            ts = int(row[1])
            payload = row[2]
            if sensor == 0 and isinstance(payload, list) and len(payload) >= 4:
                quats.append((
                    ts,
                    {
                        "x": float(payload[0]),
                        "y": float(payload[1]),
                        "z": float(payload[2]),
                        "w": float(payload[3]),
                    },
                ))
            elif sensor == 1 and isinstance(payload, list) and len(payload) >= 3:
                accels.append((
                    ts,
                    np.array([float(payload[0]), float(payload[1]), float(payload[2])]),
                ))

    quats.sort(key=lambda item: item[0])
    accels.sort(key=lambda item: item[0])
    return quats, accels


def dedupe_quats(quats: list[tuple[int, dict[str, float]]]) -> list[tuple[int, dict[str, float]]]:
    """Collapse frozen game-rotation duplicates (same as SensorStreamBuffer ingest)."""
    out: list[tuple[int, dict[str, float]]] = []
    for ts, q in quats:
        qn = _quat_normalize(q)
        if out and _quat_dot(qn, out[-1][1]) >= QUAT_DEDUP_DOT_THRESHOLD:
            out[-1] = (ts, out[-1][1])
        else:
            out.append((ts, qn))
    return out


def build_omega_series(
    quats: list[tuple[int, dict[str, float]]],
    *,
    use_window: bool,
) -> list[tuple[int, np.ndarray]]:
    if use_window:
        series_ms = [(ts / 1000.0, q) for ts, q in quats]
        out: list[tuple[int, np.ndarray]] = []
        for ts, _q in quats:
            g = gyro_from_quat_window(series_ms, ts / 1000.0, window_s=QUAT_GYRO_WINDOW_S)
            out.append((ts, np.array([g["x"], g["y"], g["z"]])))
        return out

    omega_series: list[tuple[int, np.ndarray]] = []
    for i in range(1, len(quats)):
        ts0, q0 = quats[i - 1]
        ts1, q1 = quats[i]
        dt = (ts1 - ts0) / 1_000_000.0
        if 0.0 < dt < 0.1:
            g = gyro_from_quat_pair(q0, q1, dt)
            omega_series.append((ts1, np.array([g["x"], g["y"], g["z"]])))
    return omega_series


def build_spin_samples(
    quats,
    accels,
    *,
    use_window: bool = False,
) -> list[dict]:
    omega_series = build_omega_series(quats, use_window=use_window)

    quat_ts = [t for t, _ in quats]
    omega_ts = [t for t, _ in omega_series]
    samples: list[dict] = []

    for ts_a, spec in accels:
        qi = max(0, bisect_right(quat_ts, ts_a) - 1)
        _, q = quats[qi]
        linear = spec - np.array(gravity_body_from_quat((q["x"], q["y"], q["z"], q["w"])))

        oi = max(0, bisect_right(omega_ts, ts_a) - 1)
        ts_o, omega = omega_series[oi]
        if abs(ts_a - ts_o) > 50_000:
            continue
        om = float(np.linalg.norm(omega))
        if om < MIN_OMEGA:
            continue

        if 0 < oi < len(omega_series) - 1:
            om_prev = omega_series[oi - 1][1]
            om_next = omega_series[oi + 1][1]
            dt2 = (omega_series[oi + 1][0] - omega_series[oi - 1][0]) / 1_000_000.0
            omega_dot = (om_next - om_prev) / dt2 if dt2 > 1e-6 else np.zeros(3)
        else:
            omega_dot = np.zeros(3)

        samples.append({
            "ts": ts_a,
            "linear": linear,
            "omega": omega,
            "omega_dot": omega_dot,
            "alpha": float(np.linalg.norm(omega_dot)),
            "om": om,
        })

    return samples


def build_paper_samples(
    quats,
    accels,
    *,
    use_window: bool = False,
) -> list[CalibSample]:
    """Paper method: raw type-1 accel + quat-derived omega (DoG omega_dot applied later)."""
    omega_series = build_omega_series(quats, use_window=use_window)
    quat_ts = [t for t, _ in quats]
    omega_ts = [t for t, _ in omega_series]
    samples: list[CalibSample] = []

    for ts_a, spec in accels:
        oi = max(0, bisect_right(omega_ts, ts_a) - 1)
        ts_o, omega = omega_series[oi]
        if abs(ts_a - ts_o) > 50_000:
            continue
        om = float(np.linalg.norm(omega))
        if om < MIN_OMEGA:
            continue

        if 0 < oi < len(omega_series) - 1:
            om_prev = omega_series[oi - 1][1]
            om_next = omega_series[oi + 1][1]
            dt2 = (omega_series[oi + 1][0] - omega_series[oi - 1][0]) / 1_000_000.0
            omega_dot = (om_next - om_prev) / dt2 if dt2 > 1e-6 else np.zeros(3)
        else:
            omega_dot = np.zeros(3)

        samples.append(CalibSample(
            ts=ts_a,
            accel=spec.copy(),
            omega=omega,
            omega_dot=omega_dot,
        ))

    return samples


def report_paper(
    label: str,
    data: list[CalibSample],
    *,
    single_axis_z: bool = True,
    min_omega: float = 0.0,
) -> None:
    try:
        r, info = estimate_lever_arm(
            data,
            r0=np.zeros(3),
            single_axis_z=single_axis_z,
            min_omega_rad_s=min_omega,
        )
    except ValueError as exc:
        print(f"--- {label}: {exc} ---")
        return

    err = r - MEASURED
    res = gravity_magnitude_residuals(r, data)
    comp_mags = [
        float(np.linalg.norm(motion_accel(s.omega, s.omega_dot, r) - s.accel))
        for s in data
    ]

    omega_note = f" min_omega>={min_omega}" if min_omega > 0 else ""
    print(f"--- {label}{omega_note} (n={info.get('n_samples', len(data))}) ---")
    print(
        f"  estimated r (mm): x={r[0] * 1000:.2f}  y={r[1] * 1000:.2f}  z={r[2] * 1000:.2f}"
    )
    print(
        f"  measured r (mm):  x={MEASURED[0] * 1000:.2f}  y={MEASURED[1] * 1000:.2f}  "
        f"z={MEASURED[2] * 1000:.2f}"
    )
    print(
        f"  delta (mm):       dx={err[0] * 1000:.2f}  dy={err[1] * 1000:.2f}  "
        f"dz={err[2] * 1000:.2f}  |d|={np.linalg.norm(err) * 1000:.2f}"
    )
    print(
        f"  |g| after compensate: median={np.median(comp_mags):.3f}  "
        f"p90={np.percentile(comp_mags, 90):.3f}  target={GRAVITY_MAG:.2f} m/s^2"
    )
    print(
        f"  Eq.(15) residual: median={np.median(res):.3f}  max={res.max():.3f}  "
        f"LM iters={info['iterations']} converged={info['converged']}"
    )
    if "spin_axis" in info:
        ax = info["spin_axis"]
        print(f"  detected spin axis: [{ax[0]:.3f}, {ax[1]:.3f}, {ax[2]:.3f}]")


def estimate_arm(data: list[dict], *, centripetal_only: bool = False) -> np.ndarray:
    rows_a: list[np.ndarray] = []
    rows_b: list[np.ndarray] = []
    for s in data:
        omega_dot = np.zeros(3) if centripetal_only else s["omega_dot"]
        rows_a.append(kinematic_matrix(s["omega"], omega_dot))
        rows_b.append(s["linear"])
    a_stack = np.vstack(rows_a)
    b_stack = np.concatenate(rows_b)
    result, *_ = np.linalg.lstsq(a_stack, b_stack, rcond=None)
    return result


def report(label: str, data: list[dict], *, centripetal_only: bool = False) -> None:
    if len(data) < 10:
        print(f"--- {label}: too few samples ({len(data)}) ---")
        return

    r = estimate_arm(data, centripetal_only=centripetal_only)
    err = r - MEASURED
    pred = np.array([
        kinematic_matrix(s["omega"], np.zeros(3) if centripetal_only else s["omega_dot"]) @ MEASURED
        for s in data
    ])
    obs = np.array([s["linear"] for s in data])
    res_mag = np.linalg.norm(obs - pred, axis=1)

    print(f"--- {label} (n={len(data)}) ---")
    print(
        f"  estimated r (mm): x={r[0] * 1000:.2f}  y={r[1] * 1000:.2f}  z={r[2] * 1000:.2f}"
    )
    print(
        f"  measured r (mm):  x={MEASURED[0] * 1000:.2f}  y={MEASURED[1] * 1000:.2f}  "
        f"z={MEASURED[2] * 1000:.2f}"
    )
    print(
        f"  delta (mm):       dx={err[0] * 1000:.2f}  dy={err[1] * 1000:.2f}  "
        f"dz={err[2] * 1000:.2f}  |d|={np.linalg.norm(err) * 1000:.2f}"
    )
    print(
        f"  residual |a - A*r_meas|: median={np.median(res_mag):.2f}  "
        f"p90={np.percentile(res_mag, 90):.2f}  max={res_mag.max():.2f} m/s^2"
    )


def diagnose(path: Path, *, burst_ts: int) -> None:
    quats, accels = load_samples(path)
    rows: list[tuple] = []

    for i, (ts, spec) in enumerate(accels):
        qi = max(0, bisect_right([t for t, _ in quats], ts) - 1)
        q = quats[qi][1]
        g = np.array(gravity_body_from_quat((q["x"], q["y"], q["z"], q["w"])))
        lin = spec - g

        if qi > 0:
            t0, q0 = quats[qi - 1]
            t1, q1 = quats[qi]
            dt = (t1 - t0) / 1_000_000.0
            if 0.0 < dt < 0.1:
                w = gyro_from_quat_pair(
                    {"x": q0["x"], "y": q0["y"], "z": q0["z"], "w": q0["w"]},
                    {"x": q1["x"], "y": q1["y"], "z": q1["z"], "w": q1["w"]},
                    dt,
                )
                omega = np.array([w["x"], w["y"], w["z"]])
            else:
                omega = np.zeros(3)
        else:
            omega = np.zeros(3)

        rows.append((ts, lin, omega, float(np.linalg.norm(lin)), float(np.linalg.norm(omega)), float(np.linalg.norm(spec))))

    burst = [r for r in rows if r[0] >= burst_ts]
    print(f"burst samples (ts >= {burst_ts}): {len(burst)}")
    print(f"  |specific| median={np.median([r[5] for r in burst]):.2f}  max={max(r[5] for r in burst):.2f}")
    print(f"  |linear|   median={np.median([r[3] for r in burst]):.2f}  max={max(r[3] for r in burst):.2f}")
    print(
        f"  |omega|    median={np.median([r[4] for r in burst]):.2f}  "
        f"p90={np.percentile([r[4] for r in burst], 90):.2f}"
    )

    subset = [r for r in burst if 0.5 <= r[4] <= 3.0]
    print(f"  omega in [0.5, 3] rad/s: {len(subset)} samples")
    if subset:
        res = []
        for _ts, lin, omega, _lm, _om, _sm in subset:
            wcr = np.cross(omega, np.cross(omega, MEASURED))
            res.append(float(np.linalg.norm(lin - wcr)))
        print(
            f"  |lin - omega x (omega x r_meas)| median={np.median(res):.2f}  "
            f"p90={np.percentile(res, 90):.2f} m/s^2"
        )

    gonly = [r for r in burst if 8.0 < r[5] < 12.0]
    if gonly:
        print(
            f"  near-1g |specific| in burst: {len(gonly)}  "
            f"|linear| median={np.median([r[3] for r in gonly]):.2f}"
        )

    hi_lin_lo_om = [r for r in burst if r[3] > 5.0 and r[4] < 2.0]
    print(f"  high |linear| (>5) with low |omega| (<2): {len(hi_lin_lo_om)} samples")


def run_analysis(
    path: Path,
    quats: list,
    accels: list,
    label: str,
    *,
    burst_ts: int,
    use_window: bool = False,
) -> None:
    samples = build_spin_samples(quats, accels, use_window=use_window)
    burst = [s for s in samples if s["ts"] >= burst_ts]
    steady = [s for s in burst if s["alpha"] < OMEGA_DOT_MAX_RAD_S2]
    print(f"=== {label} ===")
    print(
        f"quats={len(quats)}  spin samples={len(samples)}  burst={len(burst)}  "
        f"steady_burst={len(steady)}"
    )
    if burst:
        om = np.array([s["om"] for s in burst])
        lin = np.linalg.norm(np.array([s["linear"] for s in burst]), axis=1)
        print(
            f"  |omega| median={np.median(om):.2f} p90={np.percentile(om, 90):.2f}  "
            f"max={om.max():.2f} rad/s"
        )
        print(f"  |linear| median={np.median(lin):.2f} max={lin.max():.2f} m/s^2")
        wcr = [
            float(np.linalg.norm(s["linear"] - np.cross(s["omega"], np.cross(s["omega"], MEASURED))))
            for s in burst
        ]
        print(
            f"  |lin - w x (w x r_meas)| median={np.median(wcr):.2f}  "
            f"p90={np.percentile(wcr, 90):.2f} m/s^2"
        )
    print()
    report(f"{label} / all spin", samples)
    report(f"{label} / steady burst", steady, centripetal_only=True)
    print()


def run_paper_analysis(
    quats: list,
    accels: list,
    label: str,
    *,
    burst_ts: int,
    use_window: bool = False,
) -> None:
    paper_samples = build_paper_samples(quats, accels, use_window=use_window)
    burst = [s for s in paper_samples if s.ts >= burst_ts]
    steady = [
        s for s in burst
        if float(np.linalg.norm(s.omega_dot)) < OMEGA_DOT_MAX_RAD_S2
    ]
    print(f"=== PAPER (Arzberger 2607.25784) — {label} ===")
    print(
        f"quats={len(quats)}  spin samples={len(paper_samples)}  burst={len(burst)}  "
        f"steady_burst={len(steady)}"
    )
    if burst:
        om = np.array([float(np.linalg.norm(s.omega)) for s in burst])
        acc_mag = np.array([float(np.linalg.norm(s.accel)) for s in burst])
        print(
            f"  |omega| median={np.median(om):.2f} p90={np.percentile(om, 90):.2f}  "
            f"max={om.max():.2f} rad/s"
        )
        print(
            f"  |accel| median={np.median(acc_mag):.2f} max={acc_mag.max():.2f} m/s^2"
        )
    print()
    report_paper(f"{label} / all spin (rx,ry; rz=0)", paper_samples)
    for thr in (3.0, 5.0, 10.0):
        report_paper(f"{label} / high-spin only", paper_samples, min_omega=thr)
    report_paper(f"{label} / steady burst (rx,ry; rz=0)", steady)
    print()


def run_file(path: Path) -> None:
    quats_raw, accels = load_samples(path)
    quats_deduped = dedupe_quats(quats_raw)
    burst_ts = detect_spin_burst_ts(quats_deduped, use_window=True)
    if burst_ts == 0:
        burst_ts = detect_spin_burst_ts(quats_deduped, use_window=False)
    if burst_ts == 0:
        burst_ts = SPIN_BURST_TS if path.name == "gravitySpin.jsonl" else 0

    print(f"Loaded {path.name}: quats_raw={len(quats_raw)} quats_deduped={len(quats_deduped)} type1={len(accels)}")
    print(f"Measured lever arm (mm): x={MEASURED[0]*1000:.2f} y={MEASURED[1]*1000:.2f} z={MEASURED[2]*1000:.2f}")
    print(f"Spin burst ts: {burst_ts}")
    print()
    diagnose(path, burst_ts=burst_ts)
    print()

    run_analysis(path, quats_deduped, accels, "DEDUPED quats, window omega", burst_ts=burst_ts, use_window=True)
    run_analysis(path, quats_deduped, accels, "DEDUPED quats, pair omega", burst_ts=burst_ts, use_window=False)

    print("=" * 72)
    print("Paper-based calibration (gravity magnitude, Eq. 15 + DoG omega_dot)")
    print("=" * 72)
    print()
    run_paper_analysis(quats_deduped, accels, "window omega", burst_ts=burst_ts, use_window=True)
    print()


def main() -> None:
    import argparse

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Estimate IMU lever arm from spin captures")
    parser.add_argument(
        "files",
        nargs="*",
        type=Path,
        help="jsonl capture(s); default gravitySpin.jsonl",
    )
    args = parser.parse_args()
    paths = args.files if args.files else [root / "gravitySpin.jsonl"]
    for path in paths:
        if not path.is_absolute():
            path = root / path
        if not path.exists():
            print(f"SKIP missing: {path}")
            continue
        print("#" * 72)
        print(f"# {path.name}")
        print("#" * 72)
        run_file(path)
        print()


if __name__ == "__main__":
    main()
