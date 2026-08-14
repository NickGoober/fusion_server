"""
Offline IMU lever-arm calibration via rigid-body kinematics (MrVS).

Physics (gravity-free linear accel at an off-center IMU, CoR fixed):

    a_lin = ω̇ × r + ω × (ω × r)

Per timestep t, with skew-symmetric [v]_×:

    M_t = [ω̇_t]_× + ([ω_t]_×)²
    a_lin,t = M_t · r

Batch least squares (MrVS, arXiv:2402.04240):  A r = B  →  r = argmin ||A r - B||²
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

# Gates aligned with fusion_lever_arm_cal.c (relaxed for LS which uses all axes).
MIN_OMEGA_RAD_S = 0.08
MIN_ACCEL_MS2 = 0.004
MAX_ACCEL_MS2 = 50.0
MAX_ARM_M = 0.15
MIN_SAMPLES = 20
MAX_RESIDUAL_RMS_MPS = 5.0
GYRO_QUAT_WINDOW_S = 0.35
CROSS_AXIS_MAX_FRAC = 0.35
MAX_PLAUSIBLE_OMEGA_RAD_S = 50.0
STEADY_SPIN_OMEGA_MIN_FRAC = 0.35
STEADY_SPIN_OMEGA_MAX_FRAC = 2.5
OUTLIER_SPIKE_RATIO = 4.0


@dataclass(frozen=True)
class LeverArmCalResult:
    """Calibration output."""

    imu_lever_arm_m: dict[str, float]
    samples_used: int
    samples_rejected: int
    residual_rms_mps: float
    omega_rad_s: float
    detected_axis: str
    success: bool


@dataclass
class _AccumStats:
    """Running normal equations for A r = B (3×3 system)."""

    ata: list[list[float]] = field(
        default_factory=lambda: [[0.0, 0.0, 0.0] for _ in range(3)],
    )
    atb: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    residual_sq_sum: float = 0.0
    omega_abs_sum: float = 0.0
    omega_count: int = 0


def skew(v: Vec3) -> Mat3:
    """Skew-symmetric matrix [v]_× such that [v]_× u = v × u."""
    x, y, z = v
    return (
        (0.0, -z, y),
        (z, 0.0, -x),
        (-y, x, 0.0),
    )


def mat_add(a: Mat3, b: Mat3) -> Mat3:
    return tuple(
        tuple(a[r][c] + b[r][c] for c in range(3))
        for r in range(3)
    )


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    out: list[list[float]] = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return (
        (out[0][0], out[0][1], out[0][2]),
        (out[1][0], out[1][1], out[1][2]),
        (out[2][0], out[2][1], out[2][2]),
    )


def mat_transpose(m: Mat3) -> Mat3:
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def vec_norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def M_matrix(omega: Vec3, omega_dot: Vec3) -> Mat3:
    """
    M_t = [ω̇]_× + ([ω]_×)²  so that  a_lin = M_t · r.
    """
    w_x = skew(omega)
    w_x_sq = mat_mul(w_x, w_x)
    wdot_x = skew(omega_dot)
    return mat_add(wdot_x, w_x_sq)


def solve_3x3(a: Mat3, b: Vec3) -> Vec3 | None:
    """Solve 3×3 linear system via Gaussian elimination with partial pivot."""
    m = [list(a[0]), list(a[1]), list(a[2])]
    rhs = list(b)

    for col in range(3):
        pivot_row = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot_row][col]) < 1e-12:
            return None
        if pivot_row != col:
            m[col], m[pivot_row] = m[pivot_row], m[col]
            rhs[col], rhs[pivot_row] = rhs[pivot_row], rhs[col]
        pivot = m[col][col]
        for r in range(col + 1, 3):
            factor = m[r][col] / pivot
            for c in range(col, 3):
                m[r][c] -= factor * m[col][c]
            rhs[r] -= factor * rhs[col]

    x = [0.0, 0.0, 0.0]
    for row in reversed(range(3)):
        s = rhs[row] - sum(m[row][c] * x[c] for c in range(row + 1, 3))
        if abs(m[row][row]) < 1e-12:
            return None
        x[row] = s / m[row][row]
    return (x[0], x[1], x[2])


def solve_2x2(a: tuple[tuple[float, float], tuple[float, float]], b: tuple[float, float]) -> tuple[float, float] | None:
    """Solve 2×2 linear system."""
    (a00, a01), (a10, a11) = a
    b0, b1 = b
    det = a00 * a11 - a01 * a10
    if abs(det) < 1e-12:
        return None
    return (
        (b0 * a11 - b1 * a01) / det,
        (a00 * b1 - a10 * b0) / det,
    )


def _solve_observable_arm(spin_axis: str, stats: _AccumStats) -> Vec3 | None:
    """Solve for r using only the observable components of a single-axis spin."""
    ata = stats.ata
    atb = stats.atb
    if spin_axis == "x":
        pair = solve_2x2(
            ((ata[1][1], ata[1][2]), (ata[2][1], ata[2][2])),
            (atb[1], atb[2]),
        )
        if pair is None:
            return None
        return (0.0, pair[0], pair[1])
    if spin_axis == "y":
        pair = solve_2x2(
            ((ata[0][0], ata[0][2]), (ata[2][0], ata[2][2])),
            (atb[0], atb[2]),
        )
        if pair is None:
            return None
        return (pair[0], 0.0, pair[1])
    pair = solve_2x2(
        ((ata[0][0], ata[0][1]), (ata[1][0], ata[1][1])),
        (atb[0], atb[1]),
    )
    if pair is None:
        return None
    return (pair[0], pair[1], 0.0)


def _observable_rows(spin_axis: str) -> tuple[int, int]:
    """Acceleration rows used for single-axis spin (r_parallel is unobservable)."""
    if spin_axis == "x":
        return (1, 2)
    if spin_axis == "y":
        return (0, 2)
    return (0, 1)


def _accumulate_sample_axis(
    stats: _AccumStats,
    m: Mat3,
    accel: Vec3,
    spin_axis: str,
    *,
    weight: float = 1.0,
) -> None:
    """Add observable rows of M r = a for a dominant single-axis spin."""
    for row in _observable_rows(spin_axis):
        mr = (m[row][0], m[row][1], m[row][2])
        ai = accel[row]
        for j in range(3):
            stats.atb[j] += weight * mr[j] * ai
            for k in range(3):
                stats.ata[j][k] += weight * mr[j] * mr[k]


class AxisKalmanFilter:
    """
    2-state Kalman filter per gyro axis: state [ω, α] (angular velocity, acceleration).

    Gyro samples update ω; α is used to smooth numerical differentiation.
    """

    def __init__(
        self,
        *,
        process_noise_omega: float = 4.0,
        process_noise_alpha: float = 80.0,
        measurement_noise: float = 0.25,
    ) -> None:
        self._q_omega = process_noise_omega
        self._q_alpha = process_noise_alpha
        self._r = measurement_noise
        self._omega = 0.0
        self._alpha = 0.0
        self._p00 = 1.0
        self._p01 = 0.0
        self._p11 = 1.0
        self._initialized = False

    def reset(self, omega0: float) -> None:
        self._omega = omega0
        self._alpha = 0.0
        self._p00 = 1.0
        self._p01 = 0.0
        self._p11 = 1.0
        self._initialized = True

    def step(self, omega_meas: float, dt_s: float) -> tuple[float, float]:
        if not self._initialized:
            self.reset(omega_meas)
            return self._omega, self._alpha

        # Predict: ω += α·dt, α unchanged
        omega_pred = self._omega + self._alpha * dt_s
        p00 = self._p00 + 2.0 * dt_s * self._p01 + dt_s * dt_s * self._p11 + self._q_omega
        p01 = self._p01 + dt_s * self._p11
        p11 = self._p11 + self._q_alpha

        # Update with gyro measurement of ω
        innov = omega_meas - omega_pred
        s = p00 + self._r
        if s < 1e-12:
            self._omega = omega_pred
            self._p00, self._p01, self._p11 = p00, p01, p11
            return self._omega, self._alpha

        k0 = p00 / s
        k1 = p01 / s
        self._omega = omega_pred + k0 * innov
        self._alpha = self._alpha + k1 * innov
        self._p00 = (1.0 - k0) * p00
        self._p01 = (1.0 - k0) * p01 - k1 * p00
        self._p11 = p11 - k1 * p01
        return self._omega, self._alpha


class GyroKalmanBank:
    """Three independent axis Kalman filters for ω smoothing."""

    def __init__(self) -> None:
        self._filters = [AxisKalmanFilter() for _ in range(3)]

    def reset(self, gyro: Vec3) -> None:
        for i in range(3):
            self._filters[i].reset(gyro[i])

    def step(self, gyro: Vec3, dt_s: float) -> tuple[Vec3, Vec3]:
        omega_f: list[float] = []
        alpha_f: list[float] = []
        for i in range(3):
            w, a = self._filters[i].step(gyro[i], dt_s)
            omega_f.append(w)
            alpha_f.append(a)
        return (omega_f[0], omega_f[1], omega_f[2]), (alpha_f[0], alpha_f[1], alpha_f[2])


def _quat_rotate_vector(v: Vec3, q: dict[str, float]) -> Vec3:
    """Rotate vector v by unit quaternion q (w, x, y, z)."""
    n = math.sqrt(
        q["w"] * q["w"] + q["x"] * q["x"] + q["y"] * q["y"] + q["z"] * q["z"],
    )
    if n < 1e-12:
        return v
    w, x, y, z = q["w"] / n, q["x"] / n, q["y"] / n, q["z"] / n
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _rotation_ok_for_axis(axis: str, omega: Vec3) -> bool:
    """Require dominant spin on one axis (matches fusion_lever_arm_cal.c)."""
    ax, ay, az = abs(omega[0]), abs(omega[1]), abs(omega[2])
    dominant = max(ax, ay, az)
    if dominant < MIN_OMEGA_RAD_S:
        return False
    if dominant == ax:
        cross = max(ay, az)
    elif dominant == ay:
        cross = max(ax, az)
    else:
        cross = max(ax, ay)
    return cross <= dominant * CROSS_AXIS_MAX_FRAC


def _pick_gyro_window_s(series: Any) -> float:
    """Choose quaternion differentiation window for offline steady-spin captures."""
    if not series.quat:
        return GYRO_QUAT_WINDOW_S
    quats = sorted(series.quat, key=lambda s: s.t_ms)
    if len(quats) < 2:
        return GYRO_QUAT_WINDOW_S
    span_s = (quats[-1].t_ms - quats[0].t_ms) / 1000.0
    if span_s <= 0.0:
        return GYRO_QUAT_WINDOW_S
    mean_dt_s = span_s / max(1, len(quats) - 1)
    return max(GYRO_QUAT_WINDOW_S, min(1.0, mean_dt_s * 20.0))


def _smooth_gyro_series(
    gyro_data: Sequence[Vec3],
    timestamps: Sequence[float],
) -> list[Vec3]:
    """Clip quaternion-derived ω; wide quat windows already low-pass the signal."""
    _ = timestamps
    return [_clip_omega_magnitude(g, MAX_PLAUSIBLE_OMEGA_RAD_S) for g in gyro_data]


def _kinematic_accel(omega: Vec3, omega_dot: Vec3, arm: Vec3) -> Vec3:
    """a_lin = ω̇ × r + ω × (ω × r) = M(ω, ω̇) · r."""
    return mat_vec(M_matrix(omega, omega_dot), arm)


def _matrix_frobenius_norm(m: Mat3) -> float:
    return math.sqrt(sum(m[r][c] * m[r][c] for r in range(3) for c in range(3)))


def _median(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    mid = len(s) // 2
    if len(s) % 2:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def _clip_omega_magnitude(omega: Vec3, max_mag: float) -> Vec3:
    mag = vec_norm(omega)
    if mag <= max_mag or mag < 1e-12:
        return omega
    scale = max_mag / mag
    return (omega[0] * scale, omega[1] * scale, omega[2] * scale)


def _plausible_omega_mags(omega_mags: Sequence[float]) -> list[float]:
    return [
        m
        for m in omega_mags
        if MIN_OMEGA_RAD_S <= m <= MAX_PLAUSIBLE_OMEGA_RAD_S
    ]


def _reference_spin_omega(omega_mags: Sequence[float]) -> float:
    """Robust spin-rate reference (median of plausible ω), not max(ω)."""
    plausible = _plausible_omega_mags(omega_mags)
    if not plausible:
        return 0.0
    return _median(plausible)


def _omega_outlier(omega_mag: float, ref_omega: float) -> bool:
    if omega_mag > MAX_PLAUSIBLE_OMEGA_RAD_S:
        return True
    if ref_omega > MIN_OMEGA_RAD_S and omega_mag > ref_omega * OUTLIER_SPIKE_RATIO:
        return True
    return False


def _steady_spin_ok(omega: Vec3, *, ref_omega: float) -> bool:
    """Keep samples near the sustained spin plateau (hand or motor constant-rate)."""
    omega_mag = vec_norm(omega)
    if omega_mag < MIN_OMEGA_RAD_S:
        return False
    if _omega_outlier(omega_mag, ref_omega):
        return False
    if ref_omega <= MIN_OMEGA_RAD_S:
        return True
    return (
        omega_mag >= ref_omega * STEADY_SPIN_OMEGA_MIN_FRAC
        and omega_mag <= ref_omega * STEADY_SPIN_OMEGA_MAX_FRAC
    )


def _detect_spin_axis(omega_samples: Sequence[Vec3]) -> str:
    sx = sy = sz = 0.0
    for wx, wy, wz in omega_samples:
        sx += abs(wx)
        sy += abs(wy)
        sz += abs(wz)
    if sx >= sy and sx >= sz:
        return "x"
    if sy >= sz:
        return "y"
    return "z"


def _mean_omega_magnitude(omega_samples: Sequence[Vec3]) -> float:
    if not omega_samples:
        return 0.0
    return sum(vec_norm(w) for w in omega_samples) / len(omega_samples)


def calibrate_lever_arm(
    accel_data: Sequence[Vec3],
    gyro_data: Sequence[Vec3],
    timestamps: Sequence[float],
    *,
    min_omega_rad_s: float = MIN_OMEGA_RAD_S,
    min_accel_ms2: float = MIN_ACCEL_MS2,
    max_arm_m: float = MAX_ARM_M,
    min_samples: int = MIN_SAMPLES,
    imu_to_body: dict[str, float] | None = None,
    axis: str | None = None,
) -> LeverArmCalResult:
    """
    Estimate lever arm r (meters) from synchronized accel, gyro, and timestamps.

    Uses the MrVS batch least-squares estimator from García-de-Villa et al.
    (arXiv:2402.04240): stack ``a = M(ω, 0) · r`` across steady hand-spin samples
    (constant-rate assumption, ω̇ ≈ 0) and solve ``r = argmin ||A r - B||²``.
    Only the two acceleration axes perpendicular to the detected spin axis are used.

    ``timestamps`` may be seconds or microseconds (auto-detected from magnitude).
    Vectors are rotated into the body frame when ``imu_to_body`` is provided.
    """
    n = min(len(accel_data), len(gyro_data), len(timestamps))
    if n < 2:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": 0.0, "y": 0.0, "z": 0.0},
            samples_used=0,
            samples_rejected=n,
            residual_rms_mps=0.0,
            omega_rad_s=0.0,
            detected_axis="auto",
            success=False,
        )

    ts = list(timestamps[:n])
    if ts[-1] > 1e6:
        ts = [t / 1_000_000.0 for t in ts]

    mount = imu_to_body
    raw_accel = list(accel_data[:n])
    raw_gyro = list(gyro_data[:n])
    if mount is not None:
        raw_gyro = [_quat_rotate_vector(g, mount) for g in raw_gyro]
        raw_accel = [_quat_rotate_vector(a, mount) for a in raw_accel]

    raw_omega_mags = [
        vec_norm(w) for w in raw_gyro if vec_norm(w) >= min_omega_rad_s
    ]
    if not raw_omega_mags:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": 0.0, "y": 0.0, "z": 0.0},
            samples_used=0,
            samples_rejected=n,
            residual_rms_mps=0.0,
            omega_rad_s=0.0,
            detected_axis="auto",
            success=False,
        )

    ref_omega = _reference_spin_omega(raw_omega_mags)
    smoothed = _smooth_gyro_series(raw_gyro, ts)
    spin_axis = (
        axis
        if axis in ("x", "y", "z")
        else _detect_spin_axis(smoothed)
    )

    stats = _AccumStats()
    used_rows: list[tuple[Vec3, Vec3]] = []
    rejected = 0
    zero_alpha: Vec3 = (0.0, 0.0, 0.0)

    for accel, omega, _t_s in zip(raw_accel, smoothed, ts):
        omega_mag = vec_norm(omega)
        accel_mag = vec_norm(accel)
        if omega_mag < min_omega_rad_s:
            rejected += 1
            continue
        if accel_mag < min_accel_ms2 or accel_mag > MAX_ACCEL_MS2:
            rejected += 1
            continue
        if not _rotation_ok_for_axis(spin_axis, omega):
            rejected += 1
            continue
        if _omega_outlier(omega_mag, ref_omega):
            rejected += 1
            continue
        if not _steady_spin_ok(omega, ref_omega=ref_omega):
            rejected += 1
            continue

        m = M_matrix(omega, zero_alpha)
        if _matrix_frobenius_norm(m) < min_omega_rad_s * min_omega_rad_s:
            rejected += 1
            continue

        weight = omega_mag ** 4
        _accumulate_sample_axis(stats, m, accel, spin_axis, weight=weight)
        used_rows.append((omega, accel))

    if len(used_rows) < min_samples:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": 0.0, "y": 0.0, "z": 0.0},
            samples_used=len(used_rows),
            samples_rejected=rejected,
            residual_rms_mps=0.0,
            omega_rad_s=ref_omega,
            detected_axis=spin_axis,
            success=False,
        )

    solved = _solve_observable_arm(spin_axis, stats)
    if solved is None:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": 0.0, "y": 0.0, "z": 0.0},
            samples_used=len(used_rows),
            samples_rejected=rejected,
            residual_rms_mps=0.0,
            omega_rad_s=ref_omega,
            detected_axis=spin_axis,
            success=False,
        )

    r: Vec3 = solved

    arm_mag = vec_norm(r)
    if arm_mag > max_arm_m or arm_mag < 1e-6:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": r[0], "y": r[1], "z": r[2]},
            samples_used=len(used_rows),
            samples_rejected=rejected,
            residual_rms_mps=0.0,
            omega_rad_s=ref_omega,
            detected_axis=spin_axis,
            success=False,
        )

    residual_sq = 0.0
    obs_rows = _observable_rows(spin_axis)
    for omega, accel in used_rows:
        pred = _kinematic_accel(omega, zero_alpha, r)
        for row in obs_rows:
            err = accel[row] - pred[row]
            residual_sq += err * err

    samples_used = len(used_rows)
    residual_rms = math.sqrt(residual_sq / max(1, 2 * samples_used))
    success = residual_rms <= MAX_RESIDUAL_RMS_MPS

    return LeverArmCalResult(
        imu_lever_arm_m={"x": r[0], "y": r[1], "z": r[2]},
        samples_used=samples_used,
        samples_rejected=rejected,
        residual_rms_mps=residual_rms,
        omega_rad_s=_mean_omega_magnitude([row[0] for row in used_rows]),
        detected_axis=spin_axis,
        success=success,
    )


@dataclass
class LeverArmCalSession:
    """Online sample buffer for streaming calibration."""

    _gyro: list[Vec3] = field(default_factory=list)
    _accel: list[Vec3] = field(default_factory=list)
    _ts_s: list[float] = field(default_factory=list)
    _t0_us: int | None = None
    _rejected_feeds: int = 0

    def clear(self) -> None:
        self._gyro.clear()
        self._accel.clear()
        self._ts_s.clear()
        self._t0_us = None
        self._rejected_feeds = 0

    def add_sample(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        *,
        ts_us: int | None = None,
        dt_s: float = 0.01,
    ) -> None:
        if ts_us is not None:
            if self._t0_us is None:
                self._t0_us = ts_us
            t_s = (ts_us - self._t0_us) / 1_000_000.0
        else:
            t_s = self._ts_s[-1] + dt_s if self._ts_s else 0.0

        gyro = (gx, gy, gz)
        accel = (ax, ay, az)
        if vec_norm(gyro) < 1e-6 and vec_norm(accel) < 1e-6:
            self._rejected_feeds += 1
            return

        self._gyro.append(gyro)
        self._accel.append(accel)
        self._ts_s.append(t_s)

    @property
    def samples_buffered(self) -> int:
        return len(self._gyro)

    def finish(self) -> LeverArmCalResult:
        return calibrate_lever_arm(self._accel, self._gyro, self._ts_s)

    def preview(self) -> dict[str, float] | None:
        """Running estimate when enough samples are buffered."""
        if len(self._gyro) < MIN_SAMPLES:
            return None
        result = self.finish()
        if not result.success:
            return None
        return dict(result.imu_lever_arm_m)

    def detected_axis_hint(self) -> str | None:
        if len(self._gyro) < 5:
            return None
        return _detect_spin_axis(self._gyro[-min(80, len(self._gyro)):])


def samples_from_series(
    series: Any,
    *,
    gyro_window_s: float = GYRO_QUAT_WINDOW_S,
) -> tuple[list[Vec3], list[Vec3], list[float]]:
    """
    Pair each accel sample with gyro from a wide quaternion window.

    Avoids resampling/interpolation and uses a longer baseline for ω so
    game-rotation quaternion noise does not masquerade as fast spin.
    """
    from sensor_stream import gyro_from_quat_window

    quats = sorted((s.t_ms, s.value) for s in series.quat)
    accels = sorted((s.t_ms, s.value) for s in series.accel)
    if not quats or not accels:
        return [], [], []

    t0_ms = min(quats[0][0], accels[0][0])
    accel_out: list[Vec3] = []
    gyro_out: list[Vec3] = []
    ts_out: list[float] = []

    for t_ms, a in accels:
        g = gyro_from_quat_window(quats, t_ms, window_s=gyro_window_s)
        accel_out.append((float(a["x"]), float(a["y"]), float(a["z"])))
        gyro_out.append((float(g["x"]), float(g["y"]), float(g["z"])))
        ts_out.append((t_ms - t0_ms) / 1000.0)

    return accel_out, gyro_out, ts_out


def samples_from_capture_ticks(
    ticks: Iterable[dict[str, Any]],
) -> tuple[list[Vec3], list[Vec3], list[float]]:
    """Extract accel/gyro/time from bundled sensor ticks (replay_capture output)."""
    from sensor_stream import gyro_from_quat_pair

    accel: list[Vec3] = []
    gyro: list[Vec3] = []
    ts_s: list[float] = []
    t0_us: int | None = None
    prev_quat: dict[str, float] | None = None
    prev_ts_us: int | None = None

    for tick in ticks:
        a = tick.get("accel")
        if not a:
            continue
        ts_us = int(tick["ts_us"]) if "ts_us" in tick else int(float(tick["t_ms"]) * 1000.0)
        if t0_us is None:
            t0_us = ts_us

        g = tick.get("gyro")
        quat = tick.get("quat")
        gx = gy = gz = 0.0
        if g and (abs(g["x"]) + abs(g["y"]) + abs(g["z"])) > 1e-6:
            gx, gy, gz = float(g["x"]), float(g["y"]), float(g["z"])
        elif quat and prev_quat is not None and prev_ts_us is not None:
            dt_s = (ts_us - prev_ts_us) / 1_000_000.0
            derived = gyro_from_quat_pair(prev_quat, quat, dt_s)
            gx, gy, gz = derived["x"], derived["y"], derived["z"]

        if quat is not None:
            prev_quat = {
                "w": float(quat["w"]),
                "x": float(quat["x"]),
                "y": float(quat["y"]),
                "z": float(quat["z"]),
            }
            prev_ts_us = ts_us

        ts_s.append((ts_us - t0_us) / 1_000_000.0)
        gyro.append((gx, gy, gz))
        accel.append((float(a["x"]), float(a["y"]), float(a["z"])))

    return accel, gyro, ts_s


def calibrate_capture_file(
    path: str,
    *,
    hz: float = 100.0,
    imu_to_body: dict[str, float] | None = None,
) -> LeverArmCalResult:
    """Offline calibration from a fusion recording or sensor stream file."""
    from pathlib import Path

    from device_protocol import unpack_collar_wire_line
    from fusion_calib import imu_to_body_from_calib, load_calib
    from replay_capture import load_capture, resample_capture

    if imu_to_body is None:
        calib = load_calib()
        if calib:
            w, x, y, z = imu_to_body_from_calib(calib)
            if w != 1.0 or x != 0.0 or y != 0.0 or z != 0.0:
                imu_to_body = {"w": w, "x": x, "y": y, "z": z}

    file_path = Path(path)
    text = file_path.read_text(encoding="utf-8")
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]

    if lines and lines[0].startswith('{"_fusion_record"'):
        lines = lines[1:]

    wire_samples: list[Any] = []
    for line in lines:
        if line.startswith("["):
            wire_samples.extend(unpack_collar_wire_line(line))

    if wire_samples:
        series = _wire_samples_to_series(wire_samples)
        gyro_window_s = _pick_gyro_window_s(series)
        accel, gyro, ts_s = samples_from_series(series, gyro_window_s=gyro_window_s)
        return calibrate_lever_arm(
            accel,
            gyro,
            ts_s,
            imu_to_body=imu_to_body,
        )

    try:
        series = load_capture(file_path)
        if series.quat and series.accel:
            gyro_window_s = _pick_gyro_window_s(series)
            accel, gyro, ts_s = samples_from_series(
                series, gyro_window_s=gyro_window_s,
            )
        else:
            ticks = resample_capture(series, hz)
            accel, gyro, ts_s = samples_from_capture_ticks(ticks)
        return calibrate_lever_arm(
            accel,
            gyro,
            ts_s,
            imu_to_body=imu_to_body,
        )
    except (json.JSONDecodeError, KeyError, ValueError):
        pass

    raise ValueError(f"Could not parse capture for lever-arm calibration: {path}")


def _wire_samples_to_series(samples: list[Any]) -> Any:
    from replay_capture import CaptureSeries, Sample
    from sensor_stream import SENSOR_ACCEL, SENSOR_FLOW, SENSOR_QUAT, SENSOR_RADAR

    series = CaptureSeries()
    for s in samples:
        t_ms = s.ts_us / 1000.0
        if s.sensor == SENSOR_QUAT:
            d = s.data
            if "w" not in d:
                continue
            series.quat.append(Sample(t_ms, {
                "w": float(d["w"]),
                "x": float(d["x"]),
                "y": float(d["y"]),
                "z": float(d["z"]),
            }))
        elif s.sensor == SENSOR_ACCEL:
            d = s.data
            series.accel.append(Sample(t_ms, {
                "x": float(d["x"]),
                "y": float(d["y"]),
                "z": float(d["z"]),
            }))
        elif s.sensor == SENSOR_FLOW:
            d = s.data
            series.flow.append(Sample(t_ms, dict(d)))
        elif s.sensor == SENSOR_RADAR:
            d = s.data
            mm = int(d["mm"]) if isinstance(d, dict) else int(d)
            series.range_mm.append(Sample(t_ms, mm))
    return series
