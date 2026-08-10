"""
Offline IMU lever-arm calibration via rigid-body kinematics.

Physics (gravity-free linear accel at an off-center IMU, CoR fixed):

    a_lin = ω̇ × r + ω × (ω × r)

Per timestep t, with skew-symmetric [v]_×:

    M_t = [ω̇_t]_× + ([ω_t]_×)²
    a_lin,t = M_t · r

Stacked least squares:  A r = B  →  r = argmin ||A r - B||²
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
MAX_ALPHA_RAD_S2 = 12.0
MAX_RESIDUAL_RMS_MPS = 0.15
MAX_RESIDUAL_RMS_ROCKING_MPS = 12.0
GYRO_QUAT_WINDOW_S = 0.2
GYRO_QUAT_WINDOW_ROCKING_S = 0.2
ROCKING_OMEGA_CV_MIN = 0.35
ROCKING_PEAK_OMEGA_FRAC = 0.65
ROCKING_CROSS_AXIS_MAX_FRAC = 0.65
ROCKING_MIN_ARM_M = 0.002
CROSS_AXIS_MAX_FRAC = 0.35


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


def _accumulate_sample(stats: _AccumStats, m: Mat3, accel: Vec3) -> None:
    """Add one row triple to normal equations: (M^T M) r = M^T a."""
    mt = mat_transpose(m)
    mt_m = mat_mul(mt, m)
    mt_a = mat_vec(mt, accel)
    for i in range(3):
        for j in range(3):
            stats.ata[i][j] += mt_m[i][j]
        stats.atb[i] += mt_a[i]


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


def _signed_omega(axis: str, omega: Vec3) -> float:
    if axis == "x":
        return omega[0]
    if axis == "y":
        return omega[1]
    return omega[2]


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


def _rotation_ok_for_axis(axis: str, omega: Vec3, *, rocking: bool = False) -> bool:
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
    limit = ROCKING_CROSS_AXIS_MAX_FRAC if rocking else CROSS_AXIS_MAX_FRAC
    return cross <= dominant * limit


def _median(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def _trimmed_median(values: Sequence[float], *, max_abs: float) -> float:
    """
    Robust median preferring physically plausible lever-arm components.

    Uses in-range values when enough exist; otherwise the middle half of
    samples sorted by component magnitude (drops blow-ups from hand motion).
    """
    if not values:
        return 0.0
    in_range = [v for v in values if abs(v) <= max_abs]
    if len(in_range) >= 5:
        return _median(in_range)
    ordered = sorted(values, key=abs)
    mid = ordered[len(ordered) // 4: 3 * len(ordered) // 4]
    if mid:
        return _median(mid)
    return _median(ordered)


def _robust_axis_arm(
    spin_axis: str,
    sample_rows: Sequence[tuple[float, Vec3, Vec3, Vec3]],
    *,
    max_arm_m: float,
) -> Vec3:
    if spin_axis == "x":
        return (
            0.0,
            _trimmed_median([row[3][1] for row in sample_rows], max_abs=max_arm_m),
            _trimmed_median([row[3][2] for row in sample_rows], max_abs=max_arm_m),
        )
    if spin_axis == "y":
        return (
            _trimmed_median([row[3][0] for row in sample_rows], max_abs=max_arm_m),
            0.0,
            _trimmed_median([row[3][2] for row in sample_rows], max_abs=max_arm_m),
        )
    return (
        _trimmed_median([row[3][0] for row in sample_rows], max_abs=max_arm_m),
        _trimmed_median([row[3][1] for row in sample_rows], max_abs=max_arm_m),
        0.0,
    )


def _infer_rocking_motion(omega_samples: Sequence[Vec3]) -> bool:
    """
    Detect oscillating spin (rocking) vs near-constant rotation.

    Rocking produces a high coefficient of variation in |ω|; steady bar spin
    does not.
    """
    mags = [vec_norm(w) for w in omega_samples if vec_norm(w) > 0.02]
    if len(mags) < 20:
        return False
    mean_mag = sum(mags) / len(mags)
    if mean_mag < 1e-6:
        return False
    variance = sum((m - mean_mag) ** 2 for m in mags) / len(mags)
    return math.sqrt(variance) / mean_mag >= ROCKING_OMEGA_CV_MIN


def _pick_gyro_window_s(
    series: Any,
    *,
    rocking: bool,
) -> float:
    """Choose quaternion differentiation window for offline captures."""
    if not rocking:
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

    # Rocking: short windows track oscillation; pick the one with the most
    # usable ω samples so vigorous hand motion does not zero out the fit.
    best_window = GYRO_QUAT_WINDOW_ROCKING_S
    best_count = -1
    for window_s in (0.15, 0.2, 0.35, 0.5):
        accel, gyro, _ = samples_from_series(series, gyro_window_s=window_s)
        count = sum(1 for g in gyro if vec_norm(g) >= MIN_OMEGA_RAD_S)
        if count > best_count:
            best_count = count
            best_window = window_s
    return best_window


def _estimate_arm_from_centripetal(
    axis: str,
    omega: float,
    accel: Vec3,
) -> Vec3 | None:
    """
    Per-sample lever arm from a ≈ ω²·r⊥ (fusion_lever_arm_cal.c).

    For rotation about one axis, only the two perpendicular components of r
    are observable; the parallel component is left at zero.
    """
    ax, ay, az = accel
    if abs(omega) < MIN_OMEGA_RAD_S:
        return None
    if not all(math.isfinite(v) for v in accel):
        return None
    if max(abs(ax), abs(ay), abs(az)) > MAX_ACCEL_MS2:
        return None

    inv_omega2 = 1.0 / (omega * omega)
    if axis == "x":
        if math.hypot(ay, az) < MIN_ACCEL_MS2:
            return None
        return (0.0, -ay * inv_omega2, -az * inv_omega2)
    if axis == "y":
        if math.hypot(ax, az) < MIN_ACCEL_MS2:
            return None
        return (-ax * inv_omega2, 0.0, -az * inv_omega2)
    if math.hypot(ax, ay) < MIN_ACCEL_MS2:
        return None
    return (-ax * inv_omega2, -ay * inv_omega2, 0.0)


def _arm_component_valid(meters: float) -> bool:
    return math.isfinite(meters) and abs(meters) <= MAX_ARM_M


@dataclass
class _AxisArmStats:
    sum_rx: float = 0.0
    sum_ry: float = 0.0
    sum_rz: float = 0.0
    w_rx: float = 0.0
    w_ry: float = 0.0
    w_rz: float = 0.0
    count: int = 0
    omega_abs_sum: float = 0.0

    def add(self, axis: str, arm: Vec3, weight: float, omega_mag: float) -> None:
        if axis == "x":
            self.sum_ry += arm[1] * weight
            self.sum_rz += arm[2] * weight
            self.w_ry += weight
            self.w_rz += weight
        elif axis == "y":
            self.sum_rx += arm[0] * weight
            self.sum_rz += arm[2] * weight
            self.w_rx += weight
            self.w_rz += weight
        else:
            self.sum_rx += arm[0] * weight
            self.sum_ry += arm[1] * weight
            self.w_rx += weight
            self.w_ry += weight
        self.count += 1
        self.omega_abs_sum += omega_mag

    def result(self, axis: str) -> Vec3:
        rx = self.sum_rx / self.w_rx if self.w_rx > 0.0 else 0.0
        ry = self.sum_ry / self.w_ry if self.w_ry > 0.0 else 0.0
        rz = self.sum_rz / self.w_rz if self.w_rz > 0.0 else 0.0
        if axis == "x":
            return (0.0, ry, rz)
        if axis == "y":
            return (rx, 0.0, rz)
        return (rx, ry, 0.0)


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


def _centripetal_accel(omega: Vec3, arm: Vec3) -> Vec3:
    """ω × (ω × r) for residual checks."""
    wx, wy, wz = omega
    rx, ry, rz = arm
    cx = wy * rz - wz * ry
    cy = wz * rx - wx * rz
    cz = wx * ry - wy * rx
    return (
        wy * cz - wz * cy,
        wz * cx - wx * cz,
        wx * cy - wy * cx,
    )


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
    rocking: bool | None = None,
) -> LeverArmCalResult:
    """
    Estimate lever arm r (meters) from synchronized accel, gyro, and timestamps.

    Uses the same single-axis centripetal model as ``fusion_lever_arm_cal.c``:
    for rotation about one body axis, ``r⊥ = -a⊥ / ω²``. The component of ``r``
    along the spin axis is not observable and is set to zero.

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
    candidates_omega: list[Vec3] = []
    candidates_accel: list[Vec3] = []
    rejected = 0

    for i in range(n):
        omega = gyro_data[i]
        accel = accel_data[i]
        if mount is not None:
            omega = _quat_rotate_vector(omega, mount)
            accel = _quat_rotate_vector(accel, mount)

        omega_mag = vec_norm(omega)
        accel_mag = vec_norm(accel)
        if omega_mag < min_omega_rad_s:
            rejected += 1
            continue
        if accel_mag < min_accel_ms2 or accel_mag > MAX_ACCEL_MS2:
            rejected += 1
            continue

        candidates_omega.append(omega)
        candidates_accel.append(accel)

    rocking_mode = (
        rocking
        if rocking is not None
        else _infer_rocking_motion(candidates_omega)
    )

    if len(candidates_omega) < min_samples:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": 0.0, "y": 0.0, "z": 0.0},
            samples_used=len(candidates_omega),
            samples_rejected=rejected + (n - len(candidates_omega)),
            residual_rms_mps=0.0,
            omega_rad_s=_mean_omega_magnitude(candidates_omega),
            detected_axis=_detect_spin_axis(candidates_omega)
            if candidates_omega
            else "auto",
            success=False,
        )

    spin_axis = axis if axis in ("x", "y", "z") else _detect_spin_axis(
        candidates_omega,
    )

    sample_rows: list[tuple[float, Vec3, Vec3, Vec3]] = []

    for omega, accel in zip(candidates_omega, candidates_accel):
        if not _rotation_ok_for_axis(spin_axis, omega, rocking=rocking_mode):
            rejected += 1
            continue
        signed_omega = _signed_omega(spin_axis, omega)
        arm = _estimate_arm_from_centripetal(spin_axis, signed_omega, accel)
        if arm is None:
            rejected += 1
            continue
        if not rocking_mode:
            if spin_axis == "x":
                if not _arm_component_valid(arm[1]) or not _arm_component_valid(arm[2]):
                    rejected += 1
                    continue
            elif spin_axis == "y":
                if not _arm_component_valid(arm[0]) or not _arm_component_valid(arm[2]):
                    rejected += 1
                    continue
            elif not _arm_component_valid(arm[0]) or not _arm_component_valid(arm[1]):
                rejected += 1
                continue
        sample_rows.append((abs(signed_omega), omega, accel, arm))

    if rocking_mode and len(sample_rows) > min_samples:
        sample_rows.sort(key=lambda row: row[0], reverse=True)
        keep = max(min_samples, int(len(sample_rows) * ROCKING_PEAK_OMEGA_FRAC))
        sample_rows = sample_rows[:keep]

    if len(sample_rows) < min_samples:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": 0.0, "y": 0.0, "z": 0.0},
            samples_used=len(sample_rows),
            samples_rejected=rejected,
            residual_rms_mps=0.0,
            omega_rad_s=_mean_omega_magnitude([row[1] for row in sample_rows]),
            detected_axis=spin_axis,
            success=False,
        )

    if rocking_mode:
        r = _robust_axis_arm(spin_axis, sample_rows, max_arm_m=max_arm_m)
        used_omega = [row[1] for row in sample_rows]
        used_accel = [row[2] for row in sample_rows]
        samples_used = len(sample_rows)
    else:
        stats = _AxisArmStats()
        used_omega = []
        used_accel = []

        for abs_omega, omega, accel, arm in sample_rows:
            if spin_axis == "x":
                weight = math.hypot(accel[1], accel[2])
            elif spin_axis == "y":
                weight = math.hypot(accel[0], accel[2])
            else:
                weight = math.hypot(accel[0], accel[1])
            stats.add(spin_axis, arm, weight, vec_norm(omega))
            used_omega.append(omega)
            used_accel.append(accel)

        r = stats.result(spin_axis)
        samples_used = stats.count
    arm_mag = vec_norm(r)
    min_arm_m = ROCKING_MIN_ARM_M if rocking_mode else 1e-6
    if arm_mag > max_arm_m or arm_mag < min_arm_m:
        return LeverArmCalResult(
            imu_lever_arm_m={"x": r[0], "y": r[1], "z": r[2]},
            samples_used=samples_used,
            samples_rejected=rejected,
            residual_rms_mps=0.0,
            omega_rad_s=_mean_omega_magnitude(used_omega),
            detected_axis=spin_axis,
            success=False,
        )

    residual_sq = 0.0
    for omega, accel in zip(used_omega, used_accel):
        pred = _centripetal_accel(omega, r)
        err = vec_sub(accel, pred)
        residual_sq += err[0] ** 2 + err[1] ** 2 + err[2] ** 2

    residual_rms = math.sqrt(residual_sq / max(1, samples_used))
    residual_limit = (
        MAX_RESIDUAL_RMS_ROCKING_MPS if rocking_mode else MAX_RESIDUAL_RMS_MPS
    )
    success = residual_rms <= residual_limit

    return LeverArmCalResult(
        imu_lever_arm_m={"x": r[0], "y": r[1], "z": r[2]},
        samples_used=samples_used,
        samples_rejected=rejected,
        residual_rms_mps=residual_rms,
        omega_rad_s=_mean_omega_magnitude(used_omega),
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
        preview_accel, preview_gyro, _ = samples_from_series(
            series, gyro_window_s=GYRO_QUAT_WINDOW_ROCKING_S,
        )
        rocking = _infer_rocking_motion(preview_gyro)
        gyro_window_s = _pick_gyro_window_s(series, rocking=rocking)
        accel, gyro, ts_s = samples_from_series(series, gyro_window_s=gyro_window_s)
        return calibrate_lever_arm(
            accel,
            gyro,
            ts_s,
            imu_to_body=imu_to_body,
            rocking=rocking,
        )

    try:
        series = load_capture(file_path)
        if series.quat and series.accel:
            preview_accel, preview_gyro, _ = samples_from_series(
                series, gyro_window_s=GYRO_QUAT_WINDOW_ROCKING_S,
            )
            rocking = _infer_rocking_motion(preview_gyro)
            gyro_window_s = _pick_gyro_window_s(series, rocking=rocking)
            accel, gyro, ts_s = samples_from_series(
                series, gyro_window_s=gyro_window_s,
            )
        else:
            rocking = False
            ticks = resample_capture(series, hz)
            accel, gyro, ts_s = samples_from_capture_ticks(ticks)
        return calibrate_lever_arm(
            accel,
            gyro,
            ts_s,
            imu_to_body=imu_to_body,
            rocking=rocking if series.quat and series.accel else None,
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
