"""Lever-arm calibration from Arzberger & Nüchter (arXiv:2607.25784), Section III-D.

After subtracting rigid-body motion acceleration from the accelerometer reading,
the residual should have magnitude ||g||. We minimize Eq. (15) with Levenberg-
Marquardt. Angular acceleration uses a non-causal derivative-of-Gaussian kernel
(Section III-E).

Single-axis collar spin: rotation is primarily about body Z, so we optionally
estimate only (rx, ry) and fix rz = 0 (not observable from pure Z rotation).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
import numpy as np

GRAVITY_MAG = 9.81


def skew(w: np.ndarray) -> np.ndarray:
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]])


def motion_accel(omega: np.ndarray, omega_dot: np.ndarray, r: np.ndarray) -> np.ndarray:
  """ω × (ω × r) + ω̇ × r  (paper Eq. 4)."""
  return np.cross(omega, np.cross(omega, r)) + np.cross(omega_dot, r)


def kinematic_matrix(omega: np.ndarray, omega_dot: np.ndarray) -> np.ndarray:
  return skew(omega) @ skew(omega) + skew(omega_dot)


def dog_kernel_half_width(f_cut_hz: float, sample_hz: float) -> int:
  """K from paper Eq. (22): half-width of symmetric DoG kernel."""
  sigma = 1.0 / (2.0 * math.pi * f_cut_hz)
  w = int(math.ceil(6.0 * sigma * sample_hz))
  return max(1, w)


def dog_kernel(f_cut_hz: float, sample_hz: float) -> np.ndarray:
  """Non-causal DoG kernel h1[k] from paper Eq. (18), k in [-K, K]."""
  k_max = dog_kernel_half_width(f_cut_hz, sample_hz)
  sigma = 1.0 / (2.0 * math.pi * f_cut_hz)
  dt = 1.0 / sample_hz
  ks = np.arange(-k_max, k_max + 1, dtype=float)
  tk = ks * dt
  h = -(tk / (sigma**3 * math.sqrt(2.0 * math.pi))) * np.exp(-(tk**2) / (2.0 * sigma**2))
  return h


def dog_derivative_series(
    values: np.ndarray,
    *,
    f_cut_hz: float = 5.0,
    sample_hz: float | None = None,
    dt_s: float | None = None,
) -> np.ndarray:
  """Axis-wise non-causal DoG derivative (paper Eq. 19)."""
  n = len(values)
  if n < 3:
    return np.zeros_like(values)

  if sample_hz is None:
    if dt_s is None or dt_s <= 0.0:
      raise ValueError("dog_derivative_series needs sample_hz or dt_s")
    sample_hz = 1.0 / dt_s

  h = dog_kernel(f_cut_hz, sample_hz)
  k_max = (len(h) - 1) // 2
  out = np.zeros_like(values, dtype=float)
  for i in range(n):
    acc = 0.0
    for ki, hk in enumerate(h):
      j = i + ki - k_max
      if 0 <= j < n:
        acc += hk * values[j]
    out[i] = acc
  return out


def attach_omega_dot_dog(
    omega_series: list[tuple[int, np.ndarray]],
    *,
    f_cut_hz: float = 5.0,
) -> list[tuple[int, np.ndarray]]:
  """Replace finite-difference omega_dot with DoG-smoothed derivative."""
  if len(omega_series) < 3:
    return omega_series

  ts = np.array([t for t, _ in omega_series], dtype=float)
  om = np.array([w for _, w in omega_series], dtype=float)
  dts = np.diff(ts) / 1_000_000.0
  dt_med = float(np.median(dts[dts > 0])) if np.any(dts > 0) else 0.01
  sample_hz = 1.0 / dt_med

  omega_dot = np.column_stack([
    dog_derivative_series(om[:, i], f_cut_hz=f_cut_hz, sample_hz=sample_hz)
    for i in range(3)
  ])

  return [(omega_series[i][0], omega_dot[i]) for i in range(len(omega_series))]


@dataclass
class CalibSample:
  ts: int
  accel: np.ndarray
  omega: np.ndarray
  omega_dot: np.ndarray


def gravity_magnitude_residual(r: np.ndarray, sample: CalibSample) -> float:
  """Paper Eq. (15) per-sample term: |‖g‖² - ‖motion - a‖²|."""
  motion = motion_accel(sample.omega, sample.omega_dot, r)
  g_est = motion - sample.accel
  norm_sq = float(np.dot(g_est, g_est))
  return abs(GRAVITY_MAG**2 - norm_sq)


def gravity_magnitude_residuals(r: np.ndarray, samples: list[CalibSample]) -> np.ndarray:
  return np.array([gravity_magnitude_residual(r, s) for s in samples], dtype=float)


def _motion_jacobian(omega: np.ndarray, omega_dot: np.ndarray) -> np.ndarray:
  return kinematic_matrix(omega, omega_dot)


def _residual_and_jacobian(
    params: np.ndarray,
    samples: list[CalibSample],
    *,
    single_axis_z: bool,
) -> tuple[np.ndarray, np.ndarray]:
  if single_axis_z:
    r = np.array([params[0], params[1], 0.0])
  else:
    r = params

  residuals = np.empty(len(samples))
  jacobian = np.empty((len(samples), len(params)))

  for i, s in enumerate(samples):
    a_mat = _motion_jacobian(s.omega, s.omega_dot)
    motion = a_mat @ r
    g_est = motion - s.accel
    norm_sq = float(np.dot(g_est, g_est))
    target = GRAVITY_MAG**2
    diff = target - norm_sq
    residuals[i] = abs(diff)
    sign = -1.0 if diff < 0.0 else 1.0
    # d(||g_est||²)/dr = 2 A^T g_est
    grad_norm_sq = 2.0 * (a_mat.T @ g_est)
    if single_axis_z:
      jacobian[i] = sign * grad_norm_sq[:2]
    else:
      jacobian[i] = sign * grad_norm_sq

  return residuals, jacobian


def levenberg_marquardt(
    samples: list[CalibSample],
    r0: np.ndarray,
    *,
    single_axis_z: bool = True,
    max_iter: int = 50,
    lam: float = 1e-2,
    weights: np.ndarray | None = None,
) -> tuple[np.ndarray, dict]:
  """LM on paper Eq. (15) magnitude residuals."""
  if single_axis_z:
    p = np.array([r0[0], r0[1]], dtype=float)
  else:
    p = r0.astype(float).copy()

  if weights is None:
    weights = np.ones(len(samples))

  last_cost = math.inf
  info: dict = {"iterations": 0, "converged": False, "cost": math.inf}

  for it in range(max_iter):
    res, jac = _residual_and_jacobian(p, samples, single_axis_z=single_axis_z)
    res_w = res * np.sqrt(weights)
    jac_w = jac * np.sqrt(weights)[:, None]
    cost = float(np.dot(res_w, res_w))
    info["cost"] = cost
    info["iterations"] = it + 1

    if abs(last_cost - cost) < 1e-12 * max(1.0, last_cost):
      info["converged"] = True
      break
    last_cost = cost

    jtj = jac_w.T @ jac_w
    jtr = jac_w.T @ res_w
    step = np.linalg.solve(jtj + lam * np.eye(len(p)), jtr)
    p_try = p - step
    res_try, _ = _residual_and_jacobian(p_try, samples, single_axis_z=single_axis_z)
    cost_try = float(np.dot(res_try * np.sqrt(weights), res_try * np.sqrt(weights)))

    if cost_try < cost:
      p = p_try
      lam = max(lam * 0.5, 1e-8)
    else:
      lam = min(lam * 4.0, 1e6)

  if single_axis_z:
    r_out = np.array([p[0], p[1], 0.0])
  else:
    r_out = p

  # Compensated magnitude stats for reporting
  mags = []
  for s in samples:
    g_est = motion_accel(s.omega, s.omega_dot, r_out) - s.accel
    mags.append(float(np.linalg.norm(g_est)))
  info["compensated_mag_median"] = float(np.median(mags))
  info["compensated_mag_p90"] = float(np.percentile(mags, 90))
  info["residual_median"] = float(np.median(gravity_magnitude_residuals(r_out, samples)))

  return r_out, info


def detect_spin_axis(samples: list[CalibSample], *, min_omega: float = 0.5) -> np.ndarray:
  """Dominant rotation axis from high-spin samples (unit vector)."""
  omegas = [s.omega for s in samples if float(np.linalg.norm(s.omega)) >= min_omega]
  if not omegas:
    return np.array([0.0, 0.0, 1.0])
  mat = np.mean([np.outer(w / np.linalg.norm(w), w / np.linalg.norm(w)) for w in omegas], axis=0)
  _vals, vecs = np.linalg.eigh(mat)
  axis = vecs[:, -1]
  if axis[2] < 0:
    axis = -axis
  return axis / np.linalg.norm(axis)


def sample_weights_omega_squared(samples: list[CalibSample]) -> np.ndarray:
  """Weight by ||omega||^4 so low-spin samples do not dominate (centripetal ~ w^2 r)."""
  w = np.array([float(np.linalg.norm(s.omega)) ** 4 for s in samples], dtype=float)
  if w.max() > 0:
    w /= w.max()
  return np.maximum(w, 1e-6)


def estimate_lever_arm(
    samples: list[CalibSample],
    *,
    r0: np.ndarray | None = None,
    single_axis_z: bool = True,
    dog_f_cut_hz: float = 5.0,
    use_dog_omega_dot: bool = True,
    min_omega_rad_s: float = 0.0,
    weight_by_omega: bool = True,
) -> tuple[np.ndarray, dict]:
  """Calibrate lever arm r from spin samples using paper Eq. (15).

  Parameters
  ----------
  samples:
    Synced accel (wire type 1, gravity/specific-force vector) and omega.
  single_axis_z:
    If True, fix rz=0 (collar spins about Z; rz unobservable).
  """
  if len(samples) < 10:
    raise ValueError(f"need >= 10 samples, got {len(samples)}")

  if min_omega_rad_s > 0.0:
    samples = [s for s in samples if float(np.linalg.norm(s.omega)) >= min_omega_rad_s]
    if len(samples) < 10:
      raise ValueError(
        f"need >= 10 samples after min_omega={min_omega_rad_s}, got {len(samples)}"
      )

  if r0 is None:
    r0 = np.zeros(3)

  meta: dict = {
    "dog_f_cut_hz": dog_f_cut_hz,
    "single_axis_z": single_axis_z,
    "min_omega_rad_s": min_omega_rad_s,
    "n_samples": len(samples),
  }

  if use_dog_omega_dot:
    # Rebuild omega_dot from the omega time series embedded in samples
    series = [(s.ts, s.omega) for s in samples]
    omega_dot_series = attach_omega_dot_dog(series, f_cut_hz=dog_f_cut_hz)
    ts_to_od = {t: od for t, od in omega_dot_series}
    for s in samples:
      s.omega_dot = ts_to_od.get(s.ts, s.omega_dot)

  axis = detect_spin_axis(samples)
  meta["spin_axis"] = axis.tolist()

  weights = sample_weights_omega_squared(samples) if weight_by_omega else None
  r, lm_info = levenberg_marquardt(
    samples, r0, single_axis_z=single_axis_z, weights=weights,
  )
  meta.update(lm_info)
  return r, meta


