"""Dual-channel barbell position: raw optical-flow/radar integrator + world-frame Kalman.

Raw path is DirectPositionTracker (unchanged). Filtered path is a 6-state
constant-velocity Kalman on the same observations. IMU accel is not used
for position (attitude only).

World frame: Y-up, origin at first valid collar pose.
"""

from __future__ import annotations

from collections import deque
from typing import Any

from direct_position import (
    FLOW_FOV_DEG,
    FLOW_NPIX,
    MIN_COUPLING,
    DirectPositionTracker,
    flow_counts_to_world_delta,
    fusion_body_attitude,
    height_from_range_m,
    meters_per_pixel,
)

# State: [x, z, y, vx, vz, vy]
_IX, _IZ, _IY, _IVX, _IVZ, _IVY = 0, 1, 2, 3, 4, 5
_N = 6


def _eye(n: int) -> list[list[float]]:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def _zeros(n: int) -> list[list[float]]:
    return [[0.0] * n for _ in range(n)]


def _mat_add(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def _median(vals: list[float]) -> float:
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    if n % 2:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


class RadarOutlierGate:
    """Reject 1–2 sample XM125 height spikes; coast/interpolate instead.

    Real barbell motion is accepted via a speed limit vs the predicted height
    and a short consecutive-agree lock-in so an actual lift is not frozen out.
    Raw DirectPositionTracker is not affected — this is filtered-channel only.
    """

    def __init__(
        self,
        *,
        window: int = 5,
        n_sigma: float = 3.5,
        max_speed_mps: float = 4.0,
        max_reject_streak: int = 3,
        cluster_m: float = 0.025,
        mad_floor_m: float = 0.005,
    ) -> None:
        self.window = max(int(window), 3)
        self.n_sigma = n_sigma
        self.max_speed_mps = max_speed_mps
        self.max_reject_streak = max(int(max_reject_streak), 2)
        self.cluster_m = cluster_m
        self.mad_floor_m = mad_floor_m
        self.reset()

    def reset(self) -> None:
        self._accepted: deque[float] = deque(maxlen=self.window)
        self._reject_streak = 0
        self._pending_y: float | None = None
        self._last_accepted_y: float | None = None
        self._cluster_elapsed_s = 0.0
        self.last_reject = False

    def _hampel_outlier(self, y_m: float) -> bool:
        if len(self._accepted) < 3:
            return False
        sample = list(self._accepted) + [y_m]
        med = _median(sample[:-1])
        abs_dev = [abs(v - med) for v in sample[:-1]]
        mad = max(_median(abs_dev), self.mad_floor_m)
        return abs(y_m - med) > self.n_sigma * 1.4826 * mad

    def _speed_outlier(self, y_m: float, predicted_y: float, dt_s: float) -> bool:
        if dt_s <= 1e-4:
            return abs(y_m - predicted_y) > 0.04
        max_step = max(self.max_speed_mps * dt_s, 0.015)
        return abs(y_m - predicted_y) > max_step

    def _implied_speed_ok(self, y_m: float) -> bool:
        """Lock-in must still be reachable from the last *accepted* height."""
        if self._last_accepted_y is None:
            return True
        elapsed = max(self._cluster_elapsed_s, 1e-3)
        implied = abs(y_m - self._last_accepted_y) / elapsed
        return implied <= self.max_speed_mps

    def accept(self, y_m: float, *, predicted_y: float, dt_s: float) -> bool:
        """True if this radar height should update the Kalman. False → coast."""
        innov = abs(y_m - predicted_y)
        speed_bad = self._speed_outlier(y_m, predicted_y, dt_s)
        hampel_bad = innov > 0.025 and self._hampel_outlier(y_m)
        outlier = speed_bad or hampel_bad

        if not outlier:
            self._accepted.append(y_m)
            self._last_accepted_y = y_m
            self._reject_streak = 0
            self._pending_y = None
            self._cluster_elapsed_s = 0.0
            self.last_reject = False
            return True

        step_dt = max(dt_s, 1e-3)
        max_follow = max(self.cluster_m, self.max_speed_mps * max(dt_s, 0.02))
        if self._pending_y is not None and abs(y_m - self._pending_y) <= max_follow:
            self._reject_streak += 1
            self._pending_y = y_m
            self._cluster_elapsed_s += step_dt
        else:
            self._reject_streak = 1
            self._pending_y = y_m
            self._cluster_elapsed_s = step_dt

        # Two-frame XM125 glitches are ~20 ms apart; never treat that as a lift.
        min_lock_s = 0.055
        if (
            self._reject_streak >= self.max_reject_streak
            and self._cluster_elapsed_s >= min_lock_s
            and self._implied_speed_ok(y_m)
        ):
            self._accepted.append(y_m)
            self._last_accepted_y = y_m
            self._reject_streak = 0
            self._pending_y = None
            self._cluster_elapsed_s = 0.0
            self.last_reject = False
            return True

        self.last_reject = True
        return False


def tilt_coupling(q_body: dict[str, float] | None) -> float:
    """|body −Y · world up| from fusion-body attitude. 1 = radar looking down."""
    if q_body is None:
        return 1.0
    qw, qx, qy, qz = q_body["w"], q_body["x"], q_body["y"], q_body["z"]
    r11 = qw * qw - qx * qx + qy * qy - qz * qz
    return abs(r11)


class PositionKalmanFilter:
    """Constant-velocity filter; measurements are radar y and flow Δx/Δz."""

    def __init__(
        self,
        *,
        process_noise_vel: float = 0.5,
        range_std_m: float = 0.003,
        flow_std_base_m: float = 0.002,
        innovation_gate_sigma: float = 3.0,
    ) -> None:
        self.process_noise_vel = process_noise_vel
        self.range_std_m = range_std_m
        self.flow_std_base_m = flow_std_base_m
        self.innovation_gate_sigma = innovation_gate_sigma
        self.reset()

    def reset(self) -> None:
        self.x = [0.0] * _N
        self.P = _eye(_N)
        # Tight velocity prior — large P_vv + tight radar R injects huge vy at 100–200 Hz.
        self.P[_IX][_IX] = 0.02 ** 2
        self.P[_IZ][_IZ] = 0.02 ** 2
        self.P[_IY][_IY] = 0.05 ** 2
        self.P[_IVX][_IVX] = 0.05 ** 2
        self.P[_IVZ][_IVZ] = 0.05 ** 2
        self.P[_IVY][_IVY] = 0.05 ** 2
        self._seeded_y = False
        self.last_reject = False

    def seed_y(self, y_m: float) -> None:
        self.x[_IY] = y_m
        self.x[_IVY] = 0.0
        self.P[_IY][_IY] = self.range_std_m ** 2
        self.P[_IVY][_IVY] = 0.05 ** 2
        self.P[_IY][_IVY] = 0.0
        self.P[_IVY][_IY] = 0.0
        self._seeded_y = True

    def position(self) -> dict[str, float]:
        return {"x": self.x[_IX], "y": self.x[_IY], "z": self.x[_IZ]}

    def velocity(self) -> dict[str, float]:
        return {"x": self.x[_IVX], "y": self.x[_IVY], "z": self.x[_IVZ]}

    def predict(self, dt_s: float, *, coast_xz: bool = True) -> None:
        if dt_s <= 0.0 or dt_s > 0.5:
            return
        # Coast Y between radar samples (and across rejected glitches).
        # Coast X/Z only on flow ticks, including empty (0,0) frames.
        if coast_xz:
            self.x[_IX] += self.x[_IVX] * dt_s
            self.x[_IZ] += self.x[_IVZ] * dt_s
        self.x[_IY] += self.x[_IVY] * dt_s
        sa2 = self.process_noise_vel ** 2
        dt2 = dt_s * dt_s
        dt3 = dt2 * dt_s
        q = _zeros(_N)
        for ip, iv in ((_IX, _IVX), (_IZ, _IVZ), (_IY, _IVY)):
            q[ip][ip] = sa2 * dt3 / 3.0
            q[ip][iv] = sa2 * dt2 / 2.0
            q[iv][ip] = sa2 * dt2 / 2.0
            q[iv][iv] = sa2 * dt_s
        self.P = _mat_add(self.P, q)

    def inflate_process_noise(self, scale: float = 4.0) -> None:
        sa2 = (self.process_noise_vel ** 2) * scale
        for i in (_IVX, _IVZ, _IVY):
            self.P[i][i] += sa2
        for i in (_IX, _IZ, _IY):
            self.P[i][i] += 0.03 ** 2

    def _scalar_update(self, index: int, z: float, r: float, *, gate: bool = True) -> bool:
        """Returns False if the innovation was treated as an outlier (soft-weighted)."""
        innov = z - self.x[index]
        s = self.P[index][index] + r
        if s <= 1e-12:
            return True
        accepted = True
        if gate:
            thresh = self.innovation_gate_sigma
            n2 = innov * innov / s
            if n2 > (thresh * 2.5) ** 2:
                # ~8σ: true outlier (flow spike). Coast; do not freeze P.
                self.last_reject = True
                self.inflate_process_noise()
                return False
            if n2 > thresh * thresh:
                self.last_reject = True
                self.inflate_process_noise()
                r *= 9.0
                s = self.P[index][index] + r
                accepted = False
            else:
                self.last_reject = False
        else:
            self.last_reject = False
        k = [self.P[i][index] / s for i in range(_N)]
        for i in range(_N):
            self.x[i] += k[i] * innov
        khp = [[k[i] * self.P[index][j] for j in range(_N)] for i in range(_N)]
        self.P = [[self.P[i][j] - khp[i][j] for j in range(_N)] for i in range(_N)]
        for i in range(_N):
            floor = 1.6e-5 if i in (_IX, _IZ, _IY) else 1e-4
            if self.P[i][i] < floor:
                self.P[i][i] = floor
            for j in range(i + 1, _N):
                mid = 0.5 * (self.P[i][j] + self.P[j][i])
                self.P[i][j] = self.P[j][i] = mid
        return accepted

    def update_y(self, y_m: float, *, coupling: float) -> bool:
        r = self.range_std_m ** 2
        if coupling < MIN_COUPLING:
            r *= (MIN_COUPLING / max(coupling, 0.05)) ** 2
        if not self._seeded_y:
            self.seed_y(y_m)
            return True
        # Radar is slow and smooth vs flow spikes — do not 3σ-reject height.
        return self._scalar_update(_IY, y_m, r, gate=False)

    def update_xz_increment(
        self,
        x_m: float,
        z_m: float,
        dx_m: float,
        dz_m: float,
        dt_s: float,
        *,
        height_m: float,
        quality: int,
        coupling: float,
        fov_deg: float,
        npix: float,
        max_pixels: int,
        dx_px: int,
        dy_px: int,
    ) -> bool:
        if abs(dx_px) > max_pixels or abs(dy_px) > max_pixels:
            self.last_reject = True
            self.inflate_process_noise()
            return False

        mpp = meters_per_pixel(height_m, fov_deg, npix)
        q = max(int(quality), 1)
        r_pos = (self.flow_std_base_m + mpp) ** 2 * (255.0 / q)
        if coupling < MIN_COUPLING:
            r_pos *= (MIN_COUPLING / max(coupling, 0.05)) ** 2

        # Unused: increment/dt kept for callers / tests; velocity is inferred.
        _ = (dx_m, dz_m, dt_s)
        ok_x = self._scalar_update(_IX, x_m, r_pos)
        ok_z = self._scalar_update(_IZ, z_m, r_pos)
        self.last_reject = not (ok_x and ok_z)
        return ok_x and ok_z


class PositionFusionEngine:
    """Raw DirectPositionTracker + optional Kalman wrapper."""

    def __init__(
        self,
        *,
        kalman_enable: bool = True,
        process_noise_vel: float = 0.5,
        range_std_m: float = 0.003,
        flow_std_base_m: float = 0.002,
        innovation_gate_sigma: float = 3.0,
        flow_max_pixels_per_frame: int = 40,
        flow_min_quality: int = 25,
        radar_max_speed_mps: float = 2.5,
        radar_hampel_window: int = 5,
        radar_hampel_sigma: float = 3.5,
        radar_max_reject_streak: int = 3,
    ) -> None:
        self.kalman_enable = kalman_enable
        self.flow_max_pixels_per_frame = flow_max_pixels_per_frame
        self.flow_min_quality = flow_min_quality
        self.raw = DirectPositionTracker()
        self.kf = PositionKalmanFilter(
            process_noise_vel=process_noise_vel,
            range_std_m=range_std_m,
            flow_std_base_m=flow_std_base_m,
            innovation_gate_sigma=innovation_gate_sigma,
        )
        self.radar_gate = RadarOutlierGate(
            window=radar_hampel_window,
            n_sigma=radar_hampel_sigma,
            max_speed_mps=radar_max_speed_mps,
            max_reject_streak=radar_max_reject_streak,
        )
        self._last_ts_us: int | None = None
        self._last_radar_y: float | None = None
        self._last_radar_ts_us: int | None = None

    def reset(self) -> None:
        self.raw.reset()
        self.kf.reset()
        self.radar_gate.reset()
        self._last_ts_us = None
        self._last_radar_y = None
        self._last_radar_ts_us = None

    @property
    def floor_offset_m(self) -> float:
        return self.raw.floor_offset_m

    def raw_position(self) -> dict[str, float]:
        return self.raw.position()

    def filtered_position(self) -> dict[str, float]:
        if not self.kalman_enable:
            return self.raw.position()
        return self.kf.position()

    def filtered_velocity(self) -> dict[str, float]:
        if not self.kalman_enable:
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        return self.kf.velocity()

    def update(
        self,
        *,
        range_mm: int | None,
        flow: dict[str, Any] | None,
        imu_quat: dict[str, float] | None,
        imu_to_body: dict[str, float],
        ts_us: int | None = None,
        fov_deg: float = FLOW_FOV_DEG,
        npix: float = FLOW_NPIX,
        radar_update: bool = True,
    ) -> None:
        self.raw.update(
            range_mm=range_mm,
            flow=flow,
            imu_quat=imu_quat,
            imu_to_body=imu_to_body,
            fov_deg=fov_deg,
            npix=npix,
        )
        after = self.raw.position()

        if not self.kalman_enable:
            if ts_us is not None:
                self._last_ts_us = ts_us
            return

        dx_flow, dy_flow, quality = _parse_flow(flow, self.flow_min_quality)
        has_y = radar_update and range_mm is not None and range_mm > 0
        has_flow_motion = dx_flow is not None and (dx_flow != 0 or dy_flow != 0)

        dt_s = 0.0
        if ts_us is not None and self._last_ts_us is not None:
            dt_s = (ts_us - self._last_ts_us) / 1e6

        empty_stream_flow = (
            flow is not None
            and not has_y
            and not has_flow_motion
            and 0.004 <= dt_s <= 0.035
        )
        if not has_y and not has_flow_motion and not empty_stream_flow:
            return

        if ts_us is not None:
            self._last_ts_us = ts_us

        prev = self.kf.position()
        coast_xz = empty_stream_flow or (
            flow is not None and not has_flow_motion and 0.004 <= dt_s <= 0.035
        )
        self.kf.predict(dt_s, coast_xz=coast_xz)

        q_body = self.raw._last_q
        coupling = tilt_coupling(q_body)

        if has_y:
            predicted_y = self.kf.position()["y"]
            if self.radar_gate.accept(after["y"], predicted_y=predicted_y, dt_s=dt_s):
                y_obs = after["y"]
                max_step = max(self.radar_gate.max_speed_mps * max(dt_s, 0.02), 0.02)
                delta = y_obs - predicted_y
                if delta > max_step:
                    y_obs = predicted_y + max_step
                elif delta < -max_step:
                    y_obs = predicted_y - max_step
                if (
                    self._last_radar_y is not None
                    and ts_us is not None
                    and self._last_radar_ts_us is not None
                ):
                    rdt = (ts_us - self._last_radar_ts_us) / 1e6
                    if 1e-3 < rdt <= 0.5:
                        vy = (y_obs - self._last_radar_y) / rdt
                        cap = self.radar_gate.max_speed_mps
                        self.kf.x[_IVY] = max(-cap, min(cap, vy))
                self._last_radar_y = y_obs
                if ts_us is not None:
                    self._last_radar_ts_us = ts_us
                self.kf.update_y(y_obs, coupling=coupling)
            else:
                self.kf.inflate_process_noise()

        if has_flow_motion:
            height_m = self.raw.height_m
            if (
                self.radar_gate.last_reject
                and self.raw.origin_height_m is not None
                and self._last_radar_y is not None
            ):
                height_m = max(self._last_radar_y + self.raw.origin_height_m, 0.05)
            dx_m, dz_m = flow_counts_to_world_delta(
                dx_flow,
                dy_flow,
                height_m,
                q_body,
                fov_deg,
                npix,
            )
            self.kf.update_xz_increment(
                after["x"],
                after["z"],
                dx_m,
                dz_m,
                dt_s,
                height_m=height_m,
                quality=quality,
                coupling=coupling,
                fov_deg=fov_deg,
                npix=npix,
                max_pixels=self.flow_max_pixels_per_frame,
                dx_px=dx_flow,
                dy_px=dy_flow,
            )

        if dt_s >= 0.004 and has_flow_motion:
            now = self.kf.position()
            a = 0.4
            inst_vx = (now["x"] - prev["x"]) / dt_s
            inst_vz = (now["z"] - prev["z"]) / dt_s
            cap = 4.0
            inst_vx = max(-cap, min(cap, inst_vx))
            inst_vz = max(-cap, min(cap, inst_vz))
            self.kf.x[_IVX] = a * inst_vx + (1.0 - a) * self.kf.x[_IVX]
            self.kf.x[_IVZ] = a * inst_vz + (1.0 - a) * self.kf.x[_IVZ]
        cap = 4.0
        self.kf.x[_IVX] = max(-cap, min(cap, self.kf.x[_IVX]))
        self.kf.x[_IVZ] = max(-cap, min(cap, self.kf.x[_IVZ]))


def _parse_flow(
    flow: dict[str, Any] | None, min_quality: int
) -> tuple[int | None, int | None, int]:
    if not flow:
        return None, None, 0
    dx = int(flow.get("dx", 0))
    dy = int(flow.get("dy", 0))
    quality = int(flow.get("quality", 255))
    if quality < min_quality:
        return None, None, quality
    return dx, dy, quality


__all__ = [
    "DirectPositionTracker",
    "PositionFusionEngine",
    "PositionKalmanFilter",
    "RadarOutlierGate",
    "height_from_range_m",
    "meters_per_pixel",
    "fusion_body_attitude",
    "tilt_coupling",
]
