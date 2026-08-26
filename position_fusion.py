"""Dual-channel barbell position: raw optical-flow/radar integrator + world-frame Kalman.

Raw path is DirectPositionTracker (unchanged). Filtered path is a 6-state
constant-velocity Kalman on the same observations. IMU accel is not used
for position (attitude only).

World frame: Y-up, origin at first valid collar pose.
"""

from __future__ import annotations

from typing import Any

from direct_position import (
    FLOW_FOV_DEG,
    FLOW_NPIX,
    MIN_COUPLING,
    DirectPositionTracker,
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

    def predict(self, dt_s: float) -> None:
        if dt_s <= 0.0 or dt_s > 0.5:
            return
        # Grow covariance with a white-accel process. Do not coast x += v*dt:
        # flow measurements are already per-frame position increments, and
        # integrating leftover vx between frames overshoots on barbell motion.
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
        self._last_ts_us: int | None = None

    def reset(self) -> None:
        self.raw.reset()
        self.kf.reset()
        self._last_ts_us = None

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
        before = self.raw.position()
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
        has_flow = dx_flow is not None
        if not has_y and not has_flow:
            return

        dt_s = 0.0
        if ts_us is not None:
            if self._last_ts_us is not None:
                dt_s = (ts_us - self._last_ts_us) / 1e6
            self._last_ts_us = ts_us

        prev = self.kf.position()
        self.kf.predict(dt_s)

        q_body = self.raw._last_q
        coupling = tilt_coupling(q_body)

        if has_y:
            self.kf.update_y(after["y"], coupling=coupling)

        if has_flow:
            dx_m = after["x"] - before["x"]
            dz_m = after["z"] - before["z"]
            self.kf.update_xz_increment(
                after["x"],
                after["z"],
                dx_m,
                dz_m,
                dt_s,
                height_m=self.raw.height_m,
                quality=quality,
                coupling=coupling,
                fov_deg=fov_deg,
                npix=npix,
                max_pixels=self.flow_max_pixels_per_frame,
                dx_px=dx_flow,
                dy_px=dy_flow,
            )

        if dt_s > 1e-4:
            now = self.kf.position()
            self.kf.x[_IVX] = (now["x"] - prev["x"]) / dt_s
            self.kf.x[_IVZ] = (now["z"] - prev["z"]) / dt_s
            self.kf.x[_IVY] = (now["y"] - prev["y"]) / dt_s


def _parse_flow(
    flow: dict[str, Any] | None, min_quality: int
) -> tuple[int | None, int | None, int]:
    if not flow:
        return None, None, 0
    dx = int(flow.get("dx", 0))
    dy = int(flow.get("dy", 0))
    quality = int(flow.get("quality", 255))
    if dx == 0 and dy == 0:
        return None, None, quality
    if quality < min_quality:
        return None, None, quality
    return dx, dy, quality


__all__ = [
    "DirectPositionTracker",
    "PositionFusionEngine",
    "PositionKalmanFilter",
    "height_from_range_m",
    "meters_per_pixel",
    "fusion_body_attitude",
    "tilt_coupling",
]
