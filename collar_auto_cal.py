"""
Automatic collar calibration on TCP connect.

Phase 0 — user mounts collar on barbell with top facing up; server averages a
stable IMU quaternion (stored for frame calibration).

Phase 1 — user spins about the bar long axis; 100 rotation packets complete
mount / frame calibration (``imu_to_body``).

Phase 2 — user spins again; lever-arm calibration runs with the corrected body
frame (100 rotation packets).

Phase 3 — calibration saved to fusion_calib.json.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import TYPE_CHECKING, Any

from collar_status import (
    STATUS_DONE,
    STATUS_ERROR,
    STATUS_FRAME_SPIN,
    STATUS_LEVER_SPIN,
    STATUS_POINT_UP,
    set_collar_status,
)
from fusion_calib import write_lever_arm_calib

if TYPE_CHECKING:
    from fusion_server import ClientSession

LOG = logging.getLogger("collar_auto_cal")

UPRIGHT_HOLD_S = 0.8
UPRIGHT_WINDOW = 30
UPRIGHT_MIN_SAMPLES = 5
UPRIGHT_MIN_STABLE_FRAC = 0.55
UPRIGHT_STABLE_DOT = 0.985
UPRIGHT_FALLBACK_S = 10.0
UPRIGHT_FALLBACK_MIN_SAMPLES = 3
UPRIGHT_STATUS_LOG_S = 3.0
UPRIGHT_TIMEOUT_S = 180.0
UPRIGHT_PROGRESS_WIDTH = 30
SPIN_REQUIRED_MOTION_PACKETS = 100
LEVER_MIN_CAL_SAMPLES = 8
# Min rotation since the last counted packet (~0.9°) — ignores per-tick quat noise.
MIN_ROTATION_DELTA_RAD = 0.016
# Min spin rate about the bar axis (body +X) for rotation packet counting.
MIN_BAR_GYRO_RAD_S = 0.08
SPIN_PROGRESS_WIDTH = 40
SPIN_TIMEOUT_S = 300.0
ERROR_DISPLAY_S = 3.0

_AXIS_NAMES = ("x", "y", "z")

_PHASE_NAMES = {
    STATUS_POINT_UP: "point_up",
    STATUS_FRAME_SPIN: "frame_spin",
    STATUS_LEVER_SPIN: "lever_spin",
    STATUS_DONE: "done",
    STATUS_ERROR: "error",
}


def _quat_dot(a: dict[str, float], b: dict[str, float]) -> float:
    return (
        a["w"] * b["w"]
        + a["x"] * b["x"]
        + a["y"] * b["y"]
        + a["z"] * b["z"]
    )


def _normalize_quat(q: dict[str, float]) -> dict[str, float]:
    n = math.sqrt(q["w"] ** 2 + q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2)
    if n < 1e-9:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    return {k: q[k] / n for k in ("w", "x", "y", "z")}


def _average_quat(samples: list[dict[str, float]]) -> dict[str, float]:
    if not samples:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    ref = _normalize_quat(samples[0])
    sw = sx = sy = sz = 0.0
    for q in samples:
        qn = _normalize_quat(q)
        if _quat_dot(ref, qn) < 0.0:
            qn = {k: -qn[k] for k in qn}
        sw += qn["w"]
        sx += qn["x"]
        sy += qn["y"]
        sz += qn["z"]
    n = len(samples)
    return _normalize_quat({"w": sw / n, "x": sx / n, "y": sy / n, "z": sz / n})


def _quat_rotation_angle_rad(a: dict[str, float], b: dict[str, float]) -> float:
    dot = abs(_quat_dot(_normalize_quat(a), _normalize_quat(b)))
    dot = min(1.0, dot)
    return 2.0 * math.acos(dot)


def _upright_stable_fraction(samples: list[dict[str, float]]) -> float:
    if len(samples) < 2:
        return 0.0
    mean = _average_quat(samples)
    stable = sum(
        1
        for s in samples
        if abs(_quat_dot(_normalize_quat(s), mean)) >= UPRIGHT_STABLE_DOT
    )
    return stable / len(samples)


def _quat_rotate_vector(
    v: tuple[float, float, float],
    q: dict[str, float],
) -> tuple[float, float, float]:
    """Rotate vector v by unit quaternion q (w, x, y, z)."""
    qn = _normalize_quat(q)
    w, x, y, z = qn["w"], qn["x"], qn["y"], qn["z"]
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _bar_axis_gyro_rad_s(
    gyro: dict[str, Any],
    imu_to_body: dict[str, float] | None,
) -> float:
    """Spin rate about body +X (bar long axis), in rad/s."""
    gx = float(gyro.get("x", 0.0))
    gy = float(gyro.get("y", 0.0))
    gz = float(gyro.get("z", 0.0))
    total = math.sqrt(gx * gx + gy * gy + gz * gz)
    if imu_to_body is None:
        return total
    bx, by, bz = _quat_rotate_vector((gx, gy, gz), imu_to_body)
    bar = abs(bx)
    if total < 1e-6:
        return 0.0
    # Mostly single-axis spin: total magnitude is a reliable fallback.
    cross = math.sqrt(by * by + bz * bz)
    if cross <= total * 0.35:
        return max(bar, total)
    return bar


def _rotation_change_sufficient(
    quat: dict[str, float],
    since_counted_quat: dict[str, float] | None,
    gyro: dict[str, Any],
    *,
    imu_to_body: dict[str, float] | None,
    min_bar_gyro_rad_s: float,
) -> bool:
    if since_counted_quat is None:
        return False
    if _quat_rotation_angle_rad(since_counted_quat, quat) < MIN_ROTATION_DELTA_RAD:
        return False
    if _bar_axis_gyro_rad_s(gyro, imu_to_body) < min_bar_gyro_rad_s:
        return False
    return True


class CollarAutoCal:
    """Runs mount + lever-arm calibration for one collar session."""

    def __init__(self, session: ClientSession) -> None:
        self._session = session
        self._phase = STATUS_POINT_UP
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._upright_window: deque[dict[str, float]] = deque(maxlen=UPRIGHT_WINDOW)
        self._stable_since: float | None = None
        self._upright_samples_seen = 0
        self._upright_warned_no_quat = False
        self._upright_last_status_log = 0.0
        self._upright_stable_frac = 0.0
        self._upright_hold_elapsed_s = 0.0
        self._upright_progress_active = False
        self._last_upright_progress_pct = -1
        self._phase_started = time.monotonic()
        self._imu_to_body: dict[str, float] | None = None
        self._error_return = STATUS_POINT_UP
        self._error_until = 0.0
        self._spin_progress_active = False
        self._last_spin_progress_count = -1
        self._spin_motion_packets = 0
        self._last_counted_spin_quat: dict[str, float] | None = None
        self._spin_label = "cal frame"
        self._calibrated_imu_lever_arm_m: dict[str, float] | None = None
        self._detected_spin_axis: str | None = None

    @property
    def active(self) -> bool:
        return not self._stop.is_set() and self._phase != STATUS_DONE

    @property
    def stopped(self) -> bool:
        return self._stop.is_set()

    def done_calibration_fields(self) -> dict[str, Any]:
        """Extra webhook fields after lever-arm cal completes."""
        fields: dict[str, Any] = {}
        if self._calibrated_imu_lever_arm_m is not None:
            fields["calibrated_imu_lever_arm_m"] = dict(self._calibrated_imu_lever_arm_m)
        if self._detected_spin_axis:
            fields["detected_spin_axis"] = self._detected_spin_axis
        return fields

    @property
    def phase(self) -> int:
        return self._phase

    @property
    def needs_stream_ticks(self) -> bool:
        return self._phase in (STATUS_FRAME_SPIN, STATUS_LEVER_SPIN)

    @property
    def motion_packets(self) -> int:
        return min(self._spin_motion_packets, SPIN_REQUIRED_MOTION_PACKETS)

    def start(self) -> None:
        set_collar_status(STATUS_POINT_UP)
        buf = self._session.stream_buffer
        buf.latency_us = buf.min_latency_us
        self._thread = threading.Thread(
            target=self._monitor_loop,
            name=f"auto-cal-{self._session.session_id[:8]}",
            daemon=True,
        )
        self._thread.start()
        self._push_calibration_locked()
        LOG.info(
            "Auto-calibration started for %s — status %d (point top up)",
            self._session.addr,
            STATUS_POINT_UP,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None

    def on_upright_quat(self, quat: dict[str, Any]) -> None:
        """Process each raw quaternion sample during the face-up phase."""
        q = {
            "w": float(quat["w"]),
            "x": float(quat["x"]),
            "y": float(quat["y"]),
            "z": float(quat["z"]),
        }
        with self._lock:
            if self._phase != STATUS_POINT_UP or self._stop.is_set():
                return
            self._process_upright_sample_locked(q)

    def on_sensor_tick(self, msg: dict[str, Any], *, cal_accepted: bool = False) -> None:
        """Handle fused sensor ticks during spin phases."""
        with self._lock:
            if self._stop.is_set():
                return
            if self._phase in (STATUS_FRAME_SPIN, STATUS_LEVER_SPIN):
                self._process_spin_tick_locked(msg, cal_accepted=cal_accepted)
            if self._phase == STATUS_LEVER_SPIN:
                self._push_calibration_locked(force=True)

    def on_spin_tick(self, msg: dict[str, Any], *, cal_accepted: bool = False) -> None:
        """Backward-compatible alias."""
        self.on_sensor_tick(msg, cal_accepted=cal_accepted)

    def _process_spin_tick_locked(self, msg: dict[str, Any], *, cal_accepted: bool) -> None:
        del cal_accepted
        quat_raw = msg.get("quat")
        if not quat_raw:
            return
        quat = {
            "w": float(quat_raw["w"]),
            "x": float(quat_raw["x"]),
            "y": float(quat_raw["y"]),
            "z": float(quat_raw["z"]),
        }
        gyro = msg.get("gyro") or {}

        if self._last_counted_spin_quat is None:
            self._last_counted_spin_quat = quat
            return

        require_cal = self._phase == STATUS_LEVER_SPIN
        min_bar_gyro = MIN_BAR_GYRO_RAD_S
        mount = self._imu_to_body
        if require_cal:
            mount = self._session.engine.get_imu_to_body()

        if _rotation_change_sufficient(
            quat,
            self._last_counted_spin_quat,
            gyro,
            imu_to_body=mount,
            min_bar_gyro_rad_s=min_bar_gyro,
        ):
            if self._spin_motion_packets < SPIN_REQUIRED_MOTION_PACKETS:
                self._spin_motion_packets += 1
                self._last_counted_spin_quat = quat
            self._update_spin_progress_locked(finish_if_ready=True)

    def _end_upright_progress_line(self) -> None:
        if self._upright_progress_active:
            print(flush=True)
            self._upright_progress_active = False

    def _render_upright_progress_locked(self) -> None:
        frac_pct = int(round(self._upright_stable_frac * 100))
        hold_pct = int(
            round(min(1.0, self._upright_hold_elapsed_s / UPRIGHT_HOLD_S) * 100)
        )
        pct = min(100, int(round(frac_pct * 0.55 + hold_pct * 0.45)))
        if pct == self._last_upright_progress_pct:
            return
        self._last_upright_progress_pct = pct
        filled = int(UPRIGHT_PROGRESS_WIDTH * pct / 100)
        bar = "#" * filled + "-" * (UPRIGHT_PROGRESS_WIDTH - filled)
        print(
            f"\r[cal upright] [{bar}] {pct}% "
            f"(stable {frac_pct}%, hold {self._upright_hold_elapsed_s:.1f}/"
            f"{UPRIGHT_HOLD_S:.1f}s)",
            end="",
            flush=True,
        )
        self._upright_progress_active = True

    def _reset_upright_locked(self) -> None:
        self._upright_window.clear()
        self._stable_since = None
        self._upright_samples_seen = 0
        self._upright_warned_no_quat = False
        self._upright_last_status_log = 0.0
        self._upright_stable_frac = 0.0
        self._upright_hold_elapsed_s = 0.0
        self._last_upright_progress_pct = -1

    def _maybe_log_upright_status_locked(self) -> None:
        now = time.monotonic()
        if now - self._upright_last_status_log < UPRIGHT_STATUS_LOG_S:
            return
        self._upright_last_status_log = now
        LOG.info(
            "Upright cal %s — samples=%d window=%d stable=%.0f%% hold=%.1f/%.1fs",
            self._session.addr,
            self._upright_samples_seen,
            len(self._upright_window),
            self._upright_stable_frac * 100.0,
            self._upright_hold_elapsed_s,
            UPRIGHT_HOLD_S,
        )

    def _process_upright_sample_locked(self, q: dict[str, float]) -> None:
        self._upright_samples_seen += 1
        if self._upright_samples_seen == 1:
            LOG.info("Upright cal receiving quaternions from %s", self._session.addr)

        self._upright_window.append(q)
        window = list(self._upright_window)
        self._upright_stable_frac = _upright_stable_fraction(window)

        now = time.monotonic()
        elapsed = now - self._phase_started

        if (
            len(window) >= UPRIGHT_MIN_SAMPLES
            and self._upright_stable_frac >= UPRIGHT_MIN_STABLE_FRAC
        ):
            if self._stable_since is None:
                self._stable_since = now
            self._upright_hold_elapsed_s = now - self._stable_since
            if self._upright_hold_elapsed_s >= UPRIGHT_HOLD_S:
                self._end_upright_progress_line()
                self._complete_upright_locked(_average_quat(window))
                return
        else:
            self._stable_since = None
            self._upright_hold_elapsed_s = 0.0

        if (
            elapsed >= UPRIGHT_FALLBACK_S
            and len(window) >= UPRIGHT_FALLBACK_MIN_SAMPLES
        ):
            LOG.warning(
                "Upright cal fallback for %s — advancing with %d samples "
                "(stable %.0f%%, seen %d)",
                self._session.addr,
                len(window),
                self._upright_stable_frac * 100.0,
                self._upright_samples_seen,
            )
            self._end_upright_progress_line()
            self._complete_upright_locked(_average_quat(window))
            return

        self._render_upright_progress_locked()
        self._maybe_log_upright_status_locked()

    def _end_spin_progress_line(self) -> None:
        if self._spin_progress_active:
            print(flush=True)
            self._spin_progress_active = False

    def _render_spin_progress(self, current: int) -> None:
        current = max(0, min(current, SPIN_REQUIRED_MOTION_PACKETS))
        if current == self._last_spin_progress_count:
            return
        self._last_spin_progress_count = current
        filled = int(SPIN_PROGRESS_WIDTH * current / SPIN_REQUIRED_MOTION_PACKETS)
        bar = "#" * filled + "-" * (SPIN_PROGRESS_WIDTH - filled)
        print(
            f"\r[{self._spin_label}] [{bar}] {current}/{SPIN_REQUIRED_MOTION_PACKETS} "
            "rotation packets",
            end="",
            flush=True,
        )
        self._spin_progress_active = True

    def _update_spin_progress_locked(self, *, finish_if_ready: bool) -> None:
        self._render_spin_progress(self.motion_packets)
        if not finish_if_ready or self._spin_motion_packets < SPIN_REQUIRED_MOTION_PACKETS:
            return

        if self._phase == STATUS_FRAME_SPIN:
            self._end_spin_progress_line()
            self._finish_frame_spin_locked()
            return

        if self._phase == STATUS_LEVER_SPIN:
            self._try_finish_lever_spin_locked()
            return

    def _try_finish_lever_spin_locked(self) -> None:
        if self._phase != STATUS_LEVER_SPIN:
            return
        if self._spin_motion_packets < SPIN_REQUIRED_MOTION_PACKETS:
            return
        cal_samples = int(
            self._session.engine.lever_arm_cal_status().get("samples_used", 0)
        )
        if cal_samples < LEVER_MIN_CAL_SAMPLES:
            return
        self._end_spin_progress_line()
        self._finish_lever_spin_locked()

    def _complete_upright_locked(self, imu_to_body: dict[str, float]) -> None:
        if self._phase != STATUS_POINT_UP:
            return
        self._imu_to_body = imu_to_body
        self._reset_upright_locked()
        LOG.info(
            "Auto-cal upright captured imu_to_body w=%.4f x=%.4f y=%.4f z=%.4f",
            imu_to_body["w"],
            imu_to_body["x"],
            imu_to_body["y"],
            imu_to_body["z"],
        )
        self._begin_frame_spin_locked()

    def _reset_spin_counters_locked(self) -> None:
        self._last_spin_progress_count = -1
        self._spin_motion_packets = 0
        self._last_counted_spin_quat = None

    def _begin_frame_spin_locked(self) -> None:
        self._phase = STATUS_FRAME_SPIN
        self._phase_started = time.monotonic()
        self._spin_label = "cal frame"
        self._reset_spin_counters_locked()
        set_collar_status(STATUS_FRAME_SPIN)
        self._session.calibrating = False
        self._session.stream_buffer.reset()
        self._render_spin_progress(0)
        self._push_calibration_locked()
        LOG.info(
            "Auto-cal frame spin started for %s — need %d rotation packets",
            self._session.addr,
            SPIN_REQUIRED_MOTION_PACKETS,
        )

    def _begin_lever_spin_locked(self) -> None:
        self._phase = STATUS_LEVER_SPIN
        self._phase_started = time.monotonic()
        self._spin_label = "cal lever"
        self._reset_spin_counters_locked()
        set_collar_status(STATUS_LEVER_SPIN)

        if not self._session.live_display:
            self._session.ensure_live_display()

        cal_axis = "auto"
        ok = self._session.engine.lever_arm_cal_start(axis=cal_axis, omega_rad_s=0.0)
        if not ok:
            self._fail_locked(STATUS_LEVER_SPIN, "failed to start lever-arm calibration")
            return
        self._session.calibrating = True
        self._session.stream_buffer.reset()
        self._render_spin_progress(0)
        self._push_calibration_locked()
        LOG.info(
            "Auto-cal lever-arm spin started for %s — need %d rotation packets "
            "(spin axis auto-detected; >= %.1f° since last count, bar-axis gyro >= %.2f rad/s)",
            self._session.addr,
            SPIN_REQUIRED_MOTION_PACKETS,
            math.degrees(MIN_ROTATION_DELTA_RAD),
            MIN_BAR_GYRO_RAD_S,
        )

    def _fail_locked(self, return_code: int, reason: str) -> None:
        self._end_upright_progress_line()
        self._end_spin_progress_line()
        LOG.warning("Auto-cal error (%s) — will retry status %d", reason, return_code)
        self._session.calibrating = False
        self._session.engine.lever_arm_cal_cancel()
        self._error_return = return_code
        self._phase = STATUS_ERROR
        self._error_until = time.monotonic() + ERROR_DISPLAY_S
        self._phase_started = time.monotonic()
        self._reset_upright_locked()
        self._reset_spin_counters_locked()
        set_collar_status(STATUS_ERROR)
        self._push_calibration_locked()

    def _finish_frame_spin_locked(self) -> None:
        if self._phase != STATUS_FRAME_SPIN:
            return

        imu_to_body = self._imu_to_body or {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        engine = self._session.engine
        engine.set_imu_to_body(
            imu_to_body["w"],
            imu_to_body["x"],
            imu_to_body["y"],
            imu_to_body["z"],
        )
        engine.reset()
        imu_arm = engine.get_imu_lever_arm()
        imu_only = engine.imu_only
        if imu_only:
            write_lever_arm_calib(
                0.0,
                0.0,
                0.0,
                imu_arm["x"],
                imu_arm["y"],
                imu_arm["z"],
                imu_only=True,
                imu_to_body=imu_to_body,
                path=engine.calib_path,
            )
        else:
            flow_arm = engine.get_flow_lever_arm()
            write_lever_arm_calib(
                flow_arm["x"],
                flow_arm["y"],
                flow_arm["z"],
                imu_arm["x"],
                imu_arm["y"],
                imu_arm["z"],
                imu_only=False,
                imu_to_body=imu_to_body,
                path=engine.calib_path,
            )

        print(
            f"[cal frame] complete — {SPIN_REQUIRED_MOTION_PACKETS}/"
            f"{SPIN_REQUIRED_MOTION_PACKETS} rotation packets",
            flush=True,
        )
        LOG.info("Auto-cal frame calibration saved for %s", self._session.addr)
        self._begin_lever_spin_locked()

    def _finish_lever_spin_locked(self) -> None:
        if self._phase != STATUS_LEVER_SPIN:
            return
        self._phase = STATUS_DONE
        self._session.calibrating = False
        self._session.stream_buffer.flush()
        result = self._session.engine.lever_arm_cal_finish()
        if not result:
            self._phase = STATUS_LEVER_SPIN
            self._session.calibrating = True
            self._fail_locked(STATUS_LEVER_SPIN, "not enough valid spin samples")
            return

        imu_to_body = self._imu_to_body or {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        axis_idx = int(result["axis"])
        axis = _AXIS_NAMES[axis_idx] if 0 <= axis_idx < 3 else "auto"
        self._detected_spin_axis = axis
        self._calibrated_imu_lever_arm_m = dict(result["imu_lever_arm_m"])
        imu_only = self._session.engine.imu_only

        if imu_only:
            path = write_lever_arm_calib(
                0.0,
                0.0,
                0.0,
                result["imu_lever_arm_m"]["x"],
                result["imu_lever_arm_m"]["y"],
                result["imu_lever_arm_m"]["z"],
                axis=axis,
                omega_rad_s=float(result["omega_rad_s"]),
                samples_used=int(result["samples_used"]),
                residual_rms_mps=float(result["residual_rms_mps"]),
                imu_only=True,
                imu_to_body=imu_to_body,
                path=self._session.engine.calib_path,
            )
        else:
            path = write_lever_arm_calib(
                result["flow_lever_arm_m"]["x"],
                result["flow_lever_arm_m"]["y"],
                result["flow_lever_arm_m"]["z"],
                result["imu_lever_arm_m"]["x"],
                result["imu_lever_arm_m"]["y"],
                result["imu_lever_arm_m"]["z"],
                axis=axis,
                omega_rad_s=float(result["omega_rad_s"]),
                samples_used=int(result["samples_used"]),
                residual_rms_mps=float(result["residual_rms_mps"]),
                imu_only=False,
                imu_to_body=imu_to_body,
                path=self._session.engine.calib_path,
            )

        self._session.calibrating = False
        self._last_spin_progress_count = -1
        set_collar_status(STATUS_DONE)
        self._push_calibration_locked()
        print(
            f"[cal lever] complete — {SPIN_REQUIRED_MOTION_PACKETS}/"
            f"{SPIN_REQUIRED_MOTION_PACKETS} rotation packets",
            flush=True,
        )
        LOG.info("Auto-calibration saved to %s for %s", path, self._session.addr)

    def _push_calibration_locked(self, *, force: bool = False) -> None:
        payload: dict[str, Any] = {
            "phase": _PHASE_NAMES.get(self._phase, "unknown"),
            "status_code": self._phase,
            "motion_packets": min(self._spin_motion_packets, SPIN_REQUIRED_MOTION_PACKETS),
            "required_packets": SPIN_REQUIRED_MOTION_PACKETS,
        }
        if self._phase == STATUS_POINT_UP:
            payload["upright_stable_frac"] = round(self._upright_stable_frac, 3)
            payload["upright_hold_s"] = round(self._upright_hold_elapsed_s, 2)
            payload["upright_required_hold_s"] = UPRIGHT_HOLD_S
        engine = self._session.engine
        payload["imu_lever_arm_m"] = engine.get_imu_lever_arm()
        running = engine.lever_arm_cal_running_imu_arm()
        if running is not None:
            payload["running_imu_lever_arm_m"] = running
        if self._phase == STATUS_LEVER_SPIN:
            status = engine.lever_arm_cal_status()
            payload["cal_samples_used"] = int(status.get("samples_used", 0))
            payload["cal_samples_required"] = LEVER_MIN_CAL_SAMPLES
            payload["cal_samples_rejected"] = int(status.get("samples_rejected", 0))
            if status.get("detected_axis"):
                self._detected_spin_axis = status["detected_axis"]
                payload["detected_spin_axis"] = status["detected_axis"]
        elif self._phase == STATUS_DONE:
            payload.update(self.done_calibration_fields())
        self._session.push_calibration_update(payload, force=force)

    def _monitor_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.2)
            with self._lock:
                if self._phase == STATUS_ERROR:
                    if time.monotonic() >= self._error_until:
                        if self._error_return == STATUS_FRAME_SPIN:
                            self._begin_frame_spin_locked()
                        elif self._error_return == STATUS_LEVER_SPIN:
                            self._begin_lever_spin_locked()
                        else:
                            self._phase = STATUS_POINT_UP
                            self._phase_started = time.monotonic()
                            self._reset_upright_locked()
                            buf = self._session.stream_buffer
                            buf.latency_us = buf.min_latency_us
                            set_collar_status(STATUS_POINT_UP)
                            self._push_calibration_locked()
                    continue

                if self._phase == STATUS_POINT_UP:
                    elapsed = time.monotonic() - self._phase_started
                    if (
                        elapsed > 5.0
                        and self._upright_samples_seen == 0
                        and not self._upright_warned_no_quat
                    ):
                        self._upright_warned_no_quat = True
                        LOG.warning(
                            "Upright cal waiting for quaternions from %s — "
                            "no samples after %.0fs (check collar stream / STREAM_START)",
                            self._session.addr,
                            elapsed,
                        )
                    if elapsed > UPRIGHT_TIMEOUT_S:
                        self._fail_locked(STATUS_POINT_UP, "upright pose timeout")
                    else:
                        self._push_calibration_locked()
                    continue

                if self._phase in (STATUS_FRAME_SPIN, STATUS_LEVER_SPIN):
                    if time.monotonic() - self._phase_started > SPIN_TIMEOUT_S:
                        label = "frame" if self._phase == STATUS_FRAME_SPIN else "lever-arm"
                        self._fail_locked(self._phase, f"{label} spin calibration timeout")
                        continue

                    self._update_spin_progress_locked(finish_if_ready=True)
                    if self._phase == STATUS_LEVER_SPIN:
                        self._try_finish_lever_spin_locked()
                        self._push_calibration_locked(force=True)

                if self._phase == STATUS_DONE:
                    break
