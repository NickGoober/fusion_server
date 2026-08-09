"""
Automatic collar calibration on TCP connect.

Phase 0 — user mounts collar on barbell with top facing up; server averages a
stable IMU quaternion and stores it as ``imu_to_body``.

Phase 1 — user spins about the bar long axis; lever-arm calibration runs with
the corrected body frame.

Phase 2 — calibration saved to fusion_calib.json.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from collar_status import (
    STATUS_DONE,
    STATUS_ERROR,
    STATUS_POINT_UP,
    STATUS_SPIN,
    set_collar_status,
)
from fusion_calib import write_lever_arm_calib

if TYPE_CHECKING:
    from fusion_server import ClientSession

LOG = logging.getLogger("collar_auto_cal")

UPRIGHT_HOLD_S = 2.0
UPRIGHT_MIN_SAMPLES = 15
UPRIGHT_MAX_DOT_SPREAD = 0.002  # 1 - min(|dot|) with running mean
UPRIGHT_TIMEOUT_S = 120.0
SPIN_MIN_SAMPLES = 30
SPIN_HOLD_S = 3.0
SPIN_TIMEOUT_S = 180.0
ERROR_DISPLAY_S = 3.0

_AXIS_NAMES = ("x", "y", "z")


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


class CollarAutoCal:
    """Runs the two-step mount + spin calibration for one collar session."""

    def __init__(self, session: ClientSession) -> None:
        self._session = session
        self._phase = STATUS_POINT_UP
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._upright_samples: list[dict[str, float]] = []
        self._stable_since: float | None = None
        self._phase_started = time.monotonic()
        self._imu_to_body: dict[str, float] | None = None
        self._error_return = STATUS_POINT_UP
        self._error_until = 0.0
        self._spin_ready_since: float | None = None
        self._last_quat: dict[str, float] | None = None

    @property
    def active(self) -> bool:
        return not self._stop.is_set() and self._phase != STATUS_DONE

    @property
    def phase(self) -> int:
        return self._phase

    def start(self) -> None:
        set_collar_status(STATUS_POINT_UP)
        self._thread = threading.Thread(
            target=self._monitor_loop,
            name=f"auto-cal-{self._session.session_id[:8]}",
            daemon=True,
        )
        self._thread.start()
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

    def on_quat(self, quat: dict[str, Any]) -> None:
        q = {
            "w": float(quat["w"]),
            "x": float(quat["x"]),
            "y": float(quat["y"]),
            "z": float(quat["z"]),
        }
        with self._lock:
            if self._phase != STATUS_POINT_UP or self._stop.is_set():
                return

            if self._last_quat is not None:
                dot = abs(_quat_dot(_normalize_quat(self._last_quat), _normalize_quat(q)))
                if dot < 1.0 - UPRIGHT_MAX_DOT_SPREAD:
                    self._upright_samples.clear()
                    self._stable_since = None
            self._last_quat = q

            self._upright_samples.append(q)
            if len(self._upright_samples) > 200:
                self._upright_samples = self._upright_samples[-200:]

            if len(self._upright_samples) < 3:
                return

            mean = _average_quat(self._upright_samples[-30:])
            dots = [
                abs(_quat_dot(_normalize_quat(s), mean))
                for s in self._upright_samples[-30:]
            ]
            if min(dots) < 1.0 - UPRIGHT_MAX_DOT_SPREAD:
                self._stable_since = None
                return

            now = time.monotonic()
            if self._stable_since is None:
                self._stable_since = now
            elif (
                now - self._stable_since >= UPRIGHT_HOLD_S
                and len(self._upright_samples) >= UPRIGHT_MIN_SAMPLES
            ):
                self._complete_upright_locked(_average_quat(self._upright_samples))

    def _complete_upright_locked(self, imu_to_body: dict[str, float]) -> None:
        self._imu_to_body = imu_to_body
        engine = self._session.engine
        engine.set_imu_to_body(
            imu_to_body["w"],
            imu_to_body["x"],
            imu_to_body["y"],
            imu_to_body["z"],
        )
        engine.reset()
        LOG.info(
            "Auto-cal upright captured imu_to_body w=%.4f x=%.4f y=%.4f z=%.4f",
            imu_to_body["w"],
            imu_to_body["x"],
            imu_to_body["y"],
            imu_to_body["z"],
        )
        self._begin_spin_locked()

    def _begin_spin_locked(self) -> None:
        self._phase = STATUS_SPIN
        self._phase_started = time.monotonic()
        self._spin_ready_since = None
        set_collar_status(STATUS_SPIN)

        ok = self._session.engine.lever_arm_cal_start(axis="auto", omega_rad_s=0.0)
        if not ok:
            self._fail_locked(STATUS_POINT_UP, "failed to start lever-arm calibration")
            return
        self._session.calibrating = True
        self._session.stream_buffer.reset()
        LOG.info("Auto-cal spin phase started for %s", self._session.addr)

    def _fail_locked(self, return_code: int, reason: str) -> None:
        LOG.warning("Auto-cal error (%s) — will retry status %d", reason, return_code)
        self._session.calibrating = False
        self._session.engine.lever_arm_cal_cancel()
        self._error_return = return_code
        self._phase = STATUS_ERROR
        self._error_until = time.monotonic() + ERROR_DISPLAY_S
        self._phase_started = time.monotonic()
        self._upright_samples.clear()
        self._stable_since = None
        self._spin_ready_since = None
        set_collar_status(STATUS_ERROR)

    def _finish_spin_locked(self) -> None:
        self._session.stream_buffer.flush()
        result = self._session.engine.lever_arm_cal_finish()
        if not result:
            self._fail_locked(STATUS_SPIN, "not enough valid spin samples")
            return

        imu_to_body = self._imu_to_body or {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        axis_idx = int(result["axis"])
        axis = _AXIS_NAMES[axis_idx] if 0 <= axis_idx < 3 else "auto"
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
        self._phase = STATUS_DONE
        set_collar_status(STATUS_DONE)
        LOG.info("Auto-calibration saved to %s for %s", path, self._session.addr)

    def _monitor_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.2)
            with self._lock:
                if self._phase == STATUS_ERROR:
                    if time.monotonic() >= self._error_until:
                        if self._error_return == STATUS_SPIN:
                            self._begin_spin_locked()
                        else:
                            self._phase = STATUS_POINT_UP
                            self._phase_started = time.monotonic()
                            self._upright_samples.clear()
                            self._stable_since = None
                            set_collar_status(STATUS_POINT_UP)
                    continue

                if self._phase == STATUS_POINT_UP:
                    if time.monotonic() - self._phase_started > UPRIGHT_TIMEOUT_S:
                        self._fail_locked(STATUS_POINT_UP, "upright pose timeout")
                    continue

                if self._phase == STATUS_SPIN:
                    if time.monotonic() - self._phase_started > SPIN_TIMEOUT_S:
                        self._fail_locked(STATUS_SPIN, "spin calibration timeout")
                        continue

                    status = self._session.engine.lever_arm_cal_status()
                    samples = int(status.get("samples_used", 0))
                    axis_locked = bool(status.get("axis_locked"))
                    if samples >= SPIN_MIN_SAMPLES and axis_locked:
                        now = time.monotonic()
                        if self._spin_ready_since is None:
                            self._spin_ready_since = now
                        elif now - self._spin_ready_since >= SPIN_HOLD_S:
                            self._finish_spin_locked()
                    else:
                        self._spin_ready_since = None

                if self._phase == STATUS_DONE:
                    break
