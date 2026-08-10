#!/usr/bin/env python3
"""
Oracle Ubuntu TCP fusion server.

The collar connects once and streams sensor packets continuously:
  [sensor_type, timestamp, data_array]

  0=accel [x,y,z], 1=quat [x,y,z,w], 2=flow [dx,dy,q], 3=radar [mm]

All control (calibration, live Vercel display) is via the server admin console:
  python3 fusion_server.py          # interactive fusion> prompt
  python3 fusion_admin.py         # if server runs under systemd without TTY

Console commands:
  cal start | cal finish | cal cancel | cal status
  display start | display stop
  status | log | help
  record start | record stop | replay start <file>
"""

from __future__ import annotations

import json
import logging
import queue
import socket
import sys
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from typing import Any
from urllib import error, request

from collar_auto_cal import CollarAutoCal
from collar_registry import (
    get_active_collar_session,
    record_collar_event,
    set_active_collar_session,
)
from collar_status import (
    STATUS_FRAME_SPIN,
    STATUS_IDLE,
    STATUS_LEVER_SPIN,
    STATUS_POINT_UP,
    get_collar_status_label,
    set_collar_status,
    start_collar_status_server,
)
from fusion_admin_dispatch import admin_console_loop, dispatch_admin_command, start_admin_console_thread
from fusion_calib import write_lever_arm_calib
from fusion_lib import FusionEngine
from fusion_settings import (
    active_settings_path,
    get_bool_setting,
    get_float_setting,
    get_int_setting,
    get_setting,
)
from sensor_recorder import get_sensor_recorder
from device_protocol import unpack_collar_wire_line
from sensor_stream import (
    SENSOR_FLOW,
    SENSOR_QUAT,
    SENSOR_RADAR,
    SensorStreamBuffer,
    imu_quat_to_body_frame,
)

LOG = logging.getLogger("fusion_server")

_webhook_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="webhook")

_AXIS_NAMES = ("x", "y", "z")

SERVER_HOST = get_setting("SERVER_HOST", "0.0.0.0")
SERVER_PORT = get_int_setting("SERVER_PORT", 9000)
VERCEL_WEBHOOK_URL = get_setting("VERCEL_WEBHOOK_URL", "") or ""
WEBHOOK_SECRET = get_setting("WEBHOOK_SECRET", "") or ""
STREAM_IDLE_TIMEOUT_S = get_float_setting("STREAM_IDLE_TIMEOUT_S", 0.0)


def _parse_fixed_latency_us() -> int | None:
    raw = (get_setting("STREAM_LATENCY_S", "auto") or "auto").strip().lower()
    if raw in ("auto", ""):
        return None
    seconds = float(raw)
    if seconds <= 0:
        return None
    return int(seconds * 1_000_000)


STREAM_FIXED_LATENCY_US = _parse_fixed_latency_us()
STREAM_MIN_LATENCY_US = int(get_float_setting("STREAM_MIN_LATENCY_S", 0.05) * 1_000_000)
STREAM_MAX_LATENCY_US = int(get_float_setting("STREAM_MAX_LATENCY_S", 2.0) * 1_000_000)
ADMIN_HOST = get_setting("ADMIN_HOST", "127.0.0.1")
ADMIN_PORT = get_int_setting("ADMIN_PORT", 9001)
CAL_STATUS_HOST = get_setting("CAL_STATUS_HOST", "0.0.0.0")
CAL_STATUS_PORT = get_int_setting("CAL_STATUS_PORT", 9002)
AUTO_CAL_ON_CONNECT = get_bool_setting("AUTO_CAL_ON_CONNECT", True)
STREAM_OUTPUT_HZ = get_float_setting("STREAM_OUTPUT_HZ", 100.0)
MAX_LINE_BYTES = get_int_setting("MAX_LINE_BYTES", 2_097_152)
LINE_QUEUE_MAX = get_int_setting("LINE_QUEUE_MAX", 4096)
CAL_WEBHOOK_MIN_INTERVAL_S = get_float_setting("CAL_WEBHOOK_MIN_INTERVAL_S", 0.05)
IMU_ONLY_MODE = get_bool_setting("IMU_ONLY_MODE", True)


def _handle_admin_client(conn: socket.socket) -> None:
    try:
        data = conn.recv(8192)
        if not data:
            return
        text = data.decode("utf-8", errors="replace")
        outputs: list[str] = []
        for line in text.splitlines():
            if not line.strip():
                continue
            _, output = dispatch_admin_command(line)
            if output:
                outputs.append(output.rstrip("\n"))
        response = "\n".join(outputs) + ("\n" if outputs else "OK\n")
        conn.sendall(response.encode("utf-8"))
    except OSError as exc:
        LOG.debug("Admin client error: %s", exc)
    finally:
        try:
            conn.close()
        except OSError:
            pass


def start_admin_socket_thread() -> None:
    def _loop() -> None:
        admin_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        admin_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        admin_sock.bind((ADMIN_HOST, ADMIN_PORT))
        admin_sock.listen(8)
        LOG.info("Admin commands on %s:%d (fusion_admin.py)", ADMIN_HOST, ADMIN_PORT)
        while True:
            conn, _addr = admin_sock.accept()
            thread = threading.Thread(
                target=_handle_admin_client,
                args=(conn,),
                daemon=True,
            )
            thread.start()

    thread = threading.Thread(target=_loop, name="fusion-admin-socket", daemon=True)
    thread.start()


def now_us() -> int:
    return int(time.time() * 1_000_000)


def post_pose_webhook(payload: dict[str, Any]) -> None:
    if not VERCEL_WEBHOOK_URL:
        LOG.warning("VERCEL_WEBHOOK_URL not set — skipping webhook POST")
        return

    body = json.dumps(payload).encode("utf-8")
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {WEBHOOK_SECRET}",
    }

    def _send() -> None:
        req = request.Request(
            VERCEL_WEBHOOK_URL,
            data=body,
            headers=headers,
            method="POST",
        )
        try:
            with request.urlopen(req, timeout=10) as resp:
                LOG.debug("Webhook OK %s", resp.status)
        except error.HTTPError as exc:
            err_body = exc.read().decode("utf-8", errors="replace")
            LOG.error("Webhook HTTP error %s: %s", exc.code, err_body[:200])
        except error.URLError as exc:
            LOG.error("Webhook URL error: %s", exc.reason)

    _webhook_pool.submit(_send)


_engine: FusionEngine | None = None
_engine_lock = threading.Lock()
_cal_meta: dict[str, Any] = {"axis": "auto", "omega_rad_s": 0.0}


def get_fusion_engine() -> FusionEngine:
    global _engine
    with _engine_lock:
        if _engine is None:
            _engine = FusionEngine()
            imu_arm = _engine.get_imu_lever_arm()
            if _engine.imu_only:
                LOG.info(
                    "Fusion engine ready (IMU-only mode, imu lever arm: "
                    "x=%.4f y=%.4f z=%.4f m)",
                    imu_arm["x"], imu_arm["y"], imu_arm["z"],
                )
            else:
                arm = _engine.get_flow_lever_arm()
                LOG.info(
                    "Fusion engine ready (flow lever arm: x=%.4f y=%.4f z=%.4f m, "
                    "imu lever arm: x=%.4f y=%.4f z=%.4f m)",
                    arm["x"], arm["y"], arm["z"],
                    imu_arm["x"], imu_arm["y"], imu_arm["z"],
                )
        return _engine


class ClientSession:
    """Handles one TCP client connection and its fusion stream."""

    def __init__(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        self.conn = conn
        self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.addr = addr
        self.session_id = str(uuid.uuid4())
        self.connected_at = time.monotonic()
        self.packets_received = 0
        self.last_packet_at: float | None = None
        self.live_display = False
        self.calibrating = False
        self.auto_cal: CollarAutoCal | None = None
        self.last_pose_step = 0
        self.last_activity = time.monotonic()
        self.engine = get_fusion_engine()
        self.last_sensor_ts_us: int | None = None
        self.last_imu_quat: dict[str, float] | None = None
        self.last_push_ms: int = 0
        self.last_calibration: dict[str, Any] | None = None
        self._last_cal_webhook_mono = 0.0
        self._shutdown_requested = False
        self._recv_loop_active = False
        self._stream_start_sent = False
        self._rotation_trace_enabled = False
        self._rotation_trace_thread: threading.Thread | None = None
        self._rotation_trace_lock = threading.Lock()
        self._trace_quat_rx = 0
        self._trace_quat_webhook = 0
        self._trace_last_rx_quat: dict[str, float] | None = None
        self._trace_last_webhook_quat: dict[str, float] | None = None
        self.stream_buffer = SensorStreamBuffer(
            fixed_latency_us=STREAM_FIXED_LATENCY_US,
            min_latency_us=STREAM_MIN_LATENCY_US,
            max_latency_us=STREAM_MAX_LATENCY_US,
            output_hz=STREAM_OUTPUT_HZ,
            on_tick=self._on_stream_tick,
            on_latency_change=self._on_stream_latency_change,
        )
        self._tick_queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=256)
        self._tick_worker_stop = threading.Event()
        self._tick_worker = threading.Thread(
            target=self._tick_worker_loop,
            name=f"tick-worker-{self.session_id[:8]}",
            daemon=True,
        )
        self._tick_worker.start()
        self._line_queue: queue.Queue[str | None] = queue.Queue(maxsize=LINE_QUEUE_MAX)
        self._line_worker_stop = threading.Event()
        self._line_worker = threading.Thread(
            target=self._line_worker_loop,
            name=f"line-worker-{self.session_id[:8]}",
            daemon=True,
        )
        self._line_worker.start()

    @property
    def stream_start_sent(self) -> bool:
        return self._stream_start_sent

    def _enqueue_line(self, line: str) -> None:
        try:
            self._line_queue.put_nowait(line)
        except queue.Full:
            try:
                self._line_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._line_queue.put_nowait(line)
            except queue.Full:
                LOG.warning(
                    "Line queue saturated for %s — dropping newest packet",
                    self.addr,
                )

    def _should_reject_as_duplicate(self, prev: ClientSession) -> bool:
        """Reject a second live TCP socket from the same collar host."""
        if prev.addr[0] != self.addr[0]:
            return False
        return prev._recv_loop_active and not prev._shutdown_requested

    def _mark_disconnected(self, reason: str) -> None:
        self._recv_loop_active = False
        self._shutdown_requested = True
        if get_active_collar_session() is self:
            set_active_collar_session(None)
        self._log_disconnect(reason)

    def _schedule_stream_start_retries(self) -> None:
        def retry_loop() -> None:
            delays = (0.35, 1.0, 2.0)
            for delay in delays:
                time.sleep(delay)
                if self._shutdown_requested or not self._recv_loop_active:
                    return
                if self.packets_received > 0:
                    return
                self.notify_collar_stream_start()

        threading.Thread(
            target=retry_loop,
            name=f"stream-start-retry-{self.session_id[:8]}",
            daemon=True,
        ).start()

    def request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        LOG.info(
            "Shutting down collar session %s (%s): %s",
            self.session_id,
            self.addr,
            reason,
        )
        try:
            self.conn.shutdown(socket.SHUT_RD)
        except OSError:
            pass
        try:
            self.conn.close()
        except OSError:
            pass

    def _line_worker_loop(self) -> None:
        while not self._line_worker_stop.is_set():
            try:
                line = self._line_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if line is None:
                break
            try:
                self.handle_line(line)
            except Exception as exc:
                LOG.warning(
                    "Packet error from %s: %s — %r",
                    self.addr,
                    exc,
                    line[:120],
                    exc_info=True,
                )

    def stop_line_worker(self) -> None:
        self._line_worker_stop.set()
        try:
            self._line_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._line_worker.is_alive():
            self._line_worker.join(timeout=3.0)

    def _tick_worker_loop(self) -> None:
        while not self._tick_worker_stop.is_set():
            try:
                msg = self._tick_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                self._process_stream_tick(msg)
            except Exception:
                LOG.exception("Stream tick error session %s", self.session_id)

    def _enqueue_stream_tick(self, msg: dict[str, Any]) -> None:
        auto_cal_active = (
            self.auto_cal is not None and self.auto_cal.active
        )
        if (
            not self.live_display
            and not self.calibrating
            and not auto_cal_active
        ):
            return
        try:
            self._tick_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._tick_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._tick_queue.put_nowait(msg)
            except queue.Full:
                pass

    def stop_tick_worker(self) -> None:
        self._tick_worker_stop.set()
        try:
            self._tick_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._tick_worker.is_alive():
            self._tick_worker.join(timeout=3.0)

    def _on_stream_latency_change(
        self, latency_us: int, sensor_periods_ms: dict[str, float],
    ) -> None:
        LOG.debug(
            "Session %s adaptive latency -> %.0f ms (sensor periods ms: %s)",
            self.session_id,
            latency_us / 1000.0,
            sensor_periods_ms,
        )

    @staticmethod
    def _format_quat(q: dict[str, float] | None) -> str:
        if not q:
            return "—"
        return (
            f"w={q['w']:.4f} x={q['x']:.4f} "
            f"y={q['y']:.4f} z={q['z']:.4f}"
        )

    def _note_rotation_rx(self, quat: dict[str, Any]) -> None:
        with self._rotation_trace_lock:
            self._trace_quat_rx += 1
            self._trace_last_rx_quat = {
                "w": float(quat["w"]),
                "x": float(quat["x"]),
                "y": float(quat["y"]),
                "z": float(quat["z"]),
            }

    def _note_rotation_webhook(self, quat: dict[str, float]) -> None:
        with self._rotation_trace_lock:
            self._trace_quat_webhook += 1
            self._trace_last_webhook_quat = dict(quat)

    def _rotation_trace_loop(self) -> None:
        while self._rotation_trace_enabled:
            time.sleep(1.0)
            if not self._rotation_trace_enabled:
                break
            with self._rotation_trace_lock:
                rx_count = self._trace_quat_rx
                webhook_count = self._trace_quat_webhook
                last_rx = self._trace_last_rx_quat
                last_web = self._trace_last_webhook_quat
                self._trace_quat_rx = 0
                self._trace_quat_webhook = 0
            print(
                f"[trace] collar→server: {rx_count}/s  "
                f"{self._format_quat(last_rx)}",
                flush=True,
            )
            print(
                f"[trace] server→web:   {webhook_count}/s  "
                f"{self._format_quat(last_web)}",
                flush=True,
            )

    def console_trace_rotation_start(self) -> None:
        if self._rotation_trace_enabled:
            print("Rotation trace already running.")
            return
        with self._rotation_trace_lock:
            self._trace_quat_rx = 0
            self._trace_quat_webhook = 0
        self._rotation_trace_enabled = True
        self._rotation_trace_thread = threading.Thread(
            target=self._rotation_trace_loop,
            name=f"rotation-trace-{self.session_id[:8]}",
            daemon=True,
        )
        self._rotation_trace_thread.start()
        print("Rotation trace ON — logging quat packets every 1s.")

    def console_trace_rotation_stop(self) -> None:
        if not self._rotation_trace_enabled:
            print("Rotation trace is not running.")
            return
        self._rotation_trace_enabled = False
        thread = self._rotation_trace_thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._rotation_trace_thread = None
        print("Rotation trace OFF.")

    def console_trace_rotation_status(self) -> None:
        state = "ON" if self._rotation_trace_enabled else "off"
        print(f"Rotation trace: {state}")

    def stop_rotation_trace(self) -> None:
        if not self._rotation_trace_enabled:
            return
        self._rotation_trace_enabled = False
        thread = self._rotation_trace_thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._rotation_trace_thread = None

    def _sensor_dt_s(self, ts_us: int) -> float:
        if self.last_sensor_ts_us is None:
            return 0.01
        dt = (ts_us - self.last_sensor_ts_us) / 1_000_000.0
        if dt <= 0.0 or dt > 0.5:
            return 0.01
        return dt

    def _feed_lever_arm_cal(self, msg: dict[str, Any], ts_us: int) -> bool:
        gyro = msg.get("gyro")
        accel = msg.get("accel")
        if not gyro or not accel:
            return False

        flow = msg.get("flow") or {"dx": 0, "dy": 0}
        range_data = msg.get("range") or {"mm": 0}
        if not IMU_ONLY_MODE and (not msg.get("flow") or not msg.get("range")):
            return False

        dt_s = float(msg.get("dt_s", self._sensor_dt_s(ts_us)))

        def feed() -> bool:
            return self.engine.lever_arm_cal_feed(
                float(gyro["x"]),
                float(gyro["y"]),
                float(gyro["z"]),
                float(accel["x"]),
                float(accel["y"]),
                float(accel["z"]),
                int(flow["dx"]),
                int(flow["dy"]),
                int(range_data["mm"]),
                dt_s,
            )

        return bool(self._with_engine(feed))

    def console_status_text(self) -> str:
        cal = self._with_engine(self.engine.lever_arm_cal_status)
        buf = self.stream_buffer.stream_status()
        uptime_s = time.monotonic() - self.connected_at
        packet_age = ""
        if self.last_packet_at is not None:
            packet_age = f", last packet {time.monotonic() - self.last_packet_at:.1f}s ago"
        lines = [
            f"Collar: connected from {self.addr[0]}:{self.addr[1]} "
            f"({uptime_s:.1f}s{packet_age})",
            f"Mode: {'IMU-only (barbell)' if IMU_ONLY_MODE else 'flow + range + IMU'}",
            f"Packets received: {self.packets_received}",
            f"Live display (Vercel): {'ON' if self.live_display else 'off'}",
            f"Collar status code: {get_collar_status_label()}",
            f"Calibration: {'ACTIVE' if cal.get('active') else 'inactive'}",
            f"Buffer latency: {buf.get('latency_ms', '?')} ms",
            f"Cal samples used: {cal.get('samples_used', 0)} "
            f"(rejected: {cal.get('samples_rejected', 0)})",
        ]
        if cal.get("axis_locked") and cal.get("detected_axis"):
            lines.append(f"Detected rotation axis: {cal['detected_axis']}")
        return "\n".join(lines)

    def console_cal_start(self, *, axis: str = "auto") -> None:
        self.handle_cal_lever_arm_start(
            {"axis": axis, "omega_rad_s": 0.0},
            from_console=True,
        )

    def console_cal_finish(self) -> None:
        self.handle_cal_lever_arm_finish(from_console=True)

    def console_cal_cancel(self) -> None:
        self.handle_cal_lever_arm_cancel(from_console=True)

    def console_cal_status(self) -> dict[str, Any]:
        status = self._with_engine(self.engine.lever_arm_cal_status)
        status["flow_lever_arm_m"] = self._with_engine(self.engine.get_flow_lever_arm)
        status["imu_lever_arm_m"] = self._with_engine(self.engine.get_imu_lever_arm)
        return status

    def console_display_start(self) -> None:
        self.handle_start(from_console=True)

    def console_display_stop(self) -> None:
        self.handle_end(from_console=True)

    def handle_cal_lever_arm_start(
        self,
        msg: dict[str, Any],
        *,
        from_console: bool = False,
    ) -> None:
        axis = str(msg.get("axis", "auto"))
        omega = float(msg.get("omega_rad_s", 0.0))
        omega_tol = float(msg.get("omega_tol_rad_s", 0.0))

        def start() -> bool:
            return self.engine.lever_arm_cal_start(axis, omega, omega_tol)

        ok = bool(self._with_engine(start))
        if ok:
            _cal_meta["axis"] = axis
            _cal_meta["omega_rad_s"] = omega
            self.calibrating = True
            self.stream_buffer.reset()
            LOG.info(
                "Lever-arm calibration started axis=%s omega=%s rad/s (%s)",
                axis,
                "variable" if omega <= 0.0 else f"{omega:.4f}",
                self.addr,
            )
            if from_console:
                print(
                    "Calibration started. Rotate the barbell about its long axis "
                    "back and forth for 5+ seconds."
                )
                print("Then run: cal finish")
            else:
                self.send_ack(
                    "cal_lever_arm_start",
                    self._with_engine(self.engine.lever_arm_cal_status),
                )
        else:
            msg_text = "Failed to start calibration"
            if from_console:
                print(msg_text)
            else:
                self.send_ack("cal_lever_arm_start", {"error": msg_text})

    def handle_cal_lever_arm_finish(self, *, from_console: bool = False) -> None:
        self.stream_buffer.flush()

        def finish() -> dict | None:
            return self.engine.lever_arm_cal_finish()

        result = self._with_engine(finish)
        if not result:
            msg_text = "Calibration failed — not enough valid samples. Spin longer and retry."
            if from_console:
                print(msg_text)
            else:
                self.send_ack("cal_lever_arm_finish", {"error": "not enough valid samples"})
            return

        axis = _cal_meta.get("axis", "auto")
        if 0 <= int(result["axis"]) < 3:
            axis = _AXIS_NAMES[int(result["axis"])]

        if IMU_ONLY_MODE:
            mount = self._with_engine(self.engine.get_imu_to_body)
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
                imu_to_body=mount,
                path=self.engine.calib_path,
            )
            LOG.info(
                "IMU lever-arm calibration saved to %s: imu=%s",
                path,
                result["imu_lever_arm_m"],
            )
        else:
            mount = self._with_engine(self.engine.get_imu_to_body)
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
                imu_to_body=mount,
                path=self.engine.calib_path,
            )
            LOG.info(
                "Lever-arm calibration saved to %s: flow=%s imu=%s",
                path,
                result["flow_lever_arm_m"],
                result["imu_lever_arm_m"],
            )
        self.calibrating = False
        if from_console:
            print(f"Calibration saved to {path}")
            print(json.dumps(result, indent=2))
        else:
            self.send_ack("cal_lever_arm_finish", result)

    def handle_cal_lever_arm_cancel(self, *, from_console: bool = False) -> None:
        self._with_engine(self.engine.lever_arm_cal_cancel)
        self.calibrating = False
        LOG.info("Lever-arm calibration cancelled (%s)", self.addr)
        if from_console:
            print("Calibration cancelled.")
        else:
            self.send_ack("cal_lever_arm_cancel")

    def handle_cal_lever_arm_status(self) -> None:
        status = self._with_engine(self.engine.lever_arm_cal_status)
        status["flow_lever_arm_m"] = self._with_engine(self.engine.get_flow_lever_arm)
        status["imu_lever_arm_m"] = self._with_engine(self.engine.get_imu_lever_arm)
        self.send_ack("cal_lever_arm_status", status)

    def _with_engine(self, fn) -> Any:
        with _engine_lock:
            return fn()

    def close(self) -> None:
        try:
            self.conn.close()
        except OSError:
            pass

    def send_ack(self, msg_type: str, extra: dict[str, Any] | None = None) -> None:
        payload: dict[str, Any] = {"type": "ack", "of": msg_type, "session_id": self.session_id}
        if extra:
            payload.update(extra)
        line = (json.dumps(payload) + "\n").encode("utf-8")
        self.conn.sendall(line)

    def push_pose(self, streaming: bool, *, force: bool = False, imu_only: bool = False) -> None:
        now_ms = int(time.time() * 1000)
        if (
            not force
            and self.last_push_ms
            and now_ms - self.last_push_ms < 200
        ):
            return

        pose = None
        if imu_only:
            if self.last_imu_quat is None:
                return
        else:
            pose = self._with_engine(self.engine.get_pose)
            if pose is None and self.last_imu_quat is None:
                return

        payload: dict[str, Any] = {
            "session_id": self.session_id,
            "streaming": streaming,
            "updated_at_ms": now_ms,
        }
        if pose is not None:
            payload["pose"] = pose
        if self.last_imu_quat is not None:
            payload["imu_game_rotation"] = dict(self.last_imu_quat)
            mount = self._with_engine(self.engine.get_imu_to_body)
            payload["collar_rotation"] = imu_quat_to_body_frame(
                self.last_imu_quat,
                mount,
            )
            self._note_rotation_webhook(payload["imu_game_rotation"])
        cal = self._build_calibration_payload()
        if cal is not None:
            payload["calibration"] = cal
        elif self.last_calibration is not None:
            payload["calibration"] = dict(self.last_calibration)
        post_pose_webhook(payload)
        self.last_push_ms = now_ms

    def push_calibration_update(self, cal: dict[str, Any], *, force: bool = False) -> None:
        self.last_calibration = dict(cal)
        now_mono = time.monotonic()
        if not force and now_mono - self._last_cal_webhook_mono < CAL_WEBHOOK_MIN_INTERVAL_S:
            return
        now_ms = int(time.time() * 1000)
        payload: dict[str, Any] = {
            "session_id": self.session_id,
            "streaming": True,
            "updated_at_ms": now_ms,
            "calibration": cal,
        }
        if self.last_imu_quat is not None:
            payload["imu_game_rotation"] = dict(self.last_imu_quat)
            mount = self._with_engine(self.engine.get_imu_to_body)
            payload["collar_rotation"] = imu_quat_to_body_frame(
                self.last_imu_quat,
                mount,
            )
        post_pose_webhook(payload)
        self._last_cal_webhook_mono = now_mono

    def _build_calibration_payload(self) -> dict[str, Any] | None:
        if self.auto_cal is None or self.auto_cal.stopped:
            return self.last_calibration
        return self.auto_cal.calibration_payload()

    def handle_start(self, *, from_console: bool = False) -> None:
        self.session_id = str(uuid.uuid4())
        self.live_display = True
        self.last_pose_step = 0
        self.last_push_ms = 0
        self.last_activity = time.monotonic()
        self.last_sensor_ts_us = None
        self._with_engine(self.engine.reset)
        LOG.info("Live display started session %s (%s)", self.session_id, self.addr)
        self.push_pose(streaming=True, force=True)
        if from_console:
            print("Live display ON — poses will POST to Vercel.")
        else:
            self.send_ack("start", {"stream": self.stream_buffer.stream_status()})

    def ensure_live_display(self) -> None:
        """Turn on webhook streaming without resetting fusion state (e.g. mid auto-cal)."""
        if self.live_display:
            return
        self.session_id = str(uuid.uuid4())
        self.live_display = True
        self.last_push_ms = 0
        self._last_cal_webhook_mono = 0.0
        LOG.info("Live display enabled for calibration session %s (%s)", self.session_id, self.addr)
        self.push_calibration_update(
            self._build_calibration_payload() or {"phase": "unknown"},
            force=True,
        )

    def handle_end(self, *, from_console: bool = False) -> None:
        self.stream_buffer.flush()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not self._tick_queue.empty():
            time.sleep(0.02)
        if self.live_display:
            self.live_display = False
            self.push_pose(streaming=False, force=True)
            LOG.info("Live display stopped session %s (%s)", self.session_id, self.addr)
        if from_console:
            print("Live display OFF.")
        else:
            self.send_ack("end")

    def _ingest_bundled_sensor(self, msg: dict[str, Any], ts_us: int) -> None:
        def ingest() -> None:
            quat = msg.get("quat")
            if quat:
                self.engine.submit_quat(
                    float(quat["w"]), float(quat["x"]),
                    float(quat["y"]), float(quat["z"]),
                    ts_us,
                )

            gyro = msg.get("gyro")
            if gyro:
                self.engine.submit_gyro(
                    float(gyro["x"]), float(gyro["y"]), float(gyro["z"]),
                    ts_us,
                )

            accel = msg.get("accel")
            if accel:
                self.engine.submit_accel(
                    float(accel["x"]), float(accel["y"]), float(accel["z"]),
                    ts_us,
                )

            flow = msg.get("flow")
            if flow and not IMU_ONLY_MODE:
                self.engine.submit_flow(
                    int(flow["dx"]), int(flow["dy"]),
                    int(flow.get("quality", 255)),
                    ts_us,
                )

            range_data = msg.get("range")
            if range_data and not IMU_ONLY_MODE:
                self.engine.submit_range(int(range_data["mm"]), ts_us)

        self._with_engine(ingest)
        self.last_activity = time.monotonic()
        self.last_sensor_ts_us = ts_us

    def _update_imu_quat(self, quat: dict[str, Any]) -> None:
        self.last_imu_quat = {
            "w": float(quat["w"]),
            "x": float(quat["x"]),
            "y": float(quat["y"]),
            "z": float(quat["z"]),
        }

    def _on_stream_tick(self, msg: dict[str, Any]) -> None:
        self._enqueue_stream_tick(msg)

    def _process_stream_tick(self, msg: dict[str, Any]) -> None:
        ts_us = int(msg["ts_us"])

        if self.auto_cal is not None and self.auto_cal.active:
            cal_accepted = False
            if self.calibrating:
                cal_accepted = self._feed_lever_arm_cal(msg, ts_us)
            if self.auto_cal.needs_stream_ticks or self.calibrating:
                self.last_sensor_ts_us = ts_us
                self.auto_cal.on_sensor_tick(msg, cal_accepted=cal_accepted)

        if not self.live_display:
            return

        self._ingest_bundled_sensor(msg, ts_us)

        pose = self._with_engine(self.engine.get_pose)
        if pose and pose["step_count"] > self.last_pose_step:
            self.last_pose_step = pose["step_count"]
            self.push_pose(streaming=True)

    def notify_collar_stream_start(self) -> None:
        """Ask the collar firmware to begin streaming sensor samples."""
        try:
            self.conn.sendall(b"STREAM_START\n")
            self._stream_start_sent = True
            LOG.info("Sent STREAM_START to collar %s", self.addr)
        except OSError as exc:
            LOG.warning("Could not send STREAM_START to %s: %s", self.addr, exc)

    def handle_sensor(self, msg: dict[str, Any]) -> None:
        ts_us = int(msg.get("ts_us", now_us()))
        self.last_activity = time.monotonic()
        self.last_packet_at = self.last_activity
        self.packets_received += 1

        quat = msg.get("quat")
        if quat is not None:
            self._update_imu_quat(quat)
            self._note_rotation_rx(quat)
            if (
                self.auto_cal is not None
                and self.auto_cal.active
                and self.auto_cal.phase == STATUS_POINT_UP
            ):
                self.auto_cal.on_upright_quat(quat)

        cal_status = self._with_engine(self.engine.lever_arm_cal_status)

        if cal_status.get("active") and not self.live_display:
            self._feed_lever_arm_cal(msg, ts_us)
            self.last_sensor_ts_us = ts_us
            self.send_ack(
                "sensor",
                {
                    "cal_feed": True,
                    "cal_status": self._with_engine(self.engine.lever_arm_cal_status),
                },
            )
            return

        if not self.live_display:
            if self.auto_cal is not None and self.auto_cal.active:
                self._ingest_bundled_sensor(msg, ts_us)
                return
            self.send_ack("sensor", {"error": "live display not started — use 'display start' on server console"})
            return

        self._ingest_bundled_sensor(msg, ts_us)

        if cal_status.get("active"):
            self._feed_lever_arm_cal(msg, ts_us)

        pose = self._with_engine(self.engine.get_pose)
        if pose and pose["step_count"] > self.last_pose_step:
            self.last_pose_step = pose["step_count"]
            self.push_pose(streaming=True)

    def _dispatch_stream_samples(
        self,
        samples: list[tuple[int, int, dict[str, Any]]],
    ) -> None:
        """Unpack is done; feed the time-ordered stream and upright cal from raw quats."""
        if not samples:
            return

        self.last_activity = time.monotonic()
        self.last_packet_at = self.last_activity
        self.packets_received += 1

        upright_cal = (
            self.auto_cal is not None
            and self.auto_cal.active
            and self.auto_cal.phase == STATUS_POINT_UP
        )

        stream_samples: list[tuple[int, int, dict[str, Any]]] = []
        last_quat: dict[str, Any] | None = None
        for sensor, ts_us, data in samples:
            if IMU_ONLY_MODE and sensor in (SENSOR_FLOW, SENSOR_RADAR):
                continue
            if sensor == SENSOR_QUAT:
                last_quat = data
                self._note_rotation_rx(data)
                if upright_cal:
                    self.auto_cal.on_upright_quat(data)
            stream_samples.append((sensor, ts_us, data))

        if last_quat is not None:
            self._update_imu_quat(last_quat)

        if not stream_samples:
            return

        if len(stream_samples) == 1:
            self.stream_buffer.ingest(*stream_samples[0])
        else:
            self.stream_buffer.ingest_sequence(stream_samples)

        if self.live_display and last_quat is not None:
            spinning = (
                self.auto_cal is not None
                and self.auto_cal.phase in (STATUS_FRAME_SPIN, STATUS_LEVER_SPIN)
            )
            if not spinning:
                self.push_pose(streaming=True, imu_only=True)

    def handle_line(self, line: str) -> None:
        if self._shutdown_requested:
            return
        active = get_active_collar_session()
        if active is not None and active is not self:
            return

        get_sensor_recorder().record_line(line)

        samples = unpack_collar_wire_line(line)
        if samples:
            quat_count = sum(1 for s in samples if s.sensor == SENSOR_QUAT)
            if len(samples) > 1:
                LOG.debug(
                    "Collar batch from %s — %d samples (%d quat, ts %d..%d us)",
                    self.addr,
                    len(samples),
                    quat_count,
                    samples[0].ts_us,
                    samples[-1].ts_us,
                )
            elif quat_count == 0 and line.strip().startswith("[["):
                LOG.warning(
                    "Collar batch from %s unpacked to %d samples but 0 quaternions "
                    "(expected wire type 0 with [x,y,z,w])",
                    self.addr,
                    len(samples),
                )
            self._dispatch_stream_samples(
                [(s.sensor, s.ts_us, s.data) for s in samples],
            )
            return

        if line.strip().startswith("["):
            LOG.warning(
                "Failed to unpack collar wire line from %s (%d bytes): %s",
                self.addr,
                len(line),
                line[:160],
            )
            return

        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            LOG.warning("Invalid JSON from %s: %s", self.addr, line[:120])
            return

        msg_type = msg.get("type")
        if msg_type == "sensor":
            self.handle_sensor(msg)
        else:
            LOG.debug("Ignoring legacy control message %r from collar", msg_type)

    def idle_watchdog(self) -> None:
        if STREAM_IDLE_TIMEOUT_S <= 0:
            return
        while self.live_display:
            if time.monotonic() - self.last_activity > STREAM_IDLE_TIMEOUT_S:
                LOG.info("Session %s idle timeout — stopping live display", self.session_id)
                self.handle_end(from_console=True)
                break
            time.sleep(0.25)

    def _log_disconnect(self, reason: str) -> None:
        duration_s = time.monotonic() - self.connected_at
        LOG.info(
            "Collar disconnected from %s after %.1fs — %s (%d packets)",
            self.addr,
            duration_s,
            reason,
            self.packets_received,
        )
        record_collar_event(
            "disconnect",
            addr=f"{self.addr[0]}:{self.addr[1]}",
            session_id=self.session_id,
            duration_s=round(duration_s, 2),
            reason=reason,
            packets_received=self.packets_received,
        )

    def run(self) -> None:
        prev = get_active_collar_session()
        if prev is not None and prev is not self:
            if self._should_reject_as_duplicate(prev):
                LOG.warning(
                    "Ignoring duplicate collar TCP from %s — keeping active session %s "
                    "(%d packets, idle %.1fs)",
                    self.addr,
                    prev.addr,
                    prev.packets_received,
                    time.monotonic() - prev.last_activity,
                )
                try:
                    self.conn.close()
                except OSError:
                    pass
                return
            LOG.info(
                "Replacing active collar session %s with new connection from %s",
                prev.addr,
                self.addr,
            )
            prev.request_shutdown("replaced by new collar connection")
        set_active_collar_session(self)
        record_collar_event(
            "connect",
            addr=f"{self.addr[0]}:{self.addr[1]}",
            session_id=self.session_id,
        )
        LOG.info("Collar connected from %s — streaming packets", self.addr)

        if AUTO_CAL_ON_CONNECT:
            self.notify_collar_stream_start()
            self._schedule_stream_start_retries()
            self.auto_cal = CollarAutoCal(self)
            self.auto_cal.start()

        if STREAM_IDLE_TIMEOUT_S > 0:
            watchdog = threading.Thread(target=self.idle_watchdog, daemon=True)
            watchdog.start()

        buffer = b""
        self._recv_loop_active = True
        try:
            while True:
                chunk = self.conn.recv(65536)
                if not chunk:
                    self._mark_disconnected("peer closed connection")
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line_bytes, buffer = buffer.split(b"\n", 1)
                    if not line_bytes.strip():
                        continue
                    if len(line_bytes) > MAX_LINE_BYTES:
                        LOG.warning("Line too large from %s", self.addr)
                        continue
                    try:
                        self._enqueue_line(line_bytes.decode("utf-8"))
                    except UnicodeDecodeError:
                        LOG.warning("Invalid UTF-8 line from %s", self.addr)
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            self._mark_disconnected(str(exc))
        except Exception as exc:
            LOG.exception("Collar session %s crashed", self.session_id)
            self._mark_disconnected(f"server error: {exc}")
        finally:
            self._recv_loop_active = False
            if get_active_collar_session() is self:
                set_active_collar_session(None)
                set_collar_status(STATUS_IDLE)
            if buffer.strip():
                try:
                    self._enqueue_line(buffer.decode("utf-8"))
                except UnicodeDecodeError:
                    LOG.warning(
                        "Dropping trailing partial line from %s (%d bytes)",
                        self.addr,
                        len(buffer),
                    )
            if self.auto_cal is not None:
                self.auto_cal.stop()
                self.auto_cal = None
            if self.calibrating:
                self._with_engine(self.engine.lever_arm_cal_cancel)
                self.calibrating = False
            self.stop_rotation_trace()
            self.stop_line_worker()
            self.stop_tick_worker()
            if self.live_display:
                self.handle_end(from_console=True)
            self.close()


def serve() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    settings_path = active_settings_path()
    if settings_path.is_file():
        LOG.info("Loaded settings from %s", settings_path)
    else:
        LOG.warning(
            "No settings file at %s — copy fusion_server.json.example and edit, "
            "or set FUSION_SERVER_CONFIG",
            settings_path,
        )

    if IMU_ONLY_MODE:
        LOG.info("IMU-only barbell mode — optical flow and radar disabled")
    else:
        LOG.info("Full fusion mode — optical flow and radar required for EKF steps")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((SERVER_HOST, SERVER_PORT))
    sock.listen(8)
    if STREAM_FIXED_LATENCY_US is None:
        LOG.info(
            "Fusion server listening on %s:%d (adaptive stream latency)",
            SERVER_HOST, SERVER_PORT,
        )
    else:
        LOG.info(
            "Fusion server listening on %s:%d (fixed stream latency %.0f ms)",
            SERVER_HOST, SERVER_PORT, STREAM_FIXED_LATENCY_US / 1000.0,
        )

    if sys.stdin.isatty():
        start_admin_console_thread()
        LOG.info("Interactive admin console — type 'help' at the fusion> prompt")
    else:
        LOG.info("No TTY — use: python3 fusion_admin.py")

    start_admin_socket_thread()
    start_collar_status_server(CAL_STATUS_HOST, CAL_STATUS_PORT)
    set_collar_status(STATUS_IDLE)

    while True:
        conn, addr = sock.accept()
        LOG.info("Connection from %s", addr)
        session = ClientSession(conn, addr)
        thread = threading.Thread(target=session.run, daemon=True)
        thread.start()


if __name__ == "__main__":
    serve()
