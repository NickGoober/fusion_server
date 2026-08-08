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
"""

from __future__ import annotations

import json
import logging
import socket
import sys
import threading
import time
import uuid
from collections import deque
from typing import Any
from urllib import error, request

from fusion_admin_dispatch import admin_console_loop, dispatch_admin_command, start_admin_console_thread
from fusion_calib import write_lever_arm_calib
from fusion_lib import FusionEngine
from fusion_settings import (
    active_settings_path,
    get_float_setting,
    get_int_setting,
    get_setting,
)
from sensor_stream import (
    SensorStreamBuffer,
    parse_sample_line,
)

LOG = logging.getLogger("fusion_server")

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
STREAM_OUTPUT_HZ = get_float_setting("STREAM_OUTPUT_HZ", 100.0)
MAX_LINE_BYTES = get_int_setting("MAX_LINE_BYTES", 65536)


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
        LOG.error("Webhook HTTP error %s: %s", exc.code, exc.read())
    except error.URLError as exc:
        LOG.error("Webhook URL error: %s", exc.reason)


_engine: FusionEngine | None = None
_engine_lock = threading.Lock()
_cal_meta: dict[str, Any] = {"axis": "auto", "omega_rad_s": 0.0}
_active_collar_session: ClientSession | None = None
_collar_session_lock = threading.Lock()
_collar_events: deque[dict[str, Any]] = deque(maxlen=50)
_collar_events_lock = threading.Lock()
_last_disconnect: dict[str, Any] | None = None


def record_collar_event(kind: str, **fields: Any) -> None:
    global _last_disconnect
    event = {
        "t_utc": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        "kind": kind,
        **fields,
    }
    with _collar_events_lock:
        _collar_events.append(event)
        if kind == "disconnect":
            _last_disconnect = event


def get_collar_events(limit: int = 20) -> list[dict[str, Any]]:
    with _collar_events_lock:
        return list(_collar_events)[-limit:]


def get_last_collar_disconnect() -> dict[str, Any] | None:
    with _collar_events_lock:
        return dict(_last_disconnect) if _last_disconnect else None


def get_active_collar_session() -> ClientSession | None:
    with _collar_session_lock:
        return _active_collar_session


def _set_active_collar_session(session: ClientSession | None) -> None:
    global _active_collar_session
    with _collar_session_lock:
        _active_collar_session = session


def get_fusion_engine() -> FusionEngine:
    global _engine
    with _engine_lock:
        if _engine is None:
            _engine = FusionEngine()
            arm = _engine.get_flow_lever_arm()
            imu_arm = _engine.get_imu_lever_arm()
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
        self.last_pose_step = 0
        self.last_activity = time.monotonic()
        self.engine = get_fusion_engine()
        self.last_sensor_ts_us: int | None = None
        self.stream_buffer = SensorStreamBuffer(
            fixed_latency_us=STREAM_FIXED_LATENCY_US,
            min_latency_us=STREAM_MIN_LATENCY_US,
            max_latency_us=STREAM_MAX_LATENCY_US,
            output_hz=STREAM_OUTPUT_HZ,
            on_tick=self._on_stream_tick,
            on_latency_change=self._on_stream_latency_change,
        )

    def _on_stream_latency_change(
        self, latency_us: int, sensor_periods_ms: dict[str, float],
    ) -> None:
        LOG.info(
            "Session %s adaptive latency -> %.0f ms (sensor periods ms: %s)",
            self.session_id,
            latency_us / 1000.0,
            sensor_periods_ms,
        )

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
        flow = msg.get("flow")
        range_data = msg.get("range")
        if not gyro or not accel or not flow or not range_data:
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
            f"Packets received: {self.packets_received}",
            f"Live display (Vercel): {'ON' if self.live_display else 'off'}",
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
                print("Calibration started. Spin the collar about one axis for 5+ seconds.")
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

    def push_pose(self, streaming: bool) -> None:
        pose = self._with_engine(self.engine.get_pose)
        if pose is None:
            return

        payload = {
            "session_id": self.session_id,
            "streaming": streaming,
            "updated_at_ms": int(time.time() * 1000),
            "pose": pose,
        }
        post_pose_webhook(payload)

    def handle_start(self, *, from_console: bool = False) -> None:
        self.session_id = str(uuid.uuid4())
        self.live_display = True
        self.last_pose_step = 0
        self.last_activity = time.monotonic()
        self.last_sensor_ts_us = None
        self._with_engine(self.engine.reset)
        LOG.info("Live display started session %s (%s)", self.session_id, self.addr)
        if from_console:
            print("Live display ON — poses will POST to Vercel.")
        else:
            self.send_ack("start", {"stream": self.stream_buffer.stream_status()})

    def handle_end(self, *, from_console: bool = False) -> None:
        self.stream_buffer.flush()
        if self.live_display:
            self.live_display = False
            self.push_pose(streaming=False)
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
            if flow:
                self.engine.submit_flow(
                    int(flow["dx"]), int(flow["dy"]),
                    int(flow.get("quality", 255)),
                    ts_us,
                )

            range_data = msg.get("range")
            if range_data:
                self.engine.submit_range(int(range_data["mm"]), ts_us)

        self._with_engine(ingest)
        self.last_activity = time.monotonic()
        self.last_sensor_ts_us = ts_us

    def _on_stream_tick(self, msg: dict[str, Any]) -> None:
        ts_us = int(msg["ts_us"])
        cal_status = self._with_engine(self.engine.lever_arm_cal_status)

        if cal_status.get("active"):
            self._feed_lever_arm_cal(msg, ts_us)
            self.last_sensor_ts_us = ts_us

        if not self.live_display:
            return

        self._ingest_bundled_sensor(msg, ts_us)

        pose = self._with_engine(self.engine.get_pose)
        if pose and pose["step_count"] > self.last_pose_step:
            self.last_pose_step = pose["step_count"]
            self.push_pose(streaming=True)

    def handle_sensor(self, msg: dict[str, Any]) -> None:
        ts_us = int(msg.get("ts_us", now_us()))
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
            self.send_ack("sensor", {"error": "live display not started — use 'display start' on server console"})
            return

        self._ingest_bundled_sensor(msg, ts_us)

        if cal_status.get("active"):
            self._feed_lever_arm_cal(msg, ts_us)

        pose = self._with_engine(self.engine.get_pose)
        if pose and pose["step_count"] > self.last_pose_step:
            self.last_pose_step = pose["step_count"]
            self.push_pose(streaming=True)

    def handle_stream_sample(self, sensor: int, ts_us: int, data: dict[str, Any]) -> None:
        self.last_activity = time.monotonic()
        self.last_packet_at = self.last_activity
        self.packets_received += 1
        self.stream_buffer.ingest(sensor, ts_us, data)

    def handle_line(self, line: str) -> None:
        parsed = parse_sample_line(line)
        if parsed is not None:
            sensor, ts_us, data = parsed
            self.handle_stream_sample(sensor, ts_us, data)
            return

        if not line.strip().startswith("{"):
            LOG.debug("Unrecognized line from %s: %s", self.addr, line[:120])
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
            LOG.info(
                "Replacing active collar session %s with new connection from %s",
                prev.addr,
                self.addr,
            )
        _set_active_collar_session(self)
        record_collar_event(
            "connect",
            addr=f"{self.addr[0]}:{self.addr[1]}",
            session_id=self.session_id,
        )
        LOG.info("Collar connected from %s — streaming packets", self.addr)

        if STREAM_IDLE_TIMEOUT_S > 0:
            watchdog = threading.Thread(target=self.idle_watchdog, daemon=True)
            watchdog.start()

        buffer = b""
        try:
            while True:
                chunk = self.conn.recv(4096)
                if not chunk:
                    self._log_disconnect("peer closed connection")
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
                        self.handle_line(line_bytes.decode("utf-8"))
                    except Exception as exc:
                        LOG.warning(
                            "Packet error from %s: %s — %r",
                            self.addr,
                            exc,
                            line_bytes[:120],
                            exc_info=True,
                        )
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            self._log_disconnect(str(exc))
        except Exception as exc:
            LOG.exception("Collar session %s crashed", self.session_id)
            self._log_disconnect(f"server error: {exc}")
        finally:
            if get_active_collar_session() is self:
                _set_active_collar_session(None)
            if self.live_display:
                self.handle_end(from_console=True)
            self.close()


def serve() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
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

    while True:
        conn, addr = sock.accept()
        LOG.info("Connection from %s", addr)
        session = ClientSession(conn, addr)
        thread = threading.Thread(target=session.run, daemon=True)
        thread.start()


if __name__ == "__main__":
    serve()
