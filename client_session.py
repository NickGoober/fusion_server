"""One collar TCP session: transport, wire parsing, fusion, and webhooks."""

from __future__ import annotations

import json
import logging
import queue
import socket
import sys
import threading
import time
import uuid
from typing import Any

from collar_connection import apply_admission
from collar_registry import (
    get_active_collar_session,
    record_collar_event,
    set_active_collar_session,
)
from collar_tcp import TcpIdleTimeout, TcpReadState, read_collar_tcp_lines
from collar_wire_handler import process_collar_line
from sensor_recorder import get_sensor_recorder
from sensor_stream import (
    SENSOR_FLOW,
    SENSOR_QUAT,
    SENSOR_RADAR,
    SensorStreamBuffer,
    imu_quat_to_body_frame,
)
from server_config import (
    COLLAR_RECV_TIMEOUT_S,
    COLLAR_TCP_IDLE_DISCONNECT_S,
    IMU_ONLY_MODE,
    LINE_QUEUE_MAX,
    MAX_LINE_BYTES,
    PACKET_DEBUG_INTERVAL,
    STREAM_FIXED_LATENCY_US,
    STREAM_IDLE_TIMEOUT_S,
    STREAM_MAX_LATENCY_US,
    STREAM_MIN_LATENCY_US,
    STREAM_OUTPUT_HZ,
)
from server_engine import get_fusion_engine, with_engine
from webhook_client import now_us, post_pose_webhook

LOG = logging.getLogger("fusion_server.session")


class ClientSession:
    """Handles one TCP client connection and its fusion stream."""

    def __init__(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        self.conn = conn
        self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.addr = addr
        self.session_id = str(uuid.uuid4())
        self.connected_at = time.monotonic()
        self.packets_received = 0
        self._raw_packets_received = 0
        self._tcp_lines_enqueued = 0
        self._tcp_bytes_received = 0
        self._logged_first_batch = False
        self.last_packet_at: float | None = None
        self.live_display = False
        self.last_pose_step = 0
        self.last_activity = time.monotonic()
        self.engine = get_fusion_engine()
        self.last_sensor_ts_us: int | None = None
        self.last_imu_quat: dict[str, float] | None = None
        self.last_push_ms: int = 0
        self._shutdown_requested = False
        self._recv_loop_active = False
        self._last_tcp_recv_at: float | None = None
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

    @staticmethod
    def _debug_print(msg: str) -> None:
        print(f"[collar debug] {msg}", file=sys.stderr, flush=True)

    def _log_raw_packet(self, line_bytes: bytes) -> None:
        """Log raw TCP payloads before decode, unpack, or queueing."""
        if PACKET_DEBUG_INTERVAL <= 0:
            return
        self._raw_packets_received += 1
        count = self._raw_packets_received
        preview = line_bytes[:160].decode("utf-8", errors="replace")
        if count == 1:
            self._debug_print(
                f"{self.addr} raw packet #1 ({len(line_bytes)} bytes): {preview}"
            )
        elif count % PACKET_DEBUG_INTERVAL == 0:
            active = get_active_collar_session()
            active_tag = "active" if active is self else "inactive"
            self._debug_print(
                f"{self.addr} raw packet #{count} [{active_tag}] "
                f"({len(line_bytes)} bytes): {preview}"
            )

    def _ingest_raw_line(self, line_bytes: bytes) -> None:
        """Record and queue one raw newline-delimited TCP payload."""
        if not line_bytes.strip():
            return
        if len(line_bytes) > MAX_LINE_BYTES:
            LOG.warning("Line too large from %s", self.addr)
            return
        self._tcp_lines_enqueued += 1
        self._tcp_bytes_received += len(line_bytes)
        self._log_raw_packet(line_bytes)
        try:
            self._enqueue_line(line_bytes.decode("utf-8"))
        except UnicodeDecodeError:
            LOG.warning("Invalid UTF-8 line from %s", self.addr)

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

    def _mark_disconnected(self, reason: str) -> None:
        self._recv_loop_active = False
        self._shutdown_requested = True
        if get_active_collar_session() is self:
            set_active_collar_session(None)
        self._log_disconnect(reason)

    def request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._recv_loop_active = False
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
        if not self.live_display:
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

    def console_status_text(self) -> str:
        buf = self.stream_buffer.stream_status()
        uptime_s = time.monotonic() - self.connected_at
        packet_age = ""
        if self.last_packet_at is not None:
            packet_age = f", last packet {time.monotonic() - self.last_packet_at:.1f}s ago"
        tcp_age = ""
        if self._last_tcp_recv_at is not None:
            tcp_age = f", last TCP {time.monotonic() - self._last_tcp_recv_at:.1f}s ago"
        imu_arm = self._with_engine(self.engine.get_imu_lever_arm)
        return "\n".join([
            f"Collar: connected from {self.addr[0]}:{self.addr[1]} "
            f"({uptime_s:.1f}s{packet_age}{tcp_age})",
            f"Mode: {'IMU-only (barbell)' if IMU_ONLY_MODE else 'flow + range + IMU'}",
            f"IMU lever arm (m): x={imu_arm['x']:.4f} y={imu_arm['y']:.4f} z={imu_arm['z']:.4f}",
            f"Packets received: {self.packets_received} "
            f"({self._tcp_lines_enqueued} TCP lines, {self._tcp_bytes_received} bytes)",
            f"Live display (Vercel): {'ON' if self.live_display else 'off'}",
            f"Buffer latency: {buf.get('latency_ms', '?')} ms",
        ])

    def console_display_start(self) -> None:
        self.handle_start(from_console=True)

    def console_display_stop(self) -> None:
        self.handle_end(from_console=True)

    def _with_engine(self, fn) -> Any:
        return with_engine(fn)

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
        post_pose_webhook(payload)
        self.last_push_ms = now_ms

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

    def _process_stream_tick(self, msg: dict[str, Any]) -> None:
        if not self.live_display:
            return

        ts_us = int(msg["ts_us"])
        self._ingest_bundled_sensor(msg, ts_us)

        pose = self._with_engine(self.engine.get_pose)
        if pose and pose["step_count"] > self.last_pose_step:
            self.last_pose_step = pose["step_count"]
            self.push_pose(streaming=True)

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

    def handle_sensor(self, msg: dict[str, Any]) -> None:
        ts_us = int(msg.get("ts_us", now_us()))
        self.last_activity = time.monotonic()
        self.last_packet_at = self.last_activity
        self.packets_received += 1

        quat = msg.get("quat")
        if quat is not None:
            self._update_imu_quat(quat)
            self._note_rotation_rx(quat)

        if not self.live_display:
            self.send_ack(
                "sensor",
                {"error": "live display not started — use 'display start' on server console"},
            )
            return

        self._ingest_bundled_sensor(msg, ts_us)

        pose = self._with_engine(self.engine.get_pose)
        if pose and pose["step_count"] > self.last_pose_step:
            self.last_pose_step = pose["step_count"]
            self.push_pose(streaming=True)

    def _dispatch_stream_samples(
        self,
        samples: list[tuple[int, int, dict[str, Any]]],
    ) -> None:
        """Unpack is done; feed the time-ordered stream from raw wire samples."""
        if not samples:
            return

        self.last_activity = time.monotonic()
        self.last_packet_at = self.last_activity
        self.packets_received += 1

        stream_samples: list[tuple[int, int, dict[str, Any]]] = []
        last_quat: dict[str, Any] | None = None
        for sensor, ts_us, data in samples:
            if IMU_ONLY_MODE and sensor in (SENSOR_FLOW, SENSOR_RADAR):
                continue
            if sensor == SENSOR_QUAT:
                last_quat = data
                self._note_rotation_rx(data)
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
            self.push_pose(streaming=True, imu_only=True)

    def handle_line(self, line: str) -> None:
        if self._shutdown_requested:
            return
        active = get_active_collar_session()
        if active is not None and active is not self:
            return

        get_sensor_recorder().record_line(line)
        process_collar_line(self, line)

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
        if not apply_admission(prev, self, self.conn):
            return

        set_active_collar_session(self)
        record_collar_event(
            "connect",
            addr=f"{self.addr[0]}:{self.addr[1]}",
            session_id=self.session_id,
        )
        LOG.info("Collar connected from %s — streaming packets", self.addr)

        if STREAM_IDLE_TIMEOUT_S > 0:
            watchdog = threading.Thread(target=self.idle_watchdog, daemon=True)
            watchdog.start()

        tcp_state = TcpReadState()
        self._recv_loop_active = True
        try:
            for line_bytes in read_collar_tcp_lines(
                self.conn,
                recv_timeout_s=COLLAR_RECV_TIMEOUT_S,
                idle_disconnect_s=COLLAR_TCP_IDLE_DISCONNECT_S,
                state=tcp_state,
            ):
                if tcp_state.last_recv_at is not None:
                    self._last_tcp_recv_at = tcp_state.last_recv_at
                    self.last_activity = tcp_state.last_recv_at
                self._ingest_raw_line(line_bytes)
            self._mark_disconnected("peer closed connection")
        except TcpIdleTimeout as exc:
            self._mark_disconnected(str(exc))
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            self._mark_disconnected(str(exc))
        except Exception as exc:
            LOG.exception("Collar session %s crashed", self.session_id)
            self._mark_disconnected(f"server error: {exc}")
        finally:
            self._recv_loop_active = False
            if get_active_collar_session() is self:
                set_active_collar_session(None)
            self.stop_rotation_trace()
            self.stop_line_worker()
            self.stop_tick_worker()
            if self.live_display:
                self.handle_end(from_console=True)
            self.close()

