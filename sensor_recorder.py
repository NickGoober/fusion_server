"""
Record and replay collar sensor stream lines (JSONL).

Wire format per line (any of):
  [sensor_type, timestamp, [data...]]                    — single sample
  [[type, ts, data], [type, ts, data], ...]             — nested 1s batch
  [type, ts, data, type, ts, data, ...]                   — flat batch

JSONL files start with a metadata object:
  {"_fusion_record": 1, "version": 1, ...}
"""

from __future__ import annotations

import json
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from fusion_settings import get_setting
from sensor_stream import detect_timestamp_scale

RECORD_VERSION = 1
# Collar wire types for IMU-only captures (quat + gravity/linear + optional accel).
IMU_WIRE_SENSOR_TYPES = frozenset({0, 1, 4})


def default_record_dir() -> Path:
    raw = get_setting("RECORD_DIR")
    if raw:
        return Path(raw)
    return Path(__file__).resolve().parent / "recordings"


def _default_capture_path(*, imu_only: bool = False) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    name = f"imu_{stamp}.jsonl" if imu_only else f"capture_{stamp}.jsonl"
    return default_record_dir() / name


def filter_imu_wire_line(line: str) -> str | None:
    """
    Keep only collar IMU wire rows (type 0 = quat, type 1 = gravity/linear, type 4 = accel).

    Returns a re-encoded line, or None when no IMU samples are present.
    """
    line = line.strip()
    if not line.startswith("["):
        return None
    try:
        raw = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(raw, list) or not raw:
        return None

    def _keep(sensor: Any) -> bool:
        try:
            return int(sensor) in IMU_WIRE_SENSOR_TYPES
        except (TypeError, ValueError):
            return False

    if isinstance(raw[0], list):
        kept = [
            row for row in raw
            if isinstance(row, list) and len(row) >= 1 and _keep(row[0])
        ]
        if not kept:
            return None
        return json.dumps(kept, separators=(",", ":"))

    if len(raw) >= 3 and len(raw) % 3 == 0 and isinstance(raw[2], list):
        kept_flat: list[Any] = []
        for offset in range(0, len(raw), 3):
            if _keep(raw[offset]):
                kept_flat.extend(raw[offset: offset + 3])
        if not kept_flat:
            return None
        return json.dumps(kept_flat, separators=(",", ":"))

    if len(raw) >= 2 and _keep(raw[0]):
        return json.dumps(raw, separators=(",", ":"))
    return None


class SensorRecorder:
    """Thread-safe sensor stream recorder and replay controller."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._recording = False
        self._path: Path | None = None
        self._file = None
        self._started_at_ms = 0
        self._sample_count = 0
        self._session_id: str | None = None
        self._remote_addr: str | None = None
        self._filter_mode = "all"

        self._replay_thread: threading.Thread | None = None
        self._replay_stop = threading.Event()
        self._replay_path: Path | None = None
        self._replay_progress: dict[str, Any] = {}

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "recording": self._recording,
                "path": str(self._path) if self._path else None,
                "samples": self._sample_count,
                "session_id": self._session_id,
                "remote_addr": self._remote_addr,
                "filter": self._filter_mode,
                "replay_active": self._replay_active_locked(),
                "replay": dict(self._replay_progress),
            }

    def _replay_active_locked(self) -> bool:
        return self._replay_thread is not None and self._replay_thread.is_alive()

    def start(
        self,
        path: Path | str | None = None,
        *,
        session_id: str | None = None,
        remote_addr: str | None = None,
        filter_mode: str = "all",
    ) -> Path:
        with self._lock:
            if self._recording:
                raise RuntimeError(
                    f"Already recording to {self._path} — run 'record stop' first."
                )
            if self._replay_active_locked():
                raise RuntimeError("Replay in progress — run 'replay stop' first.")

            imu_only = filter_mode == "imu"
            out = Path(path) if path else _default_capture_path(imu_only=imu_only)
            if not out.is_absolute():
                out = default_record_dir() / out
            if out.suffix.lower() not in (".jsonl", ".json"):
                out = out.with_suffix(".jsonl")
            out.parent.mkdir(parents=True, exist_ok=True)

            self._file = open(out, "w", encoding="utf-8")
            meta = {
                "_fusion_record": RECORD_VERSION,
                "version": RECORD_VERSION,
                "started_at_ms": int(time.time() * 1000),
                "session_id": session_id,
                "remote_addr": remote_addr,
                "filter": filter_mode,
            }
            self._file.write(json.dumps(meta, separators=(",", ":")) + "\n")
            self._file.flush()

            self._recording = True
            self._path = out
            self._started_at_ms = meta["started_at_ms"]
            self._sample_count = 0
            self._session_id = session_id
            self._remote_addr = remote_addr
            self._filter_mode = filter_mode
            return out

    def stop(self) -> Path | None:
        with self._lock:
            if not self._recording:
                return None
            path = self._path
            if self._file is not None:
                self._file.flush()
                self._file.close()
                self._file = None
            self._recording = False
            return path

    def record_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        with self._lock:
            if not self._recording or self._file is None:
                return
            if line.startswith("{") and '"_fusion_record"' in line:
                return
            if self._filter_mode == "imu":
                filtered = filter_imu_wire_line(line)
                if filtered is None:
                    return
                line = filtered
            self._file.write(line + "\n")
            self._sample_count += 1
            if self._sample_count % 50 == 0:
                self._file.flush()

    def start_replay(
        self,
        path: Path | str,
        *,
        host: str = "127.0.0.1",
        port: int = 9000,
        speed: float = 1.0,
        realtime: bool = True,
        expand_batches: bool = True,
    ) -> None:
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f"Capture not found: {path}")

        with self._lock:
            if self._replay_active_locked():
                raise RuntimeError("Replay already running — run 'replay stop' first.")

        self._replay_stop.clear()
        self._replay_path = path
        self._replay_progress = {
            "path": str(path),
            "host": host,
            "port": port,
            "speed": speed,
            "realtime": realtime,
            "expand_batches": expand_batches,
            "sent": 0,
            "total": 0,
        }
        self._replay_thread = threading.Thread(
            target=self._replay_loop,
            args=(path, host, port, speed, realtime, expand_batches),
            name="sensor-replay",
            daemon=True,
        )
        self._replay_thread.start()

    def stop_replay(self) -> None:
        self._replay_stop.set()
        thread = self._replay_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
        self._replay_thread = None

    def _replay_loop(
        self,
        path: Path,
        host: str,
        port: int,
        speed: float,
        realtime: bool,
        expand_batches: bool,
    ) -> None:
        samples, ts_scale = _load_sample_lines(path, expand_batches=expand_batches)
        if not samples:
            self._replay_progress["error"] = "no samples in file"
            return

        speed = max(speed, 0.01)
        self._replay_progress["total"] = len(samples)
        if realtime and len(samples) >= 2:
            span_s = (samples[-1][0] - samples[0][0]) / ts_scale
            self._replay_progress["estimated_duration_s"] = round(span_s / speed, 2)
            self._replay_progress["timestamp_scale"] = ts_scale

        try:
            sock = socket.create_connection((host, port), timeout=10.0)
        except OSError as exc:
            self._replay_progress["error"] = str(exc)
            return

        try:
            t0_raw = samples[0][0]
            replay_start = time.monotonic()
            for i, (ts_raw, line) in enumerate(samples):
                if self._replay_stop.is_set():
                    self._replay_progress["stopped"] = True
                    break

                if realtime and i > 0:
                    target_s = (ts_raw - t0_raw) / ts_scale / speed
                    delay = target_s - (time.monotonic() - replay_start)
                    while delay > 0 and not self._replay_stop.is_set():
                        time.sleep(min(delay, 0.05))
                        delay = target_s - (time.monotonic() - replay_start)

                if self._replay_stop.is_set():
                    self._replay_progress["stopped"] = True
                    break

                sock.sendall((line + "\n").encode("utf-8"))
                self._replay_progress["sent"] = i + 1
        except OSError as exc:
            self._replay_progress["error"] = str(exc)
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._replay_progress["done"] = True


def _replay_timestamp_from_wire_array(arr: list) -> int | None:
    """
    Device timestamp for replay pacing.

    Nested batches use the first row's timestamp; single/flat rows use arr[1].
    """
    if not isinstance(arr, list) or len(arr) < 2:
        return None
    if isinstance(arr[0], list):
        first = arr[0]
        if len(first) >= 2 and isinstance(first[1], (int, float)):
            return int(first[1])
        return None
    if isinstance(arr[1], (int, float)):
        return int(arr[1])
    return None


def _replay_timestamp_from_wire_row(row: list) -> int | None:
    if len(row) < 2 or not isinstance(row[1], (int, float)):
        return None
    return int(row[1])


def _expand_replay_wire_line(line: str) -> list[tuple[int, str]]:
    """
    Expand 1-second collar batches into per-sample wire lines for replay.

    Recordings like freeMoveFB.jsonl store ~1s of samples per JSONL line:
      [[type, ts, data], [type, ts, data], ...]

    Replaying those lines whole only delivers one TCP packet per second, so the
    fusion server and Vercel viewer see ~10 pose updates for a 10s capture.
    """
    try:
        arr = json.loads(line)
    except json.JSONDecodeError:
        return []

    if not isinstance(arr, list) or not arr:
        return []

    expanded: list[tuple[int, str]] = []

    if isinstance(arr[0], list):
        for row in arr:
            if not isinstance(row, list):
                continue
            ts_raw = _replay_timestamp_from_wire_row(row)
            if ts_raw is None:
                continue
            expanded.append((ts_raw, json.dumps(row, separators=(",", ":"))))
        return expanded

    if len(arr) >= 3 and len(arr) % 3 == 0 and isinstance(arr[2], list):
        for offset in range(0, len(arr), 3):
            row = arr[offset : offset + 3]
            ts_raw = _replay_timestamp_from_wire_row(row)
            if ts_raw is None:
                continue
            expanded.append((ts_raw, json.dumps(row, separators=(",", ":"))))
        return expanded

    ts_raw = _replay_timestamp_from_wire_array(arr)
    if ts_raw is None:
        return []
    return [(ts_raw, line)]


def _load_sample_lines(
    path: Path,
    *,
    expand_batches: bool = True,
) -> tuple[list[tuple[int, str]], float]:
    """Load replay units: (device_ts, raw_line) in device time order."""
    samples: list[tuple[int, str]] = []
    raw_ts: list[int] = []
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("{"):
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(obj, dict) and obj.get("_fusion_record"):
                    continue
                continue
            if not line.startswith("["):
                continue

            if expand_batches:
                units = _expand_replay_wire_line(line)
            else:
                try:
                    arr = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(arr, list):
                    continue
                ts_raw = _replay_timestamp_from_wire_array(arr)
                if ts_raw is None:
                    continue
                units = [(ts_raw, line)]

            for ts_raw, unit_line in units:
                raw_ts.append(ts_raw)
                samples.append((ts_raw, unit_line))

    samples.sort(key=lambda s: s[0])
    ts_scale = detect_timestamp_scale(raw_ts)
    return samples, ts_scale


_recorder = SensorRecorder()


def get_sensor_recorder() -> SensorRecorder:
    return _recorder
