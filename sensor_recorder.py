"""
Record and replay collar sensor stream lines (JSONL).

Wire format per line: [sensor_type, timestamp, [data...]]

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


def default_record_dir() -> Path:
    raw = get_setting("RECORD_DIR")
    if raw:
        return Path(raw)
    return Path(__file__).resolve().parent / "recordings"


def _default_capture_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    return default_record_dir() / f"capture_{stamp}.jsonl"


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
    ) -> Path:
        with self._lock:
            if self._recording:
                raise RuntimeError(
                    f"Already recording to {self._path} — run 'record stop' first."
                )
            if self._replay_active_locked():
                raise RuntimeError("Replay in progress — run 'replay stop' first.")

            out = Path(path) if path else _default_capture_path()
            if not out.is_absolute():
                out = default_record_dir() / out
            out.parent.mkdir(parents=True, exist_ok=True)

            self._file = open(out, "w", encoding="utf-8")
            meta = {
                "_fusion_record": RECORD_VERSION,
                "version": RECORD_VERSION,
                "started_at_ms": int(time.time() * 1000),
                "session_id": session_id,
                "remote_addr": remote_addr,
            }
            self._file.write(json.dumps(meta, separators=(",", ":")) + "\n")
            self._file.flush()

            self._recording = True
            self._path = out
            self._started_at_ms = meta["started_at_ms"]
            self._sample_count = 0
            self._session_id = session_id
            self._remote_addr = remote_addr
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
            "sent": 0,
            "total": 0,
        }
        self._replay_thread = threading.Thread(
            target=self._replay_loop,
            args=(path, host, port, speed, realtime),
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
    ) -> None:
        samples, ts_scale = _load_sample_lines(path)
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


def _load_sample_lines(path: Path) -> tuple[list[tuple[int, str]], float]:
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
            try:
                arr = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(arr, list) or len(arr) < 2:
                continue
            ts_raw = int(arr[1])
            raw_ts.append(ts_raw)
            samples.append((ts_raw, line))
    samples.sort(key=lambda s: s[0])
    ts_scale = detect_timestamp_scale(raw_ts)
    return samples, ts_scale


_recorder = SensorRecorder()


def get_sensor_recorder() -> SensorRecorder:
    return _recorder
