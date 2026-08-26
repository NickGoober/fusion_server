"""Fusion server settings loaded from fusion_server.json / config.json."""

from __future__ import annotations

from fusion_settings import get_bool_setting, get_float_setting, get_int_setting, get_setting


def _parse_fixed_latency_us() -> int | None:
    raw = (get_setting("STREAM_LATENCY_S", "auto") or "auto").strip().lower()
    if raw in ("auto", ""):
        return None
    seconds = float(raw)
    if seconds <= 0:
        return None
    return int(seconds * 1_000_000)


SERVER_HOST = get_setting("SERVER_HOST", "0.0.0.0")
SERVER_PORT = get_int_setting("SERVER_PORT", 9000)
VERCEL_WEBHOOK_URL = get_setting("VERCEL_WEBHOOK_URL", "") or ""
WEBHOOK_SECRET = get_setting("WEBHOOK_SECRET", "") or ""
# When true, accumulate frames locally and POST the full timeline once on display stop.
WEBHOOK_BATCH_MODE = get_bool_setting("WEBHOOK_BATCH_MODE", True)
STREAM_IDLE_TIMEOUT_S = get_float_setting("STREAM_IDLE_TIMEOUT_S", 0.0)
STREAM_FIXED_LATENCY_US = _parse_fixed_latency_us()
STREAM_MIN_LATENCY_US = int(get_float_setting("STREAM_MIN_LATENCY_S", 0.05) * 1_000_000)
STREAM_MAX_LATENCY_US = int(get_float_setting("STREAM_MAX_LATENCY_S", 2.0) * 1_000_000)
ADMIN_HOST = get_setting("ADMIN_HOST", "127.0.0.1")
ADMIN_PORT = get_int_setting("ADMIN_PORT", 9001)
STREAM_OUTPUT_HZ = get_float_setting("STREAM_OUTPUT_HZ", 100.0)
MAX_LINE_BYTES = get_int_setting("MAX_LINE_BYTES", 2_097_152)
LINE_QUEUE_MAX = get_int_setting("LINE_QUEUE_MAX", 4096)
IMU_ONLY_MODE = get_bool_setting("IMU_ONLY_MODE", True)
# Per-sensor fusion gates (ignored when IMU_ONLY_MODE is true).
FUSION_USE_OPTICAL_FLOW = get_bool_setting("FUSION_USE_OPTICAL_FLOW", True)
FUSION_USE_RANGE = get_bool_setting("FUSION_USE_RANGE", True)
FLOW_MAX_PIXELS_PER_FRAME = get_int_setting("FLOW_MAX_PIXELS_PER_FRAME", 40)
FLOW_MAX_PIXELS_PER_WINDOW = get_int_setting("FLOW_MAX_PIXELS_PER_WINDOW", 200)
FLOW_MIN_QUALITY = get_int_setting("FLOW_MIN_QUALITY", 25)
PACKET_DEBUG_INTERVAL = get_int_setting("PACKET_DEBUG_INTERVAL", 0)
COLLAR_RAW_LOG_ONLY = get_bool_setting("COLLAR_RAW_LOG_ONLY", False)
COLLAR_RECV_TIMEOUT_S = get_float_setting("COLLAR_RECV_TIMEOUT_S", 5.0)
COLLAR_TCP_IDLE_DISCONNECT_S = get_float_setting("COLLAR_TCP_IDLE_DISCONNECT_S", 45.0)
COLLAR_DUPLICATE_REJECT_IDLE_S = get_float_setting(
    "COLLAR_DUPLICATE_REJECT_IDLE_S", 2.0,
)
