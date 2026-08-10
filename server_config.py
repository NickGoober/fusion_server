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
STREAM_IDLE_TIMEOUT_S = get_float_setting("STREAM_IDLE_TIMEOUT_S", 0.0)
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
PACKET_DEBUG_INTERVAL = get_int_setting("PACKET_DEBUG_INTERVAL", 0)
COLLAR_RAW_LOG_ONLY = get_bool_setting("COLLAR_RAW_LOG_ONLY", False)
COLLAR_RECV_TIMEOUT_S = get_float_setting("COLLAR_RECV_TIMEOUT_S", 5.0)
COLLAR_TCP_IDLE_DISCONNECT_S = get_float_setting("COLLAR_TCP_IDLE_DISCONNECT_S", 45.0)
COLLAR_DUPLICATE_REJECT_IDLE_S = get_float_setting(
    "COLLAR_DUPLICATE_REJECT_IDLE_S", 2.0,
)
