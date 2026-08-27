"""Fusion server settings loaded from fusion_server.json / config.json."""

from __future__ import annotations

from fusion_settings import get_bool_setting, get_float_setting, get_int_setting, get_setting


def _parse_fixed_latency_us() -> int | None:
    raw = (get_setting("STREAM_LATENCY_S", "0") or "0").strip().lower()
    if raw in ("auto", ""):
        return None
    seconds = float(raw)
    if seconds < 0:
        return None
    return int(seconds * 1_000_000)


SERVER_HOST = get_setting("SERVER_HOST", "0.0.0.0")
SERVER_PORT = get_int_setting("SERVER_PORT", 9000)
VERCEL_WEBHOOK_URL = get_setting("VERCEL_WEBHOOK_URL", "") or ""
WEBHOOK_SECRET = get_setting("WEBHOOK_SECRET", "") or ""
# When true, accumulate frames locally and POST the full timeline once on display stop.
# Live apps should leave this false (default).
WEBHOOK_BATCH_MODE = get_bool_setting("WEBHOOK_BATCH_MODE", False)
# HTTP webhook min interval (viewer). TCP pose stream is never throttled.
WEBHOOK_MIN_INTERVAL_MS = get_int_setting("WEBHOOK_MIN_INTERVAL_MS", 50)
POSE_STREAM_ENABLE = get_bool_setting("POSE_STREAM_ENABLE", True)
POSE_STREAM_HOST = get_setting("POSE_STREAM_HOST", "0.0.0.0")
POSE_STREAM_PORT = get_int_setting("POSE_STREAM_PORT", 9002)
POSE_STREAM_SECRET = get_setting("POSE_STREAM_SECRET", "") or ""
STREAM_IDLE_TIMEOUT_S = get_float_setting("STREAM_IDLE_TIMEOUT_S", 0.0)
STREAM_FIXED_LATENCY_US = _parse_fixed_latency_us()
STREAM_MIN_LATENCY_US = int(get_float_setting("STREAM_MIN_LATENCY_S", 0.0) * 1_000_000)
STREAM_MAX_LATENCY_US = int(get_float_setting("STREAM_MAX_LATENCY_S", 0.05) * 1_000_000)
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
# World-frame barbell position Kalman (website uses filtered; pose_raw is direct integration).
POSITION_KALMAN_ENABLE = get_bool_setting("POSITION_KALMAN_ENABLE", True)
POSITION_KALMAN_PROCESS_NOISE_VEL = get_float_setting("POSITION_KALMAN_PROCESS_NOISE_VEL", 0.5)
POSITION_KALMAN_RANGE_STD_M = get_float_setting("POSITION_KALMAN_RANGE_STD_M", 0.003)
POSITION_KALMAN_FLOW_STD_BASE_M = get_float_setting("POSITION_KALMAN_FLOW_STD_BASE_M", 0.002)
POSITION_KALMAN_INNOVATION_GATE_SIGMA = get_float_setting(
    "POSITION_KALMAN_INNOVATION_GATE_SIGMA", 3.0,
)
# Radar height glitch gate (filtered Y only). 1–2 frame spikes are rejected;
# three agreeing samples lock in as real motion. Interpolate by coasting vy.
POSITION_RADAR_MAX_SPEED_MPS = get_float_setting("POSITION_RADAR_MAX_SPEED_MPS", 2.5)
POSITION_RADAR_HAMPEL_WINDOW = get_int_setting("POSITION_RADAR_HAMPEL_WINDOW", 5)
POSITION_RADAR_HAMPEL_SIGMA = get_float_setting("POSITION_RADAR_HAMPEL_SIGMA", 3.5)
POSITION_RADAR_MAX_REJECT_STREAK = get_int_setting("POSITION_RADAR_MAX_REJECT_STREAK", 3)
PACKET_DEBUG_INTERVAL = get_int_setting("PACKET_DEBUG_INTERVAL", 0)
COLLAR_RAW_LOG_ONLY = get_bool_setting("COLLAR_RAW_LOG_ONLY", False)
COLLAR_RECV_TIMEOUT_S = get_float_setting("COLLAR_RECV_TIMEOUT_S", 5.0)
COLLAR_TCP_IDLE_DISCONNECT_S = get_float_setting("COLLAR_TCP_IDLE_DISCONNECT_S", 45.0)
COLLAR_DUPLICATE_REJECT_IDLE_S = get_float_setting(
    "COLLAR_DUPLICATE_REJECT_IDLE_S", 2.0,
)
