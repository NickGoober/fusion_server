"""Fan out a live fused-pose frame to the app TCP stream and optional HTTP webhook."""

from __future__ import annotations

import time
from typing import Any

from pose_stream import broadcast_pose
from server_config import WEBHOOK_MIN_INTERVAL_MS
from webhook_client import post_pose_webhook

_last_webhook_ms = 0


def publish_live_pose(
    payload: dict[str, Any],
    *,
    force_webhook: bool = False,
    skip_webhook: bool = False,
) -> None:
    """Push one frame. TCP stream is always immediate; HTTP webhook may be rate-limited."""
    payload.setdefault("batch_mode", False)
    payload.setdefault("batch_complete", False)
    broadcast_pose(payload)
    if skip_webhook:
        return

    global _last_webhook_ms
    now_ms = int(time.time() * 1000)
    if (
        not force_webhook
        and WEBHOOK_MIN_INTERVAL_MS > 0
        and _last_webhook_ms
        and now_ms - _last_webhook_ms < WEBHOOK_MIN_INTERVAL_MS
    ):
        return
    _last_webhook_ms = now_ms
    post_pose_webhook(payload)
