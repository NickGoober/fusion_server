"""Async POST of pose/calibration payloads to the Vercel viewer."""

from __future__ import annotations

import json
import logging
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any
from urllib import error, request

from server_config import VERCEL_WEBHOOK_URL, WEBHOOK_SECRET

LOG = logging.getLogger("fusion_server.webhook")

_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="webhook")


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

    _pool.submit(_send)
