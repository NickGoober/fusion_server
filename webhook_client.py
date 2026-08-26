"""Async POST of pose payloads to the Vercel viewer."""

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
            timeout_s = 120 if len(body) > 512_000 else 30
            with request.urlopen(req, timeout=timeout_s) as resp:
                LOG.debug("Webhook OK %s", resp.status)
        except error.HTTPError as exc:
            err_body = exc.read().decode("utf-8", errors="replace")
            LOG.error(
                "Webhook HTTP error %s to %s: %s",
                exc.code,
                VERCEL_WEBHOOK_URL,
                err_body[:200],
            )
        except error.URLError as exc:
            reason = exc.reason
            if isinstance(reason, OSError) and reason.errno == 111:
                LOG.error(
                    "Webhook connection refused to %s — is the pose viewer running "
                    "and reachable from this machine? (127.0.0.1 only works if the "
                    "viewer runs on the same host as fusion_server)",
                    VERCEL_WEBHOOK_URL,
                )
            else:
                LOG.error("Webhook URL error to %s: %s", VERCEL_WEBHOOK_URL, reason)

    _pool.submit(_send)
