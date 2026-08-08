"""
Shared collar connection state.

Kept in a separate module so admin commands see the same state when
fusion_server.py is run as `python3 fusion_server.py` (__main__).
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

_active_collar_session: Any = None
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


def get_active_collar_session() -> Any:
    with _collar_session_lock:
        return _active_collar_session


def set_active_collar_session(session: Any) -> None:
    global _active_collar_session
    with _collar_session_lock:
        _active_collar_session = session
