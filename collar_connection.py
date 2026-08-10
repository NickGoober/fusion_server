"""
Collar TCP admission policy.

Only one active collar stream per host IP. A second socket is rejected only when
the existing session is still receiving TCP data; otherwise the stale session is
replaced so reconnects are not blocked by zombie recv loops.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any, Protocol

from server_config import COLLAR_DUPLICATE_REJECT_IDLE_S

if TYPE_CHECKING:
    pass

LOG = logging.getLogger("fusion_server.connection")


class AdmissionAction(Enum):
    ADMIT = "admit"
    REJECT_DUPLICATE = "reject_duplicate"
    REPLACE_STALE = "replace_stale"


@dataclass(frozen=True)
class AdmissionDecision:
    action: AdmissionAction
    prev_addr: tuple[str, int] | None = None
    prev_packets: int = 0
    prev_idle_tcp_s: float | None = None
    prev_idle_activity_s: float | None = None


class CollarSessionView(Protocol):
    """Minimal session surface needed for admission decisions."""

    addr: tuple[str, int]
    packets_received: int
    last_activity: float
    _recv_loop_active: bool
    _shutdown_requested: bool
    _last_tcp_recv_at: float | None

    def request_shutdown(self, reason: str) -> None: ...


def evaluate_admission(
    prev: CollarSessionView | None,
    incoming: CollarSessionView,
    *,
    duplicate_reject_idle_s: float = COLLAR_DUPLICATE_REJECT_IDLE_S,
) -> AdmissionDecision:
    """Decide whether to admit, reject, or replace on a new TCP connection."""
    if prev is None or prev is incoming:
        return AdmissionDecision(action=AdmissionAction.ADMIT)

    if prev.addr[0] != incoming.addr[0]:
        return AdmissionDecision(action=AdmissionAction.ADMIT)

    prev_idle_activity_s = time.monotonic() - prev.last_activity

    if not prev._recv_loop_active or prev._shutdown_requested:
        return AdmissionDecision(
            action=AdmissionAction.REPLACE_STALE,
            prev_addr=prev.addr,
            prev_packets=prev.packets_received,
            prev_idle_activity_s=prev_idle_activity_s,
        )

    if prev._last_tcp_recv_at is None:
        return AdmissionDecision(
            action=AdmissionAction.REPLACE_STALE,
            prev_addr=prev.addr,
            prev_packets=prev.packets_received,
            prev_idle_activity_s=prev_idle_activity_s,
        )

    prev_idle_tcp_s = time.monotonic() - prev._last_tcp_recv_at
    if prev_idle_tcp_s < duplicate_reject_idle_s:
        return AdmissionDecision(
            action=AdmissionAction.REJECT_DUPLICATE,
            prev_addr=prev.addr,
            prev_packets=prev.packets_received,
            prev_idle_tcp_s=prev_idle_tcp_s,
        )

    return AdmissionDecision(
        action=AdmissionAction.REPLACE_STALE,
        prev_addr=prev.addr,
        prev_packets=prev.packets_received,
        prev_idle_tcp_s=prev_idle_tcp_s,
        prev_idle_activity_s=prev_idle_activity_s,
    )


def apply_admission(
    prev: CollarSessionView | None,
    incoming: CollarSessionView,
    conn: Any,
    *,
    duplicate_reject_idle_s: float = COLLAR_DUPLICATE_REJECT_IDLE_S,
) -> bool:
    """
    Evaluate admission and act on the incoming connection.

    Returns True if the incoming session should proceed, False if rejected.
    """
    decision = evaluate_admission(
        prev, incoming, duplicate_reject_idle_s=duplicate_reject_idle_s,
    )

    if decision.action is AdmissionAction.ADMIT:
        return True

    if decision.action is AdmissionAction.REJECT_DUPLICATE:
        LOG.warning(
            "Ignoring duplicate collar TCP from %s — keeping active session %s "
            "(%d packets, last TCP %.1fs ago). Collar firmware opened two "
            "sockets; only one can stream at a time.",
            incoming.addr,
            decision.prev_addr,
            decision.prev_packets,
            decision.prev_idle_tcp_s if decision.prev_idle_tcp_s is not None else -1.0,
        )
        try:
            conn.close()
        except OSError:
            pass
        return False

    LOG.info(
        "Replacing stale collar session %s with new connection from %s "
        "(previous idle %.1fs, %d packets)",
        decision.prev_addr,
        incoming.addr,
        decision.prev_idle_activity_s or decision.prev_idle_tcp_s or 0.0,
        decision.prev_packets,
    )
    if prev is not None:
        prev.request_shutdown("replaced by new collar connection")
    return True
