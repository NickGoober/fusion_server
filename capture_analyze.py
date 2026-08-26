"""Fingerprint collar JSONL captures (range span, flow axes) for replay sanity checks."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from flow_endian import normalize_flow_dx_dy
from lever_arm_config import FLOW_MOUNT_PITCH_X_RAD

# Matches fusion.c defaults / flow_viewer body mapping.
FLOW_SWAP_XY = True
FLOW_INVERT_X = True
FLOW_INVERT_Y = True
FLOW_CP = math.cos(FLOW_MOUNT_PITCH_X_RAD)


def _map_flow_body(dx: int, dy: int) -> tuple[int, float]:
    raw_x, raw_y = int(dx), int(dy)
    bx = raw_y if FLOW_SWAP_XY else raw_x
    by = raw_x if FLOW_SWAP_XY else raw_y
    if FLOW_INVERT_X:
        bx = -bx
    if FLOW_INVERT_Y:
        by = -by
    bz = float(by) * FLOW_CP
    return bx, bz


def _wire_rows_from_line(line: str) -> list[list[Any]]:
    line = line.strip()
    if not line or line.startswith("{"):
        return []
    try:
        batch = json.loads(line)
    except json.JSONDecodeError:
        return []
    if not isinstance(batch, list) or not batch:
        return []
    if isinstance(batch[0], list):
        return [row for row in batch if isinstance(row, list)]
    if len(batch) >= 3 and len(batch) % 3 == 0:
        return [batch[i : i + 3] for i in range(0, len(batch), 3)]
    if len(batch) == 3:
        return [batch]
    return []


def _infer_motion_label(range_span: int, flow_net_bx: int, flow_net_bz: float) -> str:
    """Heuristic motion class from capture content (not the filename)."""
    # freeMove* fixtures separate cleanly by radar span alone:
    #   UD ~950mm, LR ~600mm, FB ~400mm.
    if range_span >= 800:
        return "UD-like (large range swing)"
    if range_span >= 500:
        return "LR-like (moderate range + lateral flow)"
    if range_span >= 300:
        return "FB-like (small range swing + forward flow)"
    abs_bx = abs(flow_net_bx)
    abs_bz = abs(flow_net_bz)
    if abs_bz > abs_bx and abs_bz >= 50:
        return "FB-like (flow forward / bz)"
    if abs_bx > abs_bz and abs_bx >= 30:
        return "LR-like (flow lateral / bx)"
    return "unknown"


def analyze_capture(path: Path | str) -> dict[str, Any]:
    path = Path(path)
    ranges: list[int] = []
    flow_net_bx = 0
    flow_net_bz = 0.0
    meta: dict[str, Any] | None = None

    with path.open(encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("{"):
                try:
                    row = json.loads(stripped)
                    if isinstance(row, dict):
                        meta = row
                except json.JSONDecodeError:
                    pass
                continue
            for wire_row in _wire_rows_from_line(stripped):
                if len(wire_row) < 3:
                    continue
                sensor = int(wire_row[0])
                payload = wire_row[2]
                if sensor == 3 and isinstance(payload, list) and payload:
                    ranges.append(int(payload[0]))
                elif sensor == 2 and isinstance(payload, list) and len(payload) >= 2:
                    dx, dy = normalize_flow_dx_dy(int(payload[0]), int(payload[1]))
                    bx, bz = _map_flow_body(dx, dy)
                    flow_net_bx += bx
                    flow_net_bz += bz

    range_min = min(ranges) if ranges else None
    range_max = max(ranges) if ranges else None
    range_span = (range_max - range_min) if range_min is not None and range_max is not None else None
    motion = (
        _infer_motion_label(range_span, flow_net_bx, flow_net_bz)
        if range_span is not None
        else "no range data"
    )

    return {
        "path": str(path.resolve()),
        "name": path.name,
        "meta": meta,
        "range_count": len(ranges),
        "range_min_mm": range_min,
        "range_max_mm": range_max,
        "range_span_mm": range_span,
        "flow_net_bx_px": flow_net_bx,
        "flow_net_bz_px": round(flow_net_bz, 1),
        "motion_hint": motion,
    }


def format_capture_summary(info: dict[str, Any]) -> str:
    if info.get("range_min_mm") is None:
        rng = "no range samples"
    else:
        rng = (
            f"range {info['range_min_mm']}-{info['range_max_mm']} mm "
            f"(span {info['range_span_mm']})"
        )
    flow = (
        f"flow cum bx={info['flow_net_bx_px']:+d} bz={info['flow_net_bz_px']:+.0f} px"
    )
    hint = info.get("motion_hint", "")
    name_match = ""
    name = info.get("name", "").upper()
    if "UD" in name and "UD-like" in hint:
        name_match = "name/content OK"
    elif "FB" in name and "FB-like" in hint:
        name_match = "name/content OK"
    elif "LR" in name and "LR-like" in hint:
        name_match = "name/content OK"
    elif "UD" in name and "UD-like" in hint:
        name_match = "name/content OK"
    elif any(tag in name for tag in ("UD", "FB", "LR")):
        name_match = "WARNING: filename may not match content — check replay info"
    parts = [rng, flow, hint]
    if name_match:
        parts.append(name_match)
    return " · ".join(parts)
