"""Fix PMW3901 flow dx/dy in a collar wire capture (LE/BE int16 byte swap).

Firmware that parses big-endian UART motion as little-endian produces values
like 256 for 1 px, -257 for -2 px. This tool byte-swaps type-2 flow samples.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from flow_endian import normalize_flow_dx_dy, swap_int16


def fix_flow_payload(payload: list) -> list:
    if not isinstance(payload, list) or len(payload) < 2:
        return payload
    out = list(payload)
    dx, dy = normalize_flow_dx_dy(int(out[0]), int(out[1]))
    out[0] = dx
    out[1] = dy
    return out


def fix_wire_row(row: list) -> list:
    if not isinstance(row, list) or len(row) != 3:
        return row
    sensor, ts, payload = row
    if sensor != 2:
        return row
    if isinstance(payload, list):
        return [sensor, ts, fix_flow_payload(payload)]
    if isinstance(payload, dict):
        fixed = dict(payload)
        for key in ("dx", "dy", "delta_x", "delta_y"):
            if key in fixed:
                fixed[key] = swap_int16(int(fixed[key]))
        return [sensor, ts, fixed]
    return row


def fix_batch(raw: object) -> object:
    if not isinstance(raw, list) or not raw:
        return raw
    if isinstance(raw[0], list):
        return [fix_wire_row(row) for row in raw]
    if len(raw) >= 3 and not isinstance(raw[0], list) and isinstance(raw[2], list):
        return fix_wire_row(raw)
    if len(raw) >= 3 and len(raw) % 3 == 0 and not isinstance(raw[0], list):
        return [fix_wire_row(row[i : i + 3]) for i in range(0, len(raw), 3)]
    return raw


def fix_device_row(row: dict) -> dict:
    if row.get("kind") != "flow":
        return row
    flow = row.get("flow")
    if not isinstance(flow, dict):
        return row
    out = dict(row)
    out_flow = dict(flow)
    dx, dy = normalize_flow_dx_dy(
        int(out_flow.get("dx", out_flow.get("delta_x", 0))),
        int(out_flow.get("dy", out_flow.get("delta_y", 0))),
    )
    if "dx" in out_flow or "delta_x" in out_flow:
        out_flow["dx"] = dx
        out_flow["delta_x"] = dx
    if "dy" in out_flow or "delta_y" in out_flow:
        out_flow["dy"] = dy
        out_flow["delta_y"] = dy
    out["flow"] = out_flow
    return out


def process_line(line: str) -> str:
    stripped = line.strip()
    if not stripped:
        return line
    if stripped.startswith("{"):
        row = json.loads(stripped)
        if isinstance(row, dict) and row.get("_fusion_record"):
            return line
        if isinstance(row, dict):
            return json.dumps(fix_device_row(row), separators=(",", ":")) + "\n"
        return line
    batch = json.loads(stripped)
    return json.dumps(fix_batch(batch), separators=(",", ":")) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output path (default: <input stem>_flow_fixed.jsonl next to input)",
    )
    args = parser.parse_args()
    out = args.output or args.input.with_name(f"{args.input.stem}_flow_fixed.jsonl")

    fixed_flow = 0
    with args.input.open(encoding="utf-8") as fin, out.open("w", encoding="utf-8") as fout:
        for line in fin:
            if line.strip().startswith("[") and "[2," in line:
                batch = json.loads(line.strip())
                rows = batch if isinstance(batch[0], list) else [batch]
                for row in rows:
                    if isinstance(row, list) and row and row[0] == 2:
                        fixed_flow += 1
            fout.write(process_line(line))

    print(f"Wrote {out} ({fixed_flow} flow samples byte-swapped)")


if __name__ == "__main__":
    main()
