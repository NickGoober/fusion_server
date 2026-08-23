"""PMW3901 flow dx/dy endian fix (BE wire parsed as LE on collar firmware)."""

from __future__ import annotations

# PMW3901 reports motion as int16 pixel deltas; sane per-frame magnitudes stay well
# below this after a correct parse (server outlier gate uses 40 px/frame).
_FLOW_PLAUSIBLE_PX = 80


def swap_int16(value: int) -> int:
    """Swap low/high bytes of a signed 16-bit integer."""
    u = int(value) & 0xFFFF
    swapped = ((u & 0xFF) << 8) | (u >> 8)
    return swapped - 65536 if swapped >= 32768 else swapped


def is_endian_corrupted(value: int) -> bool:
    """True when byte-swapping yields a much smaller, plausible pixel delta."""
    if value == 0:
        return False
    av = abs(int(value))
    if av < 128:
        return False
    swapped = swap_int16(value)
    sav = abs(swapped)
    return sav <= _FLOW_PLAUSIBLE_PX and sav < av


def normalize_flow_dx_dy(dx: int, dy: int) -> tuple[int, int]:
    """Return dx/dy, byte-swapping each axis independently when corrupted."""
    if is_endian_corrupted(dx):
        dx = swap_int16(dx)
    if is_endian_corrupted(dy):
        dy = swap_int16(dy)
    return int(dx), int(dy)
