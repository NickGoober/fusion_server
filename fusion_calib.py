"""Load/save fusion calibration (IMU + flow lever arms) to JSON."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from fusion_settings import get_setting

DEFAULT_CALIB_PATH = Path(__file__).resolve().parent / "fusion_calib.json"


def default_calib_path() -> Path:
    raw = get_setting("FUSION_CALIB_PATH")
    if raw:
        return Path(raw)
    return DEFAULT_CALIB_PATH


def load_calib(path: Path | None = None) -> dict[str, Any]:
    path = path or default_calib_path()
    if not path.is_file():
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_calib(data: dict[str, Any], path: Path | None = None) -> Path:
    path = path or default_calib_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    return path


def _vec3_from_calib(calib: dict[str, Any], key: str) -> tuple[float, float, float]:
    arm = calib.get(key) or {}
    return (
        float(arm.get("x", 0.0)),
        float(arm.get("y", 0.0)),
        float(arm.get("z", 0.0)),
    )


def flow_lever_arm_from_calib(calib: dict[str, Any]) -> tuple[float, float, float]:
    return _vec3_from_calib(calib, "flow_lever_arm_m")


def imu_lever_arm_from_calib(calib: dict[str, Any]) -> tuple[float, float, float]:
    return _vec3_from_calib(calib, "imu_lever_arm_m")


def write_lever_arm_calib(
    flow_x_m: float,
    flow_y_m: float,
    flow_z_m: float,
    imu_x_m: float,
    imu_y_m: float,
    imu_z_m: float,
    *,
    axis: str = "x",
    omega_rad_s: float = 0.0,
    samples_used: int = 0,
    residual_rms_mps: float = 0.0,
    imu_only: bool = False,
    path: Path | None = None,
) -> Path:
    existing = load_calib(path)
    data = {
        **existing,
        "imu_only": imu_only,
        "flow_lever_arm_m": {"x": flow_x_m, "y": flow_y_m, "z": flow_z_m},
        "imu_lever_arm_m": {"x": imu_x_m, "y": imu_y_m, "z": imu_z_m},
        "calibrated_at_ms": int(time.time() * 1000),
        "axis": axis,
        "omega_rad_s": omega_rad_s,
        "samples_used": samples_used,
        "residual_rms_mps": residual_rms_mps,
    }
    return save_calib(data, path)
