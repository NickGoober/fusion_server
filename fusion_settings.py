"""Persistent server settings (JSON file with optional env overrides)."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_SETTINGS_PATH = REPO_ROOT / "fusion_server.json"
ALT_SETTINGS_PATH = REPO_ROOT / "config.json"
SYSTEM_SETTINGS_PATH = Path("/etc/fusion-server/config.json")

_cached_settings: dict[str, Any] | None = None
_resolved_path: Path | None = None


def resolve_settings_path() -> Path:
    """Path used for settings: env > system file > repo-local file."""
    env = os.environ.get("FUSION_SERVER_CONFIG")
    if env:
        return Path(env)
    if SYSTEM_SETTINGS_PATH.is_file():
        return SYSTEM_SETTINGS_PATH
    for candidate in (DEFAULT_SETTINGS_PATH, ALT_SETTINGS_PATH):
        if candidate.is_file():
            return candidate
    return DEFAULT_SETTINGS_PATH


def active_settings_path() -> Path:
    global _resolved_path
    if _resolved_path is None:
        _resolved_path = resolve_settings_path()
    return _resolved_path


def load_settings_file(path: Path | None = None) -> dict[str, Any]:
    path = path or active_settings_path()
    if not path.is_file():
        return {}
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Settings file must be a JSON object: {path}")
    return data


def _get_cached_settings() -> dict[str, Any]:
    global _cached_settings
    if _cached_settings is None:
        _cached_settings = load_settings_file()
    return _cached_settings


def reload_settings() -> Path:
    """Clear cache and reload from disk (tests / admin tooling)."""
    global _cached_settings, _resolved_path
    _cached_settings = None
    _resolved_path = None
    path = active_settings_path()
    _cached_settings = load_settings_file(path)
    return path


def get_setting(key: str, default: str | None = None) -> str | None:
    """Return env var if set, else value from settings file, else default."""
    env_val = os.environ.get(key)
    if env_val is not None and env_val != "":
        return env_val
    file_val = _get_cached_settings().get(key)
    if file_val is None:
        return default
    if isinstance(file_val, bool):
        return "1" if file_val else "0"
    return str(file_val)


def get_int_setting(key: str, default: int) -> int:
    raw = get_setting(key)
    if raw is None:
        return default
    return int(raw)


def get_float_setting(key: str, default: float) -> float:
    raw = get_setting(key)
    if raw is None:
        return default
    return float(raw)


def get_bool_setting(key: str, default: bool = False) -> bool:
    raw = get_setting(key)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")
