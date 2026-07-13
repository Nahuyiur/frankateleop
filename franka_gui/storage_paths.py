"""Shared local storage paths for GUI capture and NAS synchronization."""

from __future__ import annotations

import os
from pathlib import Path

DEFAULT_CACHE_ROOT = Path.home() / "Desktop" / "franka_record_cache"
CACHE_ROOT_ENV = "FRANKA_GUI_RECORD_CACHE_ROOT"


def record_cache_root(*, create: bool = True) -> Path:
    configured = os.environ.get(CACHE_ROOT_ENV, "").strip()
    root = Path(configured).expanduser() if configured else DEFAULT_CACHE_ROOT
    if create:
        root.mkdir(parents=True, exist_ok=True)
    return root
