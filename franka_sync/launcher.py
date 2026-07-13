"""Small GUI-facing launcher for the independent NAS sync daemon."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


def ensure_sync_daemon(
    repo_root: str | os.PathLike[str],
    cache_root: str | os.PathLike[str],
    nas_root: str | os.PathLike[str],
) -> int:
    """Start a detached watcher and return its PID.

    Calls are intentionally cheap and idempotent from the caller's perspective.
    Concurrent children arbitrate through the daemon's local ``flock``; all but
    the lock owner exit immediately.
    """

    repository = Path(repo_root).expanduser().absolute()
    cache = Path(cache_root).expanduser().absolute()
    nas = Path(nas_root).expanduser().absolute()
    if not (repository / "franka_sync" / "__main__.py").is_file():
        raise FileNotFoundError(f"franka_sync is not present under {repository}")

    cache.mkdir(parents=True, exist_ok=True)
    log_path = cache / "sync.log"
    environment = os.environ.copy()
    environment["PYTHONUNBUFFERED"] = "1"
    environment["FRANKA_SYNC_CACHE_ROOT"] = str(cache)
    environment["FRANKA_SYNC_NAS_ROOT"] = str(nas)

    command = [
        sys.executable,
        "-m",
        "franka_sync",
        "--watch",
        "--cache-root",
        str(cache),
        "--nas-root",
        str(nas),
    ]
    if shutil.which("nice"):
        command = ["nice", "-n", "15", *command]
    if shutil.which("ionice"):
        command = ["ionice", "-c", "3", *command]
    with log_path.open("ab", buffering=0) as log_handle:
        process = subprocess.Popen(
            command,
            cwd=repository,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )
    return process.pid
