from __future__ import annotations

import json
from pathlib import Path
from queue import Queue
import threading
import time
import uuid
from typing import Any

from .ledger import DEFAULT_DB_PATH, WorkLedger


class WorktimeTracker:
    def __init__(self, ledger: WorkLedger | None = None) -> None:
        try:
            self.ledger: WorkLedger | None = ledger or WorkLedger()
        except Exception:
            self.ledger = None
        self.session_id = uuid.uuid4().hex
        self._queue: Queue[dict[str, Any] | None] = Queue()
        ledger_path = self.ledger.path if self.ledger is not None else DEFAULT_DB_PATH
        self._emergency_path = ledger_path.with_suffix(".emergency.jsonl")
        self._emergency_lock = threading.Lock()
        self._mode_lock = threading.Lock()
        self._spool_only = self.ledger is None
        self._last_persistence_error = ""
        self._closed = False
        self._thread = threading.Thread(target=self._worker, name="worktime-ledger", daemon=True)
        self._thread.start()
        self.emit("session_start")

    def emit(
        self,
        event_type: str,
        *,
        attempt_id: str = "",
        occurred_at: float | None = None,
        payload: dict[str, Any] | None = None,
    ) -> None:
        if self._closed:
            return
        event = {
            "event_id": uuid.uuid4().hex,
            "event_type": event_type,
            "attempt_id": attempt_id,
            "session_id": self.session_id,
            "occurred_at": float(occurred_at if occurred_at is not None else time.time()),
            "payload": dict(payload or {}),
        }
        self._queue.put_nowait(event)

    def heartbeat(self) -> None:
        self.emit("session_heartbeat")

    def close(self) -> None:
        if self._closed:
            return
        self.emit("session_end")
        self._closed = True
        self._queue.put(None)
        self._thread.join(timeout=5.0)

    def flush(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while self._queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.01)
        return self._queue.unfinished_tasks == 0

    def replay_emergency(self) -> int:
        if self.ledger is None:
            return 0
        count = 0
        processing = self._processing_path
        while True:
            with self._emergency_lock:
                if not processing.is_file():
                    if not self._emergency_path.is_file():
                        return count
                    self._emergency_path.replace(processing)
            lines = processing.read_text(encoding="utf-8").splitlines()
            for offset, line in enumerate(lines):
                try:
                    self.ledger.record_event(json.loads(line))
                    count += 1
                except Exception:
                    temporary = processing.with_suffix(".rewrite.tmp")
                    temporary.write_text("\n".join(lines[offset:]) + "\n", encoding="utf-8")
                    temporary.replace(processing)
                    return count
            processing.unlink(missing_ok=True)

    def attach_ledger_and_recover(self, ledger: WorkLedger) -> int:
        """Atomically replay the ordered spool before resuming SQLite writes."""
        with self._mode_lock:
            self.ledger = ledger
            try:
                recovered = self.replay_emergency()
            except Exception as exc:
                self._spool_only = True
                self._last_persistence_error = f"{type(exc).__name__}: {exc}"
                raise
            if self._emergency_path.is_file() or self._processing_path.is_file():
                self._spool_only = True
                raise OSError("emergency spool replay was incomplete")
            self._spool_only = False
            self._last_persistence_error = ""
            return recovered

    def _worker(self) -> None:
        try:
            self.replay_emergency()
            if self._emergency_path.is_file() or self._processing_path.is_file():
                self._spool_only = True
        except Exception as exc:
            self._spool_only = True
            self._last_persistence_error = f"{type(exc).__name__}: {exc}"
        while True:
            event = self._queue.get()
            try:
                if event is None:
                    return
                with self._mode_lock:
                    if self._spool_only:
                        self._write_emergency(event)
                        continue
                    try:
                        if self.ledger is None:
                            raise OSError("worktime SQLite ledger is unavailable")
                        self.ledger.record_event(event)
                    except Exception as exc:
                        self._spool_only = True
                        self._last_persistence_error = f"{type(exc).__name__}: {exc}"
                        self._write_emergency(event)
            finally:
                self._queue.task_done()

    def _write_emergency(self, event: dict[str, Any]) -> bool:
        try:
            self._emergency_path.parent.mkdir(parents=True, exist_ok=True)
            line = json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n"
            with self._emergency_lock:
                with self._emergency_path.open("a", encoding="utf-8") as handle:
                    handle.write(line)
            return True
        except Exception as exc:
            self._last_persistence_error = f"{type(exc).__name__}: {exc}"
            return False

    @property
    def _processing_path(self) -> Path:
        return self._emergency_path.with_suffix(".processing.jsonl")
