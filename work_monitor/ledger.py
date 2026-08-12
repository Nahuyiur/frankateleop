from __future__ import annotations

import json
import os
from pathlib import Path
import sqlite3
import threading
import time
from typing import Any, Iterable

from .model import TimeInterval, WorkAttempt


DEFAULT_DB_PATH = Path("~/.local/share/frankateleop/work_monitor/worktime.sqlite3").expanduser()


class WorkLedger:
    def __init__(self, path: Path | str | None = None) -> None:
        configured = os.environ.get("FRANKA_WORKTIME_DB", "").strip()
        self.path = Path(path or configured or DEFAULT_DB_PATH).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=0.25)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA synchronous=NORMAL")
        connection.execute("PRAGMA busy_timeout=250")
        connection.execute("PRAGMA foreign_keys=ON")
        return connection

    def _initialize(self) -> None:
        with self._connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS events (
                    event_id TEXT PRIMARY KEY,
                    event_type TEXT NOT NULL,
                    attempt_id TEXT,
                    session_id TEXT NOT NULL,
                    occurred_at REAL NOT NULL,
                    payload_json TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS sessions (
                    session_id TEXT PRIMARY KEY,
                    started_at REAL NOT NULL,
                    last_heartbeat REAL NOT NULL,
                    closed_at REAL
                );
                CREATE TABLE IF NOT EXISTS attempts (
                    attempt_id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL,
                    operator_name TEXT NOT NULL,
                    mode TEXT NOT NULL,
                    task TEXT NOT NULL,
                    display_index INTEGER,
                    started_at REAL NOT NULL,
                    ended_at REAL,
                    last_heartbeat REAL NOT NULL,
                    state TEXT NOT NULL,
                    result TEXT NOT NULL DEFAULT '',
                    quality TEXT NOT NULL DEFAULT '',
                    episode_index INTEGER,
                    output_dir TEXT NOT NULL DEFAULT '',
                    save_error_count INTEGER NOT NULL DEFAULT 0,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS segments (
                    segment_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    attempt_id TEXT NOT NULL,
                    started_at REAL NOT NULL,
                    ended_at REAL,
                    UNIQUE(attempt_id, started_at),
                    FOREIGN KEY(attempt_id) REFERENCES attempts(attempt_id)
                );
                CREATE INDEX IF NOT EXISTS attempts_started_idx ON attempts(started_at);
                CREATE INDEX IF NOT EXISTS attempts_filter_idx ON attempts(operator_name, mode, task);
                CREATE INDEX IF NOT EXISTS segments_attempt_idx ON segments(attempt_id);
                """
            )
            columns = {
                str(row[1])
                for row in connection.execute("PRAGMA table_info(attempts)")
            }
            if "save_error_count" not in columns:
                connection.execute(
                    "ALTER TABLE attempts ADD COLUMN save_error_count INTEGER NOT NULL DEFAULT 0"
                )

    def record_event(self, event: dict[str, Any]) -> None:
        payload = dict(event.get("payload") or {})
        event_id = str(event["event_id"])
        event_type = str(event["event_type"])
        attempt_id = str(event.get("attempt_id") or "")
        session_id = str(event["session_id"])
        occurred_at = float(event["occurred_at"])
        with self._lock, self._connect() as connection:
            inserted = connection.execute(
                "INSERT OR IGNORE INTO events VALUES (?, ?, ?, ?, ?, ?)",
                (event_id, event_type, attempt_id or None, session_id, occurred_at, json.dumps(payload, ensure_ascii=False)),
            ).rowcount
            if not inserted:
                return
            self._apply_event(connection, event_type, attempt_id, session_id, occurred_at, payload)

    def _apply_event(
        self,
        connection: sqlite3.Connection,
        event_type: str,
        attempt_id: str,
        session_id: str,
        occurred_at: float,
        payload: dict[str, Any],
    ) -> None:
        if event_type == "session_start":
            connection.execute(
                "INSERT OR IGNORE INTO sessions VALUES (?, ?, ?, NULL)",
                (session_id, occurred_at, occurred_at),
            )
            return
        if event_type == "session_heartbeat":
            connection.execute(
                "UPDATE sessions SET last_heartbeat=? WHERE session_id=?",
                (occurred_at, session_id),
            )
            connection.execute(
                "UPDATE attempts SET last_heartbeat=?, updated_at=? WHERE session_id=? AND state NOT IN ('terminal')",
                (occurred_at, occurred_at, session_id),
            )
            return
        if event_type == "session_end":
            connection.execute(
                "UPDATE sessions SET last_heartbeat=?, closed_at=? WHERE session_id=?",
                (occurred_at, occurred_at, session_id),
            )
            return
        if event_type == "attempt_start":
            connection.execute(
                """
                INSERT INTO attempts (
                    attempt_id, session_id, operator_name, mode, task, display_index,
                    started_at, last_heartbeat, state, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, 'recording', ?)
                """,
                (
                    attempt_id,
                    session_id,
                    str(payload["operator_name"]),
                    str(payload["mode"]),
                    str(payload["task"]),
                    payload.get("display_index"),
                    occurred_at,
                    occurred_at,
                    occurred_at,
                ),
            )
            connection.execute(
                "INSERT INTO segments(attempt_id, started_at) VALUES (?, ?)",
                (attempt_id, occurred_at),
            )
            return
        if event_type == "recording_resumed":
            updated = connection.execute(
                "UPDATE attempts SET state='recording', last_heartbeat=?, updated_at=? WHERE attempt_id=? AND state!='terminal'",
                (occurred_at, occurred_at, attempt_id),
            ).rowcount
            if updated:
                connection.execute(
                    "INSERT OR IGNORE INTO segments(attempt_id, started_at) VALUES (?, ?)",
                    (attempt_id, occurred_at),
                )
            return
        if event_type == "recording_paused":
            self._close_segment(connection, attempt_id, occurred_at)
            connection.execute(
                "UPDATE attempts SET state='paused', last_heartbeat=?, updated_at=? WHERE attempt_id=? AND state!='terminal'",
                (occurred_at, occurred_at, attempt_id),
            )
            return
        if event_type == "recording_finished":
            self._close_segment(connection, attempt_id, occurred_at)
            connection.execute(
                "UPDATE attempts SET ended_at=?, state='awaiting_quality', updated_at=? WHERE attempt_id=? AND state!='terminal'",
                (occurred_at, occurred_at, attempt_id),
            )
            return
        if event_type == "quality_selected":
            connection.execute(
                "UPDATE attempts SET state='saving', quality=?, episode_index=?, updated_at=? WHERE attempt_id=? AND state!='terminal'",
                (str(payload.get("quality", "")), payload.get("episode_index"), occurred_at, attempt_id),
            )
            return
        if event_type == "save_error":
            connection.execute(
                """UPDATE attempts SET state='awaiting_quality', result='save_failed',
                    save_error_count=save_error_count+1, updated_at=?
                    WHERE attempt_id=? AND state!='terminal'""",
                (occurred_at, attempt_id),
            )
            return
        if event_type == "attempt_saved":
            quality = str(payload.get("quality", ""))
            result = {
                "High_Quality": "saved_high",
                "Low_Quality": "saved_low",
                "Failure": "saved_failure",
            }.get(quality, "saved")
            connection.execute(
                """
                UPDATE attempts SET state='terminal', result=?, quality=?, episode_index=?,
                    output_dir=?, updated_at=? WHERE attempt_id=? AND state!='terminal'
                """,
                (
                    result,
                    quality,
                    payload.get("episode_index"),
                    str(payload.get("output_dir", "")),
                    occurred_at,
                    attempt_id,
                ),
            )
            return
        if event_type in {
            "attempt_discarded",
            "attempt_interrupted",
            "attempt_validation_failed",
            "save_failed",
        }:
            self._close_segment(connection, attempt_id, occurred_at)
            result = {
                "attempt_discarded": "discarded",
                "attempt_interrupted": "interrupted",
                "attempt_validation_failed": "validation_failed",
                "save_failed": "save_failed",
            }[event_type]
            connection.execute(
                """
                UPDATE attempts SET ended_at=COALESCE(ended_at, ?), state='terminal',
                    result=?, updated_at=? WHERE attempt_id=? AND state!='terminal'
                """,
                (occurred_at, result, occurred_at, attempt_id),
            )

    @staticmethod
    def _close_segment(connection: sqlite3.Connection, attempt_id: str, ended_at: float) -> None:
        connection.execute(
            """
            UPDATE segments SET ended_at=MAX(started_at, ?)
            WHERE segment_id=(
                SELECT segment_id FROM segments
                WHERE attempt_id=? AND ended_at IS NULL
                ORDER BY started_at DESC LIMIT 1
            )
            """,
            (ended_at, attempt_id),
        )

    def recover_stale(
        self,
        now: float | None = None,
        stale_after_seconds: float = 10.0,
        exclude_session_ids: Iterable[str] = (),
    ) -> int:
        now = float(now if now is not None else time.time())
        cutoff = now - stale_after_seconds
        with self._lock, self._connect() as connection:
            excluded = tuple(str(value) for value in exclude_session_ids if value)
            exclusion_sql = ""
            values: list[Any] = [cutoff]
            if excluded:
                exclusion_sql = f" AND a.session_id NOT IN ({','.join('?' for _ in excluded)})"
                values.extend(excluded)
            rows = connection.execute(
                f"""
                SELECT a.attempt_id, a.state, s.last_heartbeat
                FROM attempts a JOIN sessions s ON s.session_id=a.session_id
                WHERE a.state != 'terminal' AND s.closed_at IS NULL AND s.last_heartbeat < ?
                {exclusion_sql}
                """,
                values,
            ).fetchall()
            for row in rows:
                ended_at = float(row["last_heartbeat"])
                self._close_segment(connection, str(row["attempt_id"]), ended_at)
                result = (
                    "interrupted"
                    if row["state"] in {"recording", "paused"}
                    else "abandoned_judging"
                )
                connection.execute(
                    """
                    UPDATE attempts SET ended_at=COALESCE(ended_at, ?), state='terminal',
                        result=?, updated_at=? WHERE attempt_id=?
                    """,
                    (ended_at, result, now, row["attempt_id"]),
                )
            return len(rows)

    def list_attempts(
        self,
        *,
        started_after: float | None = None,
        started_before: float | None = None,
        operator_name: str = "",
        mode: str = "",
        task: str = "",
        now: float | None = None,
    ) -> list[WorkAttempt]:
        clauses = ["1=1"]
        values: list[Any] = []
        for clause, value in (
            ("a.started_at>=?", started_after),
            ("a.started_at<?", started_before),
        ):
            if value is not None:
                clauses.append(clause)
                values.append(value)
        for column, value in (
            ("a.operator_name", operator_name),
            ("a.mode", mode),
            ("a.task", task),
        ):
            if value:
                clauses.append(f"{column}=?")
                values.append(value)
        query = f"SELECT a.* FROM attempts a WHERE {' AND '.join(clauses)} ORDER BY a.started_at"
        current = float(now if now is not None else time.time())
        with self._lock, self._connect() as connection:
            rows = connection.execute(query, values).fetchall()
            attempts: list[WorkAttempt] = []
            for row in rows:
                segment_rows = connection.execute(
                    "SELECT started_at, ended_at FROM segments WHERE attempt_id=? ORDER BY started_at",
                    (row["attempt_id"],),
                ).fetchall()
                segments = tuple(
                    TimeInterval(float(item["started_at"]), float(item["ended_at"] or current))
                    for item in segment_rows
                )
                attempts.append(
                    WorkAttempt(
                        attempt_id=str(row["attempt_id"]),
                        operator_name=str(row["operator_name"]),
                        mode=str(row["mode"]),
                        task=str(row["task"]),
                        started_at=float(row["started_at"]),
                        ended_at=float(row["ended_at"]) if row["ended_at"] is not None else None,
                        result=str(row["result"]),
                        quality=str(row["quality"]),
                        segments=segments,
                        save_error_count=int(row["save_error_count"]),
                    )
                )
            return attempts

    def filter_values(self) -> tuple[list[str], list[str]]:
        with self._lock, self._connect() as connection:
            operators = [str(row[0]) for row in connection.execute("SELECT DISTINCT operator_name FROM attempts ORDER BY operator_name")]
            tasks = [str(row[0]) for row in connection.execute("SELECT DISTINCT task FROM attempts ORDER BY task")]
        return operators, tasks
