from __future__ import annotations

from datetime import datetime, timedelta
import json
from pathlib import Path
import time

import pytest

from work_monitor.ledger import WorkLedger
from work_monitor.model import TimeInterval, WorkAttempt, build_day_summary, merge_intervals
from work_monitor.tracker import WorktimeTracker


def _event(
    event_id: str,
    event_type: str,
    when: float,
    *,
    attempt_id: str = "",
    session_id: str = "session",
    **payload,
):
    return {
        "event_id": event_id,
        "event_type": event_type,
        "attempt_id": attempt_id,
        "session_id": session_id,
        "occurred_at": when,
        "payload": payload,
    }


def _attempt(start: float, segments: tuple[TimeInterval, ...], result: str = "saved_high") -> WorkAttempt:
    return WorkAttempt("a", "alice", "A", "task", start, segments[-1].end, result, "", segments)


def test_merge_short_gap_boundary() -> None:
    first = TimeInterval(0.0, 10.0)
    assert merge_intervals((first, TimeInterval(70.0, 80.0)), max_gap_seconds=60.0) == (
        TimeInterval(0.0, 80.0),
    )
    assert merge_intervals((first, TimeInterval(70.001, 80.0)), max_gap_seconds=60.0) == (
        first,
        TimeInterval(70.001, 80.0),
    )


def test_day_summary_keeps_raw_time_and_merges_short_breaks() -> None:
    start = datetime.now().astimezone().replace(hour=8, minute=0, second=0, microsecond=0).timestamp()
    attempts = [
        _attempt(start, (TimeInterval(start, start + 30),)),
        _attempt(start + 90, (TimeInterval(start + 90, start + 120),)),
    ]
    summary = build_day_summary(
        datetime.fromtimestamp(start).astimezone().date(),
        attempts,
        now=start + 300,
    )
    assert summary.raw_recording_seconds == pytest.approx(60)
    assert summary.effective_work_seconds == pytest.approx(120)
    assert summary.rest_seconds == pytest.approx(180)


def test_day_summary_clips_effective_work_to_nine_hours() -> None:
    start = datetime.now().astimezone().replace(hour=7, minute=0, second=0, microsecond=0).timestamp()
    ten_hours = TimeInterval(start, start + 10 * 3600)
    summary = build_day_summary(
        datetime.fromtimestamp(start).astimezone().date(),
        [_attempt(start, (ten_hours,))],
        now=start + 11 * 3600,
    )
    assert summary.raw_recording_seconds == pytest.approx(10 * 3600)
    assert summary.effective_work_seconds == pytest.approx(9 * 3600)
    assert summary.remaining_seconds == 0


def test_work_dashboard_keeps_brand_and_metric_visual_contract(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PyQt6")
    from PyQt6 import QtWidgets
    from work_monitor.dashboard import WorktimeDashboard

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorktimeDashboard(WorkLedger(tmp_path / "ui.sqlite3"))

    assert window.findChild(QtWidgets.QFrame, "AuxHeader") is not None
    cards = window.findChildren(QtWidgets.QFrame, "MetricCard")
    assert {card.property("metric") for card in cards} == {
        "work",
        "recording",
        "rest",
        "remaining",
        "attempts",
    }
    assert window.table_count.text() == "0 条记录"

    window.close()
    app.processEvents()


def test_ledger_lifecycle_is_idempotent(tmp_path: Path) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    base = time.time()
    events = [
        _event("s", "session_start", base),
        _event("a", "attempt_start", base + 1, attempt_id="attempt", operator_name="alice", mode="A", task="stack", display_index=0),
        _event("p", "recording_paused", base + 11, attempt_id="attempt"),
        _event("r", "recording_resumed", base + 31, attempt_id="attempt"),
        _event("f", "recording_finished", base + 41, attempt_id="attempt"),
        _event("q", "quality_selected", base + 42, attempt_id="attempt", quality="High_Quality", episode_index=3),
        _event("d", "attempt_saved", base + 43, attempt_id="attempt", quality="High_Quality", episode_index=3, output_dir="/data/3"),
    ]
    for event in events + events:
        ledger.record_event(event)
    attempts = ledger.list_attempts(now=base + 100)
    assert len(attempts) == 1
    assert attempts[0].result == "saved_high"
    assert attempts[0].segments == (
        TimeInterval(base + 1, base + 11),
        TimeInterval(base + 31, base + 41),
    )


def test_recover_stale_recording_at_last_heartbeat(tmp_path: Path) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    base = time.time() - 100
    ledger.record_event(_event("s", "session_start", base))
    ledger.record_event(
        _event("a", "attempt_start", base + 1, attempt_id="attempt", operator_name="alice", mode="B", task="drawer")
    )
    ledger.record_event(_event("h", "session_heartbeat", base + 5))
    assert ledger.recover_stale(now=base + 30, stale_after_seconds=10) == 1
    attempt = ledger.list_attempts(now=base + 30)[0]
    assert attempt.result == "interrupted"
    assert attempt.ended_at == pytest.approx(base + 5)
    assert attempt.segments[0].end == pytest.approx(base + 5)


def test_recover_stale_can_exclude_the_current_gui_session(tmp_path: Path) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    base = time.time() - 100
    ledger.record_event(_event("s", "session_start", base, session_id="current"))
    ledger.record_event(
        _event(
            "a",
            "attempt_start",
            base + 1,
            attempt_id="attempt",
            session_id="current",
            operator_name="alice",
            mode="A",
            task="stack",
        )
    )
    assert ledger.recover_stale(now=base + 30, exclude_session_ids=("current",)) == 0
    assert ledger.list_attempts(now=base + 30)[0].result == ""


def test_save_error_is_counted_without_overwriting_final_result(tmp_path: Path) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    base = time.time()
    for event in (
        _event("s", "session_start", base),
        _event("a", "attempt_start", base + 1, attempt_id="attempt", operator_name="alice", mode="A", task="stack"),
        _event("f", "recording_finished", base + 2, attempt_id="attempt"),
        _event("q1", "quality_selected", base + 3, attempt_id="attempt", quality="High_Quality", episode_index=0),
        _event("e", "save_error", base + 4, attempt_id="attempt"),
        _event("q2", "quality_selected", base + 5, attempt_id="attempt", quality="High_Quality", episode_index=0),
        _event("d", "attempt_saved", base + 6, attempt_id="attempt", quality="High_Quality", episode_index=0),
        _event("late", "recording_resumed", base + 7, attempt_id="attempt"),
    ):
        ledger.record_event(event)
    attempt = ledger.list_attempts(now=base + 10)[0]
    assert attempt.result == "saved_high"
    assert attempt.save_error_count == 1
    assert len(attempt.segments) == 1


def test_validator_failure_is_a_terminal_attempt_result(tmp_path: Path) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    base = time.time()
    for event in (
        _event("s", "session_start", base),
        _event("a", "attempt_start", base + 1, attempt_id="attempt", operator_name="alice", mode="A", task="stack"),
        _event("f", "recording_finished", base + 2, attempt_id="attempt"),
        _event("q", "quality_selected", base + 3, attempt_id="attempt", quality="High_Quality", episode_index=0),
        _event("v", "attempt_validation_failed", base + 4, attempt_id="attempt"),
    ):
        ledger.record_event(event)
    attempt = ledger.list_attempts(now=base + 10)[0]
    assert attempt.result == "validation_failed"
    assert attempt.ended_at == pytest.approx(base + 2)


def test_tracker_falls_back_and_replays_emergency(tmp_path: Path, monkeypatch) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    tracker = WorktimeTracker(ledger)
    tracker.flush()
    original = ledger.record_event

    def fail(_event):
        raise OSError("locked")

    monkeypatch.setattr(ledger, "record_event", fail)
    tracker.emit("session_heartbeat")
    assert tracker.flush()
    assert tracker._emergency_path.is_file()
    monkeypatch.setattr(ledger, "record_event", original)
    assert tracker.replay_emergency() == 1
    tracker.close()


def test_tracker_initialization_failure_does_not_block_capture(tmp_path: Path, monkeypatch) -> None:
    def fail_init(_self):
        raise OSError("read-only filesystem")

    monkeypatch.setattr(WorkLedger, "_initialize", fail_init)
    monkeypatch.setattr("work_monitor.tracker.DEFAULT_DB_PATH", tmp_path / "fallback.sqlite3")
    tracker = WorktimeTracker()
    assert tracker.ledger is None
    tracker.emit("attempt_start", attempt_id="attempt", payload={"operator_name": "Alice"})
    assert tracker.flush()
    assert tracker._emergency_path.is_file()
    tracker.close()


def test_tracker_switches_to_ordered_spool_after_first_sqlite_failure(
    tmp_path: Path, monkeypatch
) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    tracker = WorktimeTracker(ledger)
    assert tracker.flush()
    original = ledger.record_event
    failed_once = False

    def fail_first_attempt(event):
        nonlocal failed_once
        if event["event_type"] == "attempt_start" and not failed_once:
            failed_once = True
            raise OSError("temporary lock")
        original(event)

    monkeypatch.setattr(ledger, "record_event", fail_first_attempt)
    base = time.time()
    tracker.emit(
        "attempt_start",
        attempt_id="ordered",
        occurred_at=base,
        payload={"operator_name": "alice", "mode": "A", "task": "stack"},
    )
    tracker.emit("recording_paused", attempt_id="ordered", occurred_at=base + 1)
    tracker.emit("recording_resumed", attempt_id="ordered", occurred_at=base + 2)
    tracker.emit("recording_finished", attempt_id="ordered", occurred_at=base + 3)
    assert tracker.flush()
    assert ledger.list_attempts(now=base + 10) == []
    monkeypatch.setattr(ledger, "record_event", original)
    assert tracker.replay_emergency() == 4
    attempt = ledger.list_attempts(now=base + 10)[0]
    assert attempt.segments == (
        TimeInterval(base, base + 1),
        TimeInterval(base + 2, base + 3),
    )
    tracker.close()


def test_tracker_atomically_recovers_spool_before_resuming_sqlite(
    tmp_path: Path, monkeypatch
) -> None:
    first_ledger = WorkLedger(tmp_path / "work.sqlite3")
    tracker = WorktimeTracker(first_ledger)
    assert tracker.flush()
    original = first_ledger.record_event

    def fail(_event):
        raise OSError("temporarily unavailable")

    monkeypatch.setattr(first_ledger, "record_event", fail)
    base = time.time()
    tracker.emit(
        "attempt_start",
        attempt_id="recovered",
        occurred_at=base,
        payload={"operator_name": "alice", "mode": "C", "task": "stack"},
    )
    assert tracker.flush()
    monkeypatch.setattr(first_ledger, "record_event", original)

    replacement = WorkLedger(first_ledger.path)
    assert tracker.attach_ledger_and_recover(replacement) == 1
    assert not tracker._spool_only
    tracker.emit("recording_finished", attempt_id="recovered", occurred_at=base + 5)
    assert tracker.flush()
    attempt = replacement.list_attempts(now=base + 10)[0]
    assert attempt.result == ""
    assert attempt.segments == (TimeInterval(base, base + 5),)
    tracker.close()


def test_tracker_recovers_crash_left_processing_before_newer_emergency(
    tmp_path: Path,
) -> None:
    ledger = WorkLedger(tmp_path / "work.sqlite3")
    emergency = ledger.path.with_suffix(".emergency.jsonl")
    processing = emergency.with_suffix(".processing.jsonl")
    base = time.time()
    start = _event(
        "old",
        "attempt_start",
        base,
        attempt_id="crash",
        operator_name="alice",
        mode="A",
        task="stack",
    )
    finish = _event(
        "new",
        "recording_finished",
        base + 4,
        attempt_id="crash",
    )
    processing.write_text(json.dumps(start) + "\n", encoding="utf-8")
    emergency.write_text(json.dumps(finish) + "\n", encoding="utf-8")

    tracker = WorktimeTracker(ledger)
    assert tracker.flush()
    attempt = ledger.list_attempts(now=base + 10)[0]
    assert attempt.segments == (TimeInterval(base, base + 4),)
    assert not processing.exists()
    assert not emergency.exists()
    tracker.close()
