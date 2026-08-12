from __future__ import annotations

from dataclasses import dataclass
from datetime import date, datetime, timedelta
from typing import Iterable, Sequence


WORK_WINDOW_SECONDS = 9 * 60 * 60
DEFAULT_SHORT_GAP_SECONDS = 60.0


@dataclass(frozen=True)
class TimeInterval:
    start: float
    end: float

    @property
    def duration(self) -> float:
        return max(0.0, self.end - self.start)


@dataclass(frozen=True)
class WorkAttempt:
    attempt_id: str
    operator_name: str
    mode: str
    task: str
    started_at: float
    ended_at: float | None
    result: str
    quality: str
    segments: tuple[TimeInterval, ...]
    save_error_count: int = 0


@dataclass(frozen=True)
class DaySummary:
    day: date
    anchor_start: float | None
    window_end: float | None
    work_intervals: tuple[TimeInterval, ...]
    raw_recording_seconds: float
    effective_work_seconds: float
    rest_seconds: float
    remaining_seconds: float
    attempt_count: int
    result_counts: dict[str, int]

    def rest_intervals(self, now: float) -> tuple[TimeInterval, ...]:
        if self.anchor_start is None or self.window_end is None:
            return ()
        elapsed_end = min(max(now, self.anchor_start), self.window_end)
        cursor = self.anchor_start
        rests: list[TimeInterval] = []
        for interval in self.work_intervals:
            if interval.start > cursor:
                rests.append(TimeInterval(cursor, min(interval.start, elapsed_end)))
            cursor = max(cursor, interval.end)
            if cursor >= elapsed_end:
                break
        if cursor < elapsed_end:
            rests.append(TimeInterval(cursor, elapsed_end))
        return tuple(item for item in rests if item.end > item.start)


def merge_intervals(
    intervals: Iterable[TimeInterval],
    *,
    max_gap_seconds: float = 0.0,
) -> tuple[TimeInterval, ...]:
    ordered = sorted(
        (item for item in intervals if item.end > item.start),
        key=lambda item: (item.start, item.end),
    )
    merged: list[TimeInterval] = []
    for item in ordered:
        if not merged or item.start - merged[-1].end > max_gap_seconds:
            merged.append(item)
        else:
            merged[-1] = TimeInterval(merged[-1].start, max(merged[-1].end, item.end))
    return tuple(merged)


def build_day_summary(
    day: date,
    attempts: Sequence[WorkAttempt],
    *,
    now: float,
    short_gap_seconds: float = DEFAULT_SHORT_GAP_SECONDS,
) -> DaySummary:
    selected = [item for item in attempts if _local_date(item.started_at) == day]
    anchor = min((item.started_at for item in selected), default=None)
    if anchor is None:
        return DaySummary(day, None, None, (), 0.0, 0.0, 0.0, WORK_WINDOW_SECONDS, 0, {})

    window_end = anchor + WORK_WINDOW_SECONDS
    elapsed_end = min(max(now, anchor), window_end)
    clipped: list[TimeInterval] = []
    raw_seconds = 0.0
    for attempt in selected:
        for segment in attempt.segments:
            end = min(segment.end, now)
            raw_seconds += max(0.0, end - segment.start)
            start = max(segment.start, anchor)
            stop = min(end, elapsed_end)
            if stop > start:
                clipped.append(TimeInterval(start, stop))
    work_intervals = merge_intervals(clipped, max_gap_seconds=short_gap_seconds)
    effective = sum(item.duration for item in work_intervals)
    elapsed = max(0.0, elapsed_end - anchor)
    rest = max(0.0, elapsed - effective)
    remaining = max(0.0, window_end - elapsed_end)
    counts: dict[str, int] = {}
    for attempt in selected:
        key = attempt.result or "active"
        counts[key] = counts.get(key, 0) + 1
    return DaySummary(
        day=day,
        anchor_start=anchor,
        window_end=window_end,
        work_intervals=work_intervals,
        raw_recording_seconds=raw_seconds,
        effective_work_seconds=effective,
        rest_seconds=rest,
        remaining_seconds=remaining,
        attempt_count=len(selected),
        result_counts=counts,
    )


def _local_date(timestamp: float) -> date:
    return datetime.fromtimestamp(timestamp).astimezone().date()


def local_day_bounds(day: date) -> tuple[float, float]:
    start = datetime.combine(day, datetime.min.time()).astimezone()
    return start.timestamp(), (start + timedelta(days=1)).timestamp()
