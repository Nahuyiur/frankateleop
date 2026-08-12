from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import date, datetime, timedelta
from pathlib import Path
import time

from PyQt6 import QtCore, QtGui, QtWidgets

from franka_gui.brand_theme import (
    AUXILIARY_WINDOW_STYLESHEET,
    MUKA_BLUE,
    MUKA_CHARCOAL,
    MUKA_DEEP_BLUE,
    MUKA_ORANGE,
    MUKA_PURPLE,
    make_aux_header,
)

from .ledger import WorkLedger
from .model import DaySummary, WorkAttempt, build_day_summary, local_day_bounds


RESULT_LABELS = {
    "active": "进行中",
    "discarded": "已丢弃",
    "interrupted": "异常中断",
    "save_failed": "保存失败",
    "saved_failure": "失败样本",
    "saved_high": "高质量",
    "saved_low": "低质量",
    "validation_failed": "核验失败",
}


def format_duration(seconds: float) -> str:
    value = max(0, int(round(seconds)))
    hours, remainder = divmod(value, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


class SevenDayTimeline(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.summaries: list[DaySummary] = []
        self.now = time.time()
        self.setMinimumHeight(165)

    def set_data(self, summaries: list[DaySummary], now: float) -> None:
        self.summaries = summaries
        self.now = now
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#ffffff"))
        left, right, top = 72, 300, 24
        width = max(1, self.width() - left - right)
        row_height = max(19, (self.height() - top - 12) / 7)
        for tick in (0, 3, 6, 9):
            x = left + width * tick / 9
            painter.setPen(QtGui.QColor("#d9e2ec"))
            painter.drawLine(QtCore.QPointF(x, top - 5), QtCore.QPointF(x, self.height() - 5))
            painter.setPen(QtGui.QColor("#66727e"))
            painter.drawText(int(x - 14), 14, f"{tick}h")
        for row, summary in enumerate(self.summaries):
            y = top + row * row_height
            if summary.day == datetime.fromtimestamp(self.now).astimezone().date():
                painter.setPen(QtCore.Qt.PenStyle.NoPen)
                painter.setBrush(QtGui.QColor("#f2f8fb"))
                painter.drawRoundedRect(QtCore.QRectF(0, y + 1, self.width(), row_height - 3), 5, 5)
            painter.setPen(QtGui.QColor(MUKA_DEEP_BLUE))
            painter.drawText(7, int(y + 20), summary.day.strftime("%m-%d"))
            track = QtCore.QRectF(left, y + 6, width, 16)
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(QtGui.QColor("#edf1f4"))
            painter.drawRoundedRect(track, 4, 4)
            if summary.anchor_start is not None and summary.window_end is not None:
                elapsed_end = min(max(self.now, summary.anchor_start), summary.window_end)
                elapsed_width = width * max(0.0, elapsed_end - summary.anchor_start) / (9 * 3600)
                painter.setBrush(QtGui.QColor("#d8e0e6"))
                painter.drawRoundedRect(QtCore.QRectF(left, y + 6, elapsed_width, 16), 4, 4)
                for interval in summary.work_intervals:
                    x1 = left + width * (interval.start - summary.anchor_start) / (9 * 3600)
                    x2 = left + width * (interval.end - summary.anchor_start) / (9 * 3600)
                    painter.setBrush(QtGui.QColor(MUKA_BLUE))
                    painter.drawRoundedRect(QtCore.QRectF(x1, y + 6, max(2, x2 - x1), 16), 4, 4)
            stats = (
                f"有效 {format_duration(summary.effective_work_seconds)}  ·  "
                f"录制 {format_duration(summary.raw_recording_seconds)}  ·  "
                f"休息 {format_duration(summary.rest_seconds)}"
            )
            painter.setPen(QtGui.QColor(MUKA_CHARCOAL))
            painter.drawText(int(left + width + 14), int(y + 20), stats)


class WorktimeDashboard(QtWidgets.QMainWindow):
    _refresh_loaded = QtCore.pyqtSignal(object, str)

    def __init__(
        self,
        ledger: WorkLedger | None = None,
        *,
        active_session_ids=(),
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.ledger = ledger or WorkLedger()
        self.active_session_ids = tuple(active_session_ids)
        self._refresh_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="work-monitor-read")
        self._refresh_running = False
        self._refresh_pending = False
        self._refresh_future = None
        self._closing = False
        self.setWindowTitle("北京莫刻机器人 · 数采工时监控")
        self.resize(1380, 860)
        self.setMinimumSize(1060, 680)
        self.setStyleSheet(AUXILIARY_WINDOW_STYLESHEET)
        self._build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(1000)
        self._refresh_loaded.connect(self._apply_refresh)
        self.refresh()

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(
            make_aux_header(
                "数采工时监控",
                "MUKA ROBOTICS  ·  OPERATIONS",
                "每秒自动刷新",
            )
        )

        body = QtWidgets.QWidget()
        body_layout = QtWidgets.QVBoxLayout(body)
        body_layout.setContentsMargins(22, 18, 22, 18)
        body_layout.setSpacing(12)

        filter_frame = QtWidgets.QFrame()
        filter_frame.setObjectName("ToolBarFrame")
        filters = QtWidgets.QHBoxLayout(filter_frame)
        filters.setContentsMargins(14, 10, 14, 10)
        filters.setSpacing(12)
        filter_heading = QtWidgets.QVBoxLayout()
        filter_heading.setSpacing(1)
        filter_title = QtWidgets.QLabel("筛选范围")
        filter_title.setObjectName("SectionTitle")
        filter_hint = QtWidgets.QLabel("查看人员与任务的 7 日记录")
        filter_hint.setObjectName("SectionMeta")
        filter_heading.addWidget(filter_title)
        filter_heading.addWidget(filter_hint)
        filters.addLayout(filter_heading)
        filters.addSpacing(8)
        self.operator_combo = QtWidgets.QComboBox()
        self.task_combo = QtWidgets.QComboBox()
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItem("全部模式", "")
        self.mode_combo.addItem("A · 左臂", "A")
        self.mode_combo.addItem("B · 右臂", "B")
        self.mode_combo.addItem("C · 双臂", "C")
        self.date_edit = QtWidgets.QDateEdit(QtCore.QDate.currentDate())
        self.date_edit.setCalendarPopup(True)
        refresh_btn = QtWidgets.QPushButton("刷新")
        refresh_btn.setObjectName("PrimaryAction")
        refresh_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_BrowserReload))
        refresh_btn.clicked.connect(self.refresh)
        for label, widget in (
            ("数采员", self.operator_combo),
            ("任务", self.task_combo),
            ("模式", self.mode_combo),
            ("日期", self.date_edit),
        ):
            group = QtWidgets.QVBoxLayout()
            group.setSpacing(2)
            caption = QtWidgets.QLabel(label)
            caption.setObjectName("FilterLabel")
            group.addWidget(caption)
            group.addWidget(widget)
            filters.addLayout(group)
        filters.addStretch()
        filters.addWidget(refresh_btn)
        body_layout.addWidget(filter_frame)
        for combo in (self.operator_combo, self.task_combo, self.mode_combo):
            combo.currentIndexChanged.connect(self.refresh)
        self.date_edit.dateChanged.connect(self.refresh)

        metrics = QtWidgets.QHBoxLayout()
        metrics.setSpacing(10)
        self.metric_labels: dict[str, QtWidgets.QLabel] = {}
        for key, label in (
            ("work", "有效工作"),
            ("recording", "实际录制"),
            ("rest", "休息"),
            ("remaining", "剩余"),
            ("attempts", "尝试数"),
        ):
            box = QtWidgets.QFrame()
            box.setObjectName("MetricCard")
            box.setProperty("metric", key)
            box.setMinimumHeight(78)
            box_layout = QtWidgets.QVBoxLayout(box)
            box_layout.setContentsMargins(13, 9, 13, 9)
            box_layout.setSpacing(2)
            caption = QtWidgets.QLabel(label)
            caption.setObjectName("MetricCaption")
            value = QtWidgets.QLabel("00:00:00" if key != "attempts" else "0")
            value.setObjectName("MetricValue")
            box_layout.addWidget(caption)
            box_layout.addWidget(value)
            self.metric_labels[key] = value
            metrics.addWidget(box)
        body_layout.addLayout(metrics)

        timeline_frame = QtWidgets.QFrame()
        timeline_frame.setObjectName("ToolBarFrame")
        timeline_layout = QtWidgets.QVBoxLayout(timeline_frame)
        timeline_layout.setContentsMargins(14, 10, 14, 10)
        timeline_layout.setSpacing(4)
        timeline_header = QtWidgets.QHBoxLayout()
        timeline_title = QtWidgets.QLabel("近 7 日工作轴")
        timeline_title.setObjectName("SectionTitle")
        timeline_legend = QtWidgets.QLabel("蓝色 有效工作（含 ≤60 秒短停）  ·  灰色 已发生休息  ·  浅色 剩余")
        timeline_legend.setObjectName("SectionMeta")
        timeline_header.addWidget(timeline_title)
        timeline_header.addStretch()
        timeline_header.addWidget(timeline_legend)
        timeline_layout.addLayout(timeline_header)
        self.timeline = SevenDayTimeline()
        timeline_layout.addWidget(self.timeline, 1)
        body_layout.addWidget(timeline_frame, 2)
        self.detail = QtWidgets.QLabel("暂无记录")
        self.detail.setWordWrap(True)
        self.detail.setObjectName("InfoStrip")
        body_layout.addWidget(self.detail)

        table_heading = QtWidgets.QHBoxLayout()
        table_title = QtWidgets.QLabel("当日录制明细")
        table_title.setObjectName("SectionTitle")
        self.table_count = QtWidgets.QLabel("0 条记录")
        self.table_count.setObjectName("SectionMeta")
        table_heading.addWidget(table_title)
        table_heading.addStretch()
        table_heading.addWidget(self.table_count)
        body_layout.addLayout(table_heading)
        self.table = QtWidgets.QTableWidget(0, 7)
        self.table.setAlternatingRowColors(True)
        self.table.setHorizontalHeaderLabels(("开始", "结束", "人员", "模式", "任务", "结果", "实际录制"))
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(34)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        body_layout.addWidget(self.table, 1)
        layout.addWidget(body, 1)

    def refresh(self, *_args) -> None:
        if self._closing:
            return
        if self._refresh_running:
            self._refresh_pending = True
            return
        now = time.time()
        query = {
            "now": now,
            "operator": str(self.operator_combo.currentData() or ""),
            "task": str(self.task_combo.currentData() or ""),
            "mode": str(self.mode_combo.currentData() or ""),
            "selected_day": self.date_edit.date().toPyDate(),
        }
        self._refresh_running = True
        future = self._refresh_executor.submit(self._query_refresh, query)
        self._refresh_future = future

        def done(completed) -> None:
            if self._closing:
                return
            try:
                result = completed.result()
                error = ""
            except Exception as exc:
                result = None
                error = f"{type(exc).__name__}: {exc}"
            self._refresh_loaded.emit(result, error)

        future.add_done_callback(done)

    def _query_refresh(self, query: dict) -> dict:
        now = float(query["now"])
        selected_day = query["selected_day"]
        self.ledger.recover_stale(now, exclude_session_ids=self.active_session_ids)
        operators, tasks = self.ledger.filter_values()
        first_day = selected_day - timedelta(days=6)
        start, _ = local_day_bounds(first_day)
        _, end = local_day_bounds(selected_day)
        attempts = self.ledger.list_attempts(
            started_after=start,
            started_before=end,
            operator_name=query["operator"],
            mode=query["mode"],
            task=query["task"],
            now=now,
        )
        summaries = [
            build_day_summary(first_day + timedelta(days=offset), attempts, now=now)
            for offset in range(7)
        ]
        return {
            "now": now,
            "selected_day": selected_day,
            "operators": operators,
            "tasks": tasks,
            "attempts": attempts,
            "summaries": summaries,
        }

    def _apply_refresh(self, result: dict | None, error: str) -> None:
        if self._closing:
            return
        self._refresh_running = False
        if error or result is None:
            self.detail.setText(f"账本读取失败: {error or '未知错误'}")
            self._finish_refresh()
            return
        try:
            now = result["now"]
            selected_day = result["selected_day"]
            attempts = result["attempts"]
            summaries = result["summaries"]
            self._sync_combo(self.operator_combo, "全部人员", result["operators"])
            self._sync_combo(self.task_combo, "全部任务", result["tasks"])
        except Exception as exc:
            self.detail.setText(f"监控结果渲染失败: {type(exc).__name__}: {exc}")
            self._finish_refresh()
            return
        selected = summaries[-1]
        self.timeline.set_data(summaries, now)
        self.metric_labels["work"].setText(format_duration(selected.effective_work_seconds))
        self.metric_labels["recording"].setText(format_duration(selected.raw_recording_seconds))
        self.metric_labels["rest"].setText(format_duration(selected.rest_seconds))
        self.metric_labels["remaining"].setText(format_duration(selected.remaining_seconds))
        self.metric_labels["attempts"].setText(str(selected.attempt_count))
        rests = selected.rest_intervals(now)
        rest_text = "；".join(
            f"{datetime.fromtimestamp(item.start).strftime('%H:%M:%S')}-{datetime.fromtimestamp(item.end).strftime('%H:%M:%S')}"
            for item in rests
        ) or "无"
        selected_attempts = [item for item in attempts if datetime.fromtimestamp(item.started_at).astimezone().date() == selected_day]
        result_text = "，".join(
            f"{RESULT_LABELS.get(key, key)}={value}"
            for key, value in sorted(selected.result_counts.items())
        ) or "无"
        save_errors = sum(item.save_error_count for item in selected_attempts)
        self.detail.setText(
            f"{selected_day.isoformat()}  休息区间：{rest_text}  |  "
            f"结果：{result_text}  |  保存异常事件={save_errors}"
        )
        self._render_attempts(selected_attempts, now)
        self._finish_refresh()

    def _finish_refresh(self) -> None:
        if self._refresh_pending:
            self._refresh_pending = False
            QtCore.QTimer.singleShot(0, self.refresh)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._closing = True
        self.timer.stop()
        if self._refresh_future is not None:
            self._refresh_future.cancel()
        self._refresh_executor.shutdown(wait=False)
        event.accept()

    @staticmethod
    def _sync_combo(combo: QtWidgets.QComboBox, all_label: str, values: list[str]) -> None:
        current = combo.currentData()
        expected = ["", *values]
        if [combo.itemData(index) for index in range(combo.count())] == expected:
            return
        combo.blockSignals(True)
        combo.clear()
        combo.addItem(all_label, "")
        for value in values:
            combo.addItem(value, value)
        index = combo.findData(current)
        combo.setCurrentIndex(max(0, index))
        combo.blockSignals(False)

    def _render_attempts(self, attempts: list[WorkAttempt], now: float) -> None:
        self.table.setRowCount(len(attempts))
        self.table_count.setText(f"{len(attempts)} 条记录")
        for row, attempt in enumerate(attempts):
            raw = sum(segment.duration for segment in attempt.segments)
            values = (
                datetime.fromtimestamp(attempt.started_at).strftime("%H:%M:%S"),
                datetime.fromtimestamp(attempt.ended_at).strftime("%H:%M:%S") if attempt.ended_at else "进行中",
                attempt.operator_name,
                attempt.mode,
                attempt.task,
                attempt.result or "active",
                format_duration(raw),
            )
            for column, value in enumerate(values):
                if column == 5:
                    value = RESULT_LABELS.get(value, value)
                item = QtWidgets.QTableWidgetItem(value)
                if column == 3:
                    mode_colors = {"A": MUKA_BLUE, "B": MUKA_ORANGE, "C": MUKA_PURPLE}
                    item.setForeground(QtGui.QColor(mode_colors.get(attempt.mode, MUKA_CHARCOAL)))
                elif column == 5:
                    result = attempt.result or "active"
                    if result.startswith("saved"):
                        item.setForeground(QtGui.QColor("#176247"))
                    elif result in {"active", "interrupted"}:
                        item.setForeground(QtGui.QColor(MUKA_BLUE if result == "active" else MUKA_ORANGE))
                    else:
                        item.setForeground(QtGui.QColor("#a63220"))
                self.table.setItem(row, column, item)


def main() -> int:
    import sys

    app = QtWidgets.QApplication(sys.argv)
    window = WorktimeDashboard()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
