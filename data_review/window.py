from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading

import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets

from franka_gui.brand_theme import (
    AUXILIARY_WINDOW_STYLESHEET,
    MUKA_BLUE,
    MUKA_ORANGE,
    MUKA_RED,
    make_aux_header,
)

from .frame_provider import EpisodeFrameProvider
from .model import (
    FAIL_JOINT_DELTA,
    MAX_JOINT_VELOCITY,
    WARN_JOINT_DELTA,
    EpisodeReview,
    ReviewEvent,
    load_episode_review,
)
from .widgets import (
    CameraView,
    EndEffectorPathWidget,
    EventStripWidget,
    JointTimelineWidget,
    ScalarTimelineWidget,
)


class DataReviewWindow(QtWidgets.QMainWindow):
    _frame_decoded = QtCore.pyqtSignal(int, int, object, str)
    _episode_loaded = QtCore.pyqtSignal(int, object, object, str)

    def __init__(self, default_root: Path | None = None, parent=None) -> None:
        super().__init__(parent)
        self.default_root = default_root or (Path.home() / "Desktop" / "Muka_NAS")
        self.review: EpisodeReview | None = None
        self.provider: EpisodeFrameProvider | None = None
        self.camera_views: dict[str, CameraView] = {}
        self.current_arm = ""
        self._playing = False
        self._closing = False
        self._decode_generation = 0
        self._decode_running = False
        self._pending_frame: int | None = None
        self._decode_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="review-video")
        self._load_generation = 0
        self._load_cancel_event: threading.Event | None = None
        self._load_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="review-load")
        self.setWindowTitle("北京莫刻机器人 · 数据复核")
        self.resize(1580, 940)
        self.setMinimumSize(1120, 720)
        self.setStyleSheet(AUXILIARY_WINDOW_STYLESHEET)
        self._build_ui()
        self.play_timer = QtCore.QTimer(self)
        self.play_timer.setSingleShot(True)
        self.play_timer.timeout.connect(self._advance_frame)
        self._frame_decoded.connect(self._apply_decoded_frame)
        self._episode_loaded.connect(self._apply_loaded_episode)

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(
            make_aux_header(
                "采集数据复核",
                "MUKA ROBOTICS  ·  DATA REVIEW",
                "视频与动作联合诊断",
            )
        )

        body = QtWidgets.QWidget()
        body_layout = QtWidgets.QVBoxLayout(body)
        body_layout.setContentsMargins(18, 14, 18, 16)
        body_layout.setSpacing(10)

        path_frame = QtWidgets.QFrame()
        path_frame.setObjectName("PathBar")
        path_layout = QtWidgets.QHBoxLayout(path_frame)
        path_layout.setContentsMargins(13, 9, 13, 9)
        path_layout.setSpacing(9)
        path_heading = QtWidgets.QVBoxLayout()
        path_heading.setSpacing(1)
        path_title = QtWidgets.QLabel("Episode 数据")
        path_title.setObjectName("SectionTitle")
        path_hint = QtWidgets.QLabel("目录 / metadata.json / pkl.gz")
        path_hint.setObjectName("SectionMeta")
        path_heading.addWidget(path_title)
        path_heading.addWidget(path_hint)
        path_layout.addLayout(path_heading)
        path_layout.addSpacing(8)
        self.path_edit = QtWidgets.QLineEdit()
        self.path_edit.setPlaceholderText("选择 task/quality/episode 目录、metadata.json 或 .pkl.gz")
        self.choose_btn = QtWidgets.QPushButton("选择数据")
        self.choose_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_DirOpenIcon))
        self.choose_btn.clicked.connect(self._choose_episode)
        self.load_btn = QtWidgets.QPushButton("打开")
        self.load_btn.setObjectName("PrimaryAction")
        self.load_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_DialogOpenButton))
        self.load_btn.clicked.connect(lambda: self.request_open_episode(self.path_edit.text()))
        path_layout.addWidget(self.path_edit, 1)
        path_layout.addWidget(self.choose_btn)
        path_layout.addWidget(self.load_btn)
        body_layout.addWidget(path_frame)

        summary_frame = QtWidgets.QFrame()
        summary_frame.setObjectName("ToolBarFrame")
        summary_layout = QtWidgets.QHBoxLayout(summary_frame)
        summary_layout.setContentsMargins(12, 7, 12, 7)
        summary_layout.setSpacing(8)
        self.summary_badge = QtWidgets.QLabel("未加载")
        self.summary_badge.setObjectName("SummaryBadge")
        self.summary_badge.setProperty("status", "idle")
        self.summary = QtWidgets.QLabel("尚未加载 episode")
        self.summary.setWordWrap(True)
        self.summary.setObjectName("ReviewSummary")
        summary_layout.addWidget(self.summary_badge)
        summary_layout.addWidget(self.summary, 1)
        body_layout.addWidget(summary_frame)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        camera_container = QtWidgets.QFrame()
        camera_container.setObjectName("MediaWorkspace")
        self.camera_grid = QtWidgets.QGridLayout(camera_container)
        self.camera_grid.setContentsMargins(9, 9, 9, 9)
        self.camera_grid.setSpacing(9)
        self.camera_empty = QtWidgets.QLabel("尚未加载视频画面")
        self.camera_empty.setObjectName("MediaEmptyState")
        self.camera_empty.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.camera_grid.addWidget(self.camera_empty, 0, 0)
        splitter.addWidget(camera_container)

        lower = QtWidgets.QWidget()
        lower_layout = QtWidgets.QVBoxLayout(lower)
        lower_layout.setContentsMargins(0, 0, 0, 0)
        lower_layout.setSpacing(8)
        playback_bar = QtWidgets.QFrame()
        playback_bar.setObjectName("PlaybackBar")
        controls = QtWidgets.QHBoxLayout(playback_bar)
        controls.setContentsMargins(9, 7, 9, 7)
        controls.setSpacing(7)
        style = self.style()
        self.play_btn = QtWidgets.QToolButton()
        self.play_btn.setIcon(style.standardIcon(QtWidgets.QStyle.StandardPixmap.SP_MediaPlay))
        self.play_btn.setObjectName("PlaybackButton")
        self.play_btn.setAccessibleName("播放或暂停")
        self.play_btn.setToolTip("播放/暂停")
        self.play_btn.clicked.connect(self._toggle_play)
        self.step_back_btn = QtWidgets.QToolButton()
        self.step_back_btn.setIcon(style.standardIcon(QtWidgets.QStyle.StandardPixmap.SP_MediaSkipBackward))
        self.step_back_btn.setObjectName("PlaybackButton")
        self.step_back_btn.setAccessibleName("上一帧")
        self.step_back_btn.setToolTip("上一帧")
        self.step_back_btn.clicked.connect(lambda: self._set_frame(self.frame_slider.value() - 1))
        self.step_next_btn = QtWidgets.QToolButton()
        self.step_next_btn.setIcon(style.standardIcon(QtWidgets.QStyle.StandardPixmap.SP_MediaSkipForward))
        self.step_next_btn.setObjectName("PlaybackButton")
        self.step_next_btn.setAccessibleName("下一帧")
        self.step_next_btn.setToolTip("下一帧")
        self.step_next_btn.clicked.connect(lambda: self._set_frame(self.frame_slider.value() + 1))
        self.frame_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.frame_slider.setRange(0, 0)
        self.frame_slider.valueChanged.connect(self._render_frame)
        self.frame_label = QtWidgets.QLabel("frame 0/0 · 0.00 s")
        self.frame_label.setObjectName("FrameReadout")
        self.speed_combo = QtWidgets.QComboBox()
        for label, value in (("0.5×", 0.5), ("1×", 1.0), ("2×", 2.0)):
            self.speed_combo.addItem(label, value)
        self.speed_combo.setCurrentIndex(1)
        self.speed_combo.currentIndexChanged.connect(self._playback_speed_changed)
        self.arm_combo = QtWidgets.QComboBox()
        self.arm_combo.currentTextChanged.connect(self._arm_changed)
        for widget in (self.step_back_btn, self.play_btn, self.step_next_btn):
            controls.addWidget(widget)
        controls.addWidget(self.frame_slider, 1)
        controls.addWidget(self.frame_label)
        controls.addWidget(QtWidgets.QLabel("速度"))
        controls.addWidget(self.speed_combo)
        controls.addWidget(QtWidgets.QLabel("机械臂"))
        controls.addWidget(self.arm_combo)
        lower_layout.addWidget(playback_bar)

        self.event_strip = EventStripWidget()
        lower_layout.addWidget(self.event_strip)
        self.tabs = QtWidgets.QTabWidget()
        joint_page = QtWidgets.QWidget()
        joint_layout = QtWidgets.QVBoxLayout(joint_page)
        joint_layout.setContentsMargins(0, 6, 0, 0)
        threshold_note = QtWidgets.QLabel(
            f"橙色虚线：Franka 物理关节限位   |   单帧变化 WARN > {WARN_JOINT_DELTA:.2f} rad   "
            f"|   FAIL > {FAIL_JOINT_DELTA:.2f} rad   |   速度 FAIL > {MAX_JOINT_VELOCITY:.1f} rad/s"
        )
        threshold_note.setStyleSheet("color:#596774;padding:0 8px;")
        self.joint_plot = JointTimelineWidget()
        joint_layout.addWidget(threshold_note)
        joint_layout.addWidget(self.joint_plot, 1)
        self.tabs.addTab(joint_page, "Joint 角度")
        motion_page = QtWidgets.QWidget()
        motion_layout = QtWidgets.QHBoxLayout(motion_page)
        self.eef_plot = EndEffectorPathWidget()
        self.gripper_plot = ScalarTimelineWidget()
        motion_layout.addWidget(self.eef_plot, 3)
        motion_layout.addWidget(self.gripper_plot, 2)
        self.tabs.addTab(motion_page, "末端与夹爪")
        diagnostics_page = QtWidgets.QWidget()
        diagnostics_layout = QtWidgets.QVBoxLayout(diagnostics_page)
        self.event_table = QtWidgets.QTableWidget(0, 6)
        self.event_table.setAlternatingRowColors(True)
        self.event_table.setHorizontalHeaderLabels(("级别", "时间", "机械臂", "关节", "类型", "详情"))
        self.event_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.event_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.event_table.verticalHeader().setVisible(False)
        self.event_table.verticalHeader().setDefaultSectionSize(34)
        self.event_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.event_table.horizontalHeader().setStretchLastSection(True)
        self.event_table.cellDoubleClicked.connect(self._jump_to_event)
        diagnostics_layout.addWidget(self.event_table)
        self.tabs.addTab(diagnostics_page, "质量事件")
        lower_layout.addWidget(self.tabs, 1)
        splitter.addWidget(lower)
        splitter.setSizes((420, 480))
        body_layout.addWidget(splitter, 1)
        layout.addWidget(body, 1)

    def _set_review_status(self, text: str, status: str, summary: str) -> None:
        self.summary_badge.setText(text)
        self.summary_badge.setProperty("status", status)
        self.summary_badge.style().unpolish(self.summary_badge)
        self.summary_badge.style().polish(self.summary_badge)
        self.summary.setText(summary)

    def _choose_episode(self) -> None:
        selected = QtWidgets.QFileDialog.getExistingDirectory(self, "选择 episode 目录", str(self.default_root))
        if selected:
            self.path_edit.setText(selected)
            self.request_open_episode(selected)

    def request_open_episode(self, path_like: str | Path) -> None:
        """Load and validate an episode without blocking the GUI thread."""
        if self._closing:
            return
        value = str(path_like).strip()
        if not value:
            return
        self._stop_playback()
        if self._load_cancel_event is not None:
            self._load_cancel_event.set()
        cancel_event = threading.Event()
        self._load_cancel_event = cancel_event
        self._load_generation += 1
        generation = self._load_generation
        self.load_btn.setEnabled(False)
        self.choose_btn.setEnabled(False)
        self._set_review_status("核验中", "loading", "正在后台加载 episode 并运行完整 V 核验…")
        future = self._load_executor.submit(self._load_episode, value, cancel_event)

        def done(completed) -> None:
            try:
                review, provider = completed.result()
                error = ""
            except Exception as exc:
                review, provider = None, None
                error = f"{type(exc).__name__}: {exc}"
            if self._closing or generation != self._load_generation or cancel_event.is_set():
                if provider is not None:
                    provider.close()
                return
            self._episode_loaded.emit(generation, review, provider, error)

        future.add_done_callback(done)

    @staticmethod
    def _load_episode(
        value: str,
        cancel_event: threading.Event | None = None,
    ) -> tuple[EpisodeReview, EpisodeFrameProvider]:
        review = load_episode_review(value, cancel_event=cancel_event)
        return review, EpisodeFrameProvider(review)

    def open_episode(self, path_like: str | Path) -> None:
        """Synchronous loader retained for deterministic tests and tooling."""
        value = str(path_like).strip()
        if not value:
            return
        self._stop_playback()
        try:
            review, provider = self._load_episode(value)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "数据无法打开", f"{type(exc).__name__}: {exc}")
            return
        self._install_episode(review, provider)

    def _apply_loaded_episode(
        self,
        generation: int,
        review: EpisodeReview | None,
        provider: EpisodeFrameProvider | None,
        error: str,
    ) -> None:
        if generation != self._load_generation:
            if provider is not None:
                provider.close()
            return
        self.load_btn.setEnabled(True)
        self.choose_btn.setEnabled(True)
        if error or review is None or provider is None:
            self._set_review_status("无法打开", "fail", f"数据无法打开: {error or '未知加载错误'}")
            return
        self._install_episode(review, provider)

    def _install_episode(
        self,
        review: EpisodeReview,
        provider: EpisodeFrameProvider,
    ) -> None:
        self._reset_decoder(close_provider=True)
        self.review, self.provider = review, provider
        self.path_edit.setText(str(review.pkl_path.parent))
        self._rebuild_cameras(review.camera_names)
        self.frame_slider.blockSignals(True)
        self.frame_slider.setRange(0, review.frame_count - 1)
        self.frame_slider.setValue(0)
        self.frame_slider.blockSignals(False)
        self.arm_combo.blockSignals(True)
        self.arm_combo.clear()
        self.arm_combo.addItems(list(review.arms))
        self.arm_combo.blockSignals(False)
        self.current_arm = next(iter(review.arms))
        self.arm_combo.setCurrentText(self.current_arm)
        self.event_strip.set_review(review)
        self._render_diagnostics(review)
        self._render_summary()
        self._arm_changed(self.current_arm)
        self._set_frame(0)

    def _rebuild_cameras(self, names: tuple[str, ...]) -> None:
        while self.camera_grid.count():
            item = self.camera_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().hide()
                item.widget().deleteLater()
        self.camera_empty = None
        self.camera_views = {}
        columns = 3 if len(names) >= 3 else max(1, len(names))
        for index, name in enumerate(names):
            view = CameraView(name)
            self.camera_views[name] = view
            self.camera_grid.addWidget(view, index // columns, index % columns)

    def _render_summary(self) -> None:
        assert self.review is not None
        metadata = self.review.metadata
        action_failures = sum(event.severity == "fail" for event in self.review.events)
        action_warnings = sum(event.severity == "warn" for event in self.review.events)
        validator = self.review.validator_report
        status = str(validator.status).upper()
        status_style = "fail" if status == "FAIL" else "warn" if status == "WARN" or validator.warning_count else "pass"
        self._set_review_status(
            f"V · {status}",
            status_style,
            f"任务: {metadata.get('task', '—')}  ·  质量: {metadata.get('quality', '—')}  ·  "
            f"数采员: {metadata.get('operator_name', '历史未标注')}  ·  "
            f"{self.review.frame_count} 帧 / {self.review.duration:.2f} s / {self.review.fps:.2f} FPS  ·  "
            f"相机 {len(self.review.camera_names)} 路  ·  V: {validator.error_count} ERROR, {validator.warning_count} WARN  ·  "
            f"Joint: {action_failures} FAIL, {action_warnings} WARN",
        )

    def _arm_changed(self, name: str) -> None:
        if self.review is None or name not in self.review.arms:
            return
        self.current_arm = name
        arm = self.review.arms[name]
        self.joint_plot.set_data(self.review.timeline, arm, self.review.events, self.review.keyframes)
        self.eef_plot.set_pose(arm.poses)
        series = []
        if arm.gripper_width.size:
            series.append(("反馈", arm.gripper_width, QtGui.QColor(MUKA_BLUE), QtCore.Qt.PenStyle.SolidLine))
        if arm.gripper_target_width.size:
            series.append(("目标", arm.gripper_target_width, QtGui.QColor(MUKA_ORANGE), QtCore.Qt.PenStyle.DashLine))
        self.gripper_plot.set_series(self.review.timeline, series)
        self._render_frame(self.frame_slider.value())

    def _render_diagnostics(self, review: EpisodeReview) -> None:
        rows = [("validator", issue) for issue in review.validator_report.issues]
        rows.extend(("joint", event) for event in review.events)
        self.event_table.setRowCount(len(rows))
        assert self.review is not None
        for row, (source, item) in enumerate(rows):
            if source == "validator":
                severity = "FAIL" if item.level == "ERROR" else "WARN"
                values = (severity, "—", "V", "—", item.code, item.message)
                frame_index = None
            else:
                severity = item.severity.upper()
                values = (
                    severity,
                    f"{self.review.timeline[item.frame_index]:.3f}s",
                    item.arm,
                    f"J{item.joint_index + 1}",
                    item.kind,
                    item.message,
                )
                frame_index = item.frame_index
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setData(QtCore.Qt.ItemDataRole.UserRole, frame_index)
                if column == 0:
                    item.setForeground(QtGui.QColor(MUKA_RED if severity == "FAIL" else MUKA_ORANGE))
                self.event_table.setItem(row, column, item)

    def _jump_to_event(self, row: int, _column: int) -> None:
        item = self.event_table.item(row, 0)
        frame_index = item.data(QtCore.Qt.ItemDataRole.UserRole) if item is not None else None
        if frame_index is not None:
            self._set_frame(int(frame_index))

    def _toggle_play(self) -> None:
        if self.review is None:
            return
        self._playing = not self._playing
        icon = QtWidgets.QStyle.StandardPixmap.SP_MediaPause if self._playing else QtWidgets.QStyle.StandardPixmap.SP_MediaPlay
        self.play_btn.setIcon(self.style().standardIcon(icon))
        if self._playing:
            self._schedule_next_frame()
        else:
            self.play_timer.stop()

    def _advance_frame(self) -> None:
        if self.review is None:
            return
        next_frame = self.frame_slider.value() + 1
        if next_frame >= self.review.frame_count:
            self._stop_playback()
            return
        self._set_frame(next_frame)
        self._schedule_next_frame()

    def _playback_speed_changed(self) -> None:
        if self._playing and self.review is not None:
            self._schedule_next_frame()

    def _schedule_next_frame(self) -> None:
        if not self._playing or self.review is None:
            return
        index = self.frame_slider.value()
        if index + 1 >= self.review.frame_count:
            self._stop_playback()
            return
        delta = float(self.review.timeline[index + 1] - self.review.timeline[index])
        speed = float(self.speed_combo.currentData())
        self.play_timer.start(max(5, int(1000.0 * max(0.0, delta) / speed)))

    def _stop_playback(self) -> None:
        self._playing = False
        if hasattr(self, "play_timer"):
            self.play_timer.stop()
        if hasattr(self, "play_btn"):
            self.play_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_MediaPlay))

    def _set_frame(self, index: int) -> None:
        if self.review is None:
            return
        self.frame_slider.setValue(max(0, min(index, self.review.frame_count - 1)))

    def _render_frame(self, index: int) -> None:
        if self.review is None or self.provider is None:
            return
        seconds = float(self.review.timeline[index])
        self.frame_label.setText(f"frame {index + 1}/{self.review.frame_count} · {seconds:.2f} s")
        self.joint_plot.set_cursor(seconds)
        self.event_strip.set_cursor(seconds)
        self.gripper_plot.set_cursor(seconds)
        self.eef_plot.set_cursor(index)
        self._pending_frame = index
        if not self._decode_running:
            self._submit_decode(index)

    def _submit_decode(self, index: int) -> None:
        if self.provider is None:
            return
        self._decode_running = True
        generation = self._decode_generation
        provider = self.provider
        future = self._decode_executor.submit(provider.read, index)

        def done(completed) -> None:
            try:
                images = completed.result()
                error = ""
            except Exception as exc:
                images = {}
                error = f"{type(exc).__name__}: {exc}"
            if not self._closing and generation == self._decode_generation:
                self._frame_decoded.emit(generation, index, images, error)

        future.add_done_callback(done)

    def _apply_decoded_frame(self, generation: int, index: int, images, error: str) -> None:
        if generation != self._decode_generation:
            return
        self._decode_running = False
        pending = self._pending_frame
        if pending is not None and pending != index:
            self._submit_decode(pending)
            return
        if error:
            self._stop_playback()
            self.summary.setText(f"读取 frame {index} 失败: {error}")
            self._pending_frame = None
            for view in self.camera_views.values():
                view.show_error()
        else:
            for name, image in images.items():
                if name in self.camera_views:
                    self.camera_views[name].set_bgr(image)

    def _reset_decoder(self, *, close_provider: bool) -> None:
        self._decode_generation += 1
        self._pending_frame = None
        old_executor = self._decode_executor
        old_provider = self.provider if close_provider else None
        if close_provider:
            self.provider = None
        if old_provider is not None:
            try:
                old_executor.submit(old_provider.close)
            except RuntimeError:
                old_provider.close()
        old_executor.shutdown(wait=False)
        self._decode_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="review-video")
        self._decode_running = False

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._closing = True
        self._stop_playback()
        if self._load_cancel_event is not None:
            self._load_cancel_event.set()
        self._load_generation += 1
        self._load_executor.shutdown(wait=False)
        self._reset_decoder(close_provider=True)
        self._decode_executor.shutdown(wait=False)
        event.accept()


def main(argv=None) -> int:
    import sys

    parser = argparse.ArgumentParser(description="复核 Franka 采集 episode 的多视角与动作轨迹")
    parser.add_argument("episode", nargs="?", help="episode 目录、metadata.json 或 .pkl.gz")
    args = parser.parse_args(argv)
    app = QtWidgets.QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
    window = DataReviewWindow()
    window.show()
    if args.episode:
        QtCore.QTimer.singleShot(0, lambda: window.request_open_episode(args.episode))
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
