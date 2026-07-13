"""Main PyQt window for FR3 capture."""

from __future__ import annotations

import json
import os
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from PyQt6 import QtCore, QtGui, QtWidgets

from franka_capture.config.fr3_single import DEFAULT_CAMERAS

from .capture_controller import (
    FAILURE_DIR,
    HIGH_QUALITY_DIR,
    LOW_QUALITY_DIR,
    CaptureController,
)
from .keyframe_snapshot import KEYFRAME_DIR_NAME, save_snapshot_frames
from .process_manager import ProcessManager
from .replay_launcher import (
    ReplayEpisodeInfo,
    inspect_replay_input,
    validate_replay_target,
)

# Display at about 15 Hz while the recording path remains fixed at 30 Hz.
# This halves GUI conversion/paint work without dropping recorded frames.
PREVIEW_POLL_INTERVAL_MS = 66


@dataclass(frozen=True)
class ReplayLaunchOptions:
    path: str
    latest: bool
    run_mode: str
    speed: float
    gripper_speed: float
    gripper_force: float
    gripper_event_delta: float
    gripper_replay_mode: str
    gripper_command_hz: float
    gripper_hold_sec: float
    approach_start: bool
    approach_start_max_delta: float
    approach_start_step_delta: float
    approach_start_hz: float


class CameraView(QtWidgets.QFrame):
    def __init__(self, name: str, parent=None) -> None:
        super().__init__(parent)
        self.name = name
        self._last_rgb: Optional[np.ndarray] = None
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setObjectName("CameraView")
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)
        self.title = QtWidgets.QLabel(name)
        self.title.setObjectName("CameraTitle")
        self.label = QtWidgets.QLabel("等待画面")
        self.label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label.setMinimumSize(280, 210)
        self.label.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        self.label.setStyleSheet("background: #050505; color: #9ca3af;")
        layout.addWidget(self.title)
        layout.addWidget(self.label, 1)

    def update_image(self, rgb: np.ndarray) -> None:
        self._last_rgb = rgb
        self._refresh_pixmap()

    def copy_current_rgb(self) -> Optional[np.ndarray]:
        if self._last_rgb is None:
            return None
        return np.ascontiguousarray(self._last_rgb.copy())

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        if self._last_rgb is not None:
            self._refresh_pixmap()

    def _refresh_pixmap(self) -> None:
        if self._last_rgb is None:
            return
        if self.label.width() <= 1 or self.label.height() <= 1:
            return
        image = _rgb_to_qimage(self._last_rgb)
        pixmap = QtGui.QPixmap.fromImage(image)
        self.label.setPixmap(
            pixmap.scaled(
                self.label.size(),
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                QtCore.Qt.TransformationMode.SmoothTransformation,
            )
        )


class AccentBar(QtWidgets.QFrame):
    def __init__(self, theme: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName(f"AccentBar{theme}")
        self.setFixedHeight(8)


class MainWindow(QtWidgets.QMainWindow):
    _tasks_loaded = QtCore.pyqtSignal(list, str)
    _disk_loaded = QtCore.pyqtSignal(object)
    _next_indices_loaded = QtCore.pyqtSignal(str, dict)
    _snapshot_saved = QtCore.pyqtSignal(str)
    _snapshot_failed = QtCore.pyqtSignal(str)

    def __init__(
        self,
        controller: CaptureController,
        process_manager: ProcessManager,
        repo_root: Path,
        profile_key: str = "",
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.process_manager = process_manager
        self.repo_root = repo_root
        self.profile_key = _safe_profile_key(profile_key or self.controller.options.mode)
        self._form_state_path = _form_state_path(self.profile_key)
        self._loading_form_state = False
        self.camera_views: Dict[str, CameraView] = {}
        self.saved_count = 0
        self._episode_state = "idle"
        self._local_save_queue_count = 0
        self._closing = False
        self._replay_process: Optional[QtCore.QProcess] = None
        self._replay_log_dialog: Optional[QtWidgets.QDialog] = None
        self._replay_log_text: Optional[QtWidgets.QPlainTextEdit] = None
        self._tasks_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gui-tasks")
        self._index_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gui-index")
        self._disk_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gui-disk")
        self._snapshot_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="gui-keyframe-save",
        )
        self._tasks_future: Optional[Future] = None
        self._disk_future: Optional[Future] = None
        self._next_indices_future: Optional[Future] = None
        self._snapshot_future: Optional[Future] = None
        self._next_indices_task = ""
        self._fixed_output_root = str(Path.home() / "Desktop" / "Muka_NAS")
        self.controller.options.output_root = self._fixed_output_root

        window_titles = {
            "single": "Franka Single-Arm Capture",
            "right": "Franka Right-Arm Capture",
            "dual": "Franka Dual-Arm Capture",
        }
        self.setWindowTitle(window_titles[self.controller.options.mode])
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.resize(1760, 960)
        self.setMinimumSize(1280, 760)
        self._build_ui()
        self._set_quality_controls_enabled(False)
        self._connect_signals()
        self._refresh_tasks()
        self._load_form_state()
        self._connect_form_state_signals()
        self.disk_label.setText("Local capture disk: idle check pending")

        self.disk_timer = QtCore.QTimer(self)
        self.disk_timer.timeout.connect(self._refresh_disk)
        self.disk_timer.start(60000)
        self.preview_timer = QtCore.QTimer(self)
        self.preview_timer.timeout.connect(self._poll_preview)
        self.preview_timer.start(PREVIEW_POLL_INTERVAL_MS)
        self.sync_timer = QtCore.QTimer(self)
        self.sync_timer.timeout.connect(self._refresh_sync_backlog)
        self.sync_timer.start(5000)
        self._refresh_sync_backlog()

        QtCore.QTimer.singleShot(250, self.controller.start_preview)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._save_form_state()
        if self._episode_state in {"recording", "paused"}:
            QtWidgets.QMessageBox.warning(
                self,
                "仍在采集中",
                "当前 episode 还没有保存或丢弃。请先按 e 保存，或按 d 丢弃。",
            )
            event.ignore()
            return
        if self._episode_state == "quality_pending":
            QtWidgets.QMessageBox.warning(
                self,
                "等待质量分层",
                "当前 episode 已结束但还没有按 h/l/f 分层保存。"
                "请先按 h 保存到高质量，按 l 保存到低质量，按 f 保存到 Failure，或按 d 丢弃。",
            )
            event.ignore()
            return
        if self.controller.has_open_episode_transition():
            QtWidgets.QMessageBox.warning(
                self,
                "采集状态切换中",
                "当前 episode 正在开始、结束或丢弃，请等待状态切换完成后再关闭。",
            )
            event.ignore()
            return
        if self._is_replay_running():
            QtWidgets.QMessageBox.warning(
                self,
                "Replay 正在运行",
                "当前 replay 进程还没有结束。请先等待完成，或在 replay 日志窗口中停止。",
            )
            event.ignore()
            return
        pending_saves = self.controller.inflight_save_count()
        if self._episode_state == "saving" and pending_saves == 0:
            pending_saves = 1
        if pending_saves > 0:
            QtWidgets.QMessageBox.warning(
                self,
                "仍在后台保存",
                f"还有 {pending_saves} 个 episode 正在后台保存。请等待保存完成后再关闭。",
            )
            event.ignore()
            return
        if self._snapshot_future is not None and not self._snapshot_future.done():
            QtWidgets.QMessageBox.warning(
                self,
                "仍在保存相机帧",
                "当前相机帧正在写入 NAS。请等待完成后再关闭。",
            )
            event.ignore()
            return
        deferred_retries = self.controller.deferred_quality_retry_count()
        if deferred_retries > 0:
            QtWidgets.QMessageBox.warning(
                self,
                "存在保存失败待处理",
                f"还有 {deferred_retries} 个保存失败的 episode 待重新分层或丢弃。"
                "请先处理后再关闭。",
            )
            event.ignore()
            return
        self._closing = True
        if not self.controller.shutdown():
            self._closing = False
            event.ignore()
            return
        for future in (
            self._tasks_future,
            self._next_indices_future,
            self._disk_future,
            self._snapshot_future,
        ):
            if future is not None:
                future.cancel()
        _shutdown_executor(self._tasks_executor)
        _shutdown_executor(self._index_executor)
        _shutdown_executor(self._disk_executor)
        _shutdown_executor(self._snapshot_executor)
        self.process_manager.stop_stack_blocking()
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)
        event.accept()

    def eventFilter(self, obj, event) -> bool:
        if QtWidgets.QApplication.activeModalWidget() is not None:
            return super().eventFilter(obj, event)
        widget = obj if isinstance(obj, QtWidgets.QWidget) else None
        if widget is not None and widget.window() is not self:
            return super().eventFilter(obj, event)

        if event.type() == QtCore.QEvent.Type.MouseButtonPress:
            self._release_text_focus_if_needed(widget)
        elif event.type() == QtCore.QEvent.Type.KeyPress:
            if self._handle_capture_shortcut(event):
                return True
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if not self._handle_capture_shortcut(event):
            super().keyPressEvent(event)

    def _handle_capture_shortcut(self, event: QtGui.QKeyEvent) -> bool:
        if event.modifiers() & (
            QtCore.Qt.KeyboardModifier.ControlModifier
            | QtCore.Qt.KeyboardModifier.AltModifier
            | QtCore.Qt.KeyboardModifier.MetaModifier
        ):
            return False
        if _is_text_input_widget(QtWidgets.QApplication.focusWidget()):
            return False

        key = event.key()
        capture_keys = {
            QtCore.Qt.Key.Key_S,
            QtCore.Qt.Key.Key_W,
            QtCore.Qt.Key.Key_E,
            QtCore.Qt.Key.Key_D,
            QtCore.Qt.Key.Key_K,
            QtCore.Qt.Key.Key_H,
            QtCore.Qt.Key.Key_L,
            QtCore.Qt.Key.Key_F,
            QtCore.Qt.Key.Key_Q,
        }
        if key in capture_keys and self._is_replay_running():
            self._set_status("Replay 运行中，采集快捷键已忽略")
            event.accept()
            return True
        if key == QtCore.Qt.Key.Key_S:
            self._start_recording()
        elif key == QtCore.Qt.Key.Key_W:
            self.controller.pause()
        elif key == QtCore.Qt.Key.Key_E:
            self.controller.finish()
        elif key == QtCore.Qt.Key.Key_D:
            self.controller.discard()
        elif key == QtCore.Qt.Key.Key_K:
            if self._episode_state == "recording":
                self.controller.add_keyframe()
            else:
                self._save_current_frame()
        elif key == QtCore.Qt.Key.Key_H:
            self.controller.mark_high_quality()
        elif key == QtCore.Qt.Key.Key_L:
            self.controller.mark_low_quality()
        elif key == QtCore.Qt.Key.Key_F:
            self.controller.mark_failure()
        elif key == QtCore.Qt.Key.Key_Q:
            self.controller.finish()
        else:
            return False
        event.accept()
        return True

    def _release_text_focus_if_needed(self, clicked_widget: Optional[QtWidgets.QWidget]) -> None:
        if clicked_widget is not None and _is_text_input_widget(clicked_widget):
            return
        focus_widget = QtWidgets.QApplication.focusWidget()
        if _is_text_input_widget(focus_widget):
            focus_widget.clearFocus()
            self.setFocus(QtCore.Qt.FocusReason.MouseFocusReason)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self.left_panel = self._build_left_panel()
        self.center_panel = self._build_center_panel()
        self.camera_panel = self._build_camera_panel()
        root.addWidget(self.left_panel)
        root.addWidget(self.center_panel)
        root.addWidget(self.camera_panel, 1)

        self.setStyleSheet(
            """
            QMainWindow { background: #f6f8fc; }
            QFrame#Panel {
                background: #ffffff;
                border: 1px solid #e5e7eb;
                border-radius: 8px;
            }
            QFrame#LeftPanel {
                background: #ffffff;
                border: 1px solid #bfdbfe;
                border-radius: 8px;
            }
            QFrame#CenterPanel {
                background: #ffffff;
                border: 1px solid #bbf7d0;
                border-radius: 8px;
            }
            QFrame#CameraPanel {
                background: #ffffff;
                border: 1px solid #ddd6fe;
                border-radius: 8px;
            }
            QFrame#CameraView {
                background: #ffffff;
                border: 1px solid #e9d5ff;
                border-radius: 8px;
            }
            QFrame#AccentBarBlue {
                border-radius: 4px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2563eb,
                    stop:1 #06b6d4
                );
            }
            QFrame#AccentBarGreen {
                border-radius: 4px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #22c55e,
                    stop:1 #facc15
                );
            }
            QFrame#AccentBarPurple {
                border-radius: 4px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #7c3aed,
                    stop:1 #ec4899
                );
            }
            QLabel#AppTitle {
                font-size: 20px;
                font-weight: 900;
                color: #0f172a;
            }
            QLabel#SectionTitle { font-weight: 800; color: #111827; }
            QLabel#CameraTitle { font-weight: 800; color: #5b21b6; }
            QLabel#OutputRoot {
                color: #475569;
                background: #f8fafc;
                border: 1px solid #e2e8f0;
                border-radius: 6px;
                padding: 6px;
            }
            QPushButton {
                min-height: 34px;
                border-radius: 6px;
                border: 1px solid transparent;
                background: #f8fafc;
                color: #0f172a;
                font-weight: 700;
            }
            QPushButton:hover { background: #eef2ff; border-color: #c7d2fe; }
            QPushButton#Danger { background: #fff1f2; color: #be123c; border-color: #fecdd3; }
            QPushButton#Danger:hover { background: #ffe4e6; }
            QPushButton#Primary {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #06b6d4,
                    stop:1 #2563eb
                );
                color: white;
            }
            QPushButton#Success {
                background: #ecfdf5;
                color: #047857;
                border-color: #a7f3d0;
            }
            QPushButton#Success:hover { background: #d1fae5; }
            QPushButton#Purple {
                background: #f5f3ff;
                color: #6d28d9;
                border-color: #ddd6fe;
            }
            QPushButton#Purple:hover { background: #ede9fe; }
            QPushButton#Neutral { background: #f8fafc; color: #0f172a; border-color: #e2e8f0; }
            QLineEdit, QComboBox {
                min-height: 30px;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 0 8px;
                background: white;
            }
            QPlainTextEdit {
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 6px;
                background: #ffffff;
            }
            QProgressBar {
                border: 1px solid #cbd5e1;
                border-radius: 5px;
                text-align: center;
                background: #f8fafc;
            }
            QProgressBar::chunk {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #22c55e,
                    stop:1 #06b6d4
                );
                border-radius: 4px;
            }
            """
        )

    def _build_left_panel(self) -> QtWidgets.QFrame:
        panel = QtWidgets.QFrame()
        panel.setObjectName("LeftPanel")
        panel.setFixedWidth(188)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        app_titles = {
            "single": "FR3 Single Capture",
            "right": "FR3 Right Capture",
            "dual": "FR3 Dual Capture",
        }
        app_title = QtWidgets.QLabel(app_titles[self.controller.options.mode])
        app_title.setObjectName("AppTitle")
        layout.addWidget(app_title)
        layout.addWidget(AccentBar("Blue"))

        title = QtWidgets.QLabel("控制中心")
        title.setObjectName("SectionTitle")
        layout.addWidget(title)

        start_labels = {
            "single": "启动 1-4",
            "right": "启动右臂 1-4",
            "dual": "启动双臂 1-4",
        }
        self.start_stack_btn = QtWidgets.QPushButton(start_labels[self.process_manager.mode])
        self.start_stack_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_MediaPlay))
        self.stop_stack_btn = QtWidgets.QPushButton("停止全部")
        self.stop_stack_btn.setObjectName("Danger")
        self.stop_env_btn = QtWidgets.QPushButton("停止遥操作")
        self.stop_env_btn.setObjectName("Neutral")
        self.preview_btn = QtWidgets.QPushButton("重启预览")
        self.preview_btn.setObjectName("Neutral")
        self.log_btn = QtWidgets.QPushButton("查看日志")
        self.log_btn.setObjectName("Neutral")

        layout.addWidget(self.start_stack_btn)
        layout.addWidget(self.stop_stack_btn)
        layout.addWidget(self.stop_env_btn)
        layout.addWidget(self.preview_btn)
        layout.addWidget(self.log_btn)

        layout.addSpacing(6)
        replay_title = QtWidgets.QLabel("数据复现")
        replay_title.setObjectName("SectionTitle")
        layout.addWidget(replay_title)
        self.replay_btn = QtWidgets.QPushButton("Replay")
        self.replay_btn.setObjectName("Purple")
        layout.addWidget(self.replay_btn)

        layout.addSpacing(6)
        capture_title = QtWidgets.QLabel("数据采集")
        capture_title.setObjectName("SectionTitle")
        layout.addWidget(capture_title)
        self.record_btn = QtWidgets.QPushButton("开始")
        self.record_btn.setObjectName("Primary")
        self.pause_btn = QtWidgets.QPushButton("暂停")
        self.pause_btn.setObjectName("Neutral")
        self.end_btn = QtWidgets.QPushButton("保存当前")
        self.end_btn.setObjectName("Success")
        self.discard_btn = QtWidgets.QPushButton("丢弃当前")
        self.discard_btn.setObjectName("Danger")
        self.keyframe_btn = QtWidgets.QPushButton("关键帧")
        self.keyframe_btn.setObjectName("Purple")
        self.save_frame_btn = QtWidgets.QPushButton("保存帧")
        self.save_frame_btn.setObjectName("Neutral")
        self.high_quality_btn = QtWidgets.QPushButton("高质量 H")
        self.high_quality_btn.setObjectName("Success")
        self.low_quality_btn = QtWidgets.QPushButton("低质量 L")
        self.low_quality_btn.setObjectName("Neutral")
        self.failure_btn = QtWidgets.QPushButton("失败 F")
        self.failure_btn.setObjectName("Purple")
        layout.addWidget(self.record_btn)
        layout.addWidget(self.pause_btn)
        layout.addWidget(self.end_btn)
        layout.addWidget(self.discard_btn)
        layout.addWidget(self.save_frame_btn)
        layout.addWidget(self.keyframe_btn)
        layout.addWidget(self.high_quality_btn)
        layout.addWidget(self.low_quality_btn)
        layout.addWidget(self.failure_btn)

        layout.addSpacing(6)
        disk_title = QtWidgets.QLabel("磁盘容量")
        disk_title.setObjectName("SectionTitle")
        layout.addWidget(disk_title)
        self.disk_bar = QtWidgets.QProgressBar()
        self.disk_label = QtWidgets.QLabel("")
        layout.addWidget(self.disk_bar)
        layout.addWidget(self.disk_label)
        layout.addStretch(1)
        return panel

    def _build_center_panel(self) -> QtWidgets.QFrame:
        panel = QtWidgets.QFrame()
        panel.setObjectName("CenterPanel")
        panel.setFixedWidth(340)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        title = QtWidgets.QLabel("任务总览")
        title.setObjectName("SectionTitle")
        layout.addWidget(title)
        layout.addWidget(AccentBar("Green"))

        self.task_combo = QtWidgets.QComboBox()
        self.task_combo.setEditable(True)
        self.task_combo.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
        self.instruction_edit = QtWidgets.QPlainTextEdit()
        self.instruction_edit.setPlaceholderText("Describe the desired behavior for this episode")
        self.instruction_edit.setMaximumHeight(72)
        self.new_task_btn = QtWidgets.QPushButton("新增采集任务")
        self.new_task_btn.setObjectName("Neutral")
        self.fps_label = QtWidgets.QLabel("30 Hz")
        self.fps_label.setObjectName("OutputRoot")
        self.output_root_label = QtWidgets.QLabel(self._fixed_output_root)
        self.output_root_label.setObjectName("OutputRoot")
        self.output_root_label.setWordWrap(True)
        self.next_path_label = QtWidgets.QLabel("")
        self.next_path_label.setObjectName("OutputRoot")
        self.next_path_label.setWordWrap(True)
        self.next_path_label.setMinimumHeight(92)
        self.next_path_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.next_path_label.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Minimum,
        )

        form = QtWidgets.QFormLayout()
        form.addRow("任务名称", self.task_combo)
        form.addRow("Text instruction", self.instruction_edit)
        form.addRow("", self.new_task_btn)
        form.addRow("采集频率", self.fps_label)
        form.addRow("保存根目录", self.output_root_label)
        layout.addLayout(form)

        next_path_title = QtWidgets.QLabel("下一条路径")
        next_path_title.setObjectName("SectionTitle")
        layout.addWidget(next_path_title)
        layout.addWidget(self.next_path_label)

        metadata_title = QtWidgets.QLabel("附加 metadata")
        metadata_title.setObjectName("SectionTitle")
        self.metadata_edit = QtWidgets.QPlainTextEdit()
        self.metadata_edit.setPlaceholderText('普通备注，或 JSON 对象，例如 {"operator": "pnp"}')
        self.metadata_edit.setMaximumHeight(64)
        layout.addWidget(metadata_title)
        layout.addWidget(self.metadata_edit)

        self.stack_state = QtWidgets.QLabel("机器人栈: 未启动")
        self.capture_state = QtWidgets.QLabel("采集状态: 等待相机")
        self.active_episode = QtWidgets.QLabel("当前 episode: -")
        self.frame_count = QtWidgets.QLabel("当前帧数: 0")
        self.saved_episodes = QtWidgets.QLabel("已保存: 0")
        self.save_queue = QtWidgets.QLabel("本地保存队列: 0")
        for label in (
            self.stack_state,
            self.capture_state,
            self.active_episode,
            self.frame_count,
            self.saved_episodes,
            self.save_queue,
        ):
            layout.addWidget(label)

        layout.addSpacing(8)
        self.status_box = QtWidgets.QLabel("READY")
        self.status_box.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_box.setMinimumHeight(72)
        self.status_box.setStyleSheet("background: #ecfdf5; color: #166534; border-radius: 8px; font-size: 26px; font-weight: 800;")
        layout.addWidget(self.status_box)

        hint = QtWidgets.QLabel(
            "快捷键:\ns 开始/继续, w 暂停, e/q 结束待分层\n"
            "h 高质量保存, l 低质量保存, f 失败保存\n"
            "d 丢弃, k 录制中标记关键帧/非录制保存帧"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        layout.addStretch(1)
        return panel

    def _build_camera_panel(self) -> QtWidgets.QFrame:
        panel = QtWidgets.QFrame()
        panel.setObjectName("CameraPanel")
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        title = QtWidgets.QLabel("相机视角")
        title.setObjectName("SectionTitle")
        layout.addWidget(title)
        layout.addWidget(AccentBar("Purple"))
        self.camera_grid_widget = QtWidgets.QWidget()
        self.camera_grid = QtWidgets.QGridLayout(self.camera_grid_widget)
        self.camera_grid.setContentsMargins(0, 0, 0, 0)
        self.camera_grid.setSpacing(8)
        for col in range(6):
            self.camera_grid.setColumnStretch(col, 1)
        self.camera_grid.setRowStretch(0, 1)
        self.camera_grid.setRowStretch(1, 1)
        self.camera_grid_widget.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        layout.addWidget(self.camera_grid_widget, 1)
        return panel

    def _connect_signals(self) -> None:
        self.start_stack_btn.clicked.connect(self._start_stack_clicked)
        self.stop_stack_btn.clicked.connect(self._stop_stack_clicked)
        self.stop_env_btn.clicked.connect(self._stop_env_clicked)
        self.preview_btn.clicked.connect(self._restart_preview)
        self.log_btn.clicked.connect(self._show_logs)
        self.replay_btn.clicked.connect(self._open_replay_dialog)

        self.record_btn.clicked.connect(self._start_recording)
        self.pause_btn.clicked.connect(self.controller.pause)
        self.end_btn.clicked.connect(self.controller.finish)
        self.discard_btn.clicked.connect(self.controller.discard)
        self.save_frame_btn.clicked.connect(self._save_current_frame)
        self.keyframe_btn.clicked.connect(self.controller.add_keyframe)
        self.high_quality_btn.clicked.connect(self.controller.mark_high_quality)
        self.low_quality_btn.clicked.connect(self.controller.mark_low_quality)
        self.failure_btn.clicked.connect(self.controller.mark_failure)
        self.new_task_btn.clicked.connect(self._create_task)
        self.task_combo.currentTextChanged.connect(self._update_next_path)

        self.controller.status_changed.connect(self._set_status)
        self.controller.error.connect(self._show_error)
        self.controller.cameras_ready.connect(self._setup_camera_views)
        self.controller.active_episode_changed.connect(self._active_episode_changed)
        self.controller.recording_frame_count.connect(lambda count: self.frame_count.setText(f"当前帧数: {count}"))
        self.controller.save_queue_changed.connect(self._save_queue_changed)
        self.controller.episode_saved.connect(self._episode_saved)
        self._snapshot_saved.connect(self._on_snapshot_saved)
        self._snapshot_failed.connect(self._on_snapshot_failed)
        self._tasks_loaded.connect(self._apply_tasks)
        self._disk_loaded.connect(self._apply_disk_usage)
        self._next_indices_loaded.connect(self._apply_next_indices)

        self.process_manager.status_changed.connect(self._set_status)
        self.process_manager.error.connect(self._show_error)
        self.process_manager.stack_state_changed.connect(self._stack_state_changed)

    def _connect_form_state_signals(self) -> None:
        self.task_combo.currentTextChanged.connect(self._save_form_state)
        self.task_combo.editTextChanged.connect(self._save_form_state)
        self.instruction_edit.textChanged.connect(self._save_form_state)
        self.metadata_edit.textChanged.connect(self._save_form_state)

    def _start_stack_clicked(self) -> None:
        if self._guard_replay_running("Replay 运行中不能启动机器人栈。"):
            return
        self.process_manager.start_stack()

    def _stop_stack_clicked(self) -> None:
        if self._guard_replay_running("Replay 运行中不能从 GUI 停止采集栈；请先停止 replay。"):
            return
        self.process_manager.stop_stack()

    def _stop_env_clicked(self) -> None:
        if self._guard_replay_running("Replay 运行中不能停止 teleop env；请先停止 replay。"):
            return
        self.process_manager.stop_teleop_env()

    def _start_recording(self) -> None:
        self._save_form_state()
        if self._guard_replay_running("Replay 运行中不能开始或继续采集。"):
            return
        task = self.task_combo.currentText().strip()
        if not task:
            self._show_error("任务名称不能为空")
            return
        if not _is_valid_task_name(task):
            self._show_error("任务名称不能为 . 或 ..，也不能包含 /、\\ 或空字符")
            return
        if task == KEYFRAME_DIR_NAME:
            self._show_error(f"任务名称 {KEYFRAME_DIR_NAME!r} 保留给相机帧保存目录。")
            return
        instruction = self.instruction_edit.toPlainText().strip()
        if not instruction:
            self._show_error("Text instruction 不能为空")
            return
        user_metadata = self._parse_user_metadata()
        if user_metadata is None:
            return
        self.controller.options.output_root = self._fixed_output_root
        self.controller.start_or_resume(task, instruction, user_metadata=user_metadata)

    def _ensure_output_root_available(self) -> bool:
        # Final output availability is checked in the background publish stage.
        return True

    def _restart_preview(self) -> None:
        if self._guard_replay_running("Replay 运行中不能重启预览。"):
            return
        if self._episode_state in {"recording", "paused", "quality_pending"}:
            self._show_error("录制、暂停或等待分层中不能重启预览，请先保存分层或丢弃当前 episode。")
            return
        if not self.controller.stop_preview():
            return
        self.controller.start_preview()

    def _setup_camera_views(self, names) -> None:
        names = list(names)
        for view in self.camera_views.values():
            view.setParent(None)
        while self.camera_grid.count():
            item = self.camera_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().setParent(None)
        self.camera_views.clear()
        if not names:
            placeholder = QtWidgets.QLabel("没有检测到可用 RealSense 相机\n无法开始录制")
            placeholder.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            placeholder.setMinimumSize(520, 320)
            placeholder.setStyleSheet(
                "background: #050505; color: #d1d5db; border-radius: 8px; font-size: 20px;"
            )
            self.camera_grid.addWidget(placeholder, 0, 0, 2, 6)
            self.camera_grid.setRowStretch(0, 1)
            self.camera_grid.setRowStretch(1, 1)
            self.capture_state.setText("采集状态: 未检测到相机")
            return
        for name in names:
            view = CameraView(name)
            self.camera_views[name] = view
        self._place_camera_views(names)
        self.capture_state.setText(f"采集状态: 相机已连接 {names}")

    def _place_camera_views(self, names) -> None:
        for col in range(6):
            self.camera_grid.setColumnStretch(col, 1)
        for row in range(8):
            self.camera_grid.setRowStretch(row, 0)

        if self.controller.options.mode == "dual":
            preferred_positions = {
                "left": (0, 0, 1, 3),
                "right": (0, 3, 1, 3),
                "left_wrist": (1, 0, 1, 2),
                "middle": (1, 2, 1, 2),
                "right_wrist": (1, 4, 1, 2),
            }
        elif self.controller.options.mode == "right":
            preferred_positions = {
                "middle": (0, 0, 1, 3),
                "right": (0, 3, 1, 3),
                "right_wrist": (1, 0, 1, 6),
            }
        else:
            preferred_positions = {
                "left": (0, 0, 1, 3),
                "middle": (0, 3, 1, 3),
                "left_wrist": (1, 0, 1, 6),
            }

        placed = set()
        for name in preferred_positions:
            view = self.camera_views.get(name)
            if view is None:
                continue
            self.camera_grid.addWidget(view, *preferred_positions[name])
            self.camera_grid.setRowStretch(preferred_positions[name][0], 1)
            placed.add(name)

        extra_names = [name for name in names if name not in placed]
        for idx, name in enumerate(extra_names):
            view = self.camera_views[name]
            row = 2 + idx // 2
            col = 0 if idx % 2 == 0 else 3
            self.camera_grid.addWidget(view, row, col, 1, 3)
            self.camera_grid.setRowStretch(row, 1)

    def _poll_preview(self) -> None:
        frames = self.controller.take_preview_frame()
        if frames is not None:
            self._update_preview(frames)

    def _update_preview(self, frames: Dict[str, np.ndarray]) -> None:
        for name, rgb in frames.items():
            view = self.camera_views.get(name)
            if view is not None:
                view.update_image(rgb)

    def _save_current_frame(self) -> None:
        if self._guard_replay_running("Replay 运行中不能保存相机帧。"):
            return
        if self._episode_state == "recording":
            self._set_status("录制中请使用 k 添加 episode 关键帧；保存帧按钮已禁用。")
            return
        if self._snapshot_future is not None and not self._snapshot_future.done():
            self._set_status("上一组相机帧仍在保存，请稍等。")
            return
        output_root = Path(self._fixed_output_root).expanduser()
        if not output_root.is_dir() or not os.path.ismount(output_root):
            self._show_error(
                f"NAS 未挂载到 {output_root}；已拒绝保存相机帧，避免误写入本地目录。"
            )
            return

        expected_names = tuple(self.controller.options.camera_names or DEFAULT_CAMERAS.keys())
        frames: Dict[str, np.ndarray] = {}
        missing = []
        for name in expected_names:
            view = self.camera_views.get(name)
            rgb = view.copy_current_rgb() if view is not None else None
            if rgb is None:
                missing.append(name)
            else:
                frames[name] = rgb
        if missing:
            self._show_error(f"以下相机尚无可保存画面: {missing}。请等待预览恢复后重试。")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._set_status(f"正在保存相机帧: keyframe/{timestamp}")
        self._snapshot_future = self._snapshot_executor.submit(
            save_snapshot_frames,
            self._fixed_output_root,
            frames,
            timestamp=timestamp,
        )
        self._update_save_frame_enabled()
        self._snapshot_future.add_done_callback(self._snapshot_future_done)

    def _snapshot_future_done(self, future: Future) -> None:
        try:
            output_dir = future.result()
        except Exception as exc:
            self._snapshot_failed.emit(str(exc))
            return
        self._snapshot_saved.emit(str(output_dir))

    def _on_snapshot_saved(self, output_dir: str) -> None:
        self._snapshot_future = None
        self._update_save_frame_enabled()
        self._set_status(f"相机帧已保存: {_display_output_path(output_dir, self._fixed_output_root)}")

    def _on_snapshot_failed(self, message: str) -> None:
        self._snapshot_future = None
        self._update_save_frame_enabled()
        self._show_error(f"保存相机帧失败: {message}")

    def _update_save_frame_enabled(self) -> None:
        if not hasattr(self, "save_frame_btn"):
            return
        busy = self._snapshot_future is not None and not self._snapshot_future.done()
        self.save_frame_btn.setEnabled(
            not self._closing
            and not self._is_replay_running()
            and self._episode_state != "recording"
            and not busy
        )

    def _active_episode_changed(self, task: str, index: int, state: str) -> None:
        self._episode_state = state
        self._update_save_frame_enabled()
        if state == "recording":
            self._set_config_controls_enabled(False)
            self._set_quality_controls_enabled(False)
            self.status_box.setText("REC")
            self.status_box.setStyleSheet("background: #fff7ed; color: #9a3412; border-radius: 8px; font-size: 26px; font-weight: 800;")
        elif state == "paused":
            self._set_config_controls_enabled(False)
            self._set_quality_controls_enabled(False)
            self.status_box.setText("PAUSE")
            self.status_box.setStyleSheet("background: #fefce8; color: #854d0e; border-radius: 8px; font-size: 26px; font-weight: 800;")
        elif state == "saving":
            self._set_config_controls_enabled(True)
            self._set_quality_controls_enabled(False)
            self.status_box.setText("SAVING")
            self.status_box.setStyleSheet("background: #ecfdf5; color: #166534; border-radius: 8px; font-size: 26px; font-weight: 800;")
        elif state == "quality_pending":
            self._set_config_controls_enabled(False)
            self._set_quality_controls_enabled(True)
            self.status_box.setText("JUDGING")
            self.status_box.setStyleSheet("background: #eff6ff; color: #1d4ed8; border-radius: 8px; font-size: 26px; font-weight: 800;")
        else:
            self._set_config_controls_enabled(True)
            self._set_quality_controls_enabled(False)
            self.status_box.setText("READY")
            self.status_box.setStyleSheet("background: #ecfdf5; color: #166534; border-radius: 8px; font-size: 26px; font-weight: 800;")
        self.active_episode.setText(f"当前 episode: {task}/{index} ({state})")

    def _episode_saved(self, task: str, index: int, output_dir: str, frames: int) -> None:
        self.saved_count += 1
        self.saved_episodes.setText(f"已保存: {self.saved_count}")
        if self._episode_state not in {"recording", "paused", "quality_pending", "finishing"}:
            prefix = (
                "NAS 已保存: "
                if self.controller.options.direct_to_output_root
                else "本地已保存，等待 NAS 同步: "
            )
            self.active_episode.setText(
                prefix + f"{_display_output_path(output_dir, self._fixed_output_root)}, frames={frames}"
            )
        self._refresh_tasks()
        self._update_next_path()
        self._refresh_sync_backlog()

    def _save_queue_changed(self, count: int) -> None:
        self._local_save_queue_count = int(count)
        self._refresh_sync_backlog()

    def _refresh_sync_backlog(self) -> None:
        pending_sync = self.controller.pending_sync_count()
        if self.controller.options.direct_to_output_root:
            text = f"NAS 保存中: {self._local_save_queue_count}"
            if pending_sync:
                text += f" | 旧 outbox 待同步: {pending_sync}"
            self.save_queue.setText(text)
            return
        self.save_queue.setText(
            f"本地保存: {self._local_save_queue_count} | 待 NAS 同步: {pending_sync}"
        )

    def _stack_state_changed(self, state: str) -> None:
        self.stack_state.setText(f"机器人栈: {state}")

    def _set_status(self, text: str) -> None:
        self.capture_state.setText(f"状态: {text}")

    def _show_error(self, text: str) -> None:
        self.capture_state.setText(f"错误: {text.splitlines()[0] if text else ''}")
        QtWidgets.QMessageBox.critical(self, "错误", text)

    def _show_logs(self) -> None:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("运行日志")
        dialog.resize(900, 620)
        layout = QtWidgets.QVBoxLayout(dialog)
        text = QtWidgets.QPlainTextEdit()
        text.setReadOnly(True)
        text.setPlainText(self.process_manager.tail_logs(120) or "暂无脚本日志")
        layout.addWidget(text)
        close_btn = QtWidgets.QPushButton("关闭")
        close_btn.clicked.connect(dialog.accept)
        layout.addWidget(close_btn)
        dialog.exec()

    def _open_replay_dialog(self) -> None:
        if self._episode_state in {"recording", "paused", "quality_pending"}:
            self._show_error("录制、暂停或等待分层中不能 replay，请先保存分层或丢弃当前 episode。")
            return
        if self.controller.inflight_save_count() > 0 or self._episode_state == "saving":
            self._show_error("后台保存队列未清空，避免 replay 和保存抢占资源。请等待保存完成。")
            return
        if self._is_replay_running():
            self._focus_replay_log_dialog()
            self._set_status("Replay 正在运行；已打开日志窗口")
            return

        dialog = ReplayOptionsDialog(Path(self._fixed_output_root).expanduser(), self)
        if dialog.exec() != QtWidgets.QDialog.DialogCode.Accepted:
            return
        options = dialog.options()

        try:
            info = inspect_replay_input(options.path, latest=options.latest)
            target = validate_replay_target(info, self.controller.options.mode)
        except Exception as exc:
            self._show_error(f"Replay 数据和当前 GUI 不匹配，或路径无效:\n{exc}")
            return

        if options.run_mode == "execute":
            message = (
                f"将执行 replay，并向机器人发送命令。\n\n"
                f"GUI 模式: {_mode_label(self.controller.options.mode)}\n"
                f"数据类型: {info.display_kind}\n"
                f"Episode: {info.episode_dir}\n"
                f"Frames: {info.frame_count_text}\n\n"
                "如果起点偏差超过普通阈值但仍在自动靠近安全上限内，"
                "执行模式会先慢速靠近 frame 0。\n"
                "执行前确认没有其他 teleop/recording 控制同一机械臂，"
                "对应机械臂已在安全位置，工作区清空，E-stop 可触达。"
            )
            response = QtWidgets.QMessageBox.warning(
                self,
                "确认执行 Replay",
                message,
                QtWidgets.QMessageBox.StandardButton.Ok
                | QtWidgets.QMessageBox.StandardButton.Cancel,
                QtWidgets.QMessageBox.StandardButton.Cancel,
            )
            if response != QtWidgets.QMessageBox.StandardButton.Ok:
                return

        try:
            self.process_manager.prepare_for_replay(target, options.run_mode)
        except Exception as exc:
            self._show_error(f"Replay 前置检查失败:\n{exc}")
            return

        self._launch_replay(info, target, options)

    def _launch_replay(
        self,
        info: ReplayEpisodeInfo,
        target: str,
        options: ReplayLaunchOptions,
    ) -> None:
        script_name = "16_replay_bi_arm_pipeline.sh" if target == "dual" else "7_replay_fr3.sh"
        script_path = self.repo_root / script_name
        if not script_path.exists():
            self._show_error(f"未找到 replay 脚本: {script_path}")
            return

        args = [str(script_path), str(info.episode_dir)]
        if target in {"left", "right"}:
            args.extend(["--arm", target])
        if options.run_mode == "file_check":
            args.append("--skip-robot-check")
        elif options.run_mode == "execute":
            args.append("--execute")

        process = QtCore.QProcess(self)
        process.setProgram("bash")
        process.setArguments(args)
        process.setWorkingDirectory(str(self.repo_root))
        process.setProcessChannelMode(QtCore.QProcess.ProcessChannelMode.MergedChannels)
        process.setProcessEnvironment(self._replay_environment(options, target))
        process.readyReadStandardOutput.connect(self._read_replay_output)
        process.readyReadStandardError.connect(self._read_replay_output)
        process.finished.connect(self._replay_finished)
        process.errorOccurred.connect(self._replay_process_error)

        self._replay_process = process
        self._show_replay_log_dialog(info, target, options, ["bash", *args])
        self._set_replay_controls_running(True)
        self._append_replay_log(">>> Starting replay process ...\n")
        process.start()
        if not process.waitForStarted(3000):
            self._show_error(f"Replay 进程启动失败: {process.errorString()}")
            self._replay_process = None
            self._set_replay_controls_running(False)
            return
        self._set_status("Replay 已启动")

    def _replay_environment(
        self,
        options: ReplayLaunchOptions,
        target: str,
    ) -> QtCore.QProcessEnvironment:
        env = QtCore.QProcessEnvironment.systemEnvironment()
        env.insert("PYTHONUNBUFFERED", "1")
        env.insert("DEFAULT_REPLAY_SPEED", _format_float(options.speed))
        env.insert("DEFAULT_GRIPPER_SPEED", _format_float(options.gripper_speed))
        env.insert("DEFAULT_GRIPPER_FORCE", _format_float(options.gripper_force))
        env.insert("DEFAULT_GRIPPER_EVENT_DELTA", _format_float(options.gripper_event_delta))
        env.insert("DEFAULT_GRIPPER_REPLAY_MODE", options.gripper_replay_mode)
        env.insert("DEFAULT_GRIPPER_COMMAND_HZ", _format_float(options.gripper_command_hz))
        env.insert("DEFAULT_GRIPPER_HOLD_SEC", _format_float(options.gripper_hold_sec))
        env.insert("DEFAULT_APPROACH_START", "1" if options.approach_start else "0")
        env.insert("DEFAULT_APPROACH_START_MAX_DELTA", _format_float(options.approach_start_max_delta))
        env.insert("DEFAULT_APPROACH_START_STEP_DELTA", _format_float(options.approach_start_step_delta))
        env.insert("DEFAULT_APPROACH_START_HZ", _format_float(options.approach_start_hz))

        if target == "dual":
            password = _read_gui_password()
            if password:
                for key in (
                    "BI_ARM_LOCAL_SUDO_PASSWORD",
                    "BI_ARM_REMOTE_SUDO_PASSWORD",
                    "BI_ARM_SSH_PASSWORD",
                ):
                    if not env.value(key):
                        env.insert(key, password)
        return env

    def _show_replay_log_dialog(
        self,
        info: ReplayEpisodeInfo,
        target: str,
        options: ReplayLaunchOptions,
        command: list[str],
    ) -> None:
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Replay 日志")
        dialog.resize(920, 640)
        layout = QtWidgets.QVBoxLayout(dialog)
        text = QtWidgets.QPlainTextEdit()
        text.setReadOnly(True)
        text.setPlainText(
            "Replay target: "
            f"{target}\n"
            f"Run mode: {options.run_mode}\n"
            f"Episode: {info.episode_dir}\n"
            f"Resolved pkl: {info.episode_file}\n"
            f"Schema: {info.schema_version}, frames={info.frame_count_text}\n"
            f"Command: {_shell_join(command)}\n\n"
        )
        layout.addWidget(text)
        buttons = QtWidgets.QDialogButtonBox()
        stop_btn = buttons.addButton("停止 replay", QtWidgets.QDialogButtonBox.ButtonRole.DestructiveRole)
        close_btn = buttons.addButton("隐藏日志", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole)
        stop_btn.clicked.connect(self._stop_replay_process)
        close_btn.clicked.connect(dialog.hide)
        layout.addWidget(buttons)
        self._replay_log_dialog = dialog
        self._replay_log_text = text
        dialog.show()

    def _append_replay_log(self, text: str) -> None:
        if not text:
            return
        if self._replay_log_text is None:
            return
        self._replay_log_text.moveCursor(QtGui.QTextCursor.MoveOperation.End)
        self._replay_log_text.insertPlainText(text)
        self._replay_log_text.moveCursor(QtGui.QTextCursor.MoveOperation.End)

    def _read_replay_output(self) -> None:
        process = self._replay_process
        if process is None:
            return
        data = bytes(process.readAllStandardOutput())
        if data:
            self._append_replay_log(data.decode("utf-8", errors="replace"))

    def _replay_finished(self, exit_code: int, exit_status: QtCore.QProcess.ExitStatus) -> None:
        self._read_replay_output()
        if exit_status == QtCore.QProcess.ExitStatus.NormalExit and exit_code == 0:
            self._append_replay_log("\n>>> Replay process finished successfully.\n")
            self._set_status("Replay 完成")
        else:
            self._append_replay_log(f"\n>>> Replay process failed: exit_code={exit_code}\n")
            self._set_status(f"Replay 失败: exit_code={exit_code}")
        self._replay_process = None
        self._set_replay_controls_running(False)

    def _replay_process_error(self, error: QtCore.QProcess.ProcessError) -> None:
        process = self._replay_process
        message = process.errorString() if process is not None else str(error)
        self._append_replay_log(f"\n>>> Replay process error: {message}\n")
        self._set_status(f"Replay 进程错误: {message}")

    def _stop_replay_process(self) -> None:
        process = self._replay_process
        if process is None:
            return
        self._append_replay_log("\n>>> Stopping replay process ...\n")
        process.terminate()
        QtCore.QTimer.singleShot(20000, self._force_kill_replay_process)

    def _force_kill_replay_process(self) -> None:
        process = self._replay_process
        if process is not None and process.state() != QtCore.QProcess.ProcessState.NotRunning:
            self._append_replay_log("\n>>> Replay process did not stop after 20s; killing it.\n")
            process.kill()

    def _is_replay_running(self) -> bool:
        return (
            self._replay_process is not None
            and self._replay_process.state() != QtCore.QProcess.ProcessState.NotRunning
        )

    def _focus_replay_log_dialog(self) -> None:
        if self._replay_log_dialog is None:
            return
        self._replay_log_dialog.show()
        self._replay_log_dialog.raise_()
        self._replay_log_dialog.activateWindow()

    def _set_replay_controls_running(self, running: bool) -> None:
        self.replay_btn.setText("Replay 日志" if running else "Replay")
        for widget in (
            self.start_stack_btn,
            self.stop_stack_btn,
            self.stop_env_btn,
            self.preview_btn,
            self.record_btn,
            self.pause_btn,
            self.end_btn,
            self.discard_btn,
            self.save_frame_btn,
            self.keyframe_btn,
            self.new_task_btn,
        ):
            widget.setEnabled(not running)
        if running:
            self._set_config_controls_enabled(False)
            self._set_quality_controls_enabled(False)
            return
        self._update_save_frame_enabled()
        config_enabled = self._episode_state not in {"recording", "paused", "quality_pending"}
        self._set_config_controls_enabled(config_enabled)
        self._set_quality_controls_enabled(self._episode_state == "quality_pending")

    def _guard_replay_running(self, message: str) -> bool:
        if not self._is_replay_running():
            return False
        self._show_error(message)
        self._focus_replay_log_dialog()
        return True

    def _refresh_tasks(self) -> None:
        if self._closing:
            return
        current = self.task_combo.currentText().strip()
        if self._tasks_future is not None and not self._tasks_future.done():
            return
        self._tasks_future = self._tasks_executor.submit(self.controller.scan_tasks)
        self._tasks_future.add_done_callback(
            lambda future, current=current: self._tasks_future_done(future, current)
        )

    def _tasks_future_done(self, future: Future, current: str) -> None:
        if self._closing or future.cancelled():
            return
        try:
            tasks = future.result()
        except Exception:
            tasks = []
        self._tasks_loaded.emit(list(tasks), current)

    def _apply_tasks(self, tasks: list, previous_current: str) -> None:
        if self._closing:
            return
        current = self.task_combo.currentText().strip() or previous_current
        self.task_combo.blockSignals(True)
        self.task_combo.clear()
        self.task_combo.addItems([str(task) for task in tasks])
        if current:
            self.task_combo.setCurrentText(current)
        elif self.task_combo.count() == 0:
            self.task_combo.setCurrentText("default")
        self.task_combo.blockSignals(False)
        self._update_next_path()

    def _create_task(self) -> None:
        if self._guard_replay_running("Replay 运行中不能新增采集任务。"):
            return
        if self._episode_state in {"recording", "paused", "quality_pending"}:
            self._show_error("录制、暂停或等待分层中不能新增任务，请先保存分层或丢弃当前 episode。")
            return

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("新增采集任务")
        layout = QtWidgets.QVBoxLayout(dialog)
        name_edit = QtWidgets.QLineEdit()
        name_edit.setPlaceholderText("例如 put_eraser_into_drawer")
        form = QtWidgets.QFormLayout()
        form.addRow("任务名称", name_edit)
        layout.addLayout(form)
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok
            | QtWidgets.QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec() != QtWidgets.QDialog.DialogCode.Accepted:
            return
        task = name_edit.text().strip()
        if not task:
            self._show_error("任务名称不能为空")
            return
        if not _is_valid_task_name(task):
            self._show_error("任务名称不能为 . 或 ..，也不能包含 /、\\ 或空字符")
            return

        if self.task_combo.findText(task) < 0:
            self.task_combo.addItem(task)
        self.task_combo.setCurrentText(task)
        self._set_status(f"已新增任务: {task}")
        self._update_next_path()

    def _set_config_controls_enabled(self, enabled: bool) -> None:
        self.task_combo.setEnabled(enabled)
        self.new_task_btn.setEnabled(enabled)
        self.instruction_edit.setEnabled(enabled)
        self.metadata_edit.setEnabled(enabled)

    def _set_quality_controls_enabled(self, enabled: bool) -> None:
        self.high_quality_btn.setEnabled(enabled)
        self.low_quality_btn.setEnabled(enabled)
        self.failure_btn.setEnabled(enabled)

    def _parse_user_metadata(self) -> Optional[Dict[str, object]]:
        text = self.metadata_edit.toPlainText().strip()
        if not text:
            return {}
        if not text.startswith("{"):
            return {"note": text}
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            self._show_error(f"附加 metadata JSON 格式错误: {exc}")
            return None
        if not isinstance(value, dict):
            self._show_error("附加 metadata 如果使用 JSON，顶层必须是 object。")
            return None
        return value

    def _load_form_state(self) -> None:
        self._loading_form_state = True
        try:
            try:
                payload = json.loads(self._form_state_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                return
            if not isinstance(payload, dict):
                return
            task = str(payload.get("task_name", "") or "").strip()
            instruction = str(payload.get("text_instruction", "") or "")
            metadata_text = str(payload.get("metadata_text", "") or "")
            if task:
                self.task_combo.setCurrentText(task)
            if instruction:
                self.instruction_edit.setPlainText(instruction)
            if metadata_text:
                self.metadata_edit.setPlainText(metadata_text)
        finally:
            self._loading_form_state = False
            self._update_next_path()

    def _save_form_state(self, *_) -> None:
        if self._loading_form_state:
            return
        payload = {
            "schema_version": 1,
            "profile_key": self.profile_key,
            "task_name": self.task_combo.currentText().strip(),
            "text_instruction": self.instruction_edit.toPlainText(),
            "metadata_text": self.metadata_edit.toPlainText(),
            "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        }
        try:
            self._form_state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = self._form_state_path.with_name(
                f".{self._form_state_path.name}.{os.getpid()}.tmp"
            )
            tmp_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            tmp_path.replace(self._form_state_path)
        except OSError:
            return

    def _update_next_path(self) -> None:
        task = self.task_combo.currentText().strip()
        if not task or not _is_valid_task_name(task):
            full_path = (
                f"{self._fixed_output_root}/<task>/"
                f"{HIGH_QUALITY_DIR}|{LOW_QUALITY_DIR}|{FAILURE_DIR}/<index>"
            )
            self.next_path_label.setText(
                f"{self._fixed_output_root}\n"
                f"<task>/{HIGH_QUALITY_DIR}/<index>\n"
                f"<task>/{LOW_QUALITY_DIR}/<index>\n"
                f"<task>/{FAILURE_DIR}/<index>"
            )
            self.next_path_label.setToolTip(full_path)
            return
        indices = {
            HIGH_QUALITY_DIR: self.controller.peek_next_episode_index(task, HIGH_QUALITY_DIR),
            LOW_QUALITY_DIR: self.controller.peek_next_episode_index(task, LOW_QUALITY_DIR),
            FAILURE_DIR: self.controller.peek_next_episode_index(task, FAILURE_DIR),
        }
        self._set_next_path_label(task, indices)
        if (
            self._next_indices_future is None
            or self._next_indices_future.done()
            or self._next_indices_task != task
        ):
            self._next_indices_task = task
            self._next_indices_future = self._index_executor.submit(
                self.controller.refresh_next_episode_indices,
                task,
            )
            self._next_indices_future.add_done_callback(
                lambda future, task=task: self._next_indices_future_done(future, task)
            )

    def _next_indices_future_done(self, future: Future, task: str) -> None:
        if self._closing or future.cancelled():
            return
        try:
            indices = future.result()
        except Exception:
            return
        self._next_indices_loaded.emit(task, dict(indices))

    def _apply_next_indices(self, task: str, indices: dict) -> None:
        if self._closing:
            return
        if task != self.task_combo.currentText().strip():
            return
        self._set_next_path_label(task, indices)

    def _set_next_path_label(self, task: str, indices: dict) -> None:
        high_index = int(indices.get(HIGH_QUALITY_DIR, 0))
        low_index = int(indices.get(LOW_QUALITY_DIR, 0))
        failure_index = int(indices.get(FAILURE_DIR, 0))
        full_path = (
            f"{self._fixed_output_root}/{task}/{HIGH_QUALITY_DIR}/{high_index}\n"
            f"{self._fixed_output_root}/{task}/{LOW_QUALITY_DIR}/{low_index}\n"
            f"{self._fixed_output_root}/{task}/{FAILURE_DIR}/{failure_index}"
        )
        self.next_path_label.setText(
            f"{self._fixed_output_root}\n"
            f"{task}/{HIGH_QUALITY_DIR}/{high_index}\n"
            f"{task}/{LOW_QUALITY_DIR}/{low_index}\n"
            f"{task}/{FAILURE_DIR}/{failure_index}"
        )
        self.next_path_label.setToolTip(full_path)

    def _refresh_disk(self) -> None:
        if self._closing:
            return
        if self._episode_state in {"recording", "paused", "quality_pending", "saving"}:
            return
        if self._disk_future is not None and not self._disk_future.done():
            return
        self._disk_future = self._disk_executor.submit(self.controller.disk_usage)
        self._disk_future.add_done_callback(self._disk_future_done)

    def _disk_future_done(self, future: Future) -> None:
        if self._closing or future.cancelled():
            return
        try:
            usage = future.result()
        except Exception:
            return
        self._disk_loaded.emit(usage)

    def _apply_disk_usage(self, usage) -> None:
        if self._closing:
            return
        used = usage.total - usage.free
        percent = int(used / usage.total * 100) if usage.total else 0
        self.disk_bar.setValue(percent)
        self.disk_label.setText(f"{_format_bytes(usage.free)} free / {_format_bytes(usage.total)}")


class ReplayOptionsDialog(QtWidgets.QDialog):
    def __init__(self, default_root: Path, parent=None) -> None:
        super().__init__(parent)
        self.default_root = default_root
        self.setWindowTitle("Replay")
        self.resize(620, 430)

        layout = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QFormLayout()

        path_row = QtWidgets.QHBoxLayout()
        self.path_edit = QtWidgets.QLineEdit()
        self.path_edit.setText(str(default_root))
        self.path_edit.setPlaceholderText("选择 episode 目录、metadata.json 或 .pkl.gz")
        dir_btn = QtWidgets.QPushButton("目录")
        file_btn = QtWidgets.QPushButton("文件")
        dir_btn.clicked.connect(self._browse_directory)
        file_btn.clicked.connect(self._browse_file)
        path_row.addWidget(self.path_edit, 1)
        path_row.addWidget(dir_btn)
        path_row.addWidget(file_btn)
        form.addRow("Replay 路径", path_row)

        self.latest_check = QtWidgets.QCheckBox("从任务/质量目录选择最新 episode")
        form.addRow("Latest", self.latest_check)

        self.run_mode_combo = QtWidgets.QComboBox()
        self.run_mode_combo.addItem("硬件 dry-run：检查起点/自动靠近可行性，不发送轨迹", "dry_run")
        self.run_mode_combo.addItem("只检查文件：不连接 robot node", "file_check")
        self.run_mode_combo.addItem("执行 replay：发送轨迹命令", "execute")
        form.addRow("运行模式", self.run_mode_combo)

        self.speed_spin = _double_spin(1.0, 0.05, 5.0, 2)
        form.addRow("轨迹速度", self.speed_spin)

        self.approach_check = QtWidgets.QCheckBox("起点偏差允许时自动慢速靠近 frame 0")
        self.approach_check.setChecked(True)
        form.addRow("Approach", self.approach_check)
        self.approach_max_delta_spin = _double_spin(0.75, 0.01, 3.0, 3)
        self.approach_step_delta_spin = _double_spin(0.02, 0.001, 0.5, 3)
        self.approach_hz_spin = _double_spin(5.0, 0.1, 60.0, 1)
        form.addRow("靠近 max delta", self.approach_max_delta_spin)
        form.addRow("靠近 step delta", self.approach_step_delta_spin)
        form.addRow("靠近 Hz", self.approach_hz_spin)

        self.gripper_mode_combo = QtWidgets.QComboBox()
        self.gripper_mode_combo.addItem("event", "event")
        self.gripper_mode_combo.addItem("continuous", "continuous")
        form.addRow("夹爪 replay", self.gripper_mode_combo)
        self.gripper_speed_spin = _double_spin(0.1, 0.001, 1.0, 3)
        self.gripper_force_spin = _double_spin(10.0, 0.1, 100.0, 1)
        self.gripper_event_delta_spin = _double_spin(0.01, 0.0001, 0.08, 4)
        self.gripper_command_hz_spin = _double_spin(15.0, 0.1, 60.0, 1)
        self.gripper_hold_spin = _double_spin(2.0, 0.0, 10.0, 2)
        form.addRow("夹爪 speed", self.gripper_speed_spin)
        form.addRow("夹爪 force", self.gripper_force_spin)
        form.addRow("夹爪事件阈值", self.gripper_event_delta_spin)
        form.addRow("夹爪 command Hz", self.gripper_command_hz_spin)
        form.addRow("夹爪事件后暂停", self.gripper_hold_spin)

        layout.addLayout(form)
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok
            | QtWidgets.QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def accept(self) -> None:
        if not self.path_edit.text().strip():
            QtWidgets.QMessageBox.warning(self, "路径为空", "请选择 replay 路径。")
            return
        super().accept()

    def options(self) -> ReplayLaunchOptions:
        return ReplayLaunchOptions(
            path=self.path_edit.text().strip(),
            latest=self.latest_check.isChecked(),
            run_mode=str(self.run_mode_combo.currentData()),
            speed=float(self.speed_spin.value()),
            gripper_speed=float(self.gripper_speed_spin.value()),
            gripper_force=float(self.gripper_force_spin.value()),
            gripper_event_delta=float(self.gripper_event_delta_spin.value()),
            gripper_replay_mode=str(self.gripper_mode_combo.currentData()),
            gripper_command_hz=float(self.gripper_command_hz_spin.value()),
            gripper_hold_sec=float(self.gripper_hold_spin.value()),
            approach_start=self.approach_check.isChecked(),
            approach_start_max_delta=float(self.approach_max_delta_spin.value()),
            approach_start_step_delta=float(self.approach_step_delta_spin.value()),
            approach_start_hz=float(self.approach_hz_spin.value()),
        )

    def _browse_directory(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "选择 replay episode 目录",
            self.path_edit.text().strip() or str(self.default_root),
        )
        if directory:
            self.path_edit.setText(directory)

    def _browse_file(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "选择 replay 文件",
            self.path_edit.text().strip() or str(self.default_root),
            "Replay input (*.pkl.gz *.json *.txt);;All files (*)",
        )
        if path:
            self.path_edit.setText(path)


def _rgb_to_qimage(rgb: np.ndarray) -> QtGui.QImage:
    contiguous = np.ascontiguousarray(rgb)
    height, width, channels = contiguous.shape
    bytes_per_line = channels * width
    return QtGui.QImage(
        contiguous.data,
        width,
        height,
        bytes_per_line,
        QtGui.QImage.Format.Format_RGB888,
    ).copy()


def _format_bytes(value: int) -> str:
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(value)
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            return f"{size:.1f}{unit}"
        size /= 1024.0
    return f"{size:.1f}TB"


def _format_float(value: float) -> str:
    return f"{float(value):g}"


def _double_spin(
    value: float,
    minimum: float,
    maximum: float,
    decimals: int,
) -> QtWidgets.QDoubleSpinBox:
    spin = QtWidgets.QDoubleSpinBox()
    spin.setRange(float(minimum), float(maximum))
    spin.setDecimals(decimals)
    spin.setSingleStep(10 ** (-max(0, decimals)))
    spin.setValue(float(value))
    return spin


def _shell_join(args: list[str]) -> str:
    return " ".join(_shell_quote(arg) for arg in args)


def _shell_quote(value: str) -> str:
    if value == "":
        return "''"
    safe = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_+-=.,/:@%")
    if all(ch in safe for ch in value):
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"


def _read_gui_password() -> Optional[str]:
    password = os.environ.get("FRANKA_GUI_SUDO_PASSWORD")
    if password:
        return password
    password_file = os.environ.get("FRANKA_GUI_SUDO_PASSWORD_FILE")
    if not password_file:
        return None
    path = Path(password_file).expanduser()
    try:
        return path.read_text(encoding="utf-8").strip() or None
    except OSError:
        return None


def _mode_label(mode: str) -> str:
    if mode == "dual":
        return "双臂"
    if mode == "right":
        return "右臂"
    return "左臂"


def _display_output_path(output_dir: str, output_root: str) -> str:
    path = Path(output_dir).expanduser()
    root = Path(output_root).expanduser()
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return str(path)


def _is_text_input_widget(widget: Optional[QtWidgets.QWidget]) -> bool:
    while widget is not None:
        if isinstance(
            widget,
            (
                QtWidgets.QLineEdit,
                QtWidgets.QPlainTextEdit,
                QtWidgets.QTextEdit,
                QtWidgets.QSpinBox,
                QtWidgets.QDoubleSpinBox,
            ),
        ):
            return True
        if isinstance(widget, QtWidgets.QComboBox) and widget.isEditable():
            return True
        widget = widget.parentWidget()
    return False


def _is_valid_task_name(task: str) -> bool:
    token = task.strip()
    return bool(token) and token not in {".", ".."} and "/" not in token and "\\" not in token and "\x00" not in token


def _shutdown_executor(executor: ThreadPoolExecutor) -> None:
    try:
        executor.shutdown(wait=False, cancel_futures=True)
    except TypeError:
        executor.shutdown(wait=False)


def _safe_profile_key(value: str) -> str:
    key = value.strip() or "default"
    safe = "".join(ch if ch.isalnum() or ch in {"-", "_", "."} else "_" for ch in key)
    return safe or "default"


def _form_state_path(profile_key: str) -> Path:
    override = os.environ.get("FRANKA_GUI_LAST_INPUTS_PATH")
    if override:
        return Path(override).expanduser()
    root = os.environ.get("XDG_STATE_HOME")
    state_root = Path(root).expanduser() if root else Path.home() / ".local" / "state"
    return state_root / "frankateleop" / "franka_gui" / f"last_inputs_{profile_key}.json"
