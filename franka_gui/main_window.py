"""Main PyQt window for FR3 capture."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from PyQt6 import QtCore, QtGui, QtWidgets

from .capture_controller import CaptureController
from .process_manager import ProcessManager


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
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self.title = QtWidgets.QLabel(name)
        self.title.setObjectName("CameraTitle")
        self.label = QtWidgets.QLabel("等待画面")
        self.label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label.setMinimumSize(240, 180)
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
    def __init__(
        self,
        controller: CaptureController,
        process_manager: ProcessManager,
        repo_root: Path,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.process_manager = process_manager
        self.repo_root = repo_root
        self.camera_views: Dict[str, CameraView] = {}
        self.saved_count = 0
        self._episode_state = "idle"
        self._fixed_output_root = str(Path.home() / "Desktop" / "franka_record_data")
        self.controller.options.output_root = self._fixed_output_root

        self.setWindowTitle(
            "Franka Dual-Arm Capture"
            if self.controller.options.mode == "dual"
            else "Franka Single-Arm Capture"
        )
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.resize(1380, 840)
        self._build_ui()
        self._connect_signals()
        self._refresh_tasks()
        self._refresh_disk()

        self.disk_timer = QtCore.QTimer(self)
        self.disk_timer.timeout.connect(self._refresh_disk)
        self.disk_timer.start(5000)

        QtCore.QTimer.singleShot(250, self.controller.start_preview)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self._episode_state in {"recording", "paused"}:
            QtWidgets.QMessageBox.warning(
                self,
                "仍在采集中",
                "当前 episode 还没有保存或丢弃。请先按 e 保存，或按 d 丢弃。",
            )
            event.ignore()
            return
        pending_saves = self.controller.saver.pending_count()
        if pending_saves > 0:
            QtWidgets.QMessageBox.warning(
                self,
                "仍在后台保存",
                f"还有 {pending_saves} 个 episode 正在后台保存。请等待保存完成后再关闭。",
            )
            event.ignore()
            return
        self.controller.shutdown()
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
        if key == QtCore.Qt.Key.Key_S:
            self._start_recording()
        elif key == QtCore.Qt.Key.Key_W:
            self.controller.pause()
        elif key == QtCore.Qt.Key.Key_E:
            self.controller.finish()
        elif key == QtCore.Qt.Key.Key_D:
            self.controller.discard()
        elif key == QtCore.Qt.Key.Key_K:
            self.controller.add_keyframe()
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
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

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
        panel.setFixedWidth(210)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        app_title = QtWidgets.QLabel(
            "FR3 Dual Capture"
            if self.controller.options.mode == "dual"
            else "FR3 Single Capture"
        )
        app_title.setObjectName("AppTitle")
        layout.addWidget(app_title)
        layout.addWidget(AccentBar("Blue"))

        title = QtWidgets.QLabel("控制中心")
        title.setObjectName("SectionTitle")
        layout.addWidget(title)

        self.start_stack_btn = QtWidgets.QPushButton(
            "启动双臂 1-4"
            if self.process_manager.mode == "dual"
            else "启动 1-4"
        )
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

        layout.addSpacing(10)
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
        layout.addWidget(self.record_btn)
        layout.addWidget(self.pause_btn)
        layout.addWidget(self.end_btn)
        layout.addWidget(self.discard_btn)
        layout.addWidget(self.keyframe_btn)

        layout.addSpacing(10)
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
        panel.setFixedWidth(440)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        title = QtWidgets.QLabel("任务总览")
        title.setObjectName("SectionTitle")
        layout.addWidget(title)
        layout.addWidget(AccentBar("Green"))

        self.task_combo = QtWidgets.QComboBox()
        self.task_combo.setEditable(True)
        self.task_combo.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
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
        self.next_path_label.setMinimumHeight(52)
        self.next_path_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.next_path_label.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Minimum,
        )

        form = QtWidgets.QFormLayout()
        form.addRow("任务名称", self.task_combo)
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
        self.metadata_edit.setMaximumHeight(82)
        layout.addWidget(metadata_title)
        layout.addWidget(self.metadata_edit)

        self.stack_state = QtWidgets.QLabel("机器人栈: 未启动")
        self.capture_state = QtWidgets.QLabel("采集状态: 等待相机")
        self.active_episode = QtWidgets.QLabel("当前 episode: -")
        self.frame_count = QtWidgets.QLabel("当前帧数: 0")
        self.saved_episodes = QtWidgets.QLabel("已保存: 0")
        self.save_queue = QtWidgets.QLabel("保存队列: 0")
        for label in (
            self.stack_state,
            self.capture_state,
            self.active_episode,
            self.frame_count,
            self.saved_episodes,
            self.save_queue,
        ):
            layout.addWidget(label)

        layout.addSpacing(16)
        self.status_box = QtWidgets.QLabel("READY")
        self.status_box.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_box.setMinimumHeight(96)
        self.status_box.setStyleSheet("background: #ecfdf5; color: #166534; border-radius: 8px; font-size: 26px; font-weight: 800;")
        layout.addWidget(self.status_box)

        hint = QtWidgets.QLabel("快捷键:\ns 开始/继续, w 暂停, e 保存\n"
                                "d 丢弃, k 关键帧, q 保存当前")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        layout.addStretch(1)
        return panel

    def _build_camera_panel(self) -> QtWidgets.QFrame:
        panel = QtWidgets.QFrame()
        panel.setObjectName("CameraPanel")
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        title = QtWidgets.QLabel("相机视角")
        title.setObjectName("SectionTitle")
        layout.addWidget(title)
        layout.addWidget(AccentBar("Purple"))
        self.camera_grid_widget = QtWidgets.QWidget()
        self.camera_grid = QtWidgets.QGridLayout(self.camera_grid_widget)
        self.camera_grid.setContentsMargins(0, 0, 0, 0)
        self.camera_grid.setSpacing(10)
        self.camera_grid.setColumnStretch(0, 1)
        self.camera_grid.setColumnStretch(1, 1)
        self.camera_grid.setRowStretch(0, 1)
        self.camera_grid.setRowStretch(1, 1)
        self.camera_grid_widget.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        layout.addWidget(self.camera_grid_widget, 1)
        return panel

    def _connect_signals(self) -> None:
        self.start_stack_btn.clicked.connect(self.process_manager.start_stack)
        self.stop_stack_btn.clicked.connect(self.process_manager.stop_stack)
        self.stop_env_btn.clicked.connect(self.process_manager.stop_teleop_env)
        self.preview_btn.clicked.connect(self._restart_preview)
        self.log_btn.clicked.connect(self._show_logs)

        self.record_btn.clicked.connect(self._start_recording)
        self.pause_btn.clicked.connect(self.controller.pause)
        self.end_btn.clicked.connect(self.controller.finish)
        self.discard_btn.clicked.connect(self.controller.discard)
        self.keyframe_btn.clicked.connect(self.controller.add_keyframe)
        self.new_task_btn.clicked.connect(self._create_task)
        self.task_combo.currentTextChanged.connect(self._update_next_path)

        self.controller.preview_frame.connect(self._update_preview)
        self.controller.status_changed.connect(self._set_status)
        self.controller.error.connect(self._show_error)
        self.controller.cameras_ready.connect(self._setup_camera_views)
        self.controller.active_episode_changed.connect(self._active_episode_changed)
        self.controller.recording_frame_count.connect(lambda count: self.frame_count.setText(f"当前帧数: {count}"))
        self.controller.save_queue_changed.connect(lambda count: self.save_queue.setText(f"保存队列: {count}"))
        self.controller.episode_saved.connect(self._episode_saved)

        self.process_manager.status_changed.connect(self._set_status)
        self.process_manager.error.connect(self._show_error)
        self.process_manager.stack_state_changed.connect(self._stack_state_changed)

    def _start_recording(self) -> None:
        task = self.task_combo.currentText().strip()
        if not task:
            self._show_error("任务名称不能为空")
            return
        if not _is_valid_task_name(task):
            self._show_error("任务名称不能包含 / 或空字符")
            return
        user_metadata = self._parse_user_metadata()
        if user_metadata is None:
            return
        self.controller.options.output_root = self._fixed_output_root
        self.controller.start_or_resume(task, "", user_metadata=user_metadata)

    def _restart_preview(self) -> None:
        if self._episode_state in {"recording", "paused"}:
            self._show_error("录制或暂停中不能重启预览，请先保存或丢弃当前 episode。")
            return
        self.controller.stop_preview()
        self.controller.start_preview()

    def _setup_camera_views(self, names) -> None:
        for view in self.camera_views.values():
            view.setParent(None)
        while self.camera_grid.count():
            item = self.camera_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().setParent(None)
        self.camera_views.clear()
        for name in names:
            view = CameraView(name)
            self.camera_views[name] = view
        self._place_camera_views(names)
        self.capture_state.setText(f"采集状态: 相机已连接 {list(names)}")

    def _place_camera_views(self, names) -> None:
        for col in range(2):
            self.camera_grid.setColumnStretch(col, 1)
        for row in range(8):
            self.camera_grid.setRowStretch(row, 0)

        preferred_positions = {
            "left_wrist": (0, 0, 1, 1),
            "left": (0, 1, 1, 1),
            "middle": (1, 0, 1, 2),
            "right": (2, 0, 1, 1),
            "right_wrist": (2, 1, 1, 1),
        }
        placed = set()
        for name in ("left_wrist", "left", "middle", "right", "right_wrist"):
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
            col = idx % 2
            self.camera_grid.addWidget(view, row, col)
            self.camera_grid.setRowStretch(row, 1)

    def _update_preview(self, frames: Dict[str, np.ndarray]) -> None:
        for name, rgb in frames.items():
            view = self.camera_views.get(name)
            if view is not None:
                view.update_image(rgb)

    def _active_episode_changed(self, task: str, index: int, state: str) -> None:
        self._episode_state = state
        if state == "recording":
            self._set_config_controls_enabled(False)
            self.status_box.setText("REC")
            self.status_box.setStyleSheet("background: #fff7ed; color: #9a3412; border-radius: 8px; font-size: 26px; font-weight: 800;")
        elif state == "paused":
            self._set_config_controls_enabled(False)
            self.status_box.setText("PAUSE")
            self.status_box.setStyleSheet("background: #fefce8; color: #854d0e; border-radius: 8px; font-size: 26px; font-weight: 800;")
        elif state == "saving":
            self._set_config_controls_enabled(True)
            self.status_box.setText("SAVING")
            self.status_box.setStyleSheet("background: #fef9c3; color: #713f12; border-radius: 8px; font-size: 26px; font-weight: 800;")
        else:
            self._set_config_controls_enabled(True)
            self.status_box.setText("READY")
            self.status_box.setStyleSheet("background: #ecfdf5; color: #166534; border-radius: 8px; font-size: 26px; font-weight: 800;")
        self.active_episode.setText(f"当前 episode: {task}/{index} ({state})")

    def _episode_saved(self, task: str, index: int, output_dir: str, frames: int) -> None:
        self.saved_count += 1
        self.saved_episodes.setText(f"已保存: {self.saved_count}")
        self.active_episode.setText(f"最近保存: {task}/{index}, frames={frames}")
        self._refresh_tasks()
        self._refresh_disk()
        self._update_next_path()

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

    def _refresh_tasks(self) -> None:
        current = self.task_combo.currentText().strip()
        self.task_combo.blockSignals(True)
        self.task_combo.clear()
        self.task_combo.addItems(self.controller.scan_tasks())
        if current:
            self.task_combo.setCurrentText(current)
        elif self.task_combo.count() == 0:
            self.task_combo.setCurrentText("default")
        self.task_combo.blockSignals(False)
        self._update_next_path()

    def _create_task(self) -> None:
        if self._episode_state in {"recording", "paused"}:
            self._show_error("录制或暂停中不能新增任务，请先保存或丢弃当前 episode。")
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
            self._show_error("任务名称不能包含 / 或空字符")
            return

        self.controller.options.output_root = self._fixed_output_root
        task_dir = Path(self._fixed_output_root).expanduser() / task
        try:
            task_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self._show_error(f"创建任务目录失败: {task_dir}\n{exc}")
            return

        self._refresh_tasks()
        self.task_combo.setCurrentText(task)
        self._set_status(f"已新增任务: {task}")
        self._update_next_path()

    def _set_config_controls_enabled(self, enabled: bool) -> None:
        self.task_combo.setEnabled(enabled)
        self.new_task_btn.setEnabled(enabled)
        self.metadata_edit.setEnabled(enabled)

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

    def _update_next_path(self) -> None:
        task = self.task_combo.currentText().strip()
        if not task or not _is_valid_task_name(task):
            full_path = f"{self._fixed_output_root}/<task>/<index>"
            self.next_path_label.setText(f"{self._fixed_output_root}\n<task>/<index>")
            self.next_path_label.setToolTip(full_path)
            return
        index = self.controller.peek_next_episode_index(task)
        full_path = f"{self._fixed_output_root}/{task}/{index}"
        self.next_path_label.setText(f"{self._fixed_output_root}\n{task}/{index}")
        self.next_path_label.setToolTip(full_path)

    def _refresh_disk(self) -> None:
        try:
            usage = self.controller.disk_usage()
        except Exception:
            return
        used = usage.total - usage.free
        percent = int(used / usage.total * 100) if usage.total else 0
        self.disk_bar.setValue(percent)
        self.disk_label.setText(f"{_format_bytes(usage.free)} free / {_format_bytes(usage.total)}")


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
    return bool(task) and "/" not in task and "\x00" not in task
