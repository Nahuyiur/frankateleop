"""Unified launcher for Franka capture, worktime monitoring, and data review."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

from PyQt6 import QtCore, QtGui, QtWidgets

from data_review.window import DataReviewWindow
from franka_sync import ensure_sync_daemon
from franka_capture.config.fr3_dual import DEFAULT_LEFT_ROBOT, DEFAULT_RIGHT_ROBOT
from franka_capture.config.fr3_single import DEFAULT_ROBOT
from work_monitor.dashboard import WorktimeDashboard
from work_monitor.ledger import WorkLedger
from work_monitor.tracker import WorktimeTracker

from .capture_controller import CaptureController, CaptureOptions, FIXED_CAPTURE_FPS
from .main_window import MainWindow
from .process_manager import ProcessManager
from .storage_paths import record_cache_root


MODE_PROFILES = {
    "single": {
        "letter": "A",
        "title": "左臂采集",
        "profile_key": "left",
        "camera_names": ["left_wrist", "left", "middle"],
    },
    "right": {
        "letter": "B",
        "title": "右臂采集",
        "profile_key": "right",
        "camera_names": ["middle", "right", "right_wrist"],
    },
    "dual": {
        "letter": "C",
        "title": "双臂采集",
        "profile_key": "dual",
        "camera_names": None,
    },
}
HUB_STATE_PATH = Path("~/.local/state/frankateleop/hub.json").expanduser()
MUKA_LOGO_PATH = Path(__file__).resolve().parent / "assets" / "muka_logo.png"


HUB_STYLESHEET = """
QMainWindow {
    background: #f5f7f9;
}
QWidget {
    color: #12181f;
    font-family: "Noto Sans CJK SC", "Noto Sans SC", "Microsoft YaHei", sans-serif;
    font-size: 14px;
}
QFrame#HeaderBar {
    background: #2b659a;
    border: 0;
}
QLabel#BrandLogo {
    background: transparent;
    border: 0;
}
QLabel#BrandLogo[fallback="true"] {
    background: #0c015e;
    color: white;
    border-radius: 7px;
    font-size: 20px;
    font-weight: 800;
}
QLabel#HeaderTitle {
    color: white;
    font-size: 25px;
    font-weight: 750;
}
QLabel#HeaderSubtitle {
    color: #cce8e9;
    font-size: 13px;
}
QLabel#SystemStatus {
    background: #255985;
    color: white;
    border: 1px solid #80cde1;
    border-radius: 7px;
    padding: 7px 11px;
    font-size: 12px;
    font-weight: 650;
}
QFrame#IdentityBar {
    background: white;
    border: 1px solid #dfe4e8;
    border-radius: 8px;
}
QLabel#SectionKicker {
    color: #2b659a;
    font-size: 11px;
    font-weight: 700;
}
QLabel#SectionTitle {
    color: #12181f;
    font-size: 19px;
    font-weight: 750;
}
QLabel#SectionHint {
    color: #596774;
    font-size: 13px;
}
QLineEdit#OperatorEdit {
    min-height: 38px;
    background: #f8fafb;
    border: 1px solid #cfd7de;
    border-radius: 7px;
    padding: 0 12px;
    selection-background-color: #2b659a;
}
QLineEdit#OperatorEdit:focus {
    background: white;
    border: 2px solid #2b659a;
    padding: 0 11px;
}
QPushButton#ConfirmButton {
    min-height: 40px;
    background: #1324a2;
    color: white;
    border: 0;
    border-radius: 7px;
    padding: 0 18px;
    font-weight: 700;
}
QPushButton#ConfirmButton:hover {
    background: #0c015e;
}
QPushButton#ConfirmButton:pressed {
    background: #441c9b;
}
QLabel#IdentityStatus[confirmed="false"] {
    color: #a04d16;
    background: #fff5e9;
    border: 1px solid #f1d2ad;
    border-radius: 7px;
    padding: 8px 11px;
    font-weight: 700;
}
QLabel#IdentityStatus[confirmed="true"] {
    color: #176247;
    background: #ebf7f1;
    border: 1px solid #b9dfcf;
    border-radius: 7px;
    padding: 8px 11px;
    font-weight: 700;
}
QPushButton#ModeButton {
    background: white;
    border: 1px solid #dfe4e8;
    border-radius: 8px;
    text-align: left;
}
QPushButton#ModeButton:hover {
    background: #fbfcfd;
    border: 2px solid #9eabb6;
}
QPushButton#ModeButton:pressed {
    background: #f2f5f7;
}
QPushButton#ModeButton:focus {
    border: 2px solid #1324a2;
}
QPushButton#ModeButton[preset="true"] {
    border: 2px solid #1324a2;
}
QPushButton#ModeButton:disabled {
    background: #eef1f3;
    border: 1px solid #dde2e6;
}
QLabel#ModeLetter {
    color: white;
    border-radius: 7px;
    font-size: 15px;
    font-weight: 800;
}
QLabel#ModeLetter[mode="single"] { background: #2b659a; }
QLabel#ModeLetter[mode="right"] { background: #f87512; color: #12181f; }
QLabel#ModeLetter[mode="dual"] { background: #603daf; }
QLabel#ModeTitle {
    color: #12181f;
    font-size: 18px;
    font-weight: 750;
}
QLabel#ModeCamera {
    color: #6b7783;
    font-size: 12px;
}
QLabel#ModeAction {
    color: #485561;
    font-size: 12px;
    font-weight: 700;
}
QFrame#ToolBar {
    background: white;
    border: 1px solid #dfe4e8;
    border-radius: 8px;
}
QToolButton#UtilityButton {
    min-height: 42px;
    background: #f7f9fa;
    color: #26323d;
    border: 1px solid #d8dee3;
    border-radius: 7px;
    padding: 0 14px;
    font-weight: 650;
}
QToolButton#UtilityButton:hover {
    background: white;
    color: #1324a2;
    border: 1px solid #2b659a;
}
"""


class ModeButton(QtWidgets.QPushButton):
    """Stable, full-card mode control while retaining QPushButton semantics."""

    def __init__(
        self,
        mode: str,
        letter: str,
        title: str,
        camera_text: str,
        *,
        preset: bool = False,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("ModeButton")
        self.setProperty("preset", preset)
        self.setAccessibleName(title)
        self.setAccessibleDescription(camera_text)
        self.setToolTip(f"进入{title}")
        self.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        self.setMinimumHeight(166)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Fixed,
        )

        content = QtWidgets.QVBoxLayout(self)
        content.setContentsMargins(20, 18, 20, 17)
        content.setSpacing(0)

        top = QtWidgets.QHBoxLayout()
        top.setSpacing(12)
        badge = QtWidgets.QLabel(letter)
        badge.setObjectName("ModeLetter")
        badge.setProperty("mode", mode)
        badge.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        badge.setFixedSize(34, 34)
        badge.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("ModeTitle")
        title_label.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        top.addWidget(badge)
        top.addWidget(title_label)
        top.addStretch()
        content.addLayout(top)
        content.addSpacing(22)

        camera_label = QtWidgets.QLabel(camera_text)
        camera_label.setObjectName("ModeCamera")
        camera_label.setWordWrap(True)
        camera_label.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        content.addWidget(camera_label)
        content.addStretch()

        action = QtWidgets.QLabel("进入采集  →")
        action.setObjectName("ModeAction")
        action.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        content.addWidget(action)

        self._opacity = QtWidgets.QGraphicsOpacityEffect(self)
        self._opacity.setOpacity(1.0)
        self.setGraphicsEffect(self._opacity)

    def setEnabled(self, enabled: bool) -> None:
        super().setEnabled(enabled)
        if hasattr(self, "_opacity"):
            self._opacity.setOpacity(1.0 if enabled else 0.58)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset-mode", choices=("single", "right", "dual"), default="")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--storage-mode", choices=("direct-nas", "local-outbox"), default=os.environ.get("FRANKA_GUI_STORAGE_MODE", "direct-nas"))
    parser.add_argument("--host", default=DEFAULT_ROBOT.host)
    parser.add_argument("--port", type=int, default=DEFAULT_ROBOT.port)
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_ROBOT.timeout_ms)
    parser.add_argument("--left-host", default=DEFAULT_LEFT_ROBOT.host)
    parser.add_argument("--left-port", type=int, default=int(os.environ.get("BI_ARM_LEFT_ZMQ_PORT", DEFAULT_LEFT_ROBOT.port)))
    parser.add_argument("--right-host", default=os.environ.get("FRANKA_RIGHT_ZMQ_HOST", DEFAULT_RIGHT_ROBOT.host))
    parser.add_argument("--right-port", type=int, default=int(os.environ.get("FRANKA_RIGHT_ZMQ_PORT", DEFAULT_RIGHT_ROBOT.port)))
    parser.add_argument("--open-monitor", action="store_true")
    parser.add_argument("--open-review", action="store_true")
    return parser.parse_args(argv)


class HubWindow(QtWidgets.QMainWindow):
    launch_capture = QtCore.pyqtSignal(str, str)
    open_monitor = QtCore.pyqtSignal()
    open_review = QtCore.pyqtSignal()
    hub_closed = QtCore.pyqtSignal()

    def __init__(self, preset_mode: str = "", parent=None) -> None:
        super().__init__(parent)
        self.preset_mode = preset_mode
        self._confirmed_name = ""
        self.setWindowTitle("北京莫刻机器人 · 数据采集中心")
        self.resize(1080, 720)
        self.setMinimumSize(900, 640)
        self.setStyleSheet(HUB_STYLESHEET)
        self._build_ui()
        self._load_state()

    @property
    def confirmed_name(self) -> str:
        return self._confirmed_name

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        header = QtWidgets.QFrame()
        header.setObjectName("HeaderBar")
        header.setFixedHeight(104)
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(38, 22, 38, 22)
        header_layout.setSpacing(14)
        self.brand_logo = QtWidgets.QLabel()
        self.brand_logo.setObjectName("BrandLogo")
        self.brand_logo.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        logo = QtGui.QPixmap(str(MUKA_LOGO_PATH))
        if logo.isNull():
            self.brand_logo.setText("M")
            self.brand_logo.setProperty("fallback", True)
            self.brand_logo.setFixedSize(48, 48)
        else:
            self.brand_logo.setProperty("fallback", False)
            visible_bounds = QtGui.QRegion(logo.mask()).boundingRect()
            if visible_bounds.isValid() and not visible_bounds.isEmpty():
                logo = logo.copy(visible_bounds)
            self.brand_logo.setFixedSize(92, 48)
            self.brand_logo.setPixmap(
                logo.scaled(
                    self.brand_logo.size(),
                    QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                    QtCore.Qt.TransformationMode.SmoothTransformation,
                )
            )
        header_layout.addWidget(self.brand_logo)
        title_group = QtWidgets.QVBoxLayout()
        title_group.setSpacing(2)
        title = QtWidgets.QLabel("北京莫刻机器人")
        title.setObjectName("HeaderTitle")
        subtitle = QtWidgets.QLabel("MUKA ROBOTICS  ·  FRANKA 数据采集工作台")
        subtitle.setObjectName("HeaderSubtitle")
        title_group.addWidget(title)
        title_group.addWidget(subtitle)
        header_layout.addLayout(title_group)
        header_layout.addStretch()
        system_status = QtWidgets.QLabel("统一采集入口")
        system_status.setObjectName("SystemStatus")
        header_layout.addWidget(system_status)
        outer.addWidget(header)

        body = QtWidgets.QWidget()
        body_layout = QtWidgets.QVBoxLayout(body)
        body_layout.setContentsMargins(38, 28, 38, 30)
        body_layout.setSpacing(20)

        identity = QtWidgets.QFrame()
        identity.setObjectName("IdentityBar")
        identity_layout = QtWidgets.QHBoxLayout(identity)
        identity_layout.setContentsMargins(20, 16, 18, 16)
        identity_layout.setSpacing(12)
        identity_copy = QtWidgets.QVBoxLayout()
        identity_copy.setSpacing(2)
        identity_kicker = QtWidgets.QLabel("CURRENT OPERATOR")
        identity_kicker.setObjectName("SectionKicker")
        identity_title = QtWidgets.QLabel("数采员身份")
        identity_title.setObjectName("SectionTitle")
        identity_copy.addWidget(identity_kicker)
        identity_copy.addWidget(identity_title)
        identity_layout.addLayout(identity_copy)
        identity_layout.addSpacing(18)
        self.operator_edit = QtWidgets.QLineEdit()
        self.operator_edit.setObjectName("OperatorEdit")
        self.operator_edit.setPlaceholderText("输入真实姓名")
        self.operator_edit.setClearButtonEnabled(True)
        self.operator_edit.setMinimumWidth(230)
        self.operator_edit.textChanged.connect(self._identity_changed)
        self.confirm_btn = QtWidgets.QPushButton("确认身份")
        self.confirm_btn.setObjectName("ConfirmButton")
        self.confirm_btn.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        self.confirm_btn.clicked.connect(self._confirm_identity)
        self.identity_status = QtWidgets.QLabel("尚未确认")
        self.identity_status.setObjectName("IdentityStatus")
        self.identity_status.setProperty("confirmed", False)
        self.identity_status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.identity_status.setMinimumWidth(90)
        identity_layout.addWidget(self.operator_edit, 1)
        identity_layout.addWidget(self.confirm_btn)
        identity_layout.addWidget(self.identity_status)
        body_layout.addWidget(identity)

        mode_header = QtWidgets.QHBoxLayout()
        mode_heading = QtWidgets.QVBoxLayout()
        mode_heading.setSpacing(2)
        mode_kicker = QtWidgets.QLabel("CAPTURE PROFILE")
        mode_kicker.setObjectName("SectionKicker")
        mode_title = QtWidgets.QLabel("选择采集模式")
        mode_title.setObjectName("SectionTitle")
        mode_heading.addWidget(mode_kicker)
        mode_heading.addWidget(mode_title)
        mode_header.addLayout(mode_heading)
        mode_header.addStretch()
        self.mode_section_hint = QtWidgets.QLabel("请先确认数采员身份")
        self.mode_section_hint.setObjectName("SectionHint")
        mode_header.addWidget(self.mode_section_hint)
        body_layout.addLayout(mode_header)

        modes = QtWidgets.QHBoxLayout()
        modes.setSpacing(14)
        self.mode_buttons: dict[str, QtWidgets.QPushButton] = {}
        descriptions = {
            "single": ("A", "左臂采集", "3 路相机  ·  left_wrist / left / middle"),
            "right": ("B", "右臂采集", "3 路相机  ·  middle / right / right_wrist"),
            "dual": ("C", "双臂采集", "全部已配置相机  ·  双臂同步"),
        }
        for mode, (letter, mode_name, camera_text) in descriptions.items():
            button = ModeButton(
                mode,
                letter,
                mode_name,
                camera_text,
                preset=mode == self.preset_mode,
            )
            button.setEnabled(False)
            button.clicked.connect(lambda _checked=False, selected=mode: self._launch(selected))
            self.mode_buttons[mode] = button
            modes.addWidget(button)
        body_layout.addLayout(modes)

        body_layout.addStretch()
        tools = QtWidgets.QFrame()
        tools.setObjectName("ToolBar")
        tools_layout = QtWidgets.QHBoxLayout(tools)
        tools_layout.setContentsMargins(18, 13, 14, 13)
        tools_layout.setSpacing(10)
        tools_copy = QtWidgets.QVBoxLayout()
        tools_copy.setSpacing(1)
        tools_title = QtWidgets.QLabel("监督与复核")
        tools_title.setObjectName("SectionTitle")
        tools_hint = QtWidgets.QLabel("独立工具，不影响当前采集配置")
        tools_hint.setObjectName("SectionHint")
        tools_copy.addWidget(tools_title)
        tools_copy.addWidget(tools_hint)
        tools_layout.addLayout(tools_copy)
        tools_layout.addStretch()
        monitor_btn = QtWidgets.QToolButton()
        monitor_btn.setObjectName("UtilityButton")
        monitor_btn.setText("工时监控")
        monitor_btn.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        monitor_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_ComputerIcon))
        monitor_btn.setIconSize(QtCore.QSize(18, 18))
        monitor_btn.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        monitor_btn.clicked.connect(self.open_monitor)
        review_btn = QtWidgets.QToolButton()
        review_btn.setObjectName("UtilityButton")
        review_btn.setText("采集数据复核")
        review_btn.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        review_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_FileDialogContentsView))
        review_btn.setIconSize(QtCore.QSize(18, 18))
        review_btn.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        review_btn.clicked.connect(self.open_review)
        tools_layout.addWidget(monitor_btn)
        tools_layout.addWidget(review_btn)
        body_layout.addWidget(tools)
        outer.addWidget(body, 1)

    def _identity_changed(self) -> None:
        if self.operator_edit.text().strip() != self._confirmed_name:
            self._confirmed_name = ""
            self._set_identity_status(False)
            for button in self.mode_buttons.values():
                button.setEnabled(False)

    def _confirm_identity(self) -> None:
        name = " ".join(self.operator_edit.text().split())
        if not name:
            QtWidgets.QMessageBox.warning(self, "姓名不能为空", "请输入数采员真实姓名后再确认。")
            return
        self.operator_edit.setText(name)
        self._confirmed_name = name
        self._set_identity_status(True, name)
        for button in self.mode_buttons.values():
            button.setEnabled(True)
        self._save_state()
        if self.preset_mode:
            QtCore.QTimer.singleShot(0, lambda: self._launch(self.preset_mode))

    def _set_identity_status(self, confirmed: bool, _name: str = "") -> None:
        self.identity_status.setText("✓  已确认" if confirmed else "待确认")
        self.identity_status.setProperty("confirmed", confirmed)
        self.identity_status.style().unpolish(self.identity_status)
        self.identity_status.style().polish(self.identity_status)
        self.mode_section_hint.setText("选择一项开始采集" if confirmed else "请先确认数采员身份")

    def _launch(self, mode: str) -> None:
        if not self._confirmed_name:
            return
        self.launch_capture.emit(mode, self._confirmed_name)

    def _load_state(self) -> None:
        try:
            payload = json.loads(HUB_STATE_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        name = str(payload.get("operator_name", "") or "").strip()
        if name:
            self.operator_edit.setText(name)

    def _save_state(self) -> None:
        HUB_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        temporary = HUB_STATE_PATH.with_suffix(".tmp")
        temporary.write_text(json.dumps({"version": 1, "operator_name": self._confirmed_name}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(HUB_STATE_PATH)

    def closeEvent(self, event) -> None:
        self.hub_closed.emit()
        event.accept()


class HubApplication(QtCore.QObject):
    def __init__(self, app: QtWidgets.QApplication, args) -> None:
        super().__init__()
        self.app = app
        self.args = args
        self.repo_root = Path(__file__).resolve().parents[1]
        self.tracker = WorktimeTracker()
        self.ledger = self.tracker.ledger
        self.hub = HubWindow(args.preset_mode)
        self.capture_window: MainWindow | None = None
        self.monitor_windows: list[WorktimeDashboard] = []
        self.review_windows: list[DataReviewWindow] = []
        self.hub.launch_capture.connect(self.launch_capture)
        self.hub.open_monitor.connect(self.open_monitor)
        self.hub.open_review.connect(self.open_review)
        self.hub.hub_closed.connect(self._hub_closed)
        self.heartbeat_timer = QtCore.QTimer(self)
        self.heartbeat_timer.timeout.connect(self.tracker.heartbeat)
        self.heartbeat_timer.start(1000)
        app.aboutToQuit.connect(self._shutdown)
        if (
            not args.mock
            and args.storage_mode == "local-outbox"
            and os.environ.get("FRANKA_GUI_DISABLE_NAS_SYNC", "") != "1"
        ):
            try:
                ensure_sync_daemon(
                    self.repo_root,
                    record_cache_root(),
                    Path.home() / "Desktop" / "Muka_NAS",
                )
            except Exception as exc:
                print(
                    f"WARNING: could not start delayed NAS sync daemon: {exc}",
                    file=sys.stderr,
                )

    def show(self) -> None:
        self.hub.show()
        if self.args.open_monitor:
            self.open_monitor()
        if self.args.open_review:
            self.open_review()

    def launch_capture(self, mode: str, operator_name: str) -> None:
        if self.capture_window is not None:
            return
        profile = MODE_PROFILES[mode]
        robot_host = self.args.right_host if mode == "right" else self.args.host
        robot_port = self.args.right_port if mode == "right" else self.args.port
        options = CaptureOptions(
            output_root=str(Path.home() / "Desktop" / "Muka_NAS"),
            mode=mode,
            camera_names=profile["camera_names"],
            camera_fps=FIXED_CAPTURE_FPS,
            video_fps=FIXED_CAPTURE_FPS,
            robot_host=robot_host,
            robot_port=robot_port,
            robot_timeout_ms=self.args.timeout_ms,
            left_robot_host=self.args.left_host,
            left_robot_port=self.args.left_port,
            right_robot_host=self.args.right_host,
            right_robot_port=self.args.right_port,
            dual_robot_timeout_ms=self.args.timeout_ms,
            mock=self.args.mock,
            direct_to_output_root=self.args.storage_mode == "direct-nas",
            operator_name=operator_name,
            capture_profile=profile["letter"],
        )
        controller = CaptureController(options)
        controller.work_event.connect(self._record_work_event)
        process_manager = ProcessManager(self.repo_root, mode=mode)
        window = MainWindow(controller, process_manager, self.repo_root, profile_key=profile["profile_key"])
        window.closed.connect(self._capture_closed)
        self.capture_window = window
        self.hub.hide()
        window.show()

    def _record_work_event(self, event: dict) -> None:
        self.tracker.emit(
            str(event.get("event_type", "")),
            attempt_id=str(event.get("attempt_id", "")),
            occurred_at=float(event.get("occurred_at", 0.0) or 0.0),
            payload=dict(event.get("payload") or {}),
        )

    def _capture_closed(self, returning_home: bool) -> None:
        window = self.capture_window
        self.capture_window = None
        if window is not None:
            window.deleteLater()
        if returning_home:
            self.hub.show()
            self.hub.raise_()
        else:
            self.app.quit()

    def open_monitor(self) -> None:
        if self.ledger is None:
            try:
                ledger = WorkLedger()
                self.tracker.attach_ledger_and_recover(ledger)
                self.ledger = ledger
            except Exception as exc:
                QtWidgets.QMessageBox.warning(
                    self.hub,
                    "工时账本不可用",
                    f"SQLite 账本暂时不可写，采集不受影响，事件将保存在 emergency spool。\n{exc}",
                )
                return
        window = WorktimeDashboard(
            self.ledger,
            active_session_ids=(self.tracker.session_id,),
        )
        window.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
        window.destroyed.connect(lambda: self._drop_window(self.monitor_windows, window))
        self.monitor_windows.append(window)
        window.show()

    def open_review(self) -> None:
        window = DataReviewWindow()
        window.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
        window.destroyed.connect(lambda: self._drop_window(self.review_windows, window))
        self.review_windows.append(window)
        window.show()

    @staticmethod
    def _drop_window(windows: list, window) -> None:
        if window in windows:
            windows.remove(window)

    def _shutdown(self) -> None:
        self.heartbeat_timer.stop()
        self.tracker.close()

    def _hub_closed(self) -> None:
        if self.capture_window is None:
            self.app.quit()


def main(argv=None) -> int:
    args = parse_args(argv)
    app = QtWidgets.QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
    app.setApplicationName("Franka Data Collection Hub")
    app.setQuitOnLastWindowClosed(False)
    manager = HubApplication(app, args)
    manager.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
