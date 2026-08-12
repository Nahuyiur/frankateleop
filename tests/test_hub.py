from __future__ import annotations

import os
from pathlib import Path
import subprocess
import threading

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt6")
from PyQt6 import QtCore, QtWidgets

import franka_gui.hub as hub_module
from franka_gui.main_window import MainWindow
import franka_gui.process_manager as process_module
from franka_gui.process_manager import ManagedProcess, ProcessManager


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_operator_must_be_confirmed_and_editing_revokes_confirmation(
    app, tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setattr(hub_module, "HUB_STATE_PATH", tmp_path / "hub.json")
    window = hub_module.HubWindow()
    assert not any(button.isEnabled() for button in window.mode_buttons.values())

    launched = []
    window.launch_capture.connect(lambda mode, name: launched.append((mode, name)))
    window.operator_edit.setText("  Alice   Chen  ")
    window._confirm_identity()
    assert window.confirmed_name == "Alice Chen"
    assert all(button.isEnabled() for button in window.mode_buttons.values())
    window._launch("single")
    assert launched == [("single", "Alice Chen")]

    window.operator_edit.setText("Alice")
    assert window.confirmed_name == ""
    assert not any(button.isEnabled() for button in window.mode_buttons.values())
    window.close()


def test_saved_name_is_prefilled_but_not_implicitly_confirmed(
    app, tmp_path: Path, monkeypatch
) -> None:
    state_path = tmp_path / "hub.json"
    state_path.write_text('{"version": 1, "operator_name": "Bob"}', encoding="utf-8")
    monkeypatch.setattr(hub_module, "HUB_STATE_PATH", state_path)
    window = hub_module.HubWindow("right")
    assert window.operator_edit.text() == "Bob"
    assert window.confirmed_name == ""
    assert not window.mode_buttons["right"].isEnabled()
    window.close()


def test_profiles_keep_existing_camera_contract() -> None:
    assert hub_module.MODE_PROFILES["single"]["camera_names"] == [
        "left_wrist",
        "left",
        "middle",
    ]
    assert hub_module.MODE_PROFILES["right"]["camera_names"] == [
        "middle",
        "right",
        "right_wrist",
    ]
    assert hub_module.MODE_PROFILES["dual"]["camera_names"] is None


def test_mode_cards_expose_names_and_keyboard_focus(app, tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(hub_module, "HUB_STATE_PATH", tmp_path / "hub.json")
    window = hub_module.HubWindow()

    assert window.mode_buttons["single"].accessibleName() == "左臂采集"
    assert window.mode_buttons["right"].accessibleName() == "右臂采集"
    assert window.mode_buttons["dual"].accessibleName() == "双臂采集"
    assert all(button.focusPolicy() != QtCore.Qt.FocusPolicy.NoFocus for button in window.mode_buttons.values())

    window.close()


def test_hub_uses_real_company_logo(app, tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(hub_module, "HUB_STATE_PATH", tmp_path / "hub.json")
    window = hub_module.HubWindow()

    assert hub_module.MUKA_LOGO_PATH.is_file()
    assert window.brand_logo.property("fallback") is False
    assert window.brand_logo.pixmap() is not None
    assert not window.brand_logo.pixmap().isNull()
    assert window.brand_logo.pixmap().width() <= window.brand_logo.width()
    assert window.brand_logo.pixmap().height() <= window.brand_logo.height()
    assert window.brand_logo.pixmap().width() >= 90
    assert window.brand_logo.pixmap().width() > 2 * window.brand_logo.pixmap().height()

    window.close()


@pytest.mark.parametrize(
    ("mode", "names", "expected"),
    (
        (
            "single",
            ("left_wrist", "left", "middle"),
            {
                "left": (0, 0, 1, 3),
                "middle": (0, 3, 1, 3),
                "left_wrist": (1, 0, 1, 6),
            },
        ),
        (
            "right",
            ("middle", "right", "right_wrist"),
            {
                "middle": (0, 0, 1, 3),
                "right": (0, 3, 1, 3),
                "right_wrist": (1, 0, 1, 6),
            },
        ),
        (
            "dual",
            ("left_wrist", "left", "middle", "right", "right_wrist"),
            {
                "left": (0, 0, 1, 3),
                "right": (0, 3, 1, 3),
                "left_wrist": (1, 0, 1, 2),
                "middle": (1, 2, 1, 2),
                "right_wrist": (1, 4, 1, 2),
            },
        ),
    ),
)
def test_branding_changes_preserve_camera_layout_contract(
    mode: str,
    names: tuple[str, ...],
    expected: dict[str, tuple[int, int, int, int]],
) -> None:
    placements = {}

    class Grid:
        def setColumnStretch(self, *_args):
            pass

        def setRowStretch(self, *_args):
            pass

        def addWidget(self, view, *position):
            placements[view] = position

    fake_window = type(
        "FakeWindow",
        (),
        {
            "camera_grid": Grid(),
            "camera_views": {name: name for name in names},
            "controller": type(
                "FakeController",
                (),
                {"options": type("Options", (), {"mode": mode})()},
            )(),
        },
    )()

    MainWindow._place_camera_views(fake_window, names)

    assert placements == expected


def test_hub_close_is_an_explicit_application_exit_signal(
    app, tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setattr(hub_module, "HUB_STATE_PATH", tmp_path / "hub.json")
    window = hub_module.HubWindow()
    closed = []
    window.hub_closed.connect(lambda: closed.append(True))
    window.show()
    window.close()
    app.processEvents()
    assert closed == [True]


def test_process_start_cancellation_prevents_late_process_launch(tmp_path: Path) -> None:
    manager = ProcessManager(tmp_path, mode="single")
    entered = threading.Event()
    launched = []

    def wait_for_cancel():
        entered.set()
        assert manager._cancel_start.wait(timeout=2.0)

    manager._prepare_sudo = wait_for_cancel
    manager._cleanup_stale_pipeline_processes = lambda: launched.append("cleanup")
    manager._start_script = lambda *args, **kwargs: launched.append("process")
    manager.start_stack()
    assert entered.wait(timeout=2.0)
    assert manager.stop_stack_blocking()
    assert manager._worker is not None
    assert not manager._worker.is_alive()
    assert launched == []


def test_process_start_is_rejected_while_stop_worker_is_active(tmp_path: Path) -> None:
    manager = ProcessManager(tmp_path, mode="single")
    release = threading.Event()
    manager._stop_worker = threading.Thread(target=release.wait, daemon=True)
    manager._stop_worker.start()
    errors = []
    manager.error.connect(errors.append)
    manager.start_stack()
    assert manager._worker is None
    assert errors == ["机器人栈正在停止，请等待清理完成后再启动。"]
    release.set()
    manager._stop_worker.join(timeout=2)


def test_remote_pipeline_has_an_independent_total_ready_timeout(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.delenv("GUI_READY_TIMEOUT", raising=False)
    monkeypatch.delenv("GUI_PIPELINE_READY_TIMEOUT", raising=False)
    manager = ProcessManager(tmp_path, mode="dual")
    assert manager.ready_timeout == 90
    assert manager.pipeline_ready_timeout == 600

    monkeypatch.setenv("GUI_READY_TIMEOUT", "75")
    monkeypatch.setenv("GUI_PIPELINE_READY_TIMEOUT", "480")
    manager = ProcessManager(tmp_path, mode="right")
    assert manager.ready_timeout == 75
    assert manager.pipeline_ready_timeout == 480


def test_remote_pipeline_force_kill_is_reported_as_unsafe(
    tmp_path: Path, monkeypatch
) -> None:
    manager = ProcessManager(tmp_path, mode="right")
    waits = []

    class FakeProcess:
        pid = 123

        def poll(self):
            return None

        def wait(self, timeout):
            waits.append(timeout)
            raise subprocess.TimeoutExpired("pipeline", timeout)

    process = FakeProcess()
    manager._processes["15_right_arm_stack"] = ManagedProcess(
        "15_right_arm_stack",
        "15_record_bi_arm_pipeline.sh",
        process,
        tmp_path / "pipeline.log",
    )
    monkeypatch.setattr(process_module, "_collect_pid_tree", lambda _pid: [123])
    monkeypatch.setattr(process_module, "_kill_pids", lambda _pids, _signal: None)
    assert not manager._stop_one("15_right_arm_stack")
    assert waits == [30, 3]
    assert not manager._stop_stack_worker(False)


@pytest.mark.parametrize(
    ("mode", "method_name"),
    (("dual", "_start_dual_stack_script"), ("right", "_start_right_stack_script")),
)
def test_gui_local_sudo_file_is_not_reused_for_right_host(
    tmp_path: Path,
    monkeypatch,
    mode: str,
    method_name: str,
) -> None:
    script = tmp_path / "15_record_bi_arm_pipeline.sh"
    script.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    manager = ProcessManager(tmp_path, mode=mode)
    manager._run_log_dir = tmp_path / "logs"
    manager._run_log_dir.mkdir()
    captured = {}
    ready_call = {}

    class FakeProcess:
        pid = 123

        def poll(self):
            return None

    def fake_popen(*_args, **kwargs):
        captured.update(kwargs["env"])
        return FakeProcess()

    for key in (
        "BI_ARM_LOCAL_SUDO_PASSWORD",
        "BI_ARM_REMOTE_SUDO_PASSWORD",
        "BI_ARM_SSH_PASSWORD",
        "FRANKA_GUI_SUDO_PASSWORD",
        "FRANKA_SUDO_PASSWORD",
    ):
        monkeypatch.setenv(key, "legacy-secret")
    monkeypatch.setattr(process_module, "_sudo_password_file", lambda: "/private/local-sudo")
    monkeypatch.setattr(process_module.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(
        manager,
        "_wait_until_ready",
        lambda *_args, **kwargs: ready_call.update(kwargs),
    )
    getattr(manager, method_name)()

    assert ready_call["timeout"] == manager.pipeline_ready_timeout == 600
    assert captured["BI_ARM_LOCAL_SUDO_PASSWORD_FILE"] == "/private/local-sudo"
    assert "BI_ARM_LOCAL_SUDO_PASSWORD" not in captured
    assert "BI_ARM_REMOTE_SUDO_PASSWORD" not in captured
    assert "BI_ARM_SSH_PASSWORD" not in captured
    assert "FRANKA_GUI_SUDO_PASSWORD" not in captured
    assert "FRANKA_SUDO_PASSWORD" not in captured
