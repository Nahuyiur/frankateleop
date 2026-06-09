"""Launch the FR3 capture GUI."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PyQt6 import QtWidgets

from franka_capture.config.fr3_single import DEFAULT_ROBOT

from .capture_controller import CaptureController, CaptureOptions, FIXED_CAPTURE_FPS
from .main_window import MainWindow
from .process_manager import ProcessManager


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mock", action="store_true", help="Use fake cameras and fake robot state.")
    parser.add_argument("--host", default=DEFAULT_ROBOT.host)
    parser.add_argument("--port", type=int, default=DEFAULT_ROBOT.port)
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_ROBOT.timeout_ms)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]

    options = CaptureOptions(
        output_root=str(Path.home() / "Desktop" / "franka_record_data"),
        camera_fps=FIXED_CAPTURE_FPS,
        video_fps=FIXED_CAPTURE_FPS,
        robot_host=args.host,
        robot_port=args.port,
        robot_timeout_ms=args.timeout_ms,
        mock=args.mock,
    )

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Franka GUI Capture")
    controller = CaptureController(options)
    process_manager = ProcessManager(repo_root)
    window = MainWindow(controller, process_manager, repo_root)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
