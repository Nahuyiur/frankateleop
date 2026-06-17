"""Launch the FR3 capture GUI."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PyQt6 import QtWidgets

from franka_capture.config.fr3_dual import DEFAULT_LEFT_ROBOT, DEFAULT_RIGHT_ROBOT
from franka_capture.config.fr3_single import (
    DEFAULT_ROBOT,
    RIGHT_GUI_CAMERA_NAMES,
    SINGLE_GUI_CAMERA_NAMES,
)

from .capture_controller import CaptureController, CaptureOptions, FIXED_CAPTURE_FPS
from .main_window import MainWindow
from .process_manager import ProcessManager


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mock", action="store_true", help="Use fake cameras and fake robot state.")
    parser.add_argument("--mode", choices=("single", "right", "dual"), default="single")
    parser.add_argument(
        "--camera-names",
        default="",
        help=(
            "Comma-separated camera names to preview and record. Empty means all "
            "configured cameras. Unavailable cameras are skipped."
        ),
    )
    parser.add_argument("--host", default=DEFAULT_ROBOT.host)
    parser.add_argument("--port", type=int, default=DEFAULT_ROBOT.port)
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_ROBOT.timeout_ms)
    parser.add_argument("--left-host", default=DEFAULT_LEFT_ROBOT.host)
    parser.add_argument("--left-port", type=int, default=DEFAULT_LEFT_ROBOT.port)
    parser.add_argument("--right-host", default=DEFAULT_RIGHT_ROBOT.host)
    parser.add_argument("--right-port", type=int, default=DEFAULT_RIGHT_ROBOT.port)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    camera_names = _parse_camera_names(args.camera_names)
    if args.mode == "single" and camera_names is None:
        camera_names = list(SINGLE_GUI_CAMERA_NAMES)
    elif args.mode == "right" and camera_names is None:
        camera_names = list(RIGHT_GUI_CAMERA_NAMES)

    robot_host = args.host
    robot_port = args.port
    if args.mode == "right":
        robot_host = args.right_host
        robot_port = args.right_port

    options = CaptureOptions(
        output_root=str(Path.home() / "Desktop" / "franka_record_data"),
        mode=args.mode,
        camera_names=camera_names,
        camera_fps=FIXED_CAPTURE_FPS,
        video_fps=FIXED_CAPTURE_FPS,
        robot_host=robot_host,
        robot_port=robot_port,
        robot_timeout_ms=args.timeout_ms,
        left_robot_host=args.left_host,
        left_robot_port=args.left_port,
        right_robot_host=args.right_host,
        right_robot_port=args.right_port,
        dual_robot_timeout_ms=args.timeout_ms,
        mock=args.mock,
    )

    app = QtWidgets.QApplication(sys.argv)
    app_names = {
        "single": "Franka Single GUI Capture",
        "right": "Franka Right GUI Capture",
        "dual": "Franka Dual GUI Capture",
    }
    app.setApplicationName(app_names[args.mode])
    controller = CaptureController(options)
    process_manager = ProcessManager(repo_root, mode=args.mode)
    window = MainWindow(controller, process_manager, repo_root)
    window.show()
    return app.exec()


def _parse_camera_names(value: str) -> list[str] | None:
    names = [item.strip() for item in value.split(",") if item.strip()]
    return names or None


if __name__ == "__main__":
    raise SystemExit(main())
