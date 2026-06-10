"""Camera/robot capture orchestration for the PyQt GUI."""

from __future__ import annotations

import queue
import shutil
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from PyQt6 import QtCore

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_dual import (
    DEFAULT_LEFT_ROBOT,
    DEFAULT_RIGHT_ROBOT,
    DUAL_SCHEMA_VERSION,
)
from franka_capture.config.fr3_single import (
    DEFAULT_CAMERAS,
    DEFAULT_ROBOT,
    SINGLE_SCHEMA_VERSION,
)
from franka_capture.core.robot_zmq_client import RobotZMQClient

from .async_episode_saver import AsyncEpisodeSaver, EpisodeSaveRequest
from .mock_sources import MockRobot, create_mock_cameras

MAX_GRIPPER_WIDTH = 0.09
GRIPPER_BINARY_THRESHOLD = 0.5
GRIPPER_BINARY_SEMANTICS = "binary_closedness_command"
FIXED_CAPTURE_FPS = 30


@dataclass
class CaptureOptions:
    output_root: str
    mode: str = "single"
    camera_names: Optional[List[str]] = None
    camera_fps: int = FIXED_CAPTURE_FPS
    video_fps: int = FIXED_CAPTURE_FPS
    robot_host: str = DEFAULT_ROBOT.host
    robot_port: int = DEFAULT_ROBOT.port
    robot_timeout_ms: int = DEFAULT_ROBOT.timeout_ms
    left_robot_host: str = DEFAULT_LEFT_ROBOT.host
    left_robot_port: int = DEFAULT_LEFT_ROBOT.port
    right_robot_host: str = DEFAULT_RIGHT_ROBOT.host
    right_robot_port: int = DEFAULT_RIGHT_ROBOT.port
    dual_robot_timeout_ms: int = DEFAULT_LEFT_ROBOT.timeout_ms
    mock: bool = False

    def __post_init__(self) -> None:
        self.camera_fps = FIXED_CAPTURE_FPS
        self.video_fps = FIXED_CAPTURE_FPS
        if self.mode not in {"single", "dual"}:
            raise ValueError(f"Unsupported capture mode: {self.mode}")
        if self.camera_names is not None:
            self.camera_names = [name for name in self.camera_names if name]


@dataclass
class ActiveEpisode:
    output_root: str
    task: str
    task_description: str
    user_metadata: Dict[str, Any]
    index: int
    started_at: float
    camera_names: List[str]
    video_fps: int
    frames: List[Dict[str, Any]]
    keyframes: List[int]


class CaptureThread(QtCore.QThread):
    preview_frame = QtCore.pyqtSignal(object)
    status_changed = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    cameras_ready = QtCore.pyqtSignal(list)
    recording_started = QtCore.pyqtSignal(str, int)
    recording_paused = QtCore.pyqtSignal(str, int, int)
    recording_resumed = QtCore.pyqtSignal(str, int)
    recording_frame_count = QtCore.pyqtSignal(str, int, int)
    episode_ready = QtCore.pyqtSignal(object)
    episode_discarded = QtCore.pyqtSignal(str, int, int)

    def __init__(self, options: CaptureOptions, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.options = options
        self._commands: queue.Queue[tuple[str, dict]] = queue.Queue()
        self._stop_requested = False
        self._recording = False
        self._active: ActiveEpisode | None = None
        self._robot = None
        self._left_robot = None
        self._right_robot = None
        self._robot_warned = False
        self._dual_read_executor = ThreadPoolExecutor(
            max_workers=2,
            thread_name_prefix="gui-dual-arm-read",
        )

    def enqueue(self, command: str, **payload) -> None:
        self._commands.put((command, payload))

    def stop(self) -> None:
        self._stop_requested = True
        self.enqueue("stop")

    def run(self) -> None:
        cameras = {}
        try:
            if self.options.mock:
                cameras = create_mock_cameras(_selected_camera_names(self.options), self.options.camera_fps)
                camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}
            else:
                cameras = create_realsense_cameras(
                    _fixed_fps_camera_configs(self.options.camera_names)
                )
                camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}

            camera_names = list(cameras.keys())
            self.cameras_ready.emit(camera_names)
            self.status_changed.emit("相机预览已启动")

            while not self._stop_requested:
                self._drain_commands(camera_names, camera_metadata)
                rgb_frames = {}
                for name, camera in cameras.items():
                    rgb, _ = camera.read()
                    rgb_frames[name] = rgb

                self.preview_frame.emit(rgb_frames)
                self._drain_commands(camera_names, camera_metadata)

                if not self._recording or self._active is None:
                    continue

                try:
                    frame = self._build_record_frame(rgb_frames)
                except Exception as exc:
                    self._recording = False
                    self.error.emit(f"读取机器人状态失败，已暂停当前 episode: {exc}")
                    continue

                self._active.frames.append(frame)
                self.recording_frame_count.emit(
                    self._active.task,
                    self._active.index,
                    len(self._active.frames),
                )
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if self._active is not None and self._active.frames:
                self._recording = False
                self.error.emit("采集线程停止时仍有未保存 episode，请重新确认是否需要重录。")
            for camera in cameras.values():
                try:
                    camera.close()
                except Exception:
                    pass
            if self._robot is not None:
                try:
                    self._robot.close()
                except Exception:
                    pass
            for robot in (self._left_robot, self._right_robot):
                if robot is not None:
                    try:
                        robot.close()
                    except Exception:
                        pass
            self._dual_read_executor.shutdown(wait=False)
            self.status_changed.emit("相机预览已停止")

    def _drain_commands(self, camera_names: List[str], camera_metadata: Dict[str, Any]) -> None:
        while True:
            try:
                command, payload = self._commands.get_nowait()
            except queue.Empty:
                return

            if command == "stop":
                self._stop_requested = True
                return
            if command == "start":
                self._start_or_resume(payload, camera_names)
            elif command == "pause":
                self._pause()
            elif command == "finish":
                self._finish(camera_metadata)
            elif command == "discard":
                self._discard()
            elif command == "keyframe":
                self._add_keyframe()

    def _start_or_resume(self, payload: Dict[str, Any], camera_names: List[str]) -> None:
        if self._active is None:
            self._active = ActiveEpisode(
                task=payload["task"],
                task_description=payload.get("task_description", ""),
                user_metadata=dict(payload.get("user_metadata", {})),
                index=int(payload["index"]),
                output_root=payload["output_root"],
                started_at=time.time(),
                camera_names=list(camera_names),
                video_fps=int(payload["video_fps"]),
                frames=[],
                keyframes=[0],
            )
            self.recording_started.emit(self._active.task, self._active.index)
        else:
            self.recording_resumed.emit(self._active.task, self._active.index)
        self._recording = True

    def _pause(self) -> None:
        if self._active is None:
            return
        self._recording = False
        self.recording_paused.emit(self._active.task, self._active.index, len(self._active.frames))

    def _finish(self, camera_metadata: Dict[str, Any]) -> None:
        if self._active is None:
            return
        active = self._active
        self._active = None
        self._recording = False
        if not active.frames:
            self.episode_discarded.emit(active.task, active.index, 0)
            return
        request = EpisodeSaveRequest(
            output_root=active.output_root,
            task=active.task,
            index=active.index,
            frames=active.frames,
            keyframes=_valid_keyframes(active.keyframes, len(active.frames)),
            camera_names=active.camera_names,
            video_fps=active.video_fps,
            metadata={
                "source": "franka_gui",
                "schema_version": (
                    DUAL_SCHEMA_VERSION
                    if self.options.mode == "dual"
                    else SINGLE_SCHEMA_VERSION
                ),
                "task_description": active.task_description,
                "user_metadata": active.user_metadata,
                "started_at_unix": active.started_at,
                "ended_at_unix": time.time(),
                **self._robot_metadata(),
                "gripper_semantics": GRIPPER_BINARY_SEMANTICS,
                "gripper_values": {"0": "open", "1": "closed"},
                "gripper_command_threshold": GRIPPER_BINARY_THRESHOLD,
                "gripper_width_field": "gripper_width",
                "gripper_target_width_field": "gripper_target_width",
                "gripper_command_source": "robot_observations.gripper_command",
                "cameras": camera_metadata,
            },
        )
        self.episode_ready.emit(request)

    def _robot_metadata(self) -> Dict[str, Any]:
        if self.options.mode == "dual":
            return {
                "robots": {
                    "left": {
                        "host": self.options.left_robot_host,
                        "port": self.options.left_robot_port,
                        "timeout_ms": self.options.dual_robot_timeout_ms,
                    },
                    "right": {
                        "host": self.options.right_robot_host,
                        "port": self.options.right_robot_port,
                        "timeout_ms": self.options.dual_robot_timeout_ms,
                    },
                }
            }
        return {
            "robot": {
                "host": self.options.robot_host,
                "port": self.options.robot_port,
                "timeout_ms": self.options.robot_timeout_ms,
            }
        }

    def _discard(self) -> None:
        if self._active is None:
            return
        active = self._active
        self._active = None
        self._recording = False
        self.episode_discarded.emit(active.task, active.index, len(active.frames))

    def _add_keyframe(self) -> None:
        if self._active is None or not self._recording:
            self.error.emit("当前没有正在录制的 episode，关键帧已忽略。")
            return
        keyframe = len(self._active.frames)
        if not self._active.keyframes or self._active.keyframes[-1] != keyframe:
            self._active.keyframes.append(keyframe)
        self.status_changed.emit(f"已添加关键帧: episode {self._active.index}, frame {keyframe}")

    def _build_record_frame(self, rgb_frames: Dict[str, np.ndarray]) -> Dict[str, Any]:
        if self.options.mode == "dual":
            return self._build_dual_record_frame(rgb_frames)
        return self._build_single_record_frame(rgb_frames)

    def _build_single_record_frame(self, rgb_frames: Dict[str, np.ndarray]) -> Dict[str, Any]:
        robot = self._ensure_robot()
        robot_observations = robot.get_observations()
        joint_state = robot.get_joint_state()
        if "ee_pose_euler" not in robot_observations:
            raise RuntimeError(
                "robot node 缺少 ee_pose_euler，请重启 3_launch_node.sh 并确认 fr3 observation 已更新。"
            )
        if "gripper_command" not in robot_observations:
            raise RuntimeError(
                "robot node 缺少 gripper_command，请重启 3_launch_node.sh 并确认 fr3 observation 已更新。"
            )
        gripper_command_value = _as_scalar(robot_observations["gripper_command"])
        gripper_command = _as_binary_gripper_command(gripper_command_value)
        gripper_command_raw = _as_scalar(
            robot_observations.get("gripper_command_raw", gripper_command_value)
        )
        gripper_target_width = _as_scalar(
            robot_observations.get(
                "gripper_target_width",
                MAX_GRIPPER_WIDTH * (1.0 - gripper_command_raw),
            )
        )
        gripper_command_timestamp = _as_scalar(
            robot_observations.get("gripper_command_timestamp", time.time())
        )
        gripper_width = _as_scalar(
            robot_observations.get(
                "gripper_width",
                joint_state[-1] * MAX_GRIPPER_WIDTH,
            )
        )
        frame = {
            "schema_version": SINGLE_SCHEMA_VERSION,
            "pose": _as_saved_value(robot_observations["ee_pose_euler"]),
            "joint": _as_saved_value(joint_state[:7]),
            "gripper": gripper_command,
            "gripper_width": gripper_width,
            "gripper_command_raw": gripper_command_raw,
            "gripper_target_width": gripper_target_width,
            "gripper_command_timestamp": gripper_command_timestamp,
            "gripper_command_source": robot_observations.get("gripper_command_source", ""),
            "timestamp": time.time(),
        }
        for name, rgb in rgb_frames.items():
            frame[f"{name}_image"] = rgb[:, :, ::-1].copy()
        return frame

    def _build_dual_record_frame(self, rgb_frames: Dict[str, np.ndarray]) -> Dict[str, Any]:
        left_robot, right_robot = self._ensure_dual_robots()
        loop_start_monotonic = time.monotonic()
        loop_start_timestamp = time.time()
        left_future = self._dual_read_executor.submit(_fetch_arm_state, left_robot)
        right_future = self._dual_read_executor.submit(_fetch_arm_state, right_robot)
        left_state = left_future.result()
        right_state = right_future.result()
        loop_end_monotonic = time.monotonic()
        timestamp = time.time()

        frame = {
            "schema_version": DUAL_SCHEMA_VERSION,
            "frame_index": len(self._active.frames) if self._active is not None else 0,
            "timestamp": timestamp,
            "loop_start_timestamp": loop_start_timestamp,
            "loop_end_timestamp": timestamp,
            "loop_start_monotonic": loop_start_monotonic,
            "loop_end_monotonic": loop_end_monotonic,
            "loop_duration_ms": (loop_end_monotonic - loop_start_monotonic) * 1000.0,
        }
        _add_prefixed_fields(frame, "left", left_state)
        _add_prefixed_fields(frame, "right", right_state)
        for name, rgb in rgb_frames.items():
            frame[f"{name}_image"] = rgb[:, :, ::-1].copy()
        return frame

    def _ensure_robot(self):
        if self.options.mock:
            if self._robot is None:
                self._robot = MockRobot()
            return self._robot
        if self._robot is None:
            robot = RobotZMQClient(
                self.options.robot_host,
                self.options.robot_port,
                timeout_ms=self.options.robot_timeout_ms,
            )
            try:
                dofs = robot.num_dofs()
            except Exception:
                robot.close()
                raise
            self._robot = robot
            if dofs != 8 and not self._robot_warned:
                self._robot_warned = True
                self.error.emit(f"robot node DOF={dofs}，预期单臂+夹爪为 8。")
        return self._robot

    def _ensure_dual_robots(self):
        if self.options.mock:
            if self._left_robot is None:
                self._left_robot = MockRobot()
            if self._right_robot is None:
                self._right_robot = MockRobot()
            return self._left_robot, self._right_robot

        if self._left_robot is None:
            self._left_robot = _connect_checked_robot(
                self.options.left_robot_host,
                self.options.left_robot_port,
                self.options.dual_robot_timeout_ms,
                "left",
            )
        if self._right_robot is None:
            self._right_robot = _connect_checked_robot(
                self.options.right_robot_host,
                self.options.right_robot_port,
                self.options.dual_robot_timeout_ms,
                "right",
            )
        return self._left_robot, self._right_robot


class CaptureController(QtCore.QObject):
    preview_frame = QtCore.pyqtSignal(object)
    status_changed = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    cameras_ready = QtCore.pyqtSignal(list)
    active_episode_changed = QtCore.pyqtSignal(str, int, str)
    recording_frame_count = QtCore.pyqtSignal(int)
    save_queue_changed = QtCore.pyqtSignal(int)
    episode_saved = QtCore.pyqtSignal(str, int, str, int)

    def __init__(self, options: CaptureOptions, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.options = options
        self.saver = AsyncEpisodeSaver(parent=self)
        self.thread: CaptureThread | None = None
        self._reserved: Dict[str, set[int]] = {}
        self._active_task: str | None = None
        self._active_index: int | None = None
        self._active_state = "idle"
        self._start_pending = False

        self.saver.queue_changed.connect(self.save_queue_changed)
        self.saver.save_started.connect(
            lambda task, index, output_dir: self.status_changed.emit(
                f"后台保存开始: {task}/{index} -> {output_dir}"
            )
        )
        self.saver.save_finished.connect(self._on_save_finished)
        self.saver.save_failed.connect(self._on_save_failed)

    def start_preview(self) -> None:
        if self.thread is not None and self.thread.isRunning():
            return
        self.thread = CaptureThread(self.options, parent=self)
        self.thread.preview_frame.connect(self.preview_frame)
        self.thread.status_changed.connect(self.status_changed)
        self.thread.error.connect(self.error)
        self.thread.cameras_ready.connect(self.cameras_ready)
        self.thread.recording_started.connect(self._on_recording_started)
        self.thread.recording_resumed.connect(self._on_recording_resumed)
        self.thread.recording_paused.connect(self._on_recording_paused)
        self.thread.recording_frame_count.connect(self._on_frame_count)
        self.thread.episode_ready.connect(self._on_episode_ready)
        self.thread.episode_discarded.connect(self._on_episode_discarded)
        self.thread.finished.connect(lambda: self.status_changed.emit("采集线程已退出"))
        self.thread.start()

    def stop_preview(self) -> None:
        if self.thread is None:
            return
        self.thread.stop()
        self.thread.wait(3000)
        if self.thread.isRunning():
            self.thread.terminate()
            self.thread.wait(1000)
        self.thread = None

    def start_or_resume(
        self,
        task: str,
        task_description: str = "",
        user_metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.start_preview()
        if self.thread is None:
            return
        if self._start_pending:
            self.status_changed.emit("已有开始命令在处理中，请稍等。")
            return
        if self._active_state == "recording":
            self.status_changed.emit("当前已经在录制中。")
            return
        metadata = dict(user_metadata or {})
        if self._active_state == "paused" and self._active_task is not None:
            self._start_pending = True
            self.thread.enqueue(
                "start",
                output_root=self.options.output_root,
                task=self._active_task,
                task_description=task_description,
                user_metadata=metadata,
                index=self._active_index,
                video_fps=self.options.video_fps,
            )
            return
        index = self._allocate_episode_index(task)
        self._start_pending = True
        self.thread.enqueue(
            "start",
            output_root=self.options.output_root,
            task=task,
            task_description=task_description,
            user_metadata=metadata,
            index=index,
            video_fps=self.options.video_fps,
        )

    def pause(self) -> None:
        if self.thread is not None:
            self.thread.enqueue("pause")

    def finish(self) -> None:
        if self.thread is not None and self._active_state in {"recording", "paused"}:
            self._active_state = "finishing"
            self.status_changed.emit("正在结束当前 episode，随后会进入后台保存队列。")
            self.thread.enqueue("finish")

    def discard(self) -> None:
        if self.thread is not None and self._active_state in {"recording", "paused"}:
            if self._active_task is not None and self._active_index is not None:
                self._release_reserved(self._active_task, self._active_index)
            self._active_state = "discarding"
            self.status_changed.emit("正在丢弃当前 episode。")
            self.thread.enqueue("discard")

    def add_keyframe(self) -> None:
        if self.thread is not None:
            self.thread.enqueue("keyframe")

    def scan_tasks(self) -> List[str]:
        root = Path(self.options.output_root).expanduser()
        if not root.exists():
            return []
        return sorted([path.name for path in root.iterdir() if path.is_dir()])

    def peek_next_episode_index(self, task: str) -> int:
        return self._next_episode_index_for_task(task)

    def disk_usage(self):
        root = Path(self.options.output_root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        return shutil.disk_usage(root)

    def shutdown(self) -> None:
        self.stop_preview()
        self.saver.shutdown()

    def _allocate_episode_index(self, task: str) -> int:
        next_index = self._next_episode_index_for_task(task)
        self._reserved.setdefault(task, set()).add(next_index)
        return next_index

    def _next_episode_index_for_task(self, task: str) -> int:
        root = Path(self.options.output_root).expanduser() / task
        existing: List[int] = []
        if root.exists():
            for child in root.iterdir():
                if child.is_dir() and child.name.isdigit():
                    existing.append(int(child.name))
        reserved = self._reserved.setdefault(task, set())
        return max(existing + list(reserved), default=-1) + 1

    def _release_reserved(self, task: str, index: int) -> None:
        self._reserved.setdefault(task, set()).discard(index)

    def _on_recording_started(self, task: str, index: int) -> None:
        self._start_pending = False
        self._active_task = task
        self._active_index = index
        self._active_state = "recording"
        self.active_episode_changed.emit(task, index, "recording")
        self.status_changed.emit(f"开始录制: {task}/{index}")

    def _on_recording_resumed(self, task: str, index: int) -> None:
        self._start_pending = False
        self._active_state = "recording"
        self.active_episode_changed.emit(task, index, "recording")
        self.status_changed.emit(f"继续录制: {task}/{index}")

    def _on_recording_paused(self, task: str, index: int, frames: int) -> None:
        self._active_state = "paused"
        self.active_episode_changed.emit(task, index, "paused")
        self.status_changed.emit(f"暂停录制: {task}/{index}, frames={frames}")

    def _on_frame_count(self, task: str, index: int, frames: int) -> None:
        self.recording_frame_count.emit(frames)

    def _on_episode_ready(self, request: EpisodeSaveRequest) -> None:
        self._active_task = None
        self._active_index = None
        self._active_state = "saving"
        self.active_episode_changed.emit(request.task, request.index, "saving")
        self.status_changed.emit(f"episode {request.task}/{request.index} 已进入后台保存队列")
        self.saver.enqueue(request)

    def _on_episode_discarded(self, task: str, index: int, frames: int) -> None:
        self._release_reserved(task, index)
        self._start_pending = False
        self._active_task = None
        self._active_index = None
        self._active_state = "idle"
        self.active_episode_changed.emit(task, index, "discarded")
        self.status_changed.emit(f"已丢弃: {task}/{index}, frames={frames}")

    def _on_save_finished(self, task: str, index: int, output_dir: str, frame_count: int) -> None:
        self.episode_saved.emit(task, index, output_dir, frame_count)
        if self._active_task is None:
            self._active_state = "idle"
            self.active_episode_changed.emit(task, index, "saved")
        self.status_changed.emit(f"保存完成: {output_dir}, frames={frame_count}")

    def _on_save_failed(self, task: str, index: int, output_dir: str, error: str) -> None:
        if self._active_task is None:
            self._active_state = "idle"
        self.error.emit(f"保存失败: {task}/{index}\n{error}")


def _selected_camera_names(options: CaptureOptions) -> List[str]:
    if options.camera_names:
        return list(options.camera_names)
    return list(DEFAULT_CAMERAS.keys())


def _fixed_fps_camera_configs(camera_names: Optional[Sequence[str]] = None):
    from dataclasses import replace

    names = list(camera_names) if camera_names else list(DEFAULT_CAMERAS.keys())
    missing = [name for name in names if name not in DEFAULT_CAMERAS]
    if missing:
        raise KeyError(f"Unknown configured camera name(s): {missing}")
    return {
        name: replace(DEFAULT_CAMERAS[name], fps=FIXED_CAPTURE_FPS)
        for name in names
    }


def _connect_checked_robot(host: str, port: int, timeout_ms: int, label: str):
    robot = RobotZMQClient(host, port, timeout_ms=timeout_ms)
    try:
        dofs = robot.num_dofs()
    except Exception:
        robot.close()
        raise
    if dofs != 8:
        robot.close()
        raise RuntimeError(f"{label} robot node DOF={dofs}，预期单臂+夹爪为 8。")
    return robot


def _fetch_arm_state(robot: RobotZMQClient) -> Dict[str, Any]:
    start_wall = time.time()
    start_monotonic = time.monotonic()
    robot_observations = robot.get_observations()
    joint_state = robot.get_joint_state()
    end_monotonic = time.monotonic()
    end_wall = time.time()

    state = _extract_arm_state(robot_observations, joint_state)
    state.update(
        {
            "robot_read_start_timestamp": start_wall,
            "robot_read_end_timestamp": end_wall,
            "robot_read_start_monotonic": start_monotonic,
            "robot_read_end_monotonic": end_monotonic,
            "robot_read_duration_ms": (end_monotonic - start_monotonic) * 1000.0,
        }
    )
    return state


def _extract_arm_state(robot_observations: Dict[str, Any], joint_state: Any) -> Dict[str, Any]:
    if "ee_pose_euler" not in robot_observations:
        raise RuntimeError(
            "robot node 缺少 ee_pose_euler，请重启 3_launch_node.sh 并确认 fr3 observation 已更新。"
        )
    if "gripper_command" not in robot_observations:
        raise RuntimeError(
            "robot node 缺少 gripper_command，请重启 3_launch_node.sh 并确认 fr3 observation 已更新。"
        )

    gripper_command_value = _as_scalar(robot_observations["gripper_command"])
    gripper_command = _as_binary_gripper_command(gripper_command_value)
    gripper_command_raw = _as_scalar(
        robot_observations.get("gripper_command_raw", gripper_command_value)
    )
    gripper_target_width = _as_scalar(
        robot_observations.get(
            "gripper_target_width",
            MAX_GRIPPER_WIDTH * (1.0 - gripper_command_raw),
        )
    )
    gripper_command_timestamp = _as_scalar(
        robot_observations.get("gripper_command_timestamp", time.time())
    )
    gripper_width = _as_scalar(
        robot_observations.get(
            "gripper_width",
            joint_state[-1] * MAX_GRIPPER_WIDTH,
        )
    )
    return {
        "pose": _as_saved_value(robot_observations["ee_pose_euler"]),
        "joint": _as_saved_value(joint_state[:7]),
        "gripper": gripper_command,
        "gripper_width": gripper_width,
        "gripper_command_raw": gripper_command_raw,
        "gripper_target_width": gripper_target_width,
        "gripper_command_timestamp": gripper_command_timestamp,
        "gripper_command_source": robot_observations.get("gripper_command_source", ""),
    }


def _add_prefixed_fields(frame: Dict[str, Any], prefix: str, values: Dict[str, Any]) -> None:
    for key, value in values.items():
        frame[f"{prefix}_{key}"] = value


def _valid_keyframes(keyframes: List[int], frame_count: int) -> List[int]:
    valid = sorted({int(k) for k in keyframes if 0 <= int(k) < max(1, frame_count)})
    return valid or [0]


def _as_saved_value(value: Any):
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def _as_scalar(value: Any) -> float:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError("Expected scalar value, got empty sequence")
        value = value[0]
    return float(value)


def _as_binary_gripper_command(value: Any) -> float:
    return 1.0 if _as_scalar(value) >= GRIPPER_BINARY_THRESHOLD else 0.0
