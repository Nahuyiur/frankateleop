"""Camera/robot capture orchestration for the PyQt GUI."""

from __future__ import annotations

import queue
import shutil
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from dataclasses import replace
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
from franka_capture.gripper_fields import gripper_metadata, observation_gripper_fields

from .async_episode_saver import AsyncEpisodeSaver, EpisodeSaveRequest
from .mock_sources import MockRobot, create_mock_cameras

FIXED_CAPTURE_FPS = 30
GUI_CAMERA_READ_TIMEOUT_MS = 3000
HIGH_QUALITY_DIR = "High_Quality"
LOW_QUALITY_DIR = "Low_Quality"
QUALITY_DIRS = (HIGH_QUALITY_DIR, LOW_QUALITY_DIR)


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
        if self.mode not in {"single", "right", "dual"}:
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
        self._robot_sampler = None
        self._left_robot_sampler = None
        self._right_robot_sampler = None
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
            requested_camera_names = _selected_camera_names(self.options)
            if self.options.mock:
                cameras = create_mock_cameras(requested_camera_names, self.options.camera_fps)
                camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}
            else:
                cameras = create_realsense_cameras(
                    _fixed_fps_camera_configs(self.options.camera_names),
                    allow_missing=True,
                )
                camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}

            camera_names = list(cameras.keys())
            missing_camera_names = [
                name for name in requested_camera_names if name not in camera_names
            ]
            self.cameras_ready.emit(camera_names)
            if not camera_names:
                self.status_changed.emit("没有检测到可用 RealSense；相机预览为空，仍可录制机器人状态")
            elif missing_camera_names:
                self.status_changed.emit(f"相机预览已启动；已跳过缺失相机: {missing_camera_names}")
            else:
                self.status_changed.emit("相机预览已启动")

            while not self._stop_requested:
                loop_started = time.monotonic()
                self._drain_commands(camera_names, camera_metadata)
                rgb_frames = {}
                unavailable_camera_names = []
                for name, camera in list(cameras.items()):
                    try:
                        rgb, _ = camera.read()
                    except Exception as exc:
                        unavailable_camera_names.append(name)
                        self.status_changed.emit(f"相机 {name} 3 秒内无画面，已跳过")
                        try:
                            camera.close()
                        except Exception:
                            pass
                        continue
                    rgb_frames[name] = rgb
                if unavailable_camera_names:
                    for name in unavailable_camera_names:
                        cameras.pop(name, None)
                        camera_metadata.pop(name, None)
                    camera_names = [name for name in camera_names if name in cameras]
                    if self._active is not None:
                        self._active.camera_names = [
                            name for name in self._active.camera_names if name in cameras
                        ]
                    self.cameras_ready.emit(camera_names)

                self.preview_frame.emit(rgb_frames)
                self._drain_commands(camera_names, camera_metadata)

                if self._recording and self._active is not None:
                    try:
                        frame = self._build_record_frame(rgb_frames)
                    except Exception as exc:
                        self._recording = False
                        self.error.emit(f"读取机器人状态失败，已暂停当前 episode: {exc}")
                    else:
                        self._active.frames.append(frame)
                        self.recording_frame_count.emit(
                            self._active.task,
                            self._active.index,
                            len(self._active.frames),
                        )
                if not cameras:
                    _sleep_to_capture_rate(loop_started, self.options.camera_fps)
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
            for sampler in (self._robot_sampler, self._left_robot_sampler, self._right_robot_sampler):
                if sampler is not None:
                    try:
                        sampler.close()
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
            text_instruction=active.task_description,
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
                **gripper_metadata(),
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
            },
            "arm_side": "right" if self.options.mode == "right" else "left",
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
        if self.options.mock:
            state = _fetch_arm_state(self._ensure_robot())
        else:
            state = self._ensure_robot_sampler().snapshot()
        now = time.time()
        frame = {
            "schema_version": SINGLE_SCHEMA_VERSION,
            "frame_index": len(self._active.frames) if self._active is not None else 0,
            "timestamp": now,
            **state,
        }
        for name, rgb in rgb_frames.items():
            frame[f"{name}_image"] = rgb[:, :, ::-1].copy()
        return frame

    def _build_dual_record_frame(self, rgb_frames: Dict[str, np.ndarray]) -> Dict[str, Any]:
        loop_start_monotonic = time.monotonic()
        loop_start_timestamp = time.time()
        if self.options.mock:
            left_robot, right_robot = self._ensure_dual_robots()
            left_future = self._dual_read_executor.submit(_fetch_arm_state, left_robot)
            right_future = self._dual_read_executor.submit(_fetch_arm_state, right_robot)
            left_state = left_future.result()
            right_state = right_future.result()
        else:
            left_sampler, right_sampler = self._ensure_dual_robot_samplers()
            left_state = left_sampler.snapshot()
            right_state = right_sampler.snapshot()
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

    def _ensure_robot_sampler(self):
        if self._robot_sampler is None:
            self._robot_sampler = _RobotStateSampler(
                self.options.robot_host,
                self.options.robot_port,
                self.options.robot_timeout_ms,
                "robot",
            )
            self._robot_sampler.start()
        return self._robot_sampler

    def _ensure_dual_robot_samplers(self):
        if self._left_robot_sampler is None:
            self._left_robot_sampler = _RobotStateSampler(
                self.options.left_robot_host,
                self.options.left_robot_port,
                self.options.dual_robot_timeout_ms,
                "left",
            )
            self._left_robot_sampler.start()
        if self._right_robot_sampler is None:
            self._right_robot_sampler = _RobotStateSampler(
                self.options.right_robot_host,
                self.options.right_robot_port,
                self.options.dual_robot_timeout_ms,
                "right",
            )
            self._right_robot_sampler.start()
        return self._left_robot_sampler, self._right_robot_sampler


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
        self._reserved: Dict[tuple[str, str], set[int]] = {}
        self._active_task: str | None = None
        self._active_index: int | None = None
        self._active_state = "idle"
        self._start_pending = False
        self._pending_quality_request: EpisodeSaveRequest | None = None
        self._saving_quality_requests: Dict[tuple[str, str, int], EpisodeSaveRequest] = {}

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
        if self._active_state == "quality_pending":
            self.status_changed.emit("当前 episode 等待质量分层，请先按 h 或 l。")
            return
        if self._active_state == "saving":
            self.status_changed.emit("当前 episode 正在后台保存，请等待保存完成后再开始下一条。")
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
        index = self._next_display_episode_index(task)
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
            self.status_changed.emit("正在结束当前 episode，随后请按 h 或 l 完成质量分层。")
            self.thread.enqueue("finish")

    def discard(self) -> None:
        if self._active_state == "quality_pending" and self._pending_quality_request is not None:
            request = self._pending_quality_request
            self._pending_quality_request = None
            self._start_pending = False
            self._active_task = None
            self._active_index = None
            self._active_state = "idle"
            self.active_episode_changed.emit(request.task, request.index, "discarded")
            self.status_changed.emit(
                f"已丢弃待分层 episode: {request.task}, frames={len(request.frames)}"
            )
            return
        if self.thread is not None and self._active_state in {"recording", "paused"}:
            self._active_state = "discarding"
            self.status_changed.emit("正在丢弃当前 episode。")
            self.thread.enqueue("discard")

    def add_keyframe(self) -> None:
        if self.thread is not None:
            self.thread.enqueue("keyframe")

    def mark_high_quality(self) -> None:
        self._save_pending_quality(HIGH_QUALITY_DIR)

    def mark_low_quality(self) -> None:
        self._save_pending_quality(LOW_QUALITY_DIR)

    def scan_tasks(self) -> List[str]:
        root = Path(self.options.output_root).expanduser()
        if not root.exists():
            return []
        return sorted([path.name for path in root.iterdir() if path.is_dir()])

    def peek_next_episode_index(self, task: str, quality: str = HIGH_QUALITY_DIR) -> int:
        return self._next_episode_index_for_task(task, quality)

    def disk_usage(self):
        root = Path(self.options.output_root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        return shutil.disk_usage(root)

    def shutdown(self) -> None:
        self.stop_preview()
        self.saver.shutdown()

    def _allocate_episode_index(self, task: str, quality: str) -> int:
        next_index = self._next_episode_index_for_task(task, quality)
        self._reserved.setdefault((task, quality), set()).add(next_index)
        return next_index

    def _next_episode_index_for_task(self, task: str, quality: str) -> int:
        root = Path(self.options.output_root).expanduser() / task / quality
        existing: List[int] = []
        if root.exists():
            for child in root.iterdir():
                if child.is_dir() and child.name.isdigit():
                    existing.append(int(child.name))
        reserved = self._reserved.setdefault((task, quality), set())
        return max(existing + list(reserved), default=-1) + 1

    def _next_display_episode_index(self, task: str) -> int:
        high = self._next_episode_index_for_task(task, HIGH_QUALITY_DIR)
        low = self._next_episode_index_for_task(task, LOW_QUALITY_DIR)
        return min(high, low)

    def _release_reserved(self, task: str, index: int, quality: str) -> None:
        self._reserved.setdefault((task, quality), set()).discard(index)

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
        self._pending_quality_request = request
        self._active_task = request.task
        self._active_index = request.index
        self._active_state = "quality_pending"
        self.active_episode_changed.emit(request.task, request.index, "quality_pending")
        self.status_changed.emit(
            f"episode {request.task}/{request.index} 等待质量分层：按 h 保存到高质量，按 l 保存到低质量。"
        )

    def _on_episode_discarded(self, task: str, index: int, frames: int) -> None:
        self._start_pending = False
        self._active_task = None
        self._active_index = None
        self._active_state = "idle"
        self.active_episode_changed.emit(task, index, "discarded")
        self.status_changed.emit(f"已丢弃: {task}/{index}, frames={frames}")

    def _on_save_finished(self, task: str, index: int, output_dir: str, frame_count: int) -> None:
        quality = Path(output_dir).parent.name
        if quality in QUALITY_DIRS:
            self._saving_quality_requests.pop((task, quality, index), None)
        self.episode_saved.emit(task, index, output_dir, frame_count)
        if self._active_task is None:
            self._active_state = "idle"
            self.active_episode_changed.emit(task, index, "saved")
        self.status_changed.emit(f"保存完成: {output_dir}, frames={frame_count}")

    def _on_save_failed(self, task: str, index: int, output_dir: str, error: str) -> None:
        quality = Path(output_dir).parent.name
        request = None
        if quality in QUALITY_DIRS:
            self._release_reserved(task, index, quality)
            request = self._saving_quality_requests.pop((task, quality, index), None)
        if request is not None and self._active_task is None:
            request = self._prepare_quality_retry_request(request)
            self._pending_quality_request = request
            self._active_task = request.task
            self._active_index = request.index
            self._active_state = "quality_pending"
            self.active_episode_changed.emit(request.task, request.index, "quality_pending")
            self.status_changed.emit(
                f"保存失败，episode 已恢复到 JUDGING，可重新按 h/l 或按 d 丢弃: {task}/{quality}/{index}"
            )
        elif self._active_task is None:
            self._active_state = "idle"
            self.active_episode_changed.emit(task, index, "save_failed")
        self.error.emit(f"保存失败: {task}/{index}\n{error}")

    def _save_pending_quality(self, quality: str) -> None:
        if quality not in QUALITY_DIRS:
            raise ValueError(f"Unsupported episode quality: {quality}")
        if self._active_state != "quality_pending" or self._pending_quality_request is None:
            self.status_changed.emit("当前没有等待分层的 episode。")
            return

        request = self._pending_quality_request
        index = self._allocate_episode_index(request.task, quality)
        metadata = dict(request.metadata)
        metadata["quality"] = quality
        request = replace(
            request,
            index=index,
            quality=quality,
            metadata=metadata,
        )
        request_key = (request.task, quality, request.index)
        self._saving_quality_requests[request_key] = request
        self._pending_quality_request = None
        self._active_task = None
        self._active_index = None
        self._active_state = "saving"
        self.active_episode_changed.emit(request.task, request.index, "saving")
        self.status_changed.emit(
            f"episode {request.task}/{quality}/{request.index} 已进入后台保存队列"
        )
        try:
            self.saver.enqueue(request)
        except Exception as exc:
            self._saving_quality_requests.pop(request_key, None)
            self._release_reserved(request.task, request.index, quality)
            request = self._prepare_quality_retry_request(request)
            self._pending_quality_request = request
            self._active_task = request.task
            self._active_index = request.index
            self._active_state = "quality_pending"
            self.active_episode_changed.emit(request.task, request.index, "quality_pending")
            self.error.emit(f"提交后台保存失败: {exc}")

    def _prepare_quality_retry_request(self, request: EpisodeSaveRequest) -> EpisodeSaveRequest:
        metadata = dict(request.metadata)
        metadata.pop("quality", None)
        return replace(
            request,
            index=self._next_display_episode_index(request.task),
            quality="",
            metadata=metadata,
        )


def _selected_camera_names(options: CaptureOptions) -> List[str]:
    if options.camera_names:
        return list(options.camera_names)
    return list(DEFAULT_CAMERAS.keys())


def _fixed_fps_camera_configs(camera_names: Optional[Sequence[str]] = None):
    names = list(camera_names) if camera_names else list(DEFAULT_CAMERAS.keys())
    missing = [name for name in names if name not in DEFAULT_CAMERAS]
    if missing:
        raise KeyError(f"Unknown configured camera name(s): {missing}")
    return {
        name: replace(
            DEFAULT_CAMERAS[name],
            fps=FIXED_CAPTURE_FPS,
            read_timeout_ms=GUI_CAMERA_READ_TIMEOUT_MS,
        )
        for name in names
    }


def _sleep_to_capture_rate(loop_started: float, fps: int) -> None:
    interval = 1.0 / max(int(fps), 1)
    remaining = interval - (time.monotonic() - loop_started)
    if remaining > 0:
        time.sleep(remaining)


class _RobotStateSampler:
    def __init__(self, host: str, port: int, timeout_ms: int, label: str) -> None:
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.label = label
        self._robot = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_state: Dict[str, Any] | None = None
        self._latest_error: str | None = None
        self._seq = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._robot = _connect_checked_robot(self.host, self.port, self.timeout_ms, self.label)
        self._thread = threading.Thread(
            target=self._run,
            name=f"gui-{self.label}-robot-state",
            daemon=True,
        )
        self._thread.start()
        self._wait_until_ready()

    def _wait_until_ready(self) -> None:
        deadline = time.monotonic() + max(1.0, self.timeout_ms / 1000.0)
        error = None
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest_state is not None:
                    return
                error = self._latest_error
            time.sleep(0.01)
        if error is not None:
            raise RuntimeError(f"{self.label} robot sampler failed: {error}")
        raise TimeoutError(f"Timed out waiting for {self.label} robot sampler")

    def _run(self) -> None:
        assert self._robot is not None
        period = 1.0 / FIXED_CAPTURE_FPS
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                state = _fetch_arm_state(self._robot)
            except Exception as exc:
                with self._lock:
                    self._latest_error = str(exc)
                self._stop.wait(0.05)
                continue
            with self._lock:
                self._seq += 1
                state["robot_state_seq"] = self._seq
                state["robot_sampler_error"] = ""
                self._latest_state = state
                self._latest_error = None
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            if self._latest_state is None:
                error = self._latest_error
                if error is not None:
                    raise RuntimeError(f"{self.label} robot sampler failed: {error}")
                raise RuntimeError(f"{self.label} robot sampler has no state yet")
            state = dict(self._latest_state)
            error = self._latest_error
        state["robot_sampler_error"] = error or state.get("robot_sampler_error", "")
        sample_time = state.get("robot_state_sample_monotonic")
        if sample_time is not None:
            state["robot_state_age_ms"] = (time.monotonic() - float(sample_time)) * 1000.0
        else:
            state["robot_state_age_ms"] = float("nan")
        state["robot_state_valid"] = True
        return state

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.timeout_ms / 1000.0))
            self._thread = None
        if self._robot is not None:
            self._robot.close()
            self._robot = None


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
    joint_state = robot_observations.get("joint_positions")
    if joint_state is None:
        joint_state = robot.get_joint_state()
    end_monotonic = time.monotonic()
    end_wall = time.time()

    state = _extract_arm_state(robot_observations, joint_state, timestamp=end_wall)
    state.update(
        {
            "robot_read_start_timestamp": start_wall,
            "robot_read_end_timestamp": end_wall,
            "robot_read_start_monotonic": start_monotonic,
            "robot_read_end_monotonic": end_monotonic,
            "robot_read_duration_ms": (end_monotonic - start_monotonic) * 1000.0,
            "robot_state_sample_timestamp": end_wall,
            "robot_state_sample_monotonic": end_monotonic,
            "robot_state_age_ms": 0.0,
            "robot_state_valid": True,
        }
    )
    return state


def _extract_arm_state(
    robot_observations: Dict[str, Any],
    joint_state: Any,
    *,
    timestamp: float | None = None,
) -> Dict[str, Any]:
    if "ee_pose_euler" not in robot_observations:
        raise RuntimeError(
            "robot node 缺少 ee_pose_euler，请重启 3_launch_node.sh 并确认 fr3 observation 已更新。"
        )
    if not any(
        key in robot_observations
        for key in (
            "gripper_closedness",
            "gripper_target_width",
        )
    ):
        raise RuntimeError(
            "robot node 缺少连续夹爪字段，请重启 3_launch_node.sh 并确认 fr3 observation 已更新。"
        )

    gripper_fields = observation_gripper_fields(
        robot_observations,
        joint_state,
        timestamp=timestamp if timestamp is not None else time.time(),
    )
    return {
        "pose": _as_saved_value(robot_observations["ee_pose_euler"]),
        "joint": _as_saved_value(joint_state[:7]),
        **gripper_fields,
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
