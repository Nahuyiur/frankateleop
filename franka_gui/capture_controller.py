"""Camera/robot capture orchestration for the PyQt GUI."""

from __future__ import annotations

import os
import json
import queue
import shutil
import threading
import time
import uuid
from copy import deepcopy
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
    DUAL_VIDEO_SCHEMA_VERSION,
)
from franka_capture.config.fr3_single import (
    DEFAULT_CAMERAS,
    DEFAULT_ROBOT,
    SINGLE_VIDEO_SCHEMA_VERSION,
)
from franka_capture.core.robot_zmq_client import RobotZMQClient
from franka_capture.gripper_fields import gripper_metadata, observation_gripper_fields

from .async_episode_saver import (
    CACHE_ROOT_ENV,
    DEFAULT_CACHE_ROOT,
    SAVE_ERROR_VALIDATION_FAILED,
    AsyncEpisodeSaver,
    EpisodeSaveRequest,
    _validate_path_token,
)
from .mock_sources import MockRobot, create_mock_cameras
from .episode_spool import StreamingEpisodeSpool
from .keyframe_snapshot import KEYFRAME_DIR_NAME

FIXED_CAPTURE_FPS = 30
GUI_CAMERA_READ_TIMEOUT_MS = 3000
HIGH_QUALITY_DIR = "High_Quality"
LOW_QUALITY_DIR = "Low_Quality"
FAILURE_DIR = "Failure"
QUALITY_DIRS = (HIGH_QUALITY_DIR, LOW_QUALITY_DIR, FAILURE_DIR)
NAS_METADATA_CACHE_TTL_SEC = 10.0
DISK_USAGE_CACHE_TTL_SEC = 60.0
DEFAULT_MIN_LOCAL_FREE_GIB = 20.0
DEFAULT_MAX_LOCAL_SAVE_BACKLOG = 2
DEFAULT_ROBOT_STATE_MAX_AGE_MS = 250.0


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
    direct_to_output_root: bool = False
    operator_name: str = ""
    capture_profile: str = ""

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
    camera_names: tuple[str, ...]
    camera_metadata: Dict[str, Any]
    video_fps: int
    frames: List[Dict[str, Any]]
    keyframes: List[int]
    spool: StreamingEpisodeSpool


class _LatestFrameMailbox:
    """A single-slot handoff that never queues stale preview frames."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: Dict[str, np.ndarray] | None = None

    def publish(self, frames: Dict[str, np.ndarray]) -> None:
        with self._lock:
            self._frames = dict(frames)

    def take(self) -> Dict[str, np.ndarray] | None:
        with self._lock:
            frames = self._frames
            self._frames = None
        return frames

    def clear(self) -> None:
        with self._lock:
            self._frames = None


class CaptureThread(QtCore.QThread):
    status_changed = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    cameras_ready = QtCore.pyqtSignal(list)
    recording_start_rejected = QtCore.pyqtSignal(str)
    recording_started = QtCore.pyqtSignal(str, int)
    recording_paused = QtCore.pyqtSignal(str, int, int)
    recording_resumed = QtCore.pyqtSignal(str, int)
    recording_frame_count = QtCore.pyqtSignal(str, int, int)
    episode_ready = QtCore.pyqtSignal(object)
    episode_discarded = QtCore.pyqtSignal(str, int, int)
    capture_interrupted = QtCore.pyqtSignal(str, int, int)

    def __init__(
        self,
        options: CaptureOptions,
        preview_mailbox: _LatestFrameMailbox | None = None,
        parent: QtCore.QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.options = options
        self._preview_mailbox = preview_mailbox or _LatestFrameMailbox()
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
                self.status_changed.emit("没有检测到可用 RealSense；相机预览为空，无法开始录制")
            elif missing_camera_names:
                self.status_changed.emit(
                    f"相机预览已启动；缺失相机 {missing_camera_names} 恢复前禁止开始录制"
                )
            else:
                self.status_changed.emit("相机预览已启动")

            while not self._stop_requested:
                loop_started = time.monotonic()
                self._drain_commands(camera_names, camera_metadata)
                if self._stop_requested:
                    break
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
                    self.cameras_ready.emit(camera_names)

                self._pause_for_missing_episode_cameras(rgb_frames)
                self._preview_mailbox.publish(rgb_frames)
                command_crossed_camera_batch = self._drain_commands(
                    camera_names, camera_metadata
                )

                if (
                    not command_crossed_camera_batch
                    and self._recording
                    and self._active is not None
                ):
                    try:
                        frame = self._build_record_frame(rgb_frames)
                        self._active.spool.append(rgb_frames)
                    except Exception as exc:
                        self._recording = False
                        self.error.emit(f"采集或本地视频写入失败，已暂停当前 episode: {exc}")
                        self.recording_paused.emit(
                            self._active.task,
                            self._active.index,
                            len(self._active.frames),
                        )
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
            if self._active is not None:
                self._recording = False
                active = self._active
                cleanup_error = self._discard_spool(active)
                if active.frames:
                    if cleanup_error is None:
                        self.error.emit("采集线程停止时仍有未保存 episode，本地临时数据已清理。")
                    else:
                        self.error.emit(
                            "采集线程停止时仍有未保存 episode；writer 未能及时退出，"
                            f"临时目录已保留在 {active.spool.session_dir}: {cleanup_error}"
                        )
                self.capture_interrupted.emit(
                    active.task,
                    active.index,
                    len(active.frames),
                )
                self._active = None
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
            self._preview_mailbox.clear()
            self.status_changed.emit("相机预览已停止")

    def _drain_commands(
        self,
        camera_names: List[str],
        camera_metadata: Dict[str, Any],
    ) -> bool:
        crossed_camera_batch = False
        while True:
            try:
                command, payload = self._commands.get_nowait()
            except queue.Empty:
                return crossed_camera_batch

            if command == "stop":
                self._stop_requested = True
                return True
            if command == "start":
                crossed_camera_batch = True
                self._start_or_resume(payload, camera_names, camera_metadata)
            elif command == "pause":
                crossed_camera_batch = True
                self._pause()
            elif command == "finish":
                crossed_camera_batch = True
                self._finish()
            elif command == "discard":
                crossed_camera_batch = True
                self._discard()
            elif command == "keyframe":
                self._add_keyframe()

    def _start_or_resume(
        self,
        payload: Dict[str, Any],
        camera_names: List[str],
        camera_metadata: Dict[str, Any],
    ) -> None:
        episode_camera_names = (
            self._active.camera_names
            if self._active is not None
            else tuple(_selected_camera_names(self.options))
        )
        missing = [name for name in episode_camera_names if name not in camera_names]
        if missing:
            self._reject_start(
                f"录制需要配置的全部相机；当前缺失 {missing}。"
                "已拒绝开始或恢复，避免 episode 相机集合变化。"
            )
            return
        missing_metadata = [
            name for name in episode_camera_names if name not in camera_metadata
        ]
        if self._active is None and missing_metadata:
            self._reject_start(
                f"相机 metadata 不完整，缺失 {missing_metadata}；录制未开始。"
            )
            return

        if self._active is None:
            spool_cache_root = None
            if self.options.direct_to_output_root:
                spool_cache_root = Path(payload["output_root"]).expanduser()
                if (
                    not self.options.mock
                    and (not spool_cache_root.is_dir() or not os.path.ismount(spool_cache_root))
                ):
                    self._reject_start(
                        "NAS 保存根目录未挂载，已拒绝开始录制，避免写入同名本地目录: "
                        f"{spool_cache_root}"
                    )
                    return
            try:
                spool = StreamingEpisodeSpool(
                    camera_names=episode_camera_names,
                    video_fps=int(payload["video_fps"]),
                    cache_root=spool_cache_root,
                )
            except Exception as exc:
                location = "NAS staging" if self.options.direct_to_output_root else "本地"
                self._reject_start(f"无法初始化{location} episode spool，录制未开始: {exc}")
                return
            self._active = ActiveEpisode(
                task=payload["task"],
                task_description=payload.get("task_description", ""),
                user_metadata=dict(payload.get("user_metadata", {})),
                index=int(payload["index"]),
                output_root=payload["output_root"],
                started_at=time.time(),
                camera_names=episode_camera_names,
                camera_metadata={
                    name: deepcopy(camera_metadata[name]) for name in episode_camera_names
                },
                video_fps=int(payload["video_fps"]),
                frames=[],
                keyframes=[0],
                spool=spool,
            )
            self.recording_started.emit(self._active.task, self._active.index)
        else:
            self.recording_resumed.emit(self._active.task, self._active.index)
        self._recording = True

    def _reject_start(self, message: str) -> None:
        self.error.emit(message)
        self.recording_start_rejected.emit(message)

    def _pause(self) -> None:
        if self._active is None:
            return
        self._recording = False
        self.recording_paused.emit(self._active.task, self._active.index, len(self._active.frames))

    def _pause_for_missing_episode_cameras(
        self, rgb_frames: Dict[str, np.ndarray]
    ) -> bool:
        if not self._recording or self._active is None:
            return False
        missing = [name for name in self._active.camera_names if name not in rgb_frames]
        if not missing:
            return False
        self._recording = False
        self.error.emit(
            f"当前 episode 相机掉线 {missing}，已在完整前缀处暂停；"
            "相机集合和已录制 metadata 保持不变。"
        )
        self.recording_paused.emit(
            self._active.task,
            self._active.index,
            len(self._active.frames),
        )
        return True

    def _finish(self) -> None:
        if self._active is None:
            return
        active = self._active
        self._recording = False
        if not active.frames:
            cleanup_error = self._discard_spool(active)
            self._active = None
            if cleanup_error is not None:
                self.error.emit(
                    "空 episode 已停止，但 writer 未能及时退出；临时目录已保留: "
                    f"{active.spool.session_dir}\n{cleanup_error}"
                )
            self.episode_discarded.emit(active.task, active.index, 0)
            return
        try:
            image_storage = active.spool.finish()
        except Exception as exc:
            cleanup_error = self._discard_spool(active, close_already_failed=True)
            self._active = None
            if cleanup_error is None:
                self.error.emit(
                    f"本地视频未能完整落盘，当前 episode 已丢弃，避免保存不同步数据: {exc}"
                )
            else:
                self.error.emit(
                    "本地视频未能完整落盘且 writer 未能及时退出；未删除仍可能在写入的目录。"
                    f"临时目录: {active.spool.session_dir}\nfinish: {exc}\ncleanup: {cleanup_error}"
                )
            self.episode_discarded.emit(active.task, active.index, len(active.frames))
            return
        self._active = None
        request = EpisodeSaveRequest(
            output_root=active.output_root,
            task=active.task,
            index=active.index,
            frames=active.frames,
            keyframes=_valid_keyframes(active.keyframes, len(active.frames)),
            camera_names=list(active.camera_names),
            video_fps=active.video_fps,
            text_instruction=active.task_description,
            local_cache_dir=str(active.spool.episode_dir),
            direct_to_output_root=self.options.direct_to_output_root,
            metadata={
                "source": "franka_gui",
                "operator_name": self.options.operator_name,
                "schema_version": (
                    DUAL_VIDEO_SCHEMA_VERSION
                    if self.options.mode == "dual"
                    else SINGLE_VIDEO_SCHEMA_VERSION
                ),
                "image_storage": image_storage,
                "task_description": active.task_description,
                "user_metadata": active.user_metadata,
                "started_at_unix": active.started_at,
                "ended_at_unix": time.time(),
                **self._robot_metadata(),
                **gripper_metadata(),
                "cameras": deepcopy(active.camera_metadata),
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
        cleanup_error = self._discard_spool(active)
        if cleanup_error is not None:
            self.error.emit(
                "episode 已停止，但本地 writer 未能及时退出；临时目录已保留，未边写边删除: "
                f"{active.spool.session_dir}\n{cleanup_error}"
            )
        self.episode_discarded.emit(active.task, active.index, len(active.frames))

    @staticmethod
    def _discard_spool(
        active: ActiveEpisode,
        *,
        close_already_failed: bool = False,
    ) -> Exception | None:
        if close_already_failed and active.spool.writer_alive:
            return RuntimeError("video writer is still running after the close timeout")
        try:
            active.spool.discard()
        except Exception as exc:
            return exc
        return None

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
            "schema_version": SINGLE_VIDEO_SCHEMA_VERSION,
            "frame_index": len(self._active.frames) if self._active is not None else 0,
            "timestamp": now,
            **state,
        }
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
            "schema_version": DUAL_VIDEO_SCHEMA_VERSION,
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
    status_changed = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    cameras_ready = QtCore.pyqtSignal(list)
    active_episode_changed = QtCore.pyqtSignal(str, int, str)
    recording_frame_count = QtCore.pyqtSignal(int)
    save_queue_changed = QtCore.pyqtSignal(int)
    validation_status_changed = QtCore.pyqtSignal(str)
    episode_saved = QtCore.pyqtSignal(str, int, str, int)
    work_event = QtCore.pyqtSignal(dict)

    def __init__(self, options: CaptureOptions, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.options = options
        self.saver = AsyncEpisodeSaver(parent=self)
        self.thread: CaptureThread | None = None
        self._preview_mailbox = _LatestFrameMailbox()
        self._reserved: Dict[tuple[str, str], set[int]] = {}
        self._active_task: str | None = None
        self._active_index: int | None = None
        self._active_state = "idle"
        self._start_pending = False
        self._pending_quality_request: EpisodeSaveRequest | None = None
        self._deferred_quality_retry_requests: List[EpisodeSaveRequest] = []
        self._saving_quality_requests: Dict[tuple[str, str, int], EpisodeSaveRequest] = {}
        self._task_cache: List[str] = []
        self._task_cache_at = 0.0
        self._index_cache: Dict[tuple[str, str], tuple[float, int]] = {}
        self._disk_usage_cache = None
        self._disk_usage_cache_at = 0.0
        self._stale_cache_count = self._reserve_stale_cache_indices()
        self._stale_cache_notice_emitted = False
        self._work_attempt_id = ""
        self._activity_marker = (
            _record_cache_root() / ".capture-active" / f"{os.getpid()}-{uuid.uuid4().hex}.json"
        )
        self._activity_marker_at = 0.0

        self.saver.queue_changed.connect(self.save_queue_changed)
        self.saver.save_started.connect(
            lambda task, index, output_dir: self.status_changed.emit(
                (
                    f"NAS staging 保存开始: {task}/{index}；目标为 {output_dir}"
                    if self.options.direct_to_output_root
                    else f"本地提交开始: {task}/{index}；NAS 目标为 {output_dir}"
                )
            )
        )
        self.saver.save_finished.connect(self._on_save_finished)
        self.saver.save_failed.connect(self._on_save_failed)
        self.saver.validation_started.connect(self._on_validation_started)
        self.saver.validation_finished.connect(self._on_validation_finished)

    def start_preview(self) -> None:
        if self.thread is not None and self.thread.isRunning():
            self._emit_stale_cache_notice()
            return
        self._preview_mailbox.clear()
        self.thread = CaptureThread(
            self.options,
            preview_mailbox=self._preview_mailbox,
            parent=self,
        )
        self.thread.status_changed.connect(self.status_changed)
        self.thread.error.connect(self.error)
        self.thread.cameras_ready.connect(self.cameras_ready)
        self.thread.recording_start_rejected.connect(self._on_recording_start_rejected)
        self.thread.recording_started.connect(self._on_recording_started)
        self.thread.recording_resumed.connect(self._on_recording_resumed)
        self.thread.recording_paused.connect(self._on_recording_paused)
        self.thread.recording_frame_count.connect(self._on_frame_count)
        self.thread.episode_ready.connect(self._on_episode_ready)
        self.thread.episode_discarded.connect(self._on_episode_discarded)
        self.thread.capture_interrupted.connect(self._on_capture_interrupted)
        self.thread.finished.connect(lambda: self.status_changed.emit("采集线程已退出"))
        self.thread.start()
        self._emit_stale_cache_notice()

    def stop_preview(self) -> bool:
        if self.thread is None:
            return True
        self.thread.stop()
        self.thread.wait(10000)
        if self.thread.isRunning():
            self.error.emit(
                "相机线程未能在 10 秒内安全退出；已拒绝强制终止，避免损坏本地视频。"
            )
            return False
        self.thread = None
        self._preview_mailbox.clear()
        return True

    def take_preview_frame(self) -> Dict[str, np.ndarray] | None:
        return self._preview_mailbox.take()

    def start_or_resume(
        self,
        task: str,
        task_description: str = "",
        user_metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        try:
            task = _validate_path_token(task, "task")
        except ValueError as exc:
            self.error.emit(str(exc))
            return
        if task == KEYFRAME_DIR_NAME:
            self.error.emit(f"任务名称 {KEYFRAME_DIR_NAME!r} 为相机帧目录保留名称，不能用于 episode。")
            return
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
            self.status_changed.emit("当前 episode 等待质量分层，请先按 h、l 或 f。")
            return
        if self._active_state in {"finishing", "discarding"}:
            self.status_changed.emit("当前 episode 正在结束或丢弃处理中，请稍等。")
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
        if not self._has_local_capture_capacity():
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
            self.status_changed.emit("正在结束当前 episode，随后请按 h、l 或 f 完成质量分层。")
            self.thread.enqueue("finish")

    def discard(self) -> None:
        if self._active_state == "quality_pending" and self._pending_quality_request is not None:
            request = self._pending_quality_request
            self._pending_quality_request = None
            self._emit_work_event("attempt_discarded", request.work_attempt_id)
            if request.work_attempt_id == self._work_attempt_id:
                self._work_attempt_id = ""
            if request.quality in QUALITY_DIRS:
                self._release_reserved(request.task, request.index, request.quality)
            self._cleanup_request_cache(request)
            self._start_pending = False
            self._active_task = None
            self._active_index = None
            self._active_state = "idle"
            self.active_episode_changed.emit(request.task, request.index, "discarded")
            self.status_changed.emit(
                f"已丢弃待分层 episode: {request.task}, frames={len(request.frames)}"
            )
            self._sync_activity_marker_to_state()
            self._maybe_restore_deferred_quality_request()
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

    def mark_failure(self) -> None:
        self._save_pending_quality(FAILURE_DIR)

    def scan_tasks(self) -> List[str]:
        now = time.monotonic()
        if now - self._task_cache_at <= NAS_METADATA_CACHE_TTL_SEC:
            return list(self._task_cache)

        root = Path(self.options.output_root).expanduser()
        try:
            if not root.exists():
                tasks: List[str] = []
            else:
                tasks = sorted(
                    [
                        path.name
                        for path in root.iterdir()
                        if path.is_dir()
                        and not path.name.startswith(".")
                        and path.name != KEYFRAME_DIR_NAME
                    ]
                )
        except OSError:
            return list(self._task_cache)

        tasks = sorted(
            ({*tasks, *[item[0] for item in _pending_outbox_entries()]})
            - {KEYFRAME_DIR_NAME}
        )
        self._task_cache = tasks
        self._task_cache_at = now
        return list(tasks)

    def peek_next_episode_index(self, task: str, quality: str = HIGH_QUALITY_DIR) -> int:
        return self._cached_next_episode_index(task, quality)

    def refresh_next_episode_indices(self, task: str) -> Dict[str, int]:
        return {
            quality: self._refresh_next_episode_index(task, quality)
            for quality in QUALITY_DIRS
        }

    def disk_usage(self):
        now = time.monotonic()
        if (
            self._disk_usage_cache is not None
            and now - self._disk_usage_cache_at <= DISK_USAGE_CACHE_TTL_SEC
        ):
            return self._disk_usage_cache

        try:
            storage_root = (
                Path(self.options.output_root).expanduser()
                if self.options.direct_to_output_root
                else _record_cache_root()
            )
            if self.options.direct_to_output_root:
                if not storage_root.is_dir() or not os.path.ismount(storage_root):
                    raise OSError(f"NAS root is not mounted: {storage_root}")
            else:
                storage_root.mkdir(parents=True, exist_ok=True)
            usage = shutil.disk_usage(storage_root)
        except OSError:
            if self._disk_usage_cache is not None:
                return self._disk_usage_cache
            raise

        self._disk_usage_cache = usage
        self._disk_usage_cache_at = now
        return usage

    def deferred_quality_retry_count(self) -> int:
        return len(self._deferred_quality_retry_requests)

    def inflight_save_count(self) -> int:
        return max(self.saver.pending_count(), len(self._saving_quality_requests))

    def stale_cache_count(self) -> int:
        return self._stale_cache_count

    def pending_sync_count(self) -> int:
        outbox_root = _record_cache_root() / "outbox"
        try:
            return sum(
                1
                for entry in outbox_root.iterdir()
                if entry.is_dir()
                and (entry / "READY").is_file()
                and (entry / "outbox.json").is_file()
            )
        except OSError:
            return 0

    def has_open_episode_transition(self) -> bool:
        return self._start_pending or self._active_state in {"finishing", "discarding"}

    def shutdown(self) -> bool:
        if not self.stop_preview():
            return False
        self.saver.shutdown()
        self._remove_activity_marker()
        return True

    def _allocate_episode_index(self, task: str, quality: str) -> int:
        next_index = self._cached_next_episode_index(task, quality)
        self._reserved.setdefault((task, quality), set()).add(next_index)
        return next_index

    def _cached_next_episode_index(self, task: str, quality: str) -> int:
        cached = self._index_cache.get((task, quality))
        next_index = cached[1] if cached is not None else 0
        return self._apply_reserved_index(task, quality, next_index)

    def _refresh_next_episode_index(self, task: str, quality: str) -> int:
        try:
            task = _validate_path_token(task, "task")
            quality = _validate_path_token(quality, "quality")
        except ValueError:
            return self._cached_next_episode_index(task, quality)
        root = Path(self.options.output_root).expanduser() / task / quality
        cache_key = (task, quality)
        now = time.monotonic()

        try:
            existing: List[int] = []
            if root.exists():
                for child in root.iterdir():
                    if child.is_dir() and child.name.isdigit():
                        existing.append(int(child.name))
            existing.extend(
                index
                for pending_task, pending_quality, index in _pending_outbox_entries()
                if pending_task == task and pending_quality == quality
            )
            next_index = max(existing, default=-1) + 1
        except OSError:
            cached = self._index_cache.get(cache_key)
            next_index = cached[1] if cached is not None else 0

        self._index_cache[cache_key] = (now, next_index)
        return self._apply_reserved_index(task, quality, next_index)

    def _apply_reserved_index(self, task: str, quality: str, next_index: int) -> int:
        reserved = self._reserved.setdefault((task, quality), set())
        return max([max(0, int(next_index)) - 1, *reserved], default=-1) + 1

    def _next_display_episode_index(self, task: str) -> int:
        return min(self._cached_next_episode_index(task, quality) for quality in QUALITY_DIRS)

    def _release_reserved(self, task: str, index: int, quality: str) -> None:
        self._reserved.setdefault((task, quality), set()).discard(index)

    def _on_recording_started(self, task: str, index: int) -> None:
        self._start_pending = False
        self._active_task = task
        self._active_index = index
        self._active_state = "recording"
        self.active_episode_changed.emit(task, index, "recording")
        self.status_changed.emit(f"开始录制: {task}/{index}")
        self._work_attempt_id = uuid.uuid4().hex
        self._emit_work_event(
            "attempt_start",
            self._work_attempt_id,
            operator_name=self.options.operator_name,
            mode=self.options.capture_profile or self.options.mode,
            task=task,
            display_index=index,
        )
        self._refresh_activity_marker(force=True)

    def _on_recording_start_rejected(self, message: str) -> None:
        self._start_pending = False
        if self._active_state not in {"paused", "recording"}:
            self._active_state = "idle"
        self.status_changed.emit(f"录制未开始: {message}")
        self._sync_activity_marker_to_state()

    def _on_recording_resumed(self, task: str, index: int) -> None:
        self._start_pending = False
        self._active_state = "recording"
        self.active_episode_changed.emit(task, index, "recording")
        self.status_changed.emit(f"继续录制: {task}/{index}")
        self._emit_work_event("recording_resumed", self._work_attempt_id)
        self._refresh_activity_marker(force=True)

    def _on_recording_paused(self, task: str, index: int, frames: int) -> None:
        self._active_state = "paused"
        self.active_episode_changed.emit(task, index, "paused")
        self.status_changed.emit(f"暂停录制: {task}/{index}, frames={frames}")
        self._emit_work_event("recording_paused", self._work_attempt_id)
        self._refresh_activity_marker(force=True)

    def _on_frame_count(self, task: str, index: int, frames: int) -> None:
        self.recording_frame_count.emit(frames)
        self._refresh_activity_marker()

    def _on_episode_ready(self, request: EpisodeSaveRequest) -> None:
        request = replace(request, work_attempt_id=self._work_attempt_id)
        self._pending_quality_request = request
        self._active_task = request.task
        self._active_index = request.index
        self._active_state = "quality_pending"
        self.active_episode_changed.emit(request.task, request.index, "quality_pending")
        self.status_changed.emit(
            f"episode {request.task}/{request.index} 等待质量分层："
            "按 h 保存到高质量，按 l 保存到低质量，按 f 保存到 Failure，按 d 丢弃。"
        )
        self._emit_work_event("recording_finished", request.work_attempt_id)
        self._refresh_activity_marker(force=True)

    def _on_episode_discarded(self, task: str, index: int, frames: int) -> None:
        self._emit_work_event("attempt_discarded", self._work_attempt_id)
        self._work_attempt_id = ""
        self._start_pending = False
        self._active_task = None
        self._active_index = None
        self._active_state = "idle"
        self.active_episode_changed.emit(task, index, "discarded")
        self.status_changed.emit(f"已丢弃: {task}/{index}, frames={frames}")
        self._sync_activity_marker_to_state()
        self._maybe_restore_deferred_quality_request()

    def _on_capture_interrupted(self, task: str, index: int, frames: int) -> None:
        self._emit_work_event("attempt_interrupted", self._work_attempt_id)
        self._work_attempt_id = ""
        self._start_pending = False
        self._pending_quality_request = None
        self._active_task = None
        self._active_index = None
        self._active_state = "idle"
        self.active_episode_changed.emit(task, index, "interrupted")
        self.status_changed.emit(
            f"采集线程异常结束，episode 已中断: {task}/{index}, frames={frames}"
        )
        self._sync_activity_marker_to_state()
        self._maybe_restore_deferred_quality_request()

    def _on_save_finished(self, task: str, index: int, output_dir: str, frame_count: int) -> None:
        quality = Path(output_dir).parent.name
        request = None
        if quality in QUALITY_DIRS:
            request = self._saving_quality_requests.pop((task, quality, index), None)
            self._release_reserved(task, index, quality)
            self._remember_saved_episode(task, quality, index)
        if request is not None:
            self._emit_work_event(
                "attempt_saved",
                request.work_attempt_id,
                quality=quality,
                episode_index=index,
                output_dir=output_dir,
            )
        self.episode_saved.emit(task, index, output_dir, frame_count)
        if self._active_task is None:
            if not self._maybe_restore_deferred_quality_request():
                self._active_state = "idle"
                self.active_episode_changed.emit(task, index, "saved")
        self.status_changed.emit(
            (
                f"NAS 保存完成: {output_dir}, frames={frame_count}"
                if self.options.direct_to_output_root
                else f"本地保存完成，已进入 NAS 延迟同步队列: {output_dir}, frames={frame_count}"
            )
        )
        self._sync_activity_marker_to_state()

    def _on_validation_started(self, task: str, index: int, output_dir: str) -> None:
        self.validation_status_changed.emit(f"核验中: {task}/{index}")

    def _on_validation_finished(
        self,
        task: str,
        index: int,
        status: str,
        issues: list,
    ) -> None:
        prefix = f"核验 {status}: {task}/{index}"
        details = [str(item) for item in issues if str(item)]
        if details:
            self.validation_status_changed.emit(prefix + " | " + " | ".join(details))
        else:
            self.validation_status_changed.emit(prefix)

    def _on_save_failed(
        self,
        task: str,
        index: int,
        output_dir: str,
        error_kind: str,
        error: str,
    ) -> None:
        quality = Path(output_dir).parent.name
        request = None
        if quality in QUALITY_DIRS:
            request = self._saving_quality_requests.pop((task, quality, index), None)
        if error_kind == SAVE_ERROR_VALIDATION_FAILED:
            if request is not None:
                self._emit_work_event(
                    "attempt_validation_failed",
                    request.work_attempt_id,
                )
                self._release_reserved(task, index, quality)
            if self._active_task is None:
                if not self._maybe_restore_deferred_quality_request():
                    self._active_state = "idle"
                    self.active_episode_changed.emit(task, index, "validation_failed")
            self.status_changed.emit(
                f"核验 FAIL，已丢弃未发布 episode: {task}/{quality}/{index}"
            )
            self._sync_activity_marker_to_state()
            return
        if request is not None:
            self._emit_work_event("save_error", request.work_attempt_id)
            self._release_reserved(task, index, quality)
            request = self._prepare_quality_retry_request(request)
            message = (
                "NAS 隐藏 staging 目录仍然保留，episode 已恢复到 JUDGING；"
                "请重新按 h/l/f 完成保存，或按 d 丢弃。"
                if self.options.direct_to_output_root
                else "本地视频缓存仍然保留，episode 已恢复到 JUDGING；"
                "请重新按 h/l/f 完成本地提交，或按 d 丢弃。"
            )
            if self._can_restore_quality_request():
                self._restore_quality_request(request, message)
            else:
                self._deferred_quality_retry_requests.append(request)
                self.status_changed.emit(message)
        elif self._active_task is None:
            self._active_state = "idle"
            self.active_episode_changed.emit(task, index, "save_failed")
        self.error.emit(f"保存失败: {task}/{index}\n{error}")
        self._sync_activity_marker_to_state()

    def _save_pending_quality(self, quality: str) -> None:
        if quality not in QUALITY_DIRS:
            raise ValueError(f"Unsupported episode quality: {quality}")
        if self._active_state != "quality_pending" or self._pending_quality_request is None:
            self.status_changed.emit("当前没有等待分层的 episode。")
            return

        request = self._pending_quality_request
        if not request.local_cache_dir:
            self.error.emit("待分层 episode 的本地视频缓存不存在，无法保存。")
            return
        if request.quality in QUALITY_DIRS:
            self._release_reserved(request.task, request.index, request.quality)
        index = self._allocate_episode_index(request.task, quality)
        metadata = dict(request.metadata)
        metadata["quality"] = quality
        request = replace(
            request,
            index=index,
            quality=quality,
            metadata=metadata,
            publish_from_cache=False,
        )
        request_key = (request.task, quality, request.index)
        self._saving_quality_requests[request_key] = request
        self._emit_work_event(
            "quality_selected",
            request.work_attempt_id,
            quality=quality,
            episode_index=request.index,
        )
        if request.work_attempt_id == self._work_attempt_id:
            self._work_attempt_id = ""
        self._pending_quality_request = None
        self._active_task = None
        self._active_index = None
        self._active_state = "saving"
        self.active_episode_changed.emit(request.task, request.index, "saving")
        self._refresh_activity_marker(force=True)
        self.status_changed.emit(
            (
                f"episode {request.task}/{quality}/{request.index} 正在保存到 NAS staging"
                if self.options.direct_to_output_root
                else f"episode {request.task}/{quality}/{request.index} 正在提交到本地同步队列"
            )
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

    def _has_local_capture_capacity(self) -> bool:
        configured_backlog = os.environ.get(
            "FRANKA_GUI_MAX_LOCAL_SAVE_BACKLOG", ""
        ).strip()
        try:
            max_backlog = (
                int(configured_backlog)
                if configured_backlog
                else DEFAULT_MAX_LOCAL_SAVE_BACKLOG
            )
            if max_backlog < 1:
                raise ValueError("backlog limit must be at least 1")
        except ValueError as exc:
            self.error.emit(f"本地保存积压上限配置无效: {exc}")
            return False
        pending_saves = self.inflight_save_count()
        if pending_saves >= max_backlog:
            self.error.emit(
                f"本地保存仍有 {pending_saves} 个 episode，已达到上限 {max_backlog}；"
                "已阻止开始新录制，请等待本地提交完成。"
            )
            return False

        configured = os.environ.get(
            "FRANKA_GUI_MIN_NAS_FREE_GIB" if self.options.direct_to_output_root
            else "FRANKA_GUI_MIN_LOCAL_FREE_GIB",
            "",
        ).strip()
        try:
            minimum_gib = float(configured) if configured else DEFAULT_MIN_LOCAL_FREE_GIB
            storage_root = (
                Path(self.options.output_root).expanduser()
                if self.options.direct_to_output_root
                else _record_cache_root()
            )
            if self.options.direct_to_output_root:
                if not storage_root.is_dir() or not os.path.ismount(storage_root):
                    raise OSError(f"NAS root is not mounted: {storage_root}")
            else:
                storage_root.mkdir(parents=True, exist_ok=True)
            usage = shutil.disk_usage(storage_root)
        except (OSError, ValueError) as exc:
            label = "NAS" if self.options.direct_to_output_root else "本地采集"
            self.error.emit(f"无法检查{label}磁盘空间: {exc}")
            return False
        minimum_bytes = max(1.0, minimum_gib) * 1024**3
        if usage.free < minimum_bytes:
            label = "NAS" if self.options.direct_to_output_root else "本地采集"
            self.error.emit(
                f"{label}磁盘空间不足，已阻止新 episode，避免录制中途损坏。"
                f"需要至少 {minimum_gib:.1f} GiB，当前可用 {usage.free / 1024**3:.1f} GiB。"
            )
            return False
        return True

    def _can_restore_quality_request(self) -> bool:
        return (
            self._active_task is None
            and self._pending_quality_request is None
            and self._active_state not in {"recording", "paused", "quality_pending", "finishing", "discarding"}
        )

    def _restore_quality_request(self, request: EpisodeSaveRequest, message: str) -> None:
        self._pending_quality_request = request
        self._active_task = request.task
        self._active_index = request.index
        self._active_state = "quality_pending"
        self.active_episode_changed.emit(request.task, request.index, "quality_pending")
        self.status_changed.emit(message)
        self._refresh_activity_marker(force=True)

    def _maybe_restore_deferred_quality_request(self) -> bool:
        if not self._deferred_quality_retry_requests or not self._can_restore_quality_request():
            return False
        request = self._deferred_quality_retry_requests.pop(0)
        self._restore_quality_request(
            request,
            f"之前后台保存失败的 episode 已恢复到 JUDGING，可重新按 h/l/f 或按 d 丢弃: "
            f"{request.task}/{request.index}",
        )
        return True

    def _prepare_quality_retry_request(
        self,
        request: EpisodeSaveRequest,
        preferred_quality: str | None = None,
    ) -> EpisodeSaveRequest:
        metadata = dict(request.metadata)
        metadata.pop("quality", None)
        if preferred_quality in QUALITY_DIRS:
            index = self._cached_next_episode_index(request.task, preferred_quality)
        else:
            index = self._next_display_episode_index(request.task)
        return replace(
            request,
            index=index,
            quality="",
            metadata=metadata,
            publish_from_cache=False,
        )

    def _cleanup_request_cache(self, request: EpisodeSaveRequest) -> None:
        if not request.local_cache_dir:
            return
        cache_dir = Path(request.local_cache_dir).expanduser()
        if request.direct_to_output_root:
            if cache_dir.name.startswith(".partial-"):
                shutil.rmtree(cache_dir, ignore_errors=True)
                return
            session_dir = cache_dir.parent
            if session_dir.parent.name == ".recording" and session_dir.exists():
                shutil.rmtree(session_dir, ignore_errors=True)
            return
        session_dir = cache_dir.parent
        if session_dir.exists():
            shutil.rmtree(session_dir, ignore_errors=True)

    def _remember_saved_episode(self, task: str, quality: str, index: int) -> None:
        if task and task not in self._task_cache:
            self._task_cache = sorted([*self._task_cache, task])
            self._task_cache_at = time.monotonic()
        cache_key = (task, quality)
        cached = self._index_cache.get(cache_key)
        cached_next = cached[1] if cached is not None else 0
        self._index_cache[cache_key] = (
            time.monotonic(),
            max(cached_next, int(index) + 1),
        )

    def _reserve_stale_cache_indices(self) -> int:
        saving_root = _record_cache_root() / ".saving"
        discovered: set[tuple[str, str, int]] = set()
        try:
            sessions = [path for path in saving_root.iterdir() if path.is_dir()]
        except OSError:
            sessions = []
        for session in sessions:
            try:
                task_dirs = [path for path in session.iterdir() if path.is_dir()]
            except OSError:
                continue
            for task_dir in task_dirs:
                task = task_dir.name
                if task in {"", ".", ".."}:
                    continue
                for quality in QUALITY_DIRS:
                    quality_dir = task_dir / quality
                    if not quality_dir.is_dir():
                        continue
                    try:
                        index_dirs = [path for path in quality_dir.iterdir() if path.is_dir()]
                    except OSError:
                        continue
                    for index_dir in index_dirs:
                        if index_dir.name.isdigit():
                            discovered.add((task, quality, int(index_dir.name)))
        for task, quality, index in _pending_outbox_entries():
            discovered.add((task, quality, index))
        for task, quality, index in discovered:
            self._reserved.setdefault((task, quality), set()).add(index)
        return len(discovered)

    def _emit_stale_cache_notice(self) -> None:
        if self._stale_cache_notice_emitted or self._stale_cache_count <= 0:
            return
        self._stale_cache_notice_emitted = True
        self.status_changed.emit(
            f"检测到 {self._stale_cache_count} 个等待 NAS 同步或旧版未发布的本地 episode，"
            f"已保留对应编号避免覆盖；缓存根目录: {_record_cache_root()}"
        )

    def _sync_activity_marker_to_state(self) -> None:
        active_states = {
            "recording",
            "paused",
            "finishing",
            "quality_pending",
            "saving",
            "discarding",
        }
        if self._active_state in active_states or self.saver.pending_count() > 0:
            self._refresh_activity_marker(force=True)
        else:
            self._remove_activity_marker()

    def _emit_work_event(self, event_type: str, attempt_id: str, **payload: Any) -> None:
        if not attempt_id:
            return
        self.work_event.emit(
            {
                "event_type": event_type,
                "attempt_id": attempt_id,
                "occurred_at": time.time(),
                "payload": payload,
            }
        )

    def _refresh_activity_marker(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._activity_marker_at < 1.0:
            return
        self._activity_marker_at = now
        try:
            self._activity_marker.parent.mkdir(parents=True, exist_ok=True)
            temp = self._activity_marker.with_suffix(".tmp")
            temp.write_text(
                json.dumps(
                    {
                        "pid": os.getpid(),
                        "state": self._active_state,
                        "updated_at_unix": time.time(),
                    },
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            temp.replace(self._activity_marker)
        except OSError:
            return

    def _remove_activity_marker(self) -> None:
        try:
            self._activity_marker.unlink(missing_ok=True)
        except OSError:
            return


def _selected_camera_names(options: CaptureOptions) -> List[str]:
    if options.camera_names:
        return list(options.camera_names)
    return list(DEFAULT_CAMERAS.keys())


def _record_cache_root() -> Path:
    configured = os.environ.get(CACHE_ROOT_ENV, "").strip()
    return Path(configured).expanduser() if configured else DEFAULT_CACHE_ROOT


def _pending_outbox_entries() -> List[tuple[str, str, int]]:
    entries: List[tuple[str, str, int]] = []
    outbox_root = _record_cache_root() / "outbox"
    try:
        sessions = [path for path in outbox_root.iterdir() if path.is_dir()]
    except OSError:
        return entries
    for session in sessions:
        manifest_path = session / "outbox.json"
        ready_path = session / "READY"
        if not ready_path.is_file():
            continue
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            task = _validate_path_token(manifest["task"], "task")
            quality = _validate_path_token(manifest["quality"], "quality")
            index = int(manifest["requested_index"])
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if quality in QUALITY_DIRS and index >= 0:
            entries.append((task, quality, index))
    return entries


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
    def __init__(
        self,
        host: str,
        port: int,
        timeout_ms: int,
        label: str,
        *,
        max_age_ms: float | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.label = label
        configured_max_age = os.environ.get(
            "FRANKA_GUI_ROBOT_STATE_MAX_AGE_MS", ""
        ).strip()
        if max_age_ms is None:
            max_age_ms = (
                float(configured_max_age)
                if configured_max_age
                else DEFAULT_ROBOT_STATE_MAX_AGE_MS
            )
        self.max_age_ms = float(max_age_ms)
        if not np.isfinite(self.max_age_ms) or self.max_age_ms <= 0:
            raise ValueError(f"Invalid robot state max age: {self.max_age_ms}")
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
                error = self._latest_error
                ready = self._latest_state is not None
            if error is not None:
                raise RuntimeError(f"{self.label} robot sampler failed: {error}")
            if ready:
                return
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
            error = self._latest_error
            if error is not None:
                raise RuntimeError(f"{self.label} robot sampler failed: {error}")
            if self._latest_state is None:
                raise RuntimeError(f"{self.label} robot sampler has no state yet")
            state = dict(self._latest_state)
        state["robot_sampler_error"] = ""
        sample_time = state.get("robot_state_sample_monotonic")
        if sample_time is None:
            raise RuntimeError(f"{self.label} robot sampler state has no monotonic timestamp")
        raw_age_ms = (time.monotonic() - float(sample_time)) * 1000.0
        if not np.isfinite(raw_age_ms):
            raise RuntimeError(f"{self.label} robot sampler timestamp is invalid")
        age_ms = max(0.0, raw_age_ms)
        if age_ms > self.max_age_ms:
            raise TimeoutError(
                f"{self.label} robot sampler state is stale: "
                f"age={age_ms:.1f} ms, limit={self.max_age_ms:.1f} ms"
            )
        state["robot_state_age_ms"] = age_ms
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
