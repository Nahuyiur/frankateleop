"""Record synchronized dual-arm FR3 observations with configured cameras."""

from __future__ import annotations

import argparse
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, Optional

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_dual import (
    DEFAULT_CAMERAS,
    DEFAULT_LEFT_ROBOT,
    DEFAULT_RECORDING,
    DEFAULT_RIGHT_ROBOT,
    DUAL_VIDEO_SCHEMA_VERSION,
)
from franka_capture.core.robot_zmq_client import RobotZMQClient
from franka_capture.gripper_fields import gripper_metadata, observation_gripper_fields
from franka_capture.recording.episode_writer import EpisodeWriter
from franka_capture.recording.preview import concatenate_rgb_images, show_rgb_preview

FIXED_RECORDING_FPS = 30


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--index",
        type=int,
        default=None,
        help="Optional starting episode index. Defaults to max existing index + 1.",
    )
    parser.add_argument("--task", default="test")
    parser.add_argument("--output-root", default=DEFAULT_RECORDING.output_root)
    parser.add_argument("--left-host", default=DEFAULT_LEFT_ROBOT.host)
    parser.add_argument("--left-port", type=int, default=DEFAULT_LEFT_ROBOT.port)
    parser.add_argument("--right-host", default=DEFAULT_RIGHT_ROBOT.host)
    parser.add_argument("--right-port", type=int, default=DEFAULT_RIGHT_ROBOT.port)
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=max(DEFAULT_LEFT_ROBOT.timeout_ms, DEFAULT_RIGHT_ROBOT.timeout_ms),
    )
    return parser.parse_args()


def _as_saved_value(value: Any):
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def _next_episode_index(output_root: str, task: str, start_index: Optional[int]) -> int:
    task_dir = Path(output_root).expanduser() / task
    existing_indices = []
    if task_dir.exists():
        for child in task_dir.iterdir():
            if child.is_dir() and child.name.isdigit():
                existing_indices.append(int(child.name))

    next_index = max(existing_indices, default=-1) + 1
    if start_index is not None:
        next_index = max(next_index, start_index)

    while (task_dir / str(next_index)).exists():
        next_index += 1
    return next_index


def _fixed_fps_camera_configs():
    return {
        name: replace(config, fps=FIXED_RECORDING_FPS, read_timeout_ms=3000)
        for name, config in DEFAULT_CAMERAS.items()
    }


def _extract_arm_state(
    robot_observations: Dict[str, Any],
    joint_state: Any,
    *,
    timestamp: float | None = None,
) -> Dict[str, Any]:
    if "ee_pose_euler" not in robot_observations:
        raise RuntimeError(
            "Robot node does not expose ee_pose_euler. "
            "Restart 3_launch_node.sh after updating teleop/teleop/robots/fr3.py."
        )
    if not any(
        key in robot_observations
        for key in (
            "gripper_closedness",
            "gripper_target_width",
        )
    ):
        raise RuntimeError(
            "Robot node does not expose continuous gripper fields. "
            "Restart 3_launch_node.sh after updating teleop/teleop/robots/fr3.py."
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


class RobotStateSampler:
    def __init__(self, host: str, port: int, timeout_ms: int, label: str) -> None:
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.label = label
        self.robot: RobotZMQClient | None = None
        self.num_dofs: int | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_state: Dict[str, Any] | None = None
        self._latest_error: str | None = None
        self._seq = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self.robot = RobotZMQClient(self.host, self.port, timeout_ms=self.timeout_ms)
        try:
            self.num_dofs = int(self.robot.num_dofs())
        except Exception:
            self.robot.close()
            self.robot = None
            raise
        if self.num_dofs != 8:
            self.robot.close()
            self.robot = None
            raise RuntimeError(f"{self.label} robot node DOF={self.num_dofs}, expected 8")
        self._thread = threading.Thread(
            target=self._run,
            name=f"dual-record-{self.label}-state",
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
        assert self.robot is not None
        period = 1.0 / FIXED_RECORDING_FPS
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                state = _fetch_arm_state(self.robot)
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
        if self.robot is not None:
            self.robot.close()
            self.robot = None


def _add_prefixed_fields(
    frame: Dict[str, Any],
    prefix: str,
    values: Dict[str, Any],
) -> None:
    for key, value in values.items():
        frame[f"{prefix}_{key}"] = value


def main() -> None:
    args = parse_args()
    import cv2

    cameras = {}
    left_sampler = None
    right_sampler = None
    writer = None
    record_flag = False
    frame_index = 0
    next_index = _next_episode_index(args.output_root, args.task, args.index)
    camera_metadata: Dict[str, Any] = {}

    def base_metadata():
        return {
            "source": "franka_capture.scripts.record_fr3_dual",
            "schema_version": DUAL_VIDEO_SCHEMA_VERSION,
            "started_at_unix": time.time(),
            "robots": {
                "left": {
                    "host": args.left_host,
                    "port": args.left_port,
                    "timeout_ms": args.timeout_ms,
                },
                "right": {
                    "host": args.right_host,
                    "port": args.right_port,
                    "timeout_ms": args.timeout_ms,
                },
            },
            "cameras": camera_metadata,
            **gripper_metadata(),
        }

    def start_episode() -> None:
        nonlocal writer, record_flag, next_index, frame_index
        if writer is None:
            frame_index = 0
            writer = EpisodeWriter(
                output_root=args.output_root,
                task=args.task,
                index=next_index,
                camera_names=camera_names,
                video_fps=FIXED_RECORDING_FPS,
                metadata=base_metadata(),
            )
            print(f"Start dual-arm recording episode {next_index}: {writer.output_dir}")
            next_index += 1
        else:
            print(f"Resume dual-arm recording episode {writer.index}: {writer.output_dir}")
        record_flag = True

    def save_episode(quiet: bool = False) -> None:
        nonlocal writer, record_flag
        if writer is None:
            if not quiet:
                print("No active episode to save")
            return
        record_flag = False
        writer.update_metadata({"ended_at_unix": time.time()})
        output_dir = writer.finish()
        print(f"Saved {len(writer.frames)} dual-arm frames to {output_dir}")
        writer = None

    def discard_episode() -> None:
        nonlocal writer, record_flag, next_index, frame_index
        if writer is None:
            print("No active episode to discard")
            return
        record_flag = False
        discarded_index = writer.index
        output_dir = writer.discard()
        next_index = min(next_index, discarded_index)
        frame_index = 0
        print(f"Discarded episode {discarded_index}: removed {output_dir}")
        print(f"Next episode index reset to {next_index}. Press s to record again.")
        writer = None

    try:
        camera_configs = _fixed_fps_camera_configs()
        cameras = create_realsense_cameras(camera_configs, allow_missing=True)
        camera_names = list(cameras.keys())
        camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}

        left_sampler = RobotStateSampler(args.left_host, args.left_port, args.timeout_ms, "left")
        right_sampler = RobotStateSampler(args.right_host, args.right_port, args.timeout_ms, "right")
        left_sampler.start()
        right_sampler.start()

        print(f"Left robot node: tcp://{args.left_host}:{args.left_port}")
        print(f"Right robot node: tcp://{args.right_host}:{args.right_port}")
        print(f"Left robot DOFs: {left_sampler.num_dofs}")
        print(f"Right robot DOFs: {right_sampler.num_dofs}")
        print(f"Connected cameras: {camera_names}")
        print(f"Schema: {DUAL_VIDEO_SCHEMA_VERSION}")
        print(f"Camera FPS: {FIXED_RECORDING_FPS}")
        print(f"Video FPS: {FIXED_RECORDING_FPS}")
        print(f"Task: {args.task}")
        print(f"Next episode index: {next_index}")
        if not camera_names:
            print("No RealSense cameras connected; recording robot state only.")
        print(
            "Click the RGB window first. "
            "s=start/resume, w=pause, e=end/save episode, "
            "d=discard episode, k=keyframe, q=quit/save."
        )

        while True:
            rgb_frames = {}
            for name, camera in cameras.items():
                rgb, _ = camera.read()
                rgb_frames[name] = rgb

            preview = concatenate_rgb_images(
                [rgb_frames[name] for name in camera_names],
                line_width=DEFAULT_RECORDING.preview_line_width,
            )
            key = show_rgb_preview("RGB", preview)

            if key in (ord("q"), ord("Q")):
                save_episode(quiet=True)
                break
            if key in (ord("s"), ord("S")):
                start_episode()
            if key in (ord("w"), ord("W")):
                record_flag = False
                print("Pause recording")
            if key in (ord("e"), ord("E")):
                save_episode()
            if key in (ord("d"), ord("D")):
                discard_episode()
            if key in (ord("k"), ord("K")):
                if not record_flag:
                    print("Keyframe ignored: recording is paused. Press s first.")
                else:
                    if writer is None:
                        start_episode()
                    keyframe = writer.add_keyframe()
                    print(f"Episode {writer.index} keyframe {keyframe} added")

            if not record_flag:
                continue
            if writer is None:
                start_episode()

            loop_start_monotonic = time.monotonic()
            loop_start_timestamp = time.time()
            left_state = left_sampler.snapshot()
            right_state = right_sampler.snapshot()
            loop_end_monotonic = time.monotonic()
            timestamp = time.time()

            frame = {
                "schema_version": DUAL_VIDEO_SCHEMA_VERSION,
                "frame_index": frame_index,
                "timestamp": timestamp,
                "loop_start_timestamp": loop_start_timestamp,
                "loop_end_timestamp": timestamp,
                "loop_start_monotonic": loop_start_monotonic,
                "loop_end_monotonic": loop_end_monotonic,
                "loop_duration_ms": (loop_end_monotonic - loop_start_monotonic) * 1000.0,
            }
            _add_prefixed_fields(frame, "left", left_state)
            _add_prefixed_fields(frame, "right", right_state)

            writer.append(frame, rgb_frames)
            frame_index += 1
    finally:
        if writer is not None:
            writer.close()
        cv2.destroyAllWindows()
        for camera in cameras.values():
            camera.close()
        if left_sampler is not None:
            left_sampler.close()
        if right_sampler is not None:
            right_sampler.close()


if __name__ == "__main__":
    main()
