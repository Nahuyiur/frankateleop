"""Helpers for launching replay from the GUI without duplicating replay logic."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

from franka_replay.replay_fr3 import (
    infer_episode_kind,
    load_episode,
    load_episode_metadata,
    resolve_episode_file,
)


@dataclass(frozen=True)
class ReplayEpisodeInfo:
    input_path: Path
    episode_file: Path
    episode_dir: Path
    kind: str
    arm_side: Optional[str]
    frame_count: Optional[int]
    schema_version: str

    @property
    def display_kind(self) -> str:
        if self.kind == "dual":
            return "dual"
        if self.arm_side:
            return self.arm_side
        return "single/unknown"

    @property
    def frame_count_text(self) -> str:
        return str(self.frame_count) if self.frame_count is not None else "unknown"


def inspect_replay_input(path_like: str, *, latest: bool = False) -> ReplayEpisodeInfo:
    input_path = Path(path_like).expanduser()
    episode_file = resolve_episode_file(str(input_path), latest=latest)
    metadata = load_episode_metadata(episode_file)
    metadata_info = _inspect_from_metadata(input_path, episode_file, metadata)
    if metadata_info is not None:
        return metadata_info

    _, frames = load_episode(episode_file)
    kind = infer_episode_kind(frames, metadata)
    schema_version = _schema_version_from_frames(frames, metadata)
    arm_side = _infer_single_arm_side_from_frames(frames, metadata, episode_file) if kind == "single" else None
    return ReplayEpisodeInfo(
        input_path=input_path,
        episode_file=episode_file,
        episode_dir=episode_file.parent,
        kind=kind,
        arm_side=arm_side,
        frame_count=len(frames),
        schema_version=schema_version,
    )


def validate_replay_target(info: ReplayEpisodeInfo, gui_mode: str) -> str:
    if gui_mode == "dual":
        if info.kind != "dual":
            raise ValueError(f"当前是双臂 GUI，但选择的数据是 {info.display_kind}。")
        return "dual"
    if gui_mode not in {"single", "right"}:
        raise ValueError(f"未知 GUI 模式: {gui_mode}")

    expected_arm = "right" if gui_mode == "right" else "left"
    if info.kind != "single":
        raise ValueError(f"当前是{_mode_label(gui_mode)} GUI，但选择的数据是 dual。")
    if info.arm_side is None:
        raise ValueError(
            "选择的是单臂数据，但无法可靠判断是左臂还是右臂。"
            "请改选包含 metadata.json 的 episode 目录，或选择相机/metadata 能标明 left/right 的数据。"
        )
    if info.arm_side != expected_arm:
        raise ValueError(
            f"当前是{_mode_label(gui_mode)} GUI，但选择的数据是 {info.arm_side} 臂。"
        )
    return expected_arm


def _inspect_from_metadata(
    input_path: Path,
    episode_file: Path,
    metadata: Dict[str, Any],
) -> Optional[ReplayEpisodeInfo]:
    schema_version = str(metadata.get("schema_version", "") or "")
    kind = _kind_from_schema(schema_version)
    if kind is None:
        return None

    arm_side = None
    if kind == "single":
        arm_side = _infer_single_arm_side_from_metadata(metadata)
        if arm_side is None:
            return None

    return ReplayEpisodeInfo(
        input_path=input_path,
        episode_file=episode_file,
        episode_dir=episode_file.parent,
        kind=kind,
        arm_side=arm_side,
        frame_count=_metadata_frame_count(metadata),
        schema_version=schema_version,
    )


def _kind_from_schema(schema_version: str) -> Optional[str]:
    if schema_version.startswith("franka_dual"):
        return "dual"
    if schema_version.startswith("franka_single"):
        return "single"
    return None


def _metadata_frame_count(metadata: Dict[str, Any]) -> Optional[int]:
    value = metadata.get("frame_count")
    if isinstance(value, bool):
        return None
    if isinstance(value, int) and value >= 0:
        return value
    return None


def _schema_version_from_frames(frames: list[Any], metadata: Dict[str, Any]) -> str:
    if metadata.get("schema_version"):
        return str(metadata["schema_version"])
    first_frame = frames[0] if frames and isinstance(frames[0], dict) else {}
    return str(first_frame.get("schema_version", "<missing>"))


def _infer_single_arm_side_from_metadata(metadata: Dict[str, Any]) -> Optional[str]:
    arm_side = str(metadata.get("arm_side", "") or "").strip().lower()
    if arm_side in {"left", "right"}:
        return arm_side
    return _infer_side_from_camera_names(_metadata_camera_names(metadata))


def _infer_single_arm_side_from_frames(
    frames: list[Any],
    metadata: Dict[str, Any],
    episode_file: Path,
) -> Optional[str]:
    side = _infer_single_arm_side_from_metadata(metadata)
    if side is not None:
        return side

    for frame in frames[:5]:
        if not isinstance(frame, dict):
            continue
        side = _infer_side_from_camera_names(_frame_camera_names(frame))
        if side is not None:
            return side

    return _infer_side_from_path(episode_file)


def _metadata_camera_names(metadata: Dict[str, Any]) -> set[str]:
    names: set[str] = set()
    camera_names = metadata.get("camera_names")
    if isinstance(camera_names, list):
        names.update(str(name) for name in camera_names)
    cameras = metadata.get("cameras")
    if isinstance(cameras, dict):
        names.update(str(name) for name in cameras)
    return names


def _frame_camera_names(frame: Dict[str, Any]) -> set[str]:
    names = set()
    for key in frame:
        if key.endswith("_image"):
            names.add(key[: -len("_image")])
        elif key.endswith("_depth"):
            names.add(key[: -len("_depth")])
    return names


def _infer_side_from_camera_names(names: set[str]) -> Optional[str]:
    has_left = bool({"left", "left_wrist"} & names)
    has_right = bool({"right", "right_wrist"} & names)
    if has_left and not has_right:
        return "left"
    if has_right and not has_left:
        return "right"
    return None


def _infer_side_from_path(path: Path) -> Optional[str]:
    parts = {_normalize_side_token(part) for part in path.parts}
    left_tokens = {"left", "left_arm", "leftarm", "single_left"}
    right_tokens = {"right", "right_arm", "rightarm", "single_right"}
    has_left = bool(parts & left_tokens)
    has_right = bool(parts & right_tokens)
    if has_left and not has_right:
        return "left"
    if has_right and not has_left:
        return "right"
    return None


def _normalize_side_token(value: str) -> str:
    return value.strip().lower().replace("-", "_")


def _mode_label(mode: str) -> str:
    if mode == "right":
        return "右臂"
    if mode == "dual":
        return "双臂"
    return "左臂"
