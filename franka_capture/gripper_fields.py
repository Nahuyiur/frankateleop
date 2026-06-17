"""Shared gripper field semantics for capture, conversion, and replay."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np


MAX_GRIPPER_WIDTH = 0.09
GRIPPER_CLOSED_THRESHOLD = 0.5
GRIPPER_SEMANTICS = "continuous_closedness_with_binary_compat"
GRIPPER_01CLOSEDNESS_FIELD = "gripper_01closedness"


def as_scalar(value: Any) -> float:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError("Expected scalar value, got empty sequence")
        value = value[0]
    return float(value)


def clipped_closedness(value: Any) -> float:
    return float(np.clip(as_scalar(value), 0.0, 1.0))


def clipped_width(value: Any, max_width: float = MAX_GRIPPER_WIDTH) -> float:
    width = as_scalar(value)
    if width < -1e-4 or width > max_width + 1e-3:
        raise ValueError(
            "Recorded gripper target width is outside the configured range: "
            f"{width:.6f} m"
        )
    return float(np.clip(width, 0.0, max_width))


def closedness_to_binary(
    closedness: Any,
    threshold: float = GRIPPER_CLOSED_THRESHOLD,
) -> float:
    return 1.0 if clipped_closedness(closedness) >= threshold else 0.0


def closedness_to_width(
    closedness: Any,
    max_width: float = MAX_GRIPPER_WIDTH,
) -> float:
    return float(max_width * (1.0 - clipped_closedness(closedness)))


def width_to_closedness(
    width: Any,
    max_width: float = MAX_GRIPPER_WIDTH,
) -> float:
    return float(np.clip(1.0 - clipped_width(width, max_width) / max_width, 0.0, 1.0))


def prefixed_key(prefix: str, key: str) -> str:
    return f"{prefix}_{key}" if prefix else key


def frame_gripper_closedness(
    frame: Mapping[str, Any],
    prefix: str = "",
    max_width: float = MAX_GRIPPER_WIDTH,
) -> float:
    """Return continuous command closedness, with v1/legacy fallbacks."""

    closedness_key = prefixed_key(prefix, "gripper_closedness")
    raw_key = prefixed_key(prefix, "gripper_command_raw")
    target_width_key = prefixed_key(prefix, "gripper_target_width")
    closed01_key = prefixed_key(prefix, GRIPPER_01CLOSEDNESS_FIELD)
    gripper_key = prefixed_key(prefix, "gripper")
    width_key = prefixed_key(prefix, "gripper_width")

    if closedness_key in frame:
        return clipped_closedness(frame[closedness_key])
    if raw_key in frame:
        return clipped_closedness(frame[raw_key])
    if target_width_key in frame:
        return width_to_closedness(frame[target_width_key], max_width)
    if width_key in frame:
        return width_to_closedness(frame[width_key], max_width)
    if closed01_key in frame:
        return clipped_closedness(frame[closed01_key])
    if gripper_key in frame:
        value = as_scalar(frame[gripper_key])
        if -1e-4 <= value <= max_width + 1e-3:
            return width_to_closedness(value, max_width)
        if -1e-4 <= value <= 1.0 + 1e-4:
            return clipped_closedness(value)
        raise ValueError(f"Invalid recorded gripper value: {value}")
    raise KeyError(closedness_key)


def frame_gripper_01closedness(
    frame: Mapping[str, Any],
    prefix: str = "",
    threshold: float = GRIPPER_CLOSED_THRESHOLD,
    max_width: float = MAX_GRIPPER_WIDTH,
) -> float:
    closed01_key = prefixed_key(prefix, GRIPPER_01CLOSEDNESS_FIELD)
    if closed01_key in frame:
        return closedness_to_binary(frame[closed01_key], threshold)
    closed_key = prefixed_key(prefix, "gripper_closed")
    if closed_key in frame:
        return closedness_to_binary(frame[closed_key], threshold)
    legacy_gripper_key = prefixed_key(prefix, "gripper")
    if legacy_gripper_key in frame:
        value = as_scalar(frame[legacy_gripper_key])
        width_key = prefixed_key(prefix, "gripper_width")
        if width_key in frame and -1e-4 <= value <= 1.0 + 1e-4:
            return closedness_to_binary(value, threshold)
        if -1e-4 <= value <= max_width + 1e-3:
            return closedness_to_binary(width_to_closedness(value, max_width), threshold)
        if -1e-4 <= value <= 1.0 + 1e-4:
            return closedness_to_binary(value, threshold)
    return closedness_to_binary(frame_gripper_closedness(frame, prefix, max_width), threshold)


def frame_gripper_closed(
    frame: Mapping[str, Any],
    prefix: str = "",
    threshold: float = GRIPPER_CLOSED_THRESHOLD,
    max_width: float = MAX_GRIPPER_WIDTH,
) -> float:
    """Legacy alias for older converter code."""

    return frame_gripper_01closedness(frame, prefix, threshold, max_width)


def frame_gripper_target_width(
    frame: Mapping[str, Any],
    prefix: str = "",
    max_width: float = MAX_GRIPPER_WIDTH,
) -> float:
    """Return command target width for replay.

    Priority follows the replay contract:
    target_width > closedness > command_raw > 01closedness > legacy gripper > feedback width.
    """

    target_width_key = prefixed_key(prefix, "gripper_target_width")
    closedness_key = prefixed_key(prefix, "gripper_closedness")
    raw_key = prefixed_key(prefix, "gripper_command_raw")
    closed01_key = prefixed_key(prefix, GRIPPER_01CLOSEDNESS_FIELD)
    gripper_key = prefixed_key(prefix, "gripper")
    width_key = prefixed_key(prefix, "gripper_width")

    if target_width_key in frame:
        return clipped_width(frame[target_width_key], max_width)
    if closedness_key in frame:
        return closedness_to_width(frame[closedness_key], max_width)
    if raw_key in frame:
        return closedness_to_width(frame[raw_key], max_width)
    if closed01_key in frame:
        return closedness_to_width(frame[closed01_key], max_width)
    if gripper_key in frame:
        value = as_scalar(frame[gripper_key])
        if width_key not in frame and -1e-4 <= value <= max_width + 1e-3:
            return clipped_width(value, max_width)
        if -1e-4 <= value <= 1.0 + 1e-4:
            return closedness_to_width(value, max_width)
        raise ValueError(f"Invalid recorded gripper value: {value}")
    if width_key in frame:
        return clipped_width(frame[width_key], max_width)
    raise KeyError(target_width_key)


def frame_gripper_width(
    frame: Mapping[str, Any],
    prefix: str = "",
    max_width: float = MAX_GRIPPER_WIDTH,
) -> float:
    width_key = prefixed_key(prefix, "gripper_width")
    if width_key in frame:
        return clipped_width(frame[width_key], max_width)
    target_width_key = prefixed_key(prefix, "gripper_target_width")
    if target_width_key in frame:
        return clipped_width(frame[target_width_key], max_width)
    return closedness_to_width(frame_gripper_closedness(frame, prefix, max_width), max_width)


def observation_gripper_fields(
    robot_observations: Mapping[str, Any],
    joint_state: Any,
    *,
    max_width: float = MAX_GRIPPER_WIDTH,
    timestamp: float | None = None,
) -> dict[str, Any]:
    """Build capture-frame gripper fields from robot node observations."""

    if "gripper_closedness" in robot_observations:
        closedness = clipped_closedness(robot_observations["gripper_closedness"])
    elif "gripper_command_raw" in robot_observations:
        closedness = clipped_closedness(robot_observations["gripper_command_raw"])
    elif "gripper_target_width" in robot_observations:
        closedness = width_to_closedness(robot_observations["gripper_target_width"], max_width)
    elif "gripper_command" in robot_observations:
        closedness = clipped_closedness(robot_observations["gripper_command"])
    else:
        raise KeyError("gripper_closedness")

    gripper_01closedness = closedness_to_binary(closedness)
    target_width = clipped_width(
        robot_observations.get("gripper_target_width", closedness_to_width(closedness, max_width)),
        max_width,
    )
    joint_state_array = np.asarray(joint_state, dtype=float)
    width = clipped_width(
        robot_observations.get("gripper_width", joint_state_array[-1] * max_width),
        max_width,
    )
    fields: dict[str, Any] = {
        "gripper_closedness": closedness,
        GRIPPER_01CLOSEDNESS_FIELD: gripper_01closedness,
        "gripper_width": width,
        "gripper_target_width": target_width,
    }
    return fields


def gripper_metadata() -> dict[str, Any]:
    return {
        "gripper_semantics": GRIPPER_SEMANTICS,
        "gripper_closedness_field": "gripper_closedness",
        "gripper_01closedness_field": GRIPPER_01CLOSEDNESS_FIELD,
        "gripper_values": {"0": "open", "1": "closed"},
        "gripper_closed_threshold": float(GRIPPER_CLOSED_THRESHOLD),
        "gripper_width_field": "gripper_width",
        "gripper_target_width_field": "gripper_target_width",
    }
