"""Validated, file-based GELLO damping profiles used by the tuning GUI."""

import json
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence


PROFILE_VERSION = 1
VECTOR_FIELDS = (
    "torque_enable",
    "goal_current_ma",
    "kp_p",
    "kp_i",
    "kp_d",
    "goal_position",
)
GAIN_FIELDS = ("kp_p", "kp_i", "kp_d")


class ProfileError(ValueError):
    """Raised when a damping profile is malformed or outside safe bounds."""


@dataclass
class BinarySearchBounds:
    low: int
    high: int

    def __post_init__(self) -> None:
        if self.low < 0 or self.high < self.low:
            raise ProfileError("binary-search bounds require 0 <= low <= high")

    @property
    def midpoint(self) -> int:
        return (self.low + self.high) // 2

    def too_weak(self) -> int:
        self.low = min(self.high, self.midpoint + 1)
        return self.midpoint

    def too_strong(self) -> int:
        self.high = max(self.low, self.midpoint - 1)
        return self.midpoint


def nearest_safe_goals(
    current: Sequence[float], requested: Sequence[float], max_delta_rad: float
) -> List[float]:
    """Wrap requested positions to the nearest turn and enforce a start bound."""
    if len(current) != len(requested):
        raise ProfileError("current and requested goal lengths do not match")
    if max_delta_rad <= 0 or not math.isfinite(max_delta_rad):
        raise ProfileError("max_delta_rad must be a positive finite number")
    result = []
    for index, (current_value, requested_value) in enumerate(
        zip(current, requested), 1
    ):
        if not math.isfinite(float(current_value)) or not math.isfinite(
            float(requested_value)
        ):
            raise ProfileError("current and requested goals must be finite")
        delta = (
            (float(requested_value) - float(current_value) + math.pi)
            % (2.0 * math.pi)
            - math.pi
        )
        if abs(delta) > max_delta_rad:
            raise ProfileError(
                f"J{index} goal delta {delta:+.3f} rad exceeds startup limit "
                f"{max_delta_rad:.3f} rad; capture the current pose first"
            )
        result.append(float(current_value) + delta)
    return result


def _number_list(
    profile: Mapping[str, object], name: str, count: int
) -> List[float]:
    value = profile.get(name)
    if not isinstance(value, list) or len(value) != count:
        raise ProfileError(f"{name} must be a list of {count} values")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise ProfileError(f"{name} must contain only numbers")
    if any(not math.isfinite(float(item)) for item in value):
        raise ProfileError(f"{name} must contain only finite numbers")
    return [float(item) for item in value]


def validate_profile(
    profile: Mapping[str, object],
    joint_count: int = 7,
    max_current_ma: Optional[Sequence[int]] = None,
) -> Dict[str, object]:
    """Return a normalized profile or raise ``ProfileError``.

    ``max_current_ma`` can contain the Current Limit register read from each
    connected motor. It is deliberately optional so profiles can be edited on a
    laptop without any hardware attached.
    """
    if not isinstance(profile, Mapping):
        raise ProfileError("profile root must be an object")
    version = profile.get("version", PROFILE_VERSION)
    if version != PROFILE_VERSION:
        raise ProfileError(f"unsupported profile version: {version}")

    torque = _number_list(profile, "torque_enable", joint_count)
    if any(item not in (0.0, 1.0) for item in torque):
        raise ProfileError("torque_enable values must be 0 or 1")

    current = _number_list(profile, "goal_current_ma", joint_count)
    if any(item < 0 or int(item) != item for item in current):
        raise ProfileError("goal_current_ma values must be non-negative integers")
    if max_current_ma is not None:
        if len(max_current_ma) != joint_count:
            raise ProfileError("max_current_ma must contain one value per joint")
        for index, (requested, limit) in enumerate(zip(current, max_current_ma), 1):
            if requested > int(limit):
                raise ProfileError(
                    f"J{index} goal current {int(requested)} mA exceeds "
                    f"device Current Limit {int(limit)} mA"
                )

    gains = {}
    for name in GAIN_FIELDS:
        values = _number_list(profile, name, joint_count)
        if any(item < 0 or item > 16383 or int(item) != item for item in values):
            raise ProfileError(f"{name} values must be integers in [0, 16383]")
        gains[name] = [int(item) for item in values]

    goal = _number_list(profile, "goal_position", joint_count)
    normalized = dict(profile)
    normalized.update(
        {
            "version": PROFILE_VERSION,
            "torque_enable": [int(item) for item in torque],
            "goal_current_ma": [int(item) for item in current],
            "kp_p": gains["kp_p"],
            "kp_i": gains["kp_i"],
            "kp_d": gains["kp_d"],
            "goal_position": goal,
        }
    )
    return normalized


def load_profile(path: str, joint_count: int = 7) -> Dict[str, object]:
    with Path(path).expanduser().open("r", encoding="utf-8") as handle:
        profile = json.load(handle)
    return validate_profile(profile, joint_count=joint_count)


def save_profile(path: str, profile: Mapping[str, object], joint_count: int = 7) -> None:
    """Atomically save a validated profile without partially written JSON."""
    normalized = validate_profile(profile, joint_count=joint_count)
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=str(destination.parent)
    )
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as handle:
            json.dump(normalized, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
