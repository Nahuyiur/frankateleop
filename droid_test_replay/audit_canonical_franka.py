#!/usr/bin/env python3
"""Measure canonical DROID-to-Franka geometric and command conversion errors."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from polymetis import RobotInterface
from scipy.spatial.transform import Rotation


def pose_matrix(pose: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(pose[3:]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def rotation_error(left: np.ndarray, right: np.ndarray) -> float:
    return float(Rotation.from_matrix(left.T @ right).magnitude())


def stats(values: list[float] | np.ndarray) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(array)),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
        "p99": float(np.quantile(array, 0.99)),
        "max": float(np.max(array)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sample", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=50051)
    parser.add_argument("--ik-samples", type=int, default=100)
    args = parser.parse_args()

    with np.load(args.sample, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    robot = RobotInterface(ip_address=args.host, port=args.robot_port)

    fk_position_errors = []
    fk_rotation_errors = []
    euler_position_errors = []
    euler_rotation_errors = []
    for joints, state_pose, mask in zip(
        data["joint_position"], data["state_pose"], data["joint_mask"]
    ):
        if not np.all(mask[:7]):
            raise RuntimeError("Invalid sampled arm joint mask")
        position, quaternion = robot.robot_model.forward_kinematics(
            torch.as_tensor(joints[:7], dtype=torch.float32)
        )
        fk_position = position.detach().cpu().numpy()
        fk_rotation = Rotation.from_quat(quaternion.detach().cpu().numpy()).as_matrix()
        canonical = pose_matrix(state_pose)
        fk_position_errors.append(np.linalg.norm(fk_position - canonical[:3, 3]))
        fk_rotation_errors.append(rotation_error(fk_rotation, canonical[:3, :3]))

        euler = Rotation.from_matrix(canonical[:3, :3]).as_euler("xyz")
        roundtrip = Rotation.from_euler("xyz", euler).as_matrix()
        euler_position_errors.append(np.linalg.norm(state_pose[:3] - canonical[:3, 3]))
        euler_rotation_errors.append(rotation_error(roundtrip, canonical[:3, :3]))

    transition_mask = data["has_next"] & np.all(data["delta_mask"], axis=1)
    delta_position_errors = []
    delta_rotation_errors = []
    delta_dt_errors = []
    delta_step_translation = []
    delta_step_rotation = []
    for state, delta, next_state, dt, timestamp, next_timestamp in zip(
        data["state_pose"][transition_mask],
        data["delta_pose"][transition_mask],
        data["next_state_pose"][transition_mask],
        data["delta_dt"][transition_mask],
        data["timestamp"][transition_mask],
        data["next_timestamp"][transition_mask],
    ):
        predicted = pose_matrix(state) @ pose_matrix(delta)
        actual = pose_matrix(next_state)
        delta_position_errors.append(np.linalg.norm(predicted[:3, 3] - actual[:3, 3]))
        delta_rotation_errors.append(rotation_error(predicted[:3, :3], actual[:3, :3]))
        delta_dt_errors.append(abs(dt - (next_timestamp - timestamp)))
        delta_step_translation.append(np.linalg.norm(delta[:3]))
        delta_step_rotation.append(np.linalg.norm(delta[3:]))

    unique_episodes = np.unique(data["episode_index"])
    ik_indices = []
    for episode in unique_episodes:
        episode_indices = np.flatnonzero(data["episode_index"] == episode)
        ik_indices.append(int(episode_indices[len(episode_indices) // 2]))
    ik_indices = ik_indices[: args.ik_samples]
    ik_success = 0
    ik_position_errors = []
    ik_rotation_errors = []
    for index in ik_indices:
        target = pose_matrix(data["state_pose"][index])
        position = torch.as_tensor(target[:3, 3], dtype=torch.float32)
        quaternion = torch.as_tensor(
            Rotation.from_matrix(target[:3, :3]).as_quat(), dtype=torch.float32
        )
        seed = torch.as_tensor(data["joint_position"][index, :7], dtype=torch.float32)
        solution, success = robot.solve_inverse_kinematics(position, quaternion, seed)
        ik_success += int(success)
        solved_position, solved_quaternion = robot.robot_model.forward_kinematics(solution)
        ik_position_errors.append(
            np.linalg.norm(solved_position.detach().cpu().numpy() - target[:3, 3])
        )
        ik_rotation_errors.append(
            rotation_error(
                Rotation.from_quat(solved_quaternion.detach().cpu().numpy()).as_matrix(),
                target[:3, :3],
            )
        )

    valid_gripper = data["gripper_opening"][data["gripper_mask"]]
    joint_lower = np.asarray([-2.64, -1.68, -2.80, -2.94, -2.70, 0.64, -2.91])
    joint_upper = np.asarray([2.64, 1.68, 2.80, -0.25, 2.70, 4.41, 2.91])
    joints = data["joint_position"][:, :7]
    joint_limit_violations = int(np.sum((joints < joint_lower) | (joints > joint_upper)))
    step_translation_array = np.asarray(delta_step_translation)
    step_rotation_array = np.asarray(delta_step_rotation)
    sampled_step_violation = (step_translation_array > 0.010) | (
        step_rotation_array > 0.050
    )
    transition_episodes = data["episode_index"][transition_mask]
    episodes_with_sampled_violation = len(
        np.unique(transition_episodes[sampled_step_violation])
    )

    report = {
        "sampled_episodes": int(len(unique_episodes)),
        "sampled_frames": int(len(data["episode_index"])),
        "sampled_valid_transitions": int(np.sum(transition_mask)),
        "joint_fk_vs_canonical": {
            "position_error_m": stats(fk_position_errors),
            "rotation_error_rad": stats(fk_rotation_errors),
        },
        "body_delta_reconstruction": {
            "position_error_m": stats(delta_position_errors),
            "rotation_error_rad": stats(delta_rotation_errors),
            "delta_dt_error_s": stats(delta_dt_errors),
        },
        "canonical_rotvec_to_franka_euler_roundtrip": {
            "position_error_m": stats(euler_position_errors),
            "rotation_error_rad": stats(euler_rotation_errors),
        },
        "franka_ik": {
            "samples": len(ik_indices),
            "successes": ik_success,
            "success_rate": ik_success / len(ik_indices),
            "position_error_m": stats(ik_position_errors),
            "rotation_error_rad": stats(ik_rotation_errors),
        },
        "sampled_action_magnitude": {
            "translation_m": stats(delta_step_translation),
            "rotation_rad": stats(delta_step_rotation),
        },
        "current_replay_step_limits": {
            "translation_limit_m": 0.010,
            "rotation_limit_rad": 0.050,
            "sampled_translation_violations": int(
                np.sum(step_translation_array > 0.010)
            ),
            "sampled_rotation_violations": int(
                np.sum(step_rotation_array > 0.050)
            ),
            "sampled_either_violations": int(np.sum(sampled_step_violation)),
            "sampled_transition_count": int(len(sampled_step_violation)),
            "episodes_with_at_least_one_sampled_violation": int(
                episodes_with_sampled_violation
            ),
        },
        "gripper": {
            "valid_samples": int(len(valid_gripper)),
            "opening_min": float(np.min(valid_gripper)),
            "opening_max": float(np.max(valid_gripper)),
            "mapped_width_min_m": float(0.085 * np.min(valid_gripper)),
            "mapped_width_max_m": float(0.085 * np.max(valid_gripper)),
        },
        "joint_limit_violations": joint_limit_violations,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
