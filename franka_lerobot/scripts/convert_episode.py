from __future__ import annotations

import argparse

from franka_lerobot.converter import (
    DEFAULT_FPS,
    DEFAULT_ROBOT_TYPE,
    ConversionError,
    convert_episode_dataset,
    default_episode_output_root,
    infer_task_description_from_episode,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert one franka_capture episode to LeRobot v2.1.")
    parser.add_argument("episode", help="Episode directory or .pkl.gz file.")
    parser.add_argument("--output-root", default=None, help="Output LeRobot dataset root.")
    parser.add_argument("--task-description", default=None, help="Task text stored in LeRobot metadata.")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Dataset/video FPS.")
    parser.add_argument("--robot-type", default=DEFAULT_ROBOT_TYPE, help="LeRobot robot_type metadata.")
    parser.add_argument("--overwrite", action="store_true", help="Replace output directory if it already exists.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = args.output_root or default_episode_output_root(args.episode)
    task_description = args.task_description or infer_task_description_from_episode(args.episode)

    print(f"Input episode: {args.episode}")
    print(f"Output dataset: {output_root}")
    print(f"Task description: {task_description}")
    print(f"FPS: {args.fps}")

    try:
        convert_episode_dataset(
            episode_path=args.episode,
            output_root=output_root,
            task_description=task_description,
            fps=args.fps,
            robot_type=args.robot_type,
            overwrite=args.overwrite,
        )
    except ConversionError as exc:
        raise SystemExit(f"Conversion failed:\n{exc}") from exc


if __name__ == "__main__":
    main()

