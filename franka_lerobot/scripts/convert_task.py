from __future__ import annotations

import argparse

from franka_lerobot.converter import (
    DEFAULT_FPS,
    DEFAULT_ROBOT_TYPE,
    ConversionError,
    convert_task_dataset,
    default_task_output_root,
    infer_task_description_from_task_dir,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert a franka_capture task directory to LeRobot v2.1.")
    parser.add_argument("task_dir", help="Task directory, e.g. /home/pnp/Desktop/franka_record_data/pick_block.")
    parser.add_argument("--output-root", default=None, help="Output LeRobot dataset root.")
    parser.add_argument("--task-description", default=None, help="Task text stored in LeRobot metadata.")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Dataset/video FPS.")
    parser.add_argument("--robot-type", default=DEFAULT_ROBOT_TYPE, help="LeRobot robot_type metadata.")
    parser.add_argument("--overwrite", action="store_true", help="Replace output directory if it already exists.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = args.output_root or default_task_output_root(args.task_dir)
    task_description = args.task_description or infer_task_description_from_task_dir(args.task_dir)

    print(f"Input task directory: {args.task_dir}")
    print(f"Output dataset: {output_root}")
    print(f"Task description: {task_description}")
    print(f"FPS: {args.fps}")

    try:
        _, skipped = convert_task_dataset(
            task_dir=args.task_dir,
            output_root=output_root,
            task_description=task_description,
            fps=args.fps,
            robot_type=args.robot_type,
            overwrite=args.overwrite,
        )
    except ConversionError as exc:
        raise SystemExit(f"Conversion failed:\n{exc}") from exc

    if skipped:
        print("Skipped episode directories without .pkl.gz:")
        for path in skipped:
            print(f"  {path}")


if __name__ == "__main__":
    main()

