from __future__ import annotations

import argparse

from franka_hdf5.converter import (
    DEFAULT_FPS,
    DEFAULT_ROBOT_TYPE,
    ConversionError,
    convert_episode_file,
    default_episode_output_file,
    infer_task_description_from_episode,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert one franka_capture episode to one HDF5 file.")
    parser.add_argument("episode", help="Episode directory or .pkl.gz file.")
    parser.add_argument("--output-file", default=None, help="Output .hdf5 file.")
    parser.add_argument("--task-description", default=None, help="Task text stored in HDF5 attributes.")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Dataset FPS for relative timestamp.")
    parser.add_argument("--robot-type", default=DEFAULT_ROBOT_TYPE, help="Robot type metadata.")
    parser.add_argument("--camera", default=None, help="Optional single camera to keep, e.g. right.")
    parser.add_argument("--overwrite", action="store_true", help="Replace output file if it already exists.")
    parser.add_argument(
        "--compression",
        default="gzip",
        choices=["gzip", "lzf", "none"],
        help="HDF5 image dataset compression.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_file = args.output_file or default_episode_output_file(args.episode)
    task_description = args.task_description or infer_task_description_from_episode(args.episode)
    compression = None if args.compression == "none" else args.compression

    print(f"Input episode: {args.episode}")
    print(f"Output HDF5: {output_file}")
    print(f"Task description: {task_description}")
    print(f"FPS: {args.fps}")
    print(f"Camera: {args.camera or 'all'}")
    print(f"Compression: {args.compression}")

    try:
        convert_episode_file(
            episode_path=args.episode,
            output_file=output_file,
            task_description=task_description,
            fps=args.fps,
            robot_type=args.robot_type,
            overwrite=args.overwrite,
            compression=compression,
            camera=args.camera,
        )
    except ConversionError as exc:
        raise SystemExit(f"Conversion failed:\n{exc}") from exc


if __name__ == "__main__":
    main()
