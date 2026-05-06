from __future__ import annotations

import argparse
from pathlib import Path

from franka_downsample.downsample import (
    DEFAULT_CAMERA,
    DEFAULT_SOURCE_FPS,
    DEFAULT_TARGET_FPS,
    DownsampleError,
    downsample_task,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Downsample a franka_capture task to fixed-stride capture format.")
    parser.add_argument("input_task_dir", help="Input task directory, e.g. /home/pnp/Desktop/franka_record_data/task.")
    parser.add_argument("output_task_dir", nargs="?", default=None, help="Output task directory.")
    parser.add_argument(
        "--camera",
        default=DEFAULT_CAMERA,
        help="Camera name to keep, comma-separated names, or all. Examples: right, wrist,right, all.",
    )
    parser.add_argument("--source-fps", type=int, default=DEFAULT_SOURCE_FPS, help="Source dataset FPS.")
    parser.add_argument("--target-fps", type=int, default=DEFAULT_TARGET_FPS, help="Target dataset FPS.")
    parser.add_argument("--overwrite", action="store_true", help="Replace output directory if it already exists.")
    return parser.parse_args()


def _default_output_task_dir(input_task_dir: str) -> Path:
    task_name = Path(input_task_dir).expanduser().resolve().name
    return Path.home() / "Desktop" / "franka_record_data_10hz" / task_name


def main() -> None:
    args = parse_args()
    output_task_dir = args.output_task_dir or _default_output_task_dir(args.input_task_dir)

    print(f"Input task: {args.input_task_dir}")
    print(f"Output task: {output_task_dir}")
    print(f"Camera: {args.camera}")
    print(f"Source/target FPS: {args.source_fps} -> {args.target_fps}")

    try:
        downsample_task(
            input_task_dir=args.input_task_dir,
            output_task_dir=output_task_dir,
            camera=args.camera,
            source_fps=args.source_fps,
            target_fps=args.target_fps,
            overwrite=args.overwrite,
        )
    except DownsampleError as exc:
        raise SystemExit(f"Downsample failed:\n{exc}") from exc


if __name__ == "__main__":
    main()
