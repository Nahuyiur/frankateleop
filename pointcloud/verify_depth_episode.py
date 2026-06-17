"""Generate depth proof artifacts for a recorded episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pointcloud.depth_proof import load_episode, write_depth_proof


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_dir", help="Episode directory, e.g. ~/Desktop/franka_record_data/task/0")
    parser.add_argument("--index", type=int, default=None, help="Episode index. Defaults to the numeric pkl.gz in the directory.")
    parser.add_argument("--pointcloud-stride", type=int, default=4)
    parser.add_argument("--pointcloud-max-points", type=int, default=80000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    episode_dir = Path(args.episode_dir).expanduser()
    frames, metadata, index = load_episode(episode_dir, args.index)
    summary = write_depth_proof(
        episode_dir,
        index,
        frames,
        metadata,
        stride=args.pointcloud_stride,
        max_points=args.pointcloud_max_points,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"Depth proof written to: {episode_dir / 'depth_proof'}")


if __name__ == "__main__":
    main()
