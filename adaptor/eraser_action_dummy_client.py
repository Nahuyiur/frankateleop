#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import base64
import json
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
import websockets


def _encode_frame(frame: np.ndarray, seed: int | None = None, route: str = "predict") -> str:
    return json.dumps(
        {
            "route": route,
            "shape": list(frame.shape),
            "dtype": str(frame.dtype),
            "frame_b64": base64.b64encode(np.ascontiguousarray(frame).tobytes()).decode("ascii"),
            "seed": seed,
        }
    )


def _load_episode(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with h5py.File(path, "r") as f:
        frames = f["observations/images/right"][()]
        action_full = f["action"][()].astype(np.float32)
    actions = np.concatenate([action_full[:, :6], action_full[:, -1:]], axis=-1).astype(np.float32)
    return frames, actions


def _slice_action_chunk_with_last_repeat(actions: np.ndarray, start_idx: int, chunk_size: int) -> tuple[np.ndarray, np.ndarray]:
    chunk = np.asarray(actions[start_idx:min(start_idx + chunk_size, len(actions))], dtype=np.float32)
    if chunk.shape[0] == 0:
        raise ValueError(f"Empty action chunk for start_idx={start_idx}, len(actions)={len(actions)}")
    valid_mask = np.zeros(chunk_size, dtype=bool)
    valid_mask[:chunk.shape[0]] = True
    if chunk.shape[0] < chunk_size:
        chunk = np.concatenate([chunk, np.repeat(chunk[-1:], chunk_size - chunk.shape[0], axis=0)], axis=0)
    return chunk, valid_mask


def _plot_actions(pred: np.ndarray, gt: np.ndarray, valid: np.ndarray, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    dims = int(gt.shape[1])
    fig, axes = plt.subplots(dims, 1, figsize=(12, max(4, dims * 1.7)), sharex=True)
    axes = np.atleast_1d(axes)
    x = np.arange(gt.shape[0])
    for dim, ax in enumerate(axes):
        ax.plot(x[valid], gt[valid, dim], label="gt", linewidth=1)
        ax.plot(x[valid], pred[valid, dim], label="pred", linewidth=1)
        ax.set_ylabel(f"a{dim}")
    axes[0].legend()
    axes[-1].set_xlabel("10Hz action step")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


async def main():
    parser = argparse.ArgumentParser(description="Dummy client for put-eraser websocket action policy.")
    parser.add_argument("--uri", default="ws://127.0.0.1:8765")
    parser.add_argument("--h5-root", default="/mnt/zezhong/datasets/put_eraser_into_drawer_10hz_right")
    parser.add_argument("--output-dir", default="output/eraser_action_eval")
    parser.add_argument("--test-count", type=int, default=3)
    parser.add_argument("--use-train-split", action="store_true")
    parser.add_argument("--train-offset", type=int, default=5)
    parser.add_argument("--chunk-size", type=int, default=160)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    all_paths = sorted((Path(args.h5_root) / "episodes").glob("*.hdf5"))
    if args.use_train_split:
        h5_paths = all_paths[args.train_offset:args.train_offset + args.test_count]
    else:
        h5_paths = all_paths[: args.test_count]
    if not h5_paths:
        raise ValueError(f"No episodes selected from {args.h5_root}")

    out_dir = Path(args.output_dir)
    all_pred, all_gt, all_valid = [], [], []
    async with websockets.connect(args.uri, max_size=None, ping_interval=30, ping_timeout=300, close_timeout=300) as ws:
        for ep_idx, path in enumerate(tqdm(h5_paths, desc="episodes")):
            frames, gt_actions = _load_episode(path)
            await ws.send(_encode_frame(frames[0], seed=args.seed, route="/reset"))
            await ws.recv()

            pred_chunks, gt_chunks, valid_chunks = [], [], []
            starts = list(range(0, max(1, len(frames) - 4), args.chunk_size))
            for start in tqdm(starts, desc=path.stem, leave=False):
                await ws.send(_encode_frame(frames[start], seed=args.seed + start))
                response = json.loads(await ws.recv())
                pred = np.asarray(response["action"], dtype=np.float32)
                gt, valid = _slice_action_chunk_with_last_repeat(gt_actions, start, args.chunk_size)
                pred_chunks.append(pred)
                gt_chunks.append(gt)
                valid_chunks.append(valid)

            pred_ep = np.concatenate(pred_chunks, axis=0)
            gt_ep = np.concatenate(gt_chunks, axis=0)
            valid_ep = np.concatenate(valid_chunks, axis=0)
            all_pred.append(pred_ep)
            all_gt.append(gt_ep)
            all_valid.append(valid_ep)
            episode_out = out_dir / path.stem
            episode_out.mkdir(parents=True, exist_ok=True)
            np.save(episode_out / "pred_action.npy", pred_ep)
            np.save(episode_out / "gt_action.npy", gt_ep)
            np.save(episode_out / "valid_mask.npy", valid_ep)
            np.savez_compressed(
                episode_out / "action_comparison.npz",
                pred=pred_ep,
                gt=gt_ep,
                valid=valid_ep,
                source_h5=str(path),
            )
            _plot_actions(pred_ep, gt_ep, valid_ep, out_dir / f"{path.stem}_actions.png")
            _plot_actions(pred_ep, gt_ep, valid_ep, episode_out / "action_comparison.png")

    pred = np.concatenate(all_pred, axis=0)
    gt = np.concatenate(all_gt, axis=0)
    valid = np.concatenate(all_valid, axis=0)
    diff = pred[valid] - gt[valid]
    metrics = {
        "l1": float(np.abs(diff).mean()),
        "l2": float((diff ** 2).mean()),
        "num_steps": int(pred.shape[0]),
        "num_valid_steps": int(valid.sum()),
        "num_padding_steps": int((~valid).sum()),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    asyncio.run(main())
