import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from pointcloud.depth_proof import (
    color_from_frame,
    depth_from_frame,
    depth_to_point_cloud,
    load_episode,
)


def put_label(image, text, org=(12, 28), scale=0.7):
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def resize_to(image, size):
    return cv2.resize(image, size, interpolation=cv2.INTER_AREA)


def draw_wrapped_lines(canvas, lines, x, y, scale=0.52, color=(30, 30, 30), max_chars=68, line_height=27):
    for line in lines:
        text = str(line)
        chunks = []
        while len(text) > max_chars:
            split_at = text.rfind(" ", 0, max_chars)
            if split_at <= 0:
                split_at = max_chars
            chunks.append(text[:split_at])
            text = text[split_at:].lstrip()
        chunks.append(text)
        for chunk in chunks:
            cv2.putText(canvas, chunk, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
            y += line_height
    return y


def depth_colormap(depth):
    mask = np.isfinite(depth) & (depth > 0)
    if not np.any(mask):
        return np.zeros((*depth.shape, 3), dtype=np.uint8)
    values = depth[mask]
    vmin = float(np.percentile(values, 1))
    vmax = float(np.percentile(values, 99))
    if vmax <= vmin:
        vmax = vmin + 1e-6
    scaled = np.zeros(depth.shape, dtype=np.uint8)
    scaled[mask] = np.clip((depth[mask] - vmin) * 255.0 / (vmax - vmin), 0, 255).astype(np.uint8)
    out = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    out[~mask] = 0
    put_label(out, f"Depth preview, p1={vmin:.2f}m p99={vmax:.2f}m")
    return out


def valid_mask_view(depth):
    mask = np.isfinite(depth) & (depth > 0)
    out = np.zeros((*depth.shape, 3), dtype=np.uint8)
    out[mask] = (70, 210, 80)
    out[~mask] = (25, 25, 25)
    valid = float(mask.mean()) if mask.size else 0.0
    put_label(out, f"Valid depth mask: {valid:.1%}")
    return out


def draw_projection(points, colors_rgb, axes, title, size=(640, 420), margin=34):
    canvas = np.full((size[1], size[0], 3), 245, dtype=np.uint8)
    if points.size == 0:
        put_label(canvas, title + " (no points)")
        return canvas

    a, b = axes
    xy = points[:, [a, b]]
    lo = np.percentile(xy, 1, axis=0)
    hi = np.percentile(xy, 99, axis=0)
    span = np.maximum(hi - lo, 1e-6)
    norm = (np.clip(xy, lo, hi) - lo) / span
    px = (margin + norm[:, 0] * (size[0] - 2 * margin)).astype(np.int32)
    py = (size[1] - margin - norm[:, 1] * (size[1] - 2 * margin)).astype(np.int32)

    order = np.argsort(points[:, 2])[::-1]
    bgr = colors_rgb[:, ::-1]
    for idx in order:
        cv2.circle(canvas, (int(px[idx]), int(py[idx])), 1, tuple(int(x) for x in bgr[idx]), -1, cv2.LINE_AA)

    cv2.rectangle(canvas, (margin, margin), (size[0] - margin, size[1] - margin), (80, 80, 80), 1)
    put_label(canvas, title)
    labels = ["x right", "y down", "z forward"]
    cv2.putText(canvas, labels[a], (margin, size[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(canvas, labels[b], (8, margin + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(
        canvas,
        f"{labels[a]}: {lo[0]:.2f}..{hi[0]:.2f} m | {labels[b]}: {lo[1]:.2f}..{hi[1]:.2f} m",
        (margin, size[1] - 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (30, 30, 30),
        1,
        cv2.LINE_AA,
    )
    return canvas


def depth_histogram(depth, size=(640, 420)):
    mask = np.isfinite(depth) & (depth > 0)
    canvas = np.full((size[1], size[0], 3), 248, dtype=np.uint8)
    if not np.any(mask):
        put_label(canvas, "Depth histogram (no valid depth)")
        return canvas
    values = depth[mask].astype(np.float64)
    lo = float(np.percentile(values, 1))
    hi = float(np.percentile(values, 99))
    hist, edges = np.histogram(values, bins=64, range=(lo, hi))
    hist = hist.astype(np.float64) / max(1.0, hist.max())
    margin = 42
    width = size[0] - 2 * margin
    height = size[1] - 2 * margin
    for i, h in enumerate(hist):
        x0 = margin + int(i * width / len(hist))
        x1 = margin + int((i + 1) * width / len(hist)) - 1
        y1 = size[1] - margin
        y0 = y1 - int(h * height)
        cv2.rectangle(canvas, (x0, y0), (max(x0, x1), y1), (100, 140, 220), -1)
    cv2.rectangle(canvas, (margin, margin), (size[0] - margin, size[1] - margin), (80, 80, 80), 1)
    put_label(canvas, "Depth distribution")
    cv2.putText(canvas, f"p1={lo:.2f}m", (margin, size[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"p99={hi:.2f}m", (size[0] - 150, size[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(
        canvas,
        f"mean={values.mean():.2f}m, valid={values.size}/{depth.size}",
        (margin, margin + 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (30, 30, 30),
        1,
        cv2.LINE_AA,
    )
    return canvas


def storage_chart(episode_dir, frame, camera_name):
    files = {
        "light pkl": episode_dir / "0.pkl.gz",
        "RGB mp4": episode_dir / f"{camera_name}.mp4",
        "depth PNGs": episode_dir / "depth" / camera_name,
        "proof/view": episode_dir / "rgb_pointcloud_view",
    }
    sizes = {}
    for name, path in files.items():
        if path.is_dir():
            sizes[name] = sum(p.stat().st_size for p in path.rglob("*") if p.is_file())
        elif path.exists():
            sizes[name] = path.stat().st_size
        else:
            sizes[name] = 0
    canvas = np.full((420, 640, 3), 248, dtype=np.uint8)
    put_label(canvas, "Storage layout after change")
    max_size = max(sizes.values()) or 1
    y = 80
    for name, size in sizes.items():
        bar_w = int(310 * size / max_size)
        cv2.rectangle(canvas, (170, y - 20), (170 + bar_w, y + 10), (70, 170, 210), -1)
        cv2.rectangle(canvas, (170, y - 20), (480, y + 10), (80, 80, 80), 1)
        cv2.putText(canvas, name, (24, y + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (30, 30, 30), 1, cv2.LINE_AA)
        cv2.putText(canvas, human_bytes(size), (498, y + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (30, 30, 30), 1, cv2.LINE_AA)
        y += 58
    cv2.putText(canvas, "pkl frame stores paths only:", (24, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (30, 30, 30), 1, cv2.LINE_AA)
    draw_wrapped_lines(canvas, [", ".join(sorted(frame.keys()))], 24, 360, scale=0.42, max_chars=82, line_height=24)
    return canvas


def human_bytes(size):
    units = ["B", "KiB", "MiB", "GiB"]
    value = float(size)
    for unit in units:
        if value < 1024 or unit == units[-1]:
            return f"{value:.1f} {unit}" if unit != "B" else f"{int(value)} B"
        value /= 1024


def make_contact_sheet(images, out_path, cell=(640, 420)):
    resized = []
    for image in images:
        resized.append(resize_to(image, cell))
    rows = [
        np.concatenate(resized[0:3], axis=1),
        np.concatenate(resized[3:6], axis=1),
        np.concatenate(resized[6:9], axis=1),
    ]
    sheet = np.concatenate(rows, axis=0)
    cv2.imwrite(str(out_path), sheet)


def parse_args():
    parser = argparse.ArgumentParser(description="Render explanatory RGB-D/point-cloud views for one recorded episode.")
    parser.add_argument("episode_dir", help="Episode directory, e.g. ~/Desktop/franka_record_data/rgb_pointcloud/0")
    parser.add_argument("--camera", default="middle", help="Camera name to visualize.")
    parser.add_argument("--frame", default="middle", help="Frame index, first, middle, or last.")
    parser.add_argument("--output-dir", default=None, help="Defaults to episode_dir/visual_explain.")
    parser.add_argument("--pointcloud-stride", type=int, default=8)
    parser.add_argument("--pointcloud-max-points", type=int, default=20000)
    return parser.parse_args()


def resolve_frame_index(spec, frame_count):
    text = str(spec).strip().lower()
    if text == "first":
        return 0
    if text == "middle":
        return frame_count // 2
    if text == "last":
        return frame_count - 1
    index = int(text)
    if index < 0:
        index = frame_count + index
    if index < 0 or index >= frame_count:
        raise IndexError(f"frame index {index} out of range [0, {frame_count - 1}]")
    return index


def main():
    args = parse_args()
    episode_dir = Path(args.episode_dir).expanduser()
    output_dir = Path(args.output_dir).expanduser() if args.output_dir else episode_dir / "visual_explain"
    output_dir.mkdir(parents=True, exist_ok=True)
    frames, metadata, _ = load_episode(episode_dir)
    frame_index = resolve_frame_index(args.frame, len(frames))
    frame = frames[frame_index]
    camera_metadata = metadata["cameras"][args.camera]
    bgr = color_from_frame(episode_dir, frame, args.camera, frame_index)
    depth = depth_from_frame(episode_dir, frame, args.camera, camera_metadata)
    points, colors = depth_to_point_cloud(
        depth=depth,
        intrinsics=camera_metadata["intrinsics"],
        bgr_image=bgr,
        stride=args.pointcloud_stride,
        max_points=args.pointcloud_max_points,
    )

    rgb = bgr.copy()
    put_label(rgb, f"RGB frame {frame_index}, camera={args.camera}")
    depth_img = depth_colormap(depth)
    mask_img = valid_mask_view(depth)
    hist = depth_histogram(depth)
    xy = draw_projection(points, colors, (0, 1), "Point cloud XY projection")
    xz = draw_projection(points, colors, (0, 2), "Point cloud XZ projection")
    yz = draw_projection(points, colors, (2, 1), "Point cloud ZY projection")
    storage = storage_chart(episode_dir, frame, args.camera)
    file_layout = np.full((420, 640, 3), 248, dtype=np.uint8)
    put_label(file_layout, "Actual stored frame references")
    lines = [
        "episode: .../franka_record_data/rgb_pointcloud_real_smoke/0",
        f"frame_count: {len(frames)}",
        f"frame_index: {frame_index}",
        f"image_path: {frame[f'{args.camera}_image_path']}",
        f"depth_path: {frame[f'{args.camera}_depth_path']}",
        f"depth_scale: {frame[f'{args.camera}_depth_scale']}",
        "point cloud = depth PNG + RGB mp4 frame + intrinsics",
    ]
    draw_wrapped_lines(file_layout, lines, 24, 78, scale=0.55, max_chars=62, line_height=38)

    cv2.imwrite(str(output_dir / "rgb_frame.png"), rgb)
    cv2.imwrite(str(output_dir / "depth_preview.png"), depth_img)
    cv2.imwrite(str(output_dir / "valid_mask.png"), mask_img)
    cv2.imwrite(str(output_dir / "pointcloud_xy.png"), xy)
    cv2.imwrite(str(output_dir / "pointcloud_xz.png"), xz)
    cv2.imwrite(str(output_dir / "pointcloud_zy.png"), yz)
    cv2.imwrite(str(output_dir / "depth_histogram.png"), hist)
    cv2.imwrite(str(output_dir / "storage_breakdown.png"), storage)
    cv2.imwrite(str(output_dir / "file_layout.png"), file_layout)
    make_contact_sheet([rgb, depth_img, mask_img, xy, xz, yz, hist, storage, file_layout], output_dir / "overview.png")

    summary = {
        "episode_dir": str(episode_dir),
        "output_dir": str(output_dir),
        "frame_index": frame_index,
        "frame_count": len(frames),
        "camera": args.camera,
        "pointcloud_stride": int(args.pointcloud_stride),
        "point_count": int(points.shape[0]),
        "depth_valid_ratio": float(((np.isfinite(depth) & (depth > 0)).mean())),
        "depth_mean_m": float(depth[np.isfinite(depth) & (depth > 0)].mean()),
        "files": {p.name: str(p) for p in output_dir.glob("*.png")},
    }
    (output_dir / "visual_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
