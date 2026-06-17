import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from pointcloud.depth_proof import (
    color_from_frame,
    depth_from_frame,
    depth_to_point_cloud,
    load_episode,
)


def put_label(image, text, org=(14, 30), scale=0.7):
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def human_range(values):
    return [float(np.min(values)), float(np.max(values))]


def parse_args():
    parser = argparse.ArgumentParser(description="Reconstruct colored point clouds and an organized depth mesh from a recorded RGB-D episode.")
    parser.add_argument("episode_dir", help="Episode directory, e.g. ~/Desktop/franka_record_data/rgb_pointcloud/0")
    parser.add_argument("--camera", default="middle", help="Camera name to reconstruct.")
    parser.add_argument("--frame", default="middle", help="Frame index, first, middle, or last.")
    parser.add_argument("--output-dir", default=None, help="Defaults to episode_dir/reconstruction_check.")
    parser.add_argument("--dense-stride", type=int, default=2)
    parser.add_argument("--mesh-stride", type=int, default=3)
    parser.add_argument("--mesh-max-depth-jump-m", type=float, default=0.06)
    parser.add_argument("--fused-frame-step", type=int, default=4)
    parser.add_argument("--fused-stride", type=int, default=4)
    parser.add_argument("--fused-voxel-size-m", type=float, default=0.012)
    parser.add_argument("--fused-max-points", type=int, default=180000)
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


def camera_points_from_depth(depth, intrinsics, stride=3):
    height, width = depth.shape
    yy, xx = np.mgrid[0:height:stride, 0:width:stride]
    sampled = depth[::stride, ::stride]
    mask = np.isfinite(sampled) & (sampled > 0)

    z = sampled.astype(np.float32)
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])
    x = (xx.astype(np.float32) - ppx) * z / fx
    y = (yy.astype(np.float32) - ppy) * z / fy
    points_grid = np.stack([x, y, z], axis=-1).astype(np.float32)
    return points_grid, mask


def build_depth_mesh(depth, bgr, intrinsics, stride=3, max_depth_jump_m=0.06):
    points_grid, mask = camera_points_from_depth(depth, intrinsics, stride=stride)
    sampled_bgr = bgr[::stride, ::stride]

    vertex_ids = -np.ones(mask.shape, dtype=np.int32)
    valid_y, valid_x = np.where(mask)
    vertices = points_grid[valid_y, valid_x]
    colors = sampled_bgr[valid_y, valid_x][:, ::-1].astype(np.uint8)
    vertex_ids[valid_y, valid_x] = np.arange(vertices.shape[0], dtype=np.int32)

    faces = []
    h, w = mask.shape
    z = points_grid[:, :, 2]
    for row in range(h - 1):
        for col in range(w - 1):
            ids = [
                vertex_ids[row, col],
                vertex_ids[row, col + 1],
                vertex_ids[row + 1, col],
                vertex_ids[row + 1, col + 1],
            ]
            if min(ids) < 0:
                continue
            zs = [z[row, col], z[row, col + 1], z[row + 1, col], z[row + 1, col + 1]]
            if max(zs) - min(zs) > max_depth_jump_m:
                continue
            faces.append((ids[0], ids[1], ids[2]))
            faces.append((ids[1], ids[3], ids[2]))

    return vertices, colors, np.asarray(faces, dtype=np.int32)


def write_cloud_ply(path, points, colors):
    with Path(path).open("w", encoding="ascii") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for point, color in zip(points, colors):
            f.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def write_mesh_ply(path, vertices, colors, faces):
    with Path(path).open("w", encoding="ascii") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {vertices.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write(f"element face {faces.shape[0]}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        for point, color in zip(vertices, colors):
            f.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )
        for face in faces:
            f.write(f"3 {int(face[0])} {int(face[1])} {int(face[2])}\n")


def rotation_matrix(yaw_deg=0.0, pitch_deg=0.0):
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float32)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]], dtype=np.float32)
    return rx @ ry


def render_points(points, colors, title, view="camera", size=(900, 650), point_radius=1):
    canvas = np.full((size[1], size[0], 3), 248, dtype=np.uint8)
    if points.size == 0:
        put_label(canvas, title + " (empty)")
        return canvas

    if view == "camera":
        projected = points[:, [0, 1]]
        depth_order = points[:, 2]
        axis_text = "x right / y down"
        invert_second_axis = False
    elif view == "top":
        projected = points[:, [0, 2]]
        depth_order = -points[:, 1]
        axis_text = "x right / z forward"
        invert_second_axis = True
    elif view == "side":
        projected = points[:, [2, 1]]
        depth_order = points[:, 0]
        axis_text = "z forward / y down"
        invert_second_axis = True
    else:
        rot = rotation_matrix(yaw_deg=-38.0, pitch_deg=-18.0)
        rotated = points @ rot.T
        projected = rotated[:, [0, 1]]
        depth_order = rotated[:, 2]
        axis_text = "rotated 3D view"
        invert_second_axis = True

    lo = np.percentile(projected, 1, axis=0)
    hi = np.percentile(projected, 99, axis=0)
    span = np.maximum(hi - lo, 1e-6)
    margin = 45
    norm = (np.clip(projected, lo, hi) - lo) / span
    px = (margin + norm[:, 0] * (size[0] - 2 * margin)).astype(np.int32)
    if invert_second_axis:
        py = (size[1] - margin - norm[:, 1] * (size[1] - 2 * margin)).astype(np.int32)
    else:
        py = (margin + norm[:, 1] * (size[1] - 2 * margin)).astype(np.int32)

    order = np.argsort(depth_order)[::-1]
    bgr = colors[:, ::-1]
    for idx in order:
        cv2.circle(canvas, (int(px[idx]), int(py[idx])), point_radius, tuple(int(x) for x in bgr[idx]), -1, cv2.LINE_AA)

    cv2.rectangle(canvas, (margin, margin), (size[0] - margin, size[1] - margin), (70, 70, 70), 1)
    put_label(canvas, title)
    cv2.putText(canvas, axis_text, (margin, size[1] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
    return canvas


def render_mesh(vertices, colors, faces, title, size=(900, 650)):
    canvas = np.full((size[1], size[0], 3), 248, dtype=np.uint8)
    if vertices.size == 0 or faces.size == 0:
        put_label(canvas, title + " (empty)")
        return canvas

    rot = rotation_matrix(yaw_deg=-38.0, pitch_deg=-18.0)
    rotated = vertices @ rot.T
    projected = rotated[:, [0, 1]]
    lo = np.percentile(projected, 1, axis=0)
    hi = np.percentile(projected, 99, axis=0)
    span = np.maximum(hi - lo, 1e-6)
    margin = 45
    norm = (np.clip(projected, lo, hi) - lo) / span
    pix = np.zeros((vertices.shape[0], 2), dtype=np.int32)
    pix[:, 0] = (margin + norm[:, 0] * (size[0] - 2 * margin)).astype(np.int32)
    pix[:, 1] = (size[1] - margin - norm[:, 1] * (size[1] - 2 * margin)).astype(np.int32)

    face_depth = rotated[faces, 2].mean(axis=1)
    order = np.argsort(face_depth)[::-1]
    for face_idx in order:
        face = faces[face_idx]
        pts = pix[face].reshape((-1, 1, 2))
        color = colors[face].mean(axis=0).astype(np.uint8)[::-1]
        cv2.fillConvexPoly(canvas, pts, tuple(int(x) for x in color), lineType=cv2.LINE_AA)

    cv2.rectangle(canvas, (margin, margin), (size[0] - margin, size[1] - margin), (70, 70, 70), 1)
    put_label(canvas, title)
    cv2.putText(canvas, "depth mesh from neighboring valid pixels", (margin, size[1] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
    return canvas


def voxel_downsample(points, colors, voxel_size=0.012, max_points=180000):
    if points.shape[0] == 0:
        return points, colors
    keys = np.floor(points / float(voxel_size)).astype(np.int32)
    _, unique_indices = np.unique(keys, axis=0, return_index=True)
    unique_indices.sort()
    if unique_indices.shape[0] > max_points:
        sample = np.linspace(0, unique_indices.shape[0] - 1, max_points).astype(np.int64)
        unique_indices = unique_indices[sample]
    return points[unique_indices], colors[unique_indices]


def make_sheet(images, out_path, cell=(900, 650), cols=2):
    resized = [cv2.resize(image, cell, interpolation=cv2.INTER_AREA) for image in images]
    rows = []
    for start in range(0, len(resized), cols):
        row = resized[start:start + cols]
        while len(row) < cols:
            row.append(np.full((cell[1], cell[0], 3), 248, dtype=np.uint8))
        rows.append(np.concatenate(row, axis=1))
    cv2.imwrite(str(out_path), np.concatenate(rows, axis=0))


def write_orbit_video(path, points, colors, size=(900, 650), frames=72):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 18.0, size)
    if not writer.isOpened():
        return False
    try:
        for i in range(frames):
            yaw = -180.0 + 360.0 * i / frames
            rot = rotation_matrix(yaw_deg=yaw, pitch_deg=-16.0)
            rotated = points @ rot.T
            canvas = render_points(rotated, colors, f"Temporal fused point cloud orbit {i + 1}/{frames}", view="camera", size=size, point_radius=1)
            writer.write(canvas)
    finally:
        writer.release()
    return True


def main():
    args = parse_args()
    episode_dir = Path(args.episode_dir).expanduser()
    out_dir = Path(args.output_dir).expanduser() if args.output_dir else episode_dir / "reconstruction_check"
    out_dir.mkdir(parents=True, exist_ok=True)

    frames, metadata, _ = load_episode(episode_dir)
    camera_metadata = metadata["cameras"][args.camera]
    intrinsics = camera_metadata["intrinsics"]
    frame_index = resolve_frame_index(args.frame, len(frames))
    frame = frames[frame_index]
    bgr = color_from_frame(episode_dir, frame, args.camera, frame_index)
    depth = depth_from_frame(episode_dir, frame, args.camera, camera_metadata)
    if bgr is None or depth is None:
        raise RuntimeError("Missing RGB or depth for reconstruction")

    dense_points, dense_colors = depth_to_point_cloud(
        depth=depth,
        intrinsics=intrinsics,
        bgr_image=bgr,
        stride=args.dense_stride,
        max_points=200000,
    )
    mesh_vertices, mesh_colors, mesh_faces = build_depth_mesh(
        depth,
        bgr,
        intrinsics,
        stride=args.mesh_stride,
        max_depth_jump_m=args.mesh_max_depth_jump_m,
    )

    fused_points = []
    fused_colors = []
    fused_frame_indices = list(range(0, len(frames), max(1, int(args.fused_frame_step))))
    if frame_index not in fused_frame_indices:
        fused_frame_indices.append(frame_index)
    fused_frame_indices = sorted(set(fused_frame_indices))
    for idx in fused_frame_indices:
        this_bgr = color_from_frame(episode_dir, frames[idx], args.camera, idx)
        this_depth = depth_from_frame(episode_dir, frames[idx], args.camera, camera_metadata)
        if this_bgr is None or this_depth is None:
            continue
        pts, cols = depth_to_point_cloud(
            depth=this_depth,
            intrinsics=intrinsics,
            bgr_image=this_bgr,
            stride=args.fused_stride,
            max_points=90000,
        )
        fused_points.append(pts)
        fused_colors.append(cols)

    if fused_points:
        fused_points = np.concatenate(fused_points, axis=0)
        fused_colors = np.concatenate(fused_colors, axis=0)
        fused_points, fused_colors = voxel_downsample(
            fused_points,
            fused_colors,
            voxel_size=args.fused_voxel_size_m,
            max_points=args.fused_max_points,
        )
    else:
        fused_points = np.zeros((0, 3), dtype=np.float32)
        fused_colors = np.zeros((0, 3), dtype=np.uint8)

    write_cloud_ply(out_dir / "single_frame_dense_cloud.ply", dense_points, dense_colors)
    write_mesh_ply(out_dir / "single_frame_depth_mesh.ply", mesh_vertices, mesh_colors, mesh_faces)
    write_cloud_ply(out_dir / "temporal_fused_cloud.ply", fused_points, fused_colors)

    rgb_label = bgr.copy()
    put_label(rgb_label, f"Original RGB frame {frame_index}")
    cloud_camera = render_points(dense_points, dense_colors, "Reprojected colored point cloud, camera view", view="camera")
    cloud_angle = render_points(dense_points, dense_colors, "Reconstructed 3D point cloud, angled view", view="angle")
    cloud_top = render_points(dense_points, dense_colors, "Top view of reconstructed point cloud", view="top")
    mesh_render = render_mesh(mesh_vertices, mesh_colors, mesh_faces, "Single-frame surface mesh")
    fused_render = render_points(fused_points, fused_colors, f"Fused point cloud from {len(fused_frame_indices)} frames", view="angle")

    cv2.imwrite(str(out_dir / "original_rgb.png"), rgb_label)
    cv2.imwrite(str(out_dir / "cloud_camera_view.png"), cloud_camera)
    cv2.imwrite(str(out_dir / "cloud_angle_view.png"), cloud_angle)
    cv2.imwrite(str(out_dir / "cloud_top_view.png"), cloud_top)
    cv2.imwrite(str(out_dir / "depth_mesh_render.png"), mesh_render)
    cv2.imwrite(str(out_dir / "temporal_fused_render.png"), fused_render)
    make_sheet(
        [rgb_label, cloud_camera, cloud_angle, cloud_top, mesh_render, fused_render],
        out_dir / "reconstruction_sheet.png",
        cell=(900, 650),
        cols=2,
    )

    orbit_written = write_orbit_video(out_dir / "temporal_fused_orbit.mp4", fused_points, fused_colors)

    valid_mask = np.isfinite(depth) & (depth > 0)
    summary = {
        "episode_dir": str(episode_dir),
        "output_dir": str(out_dir),
        "camera": args.camera,
        "frame_index": int(frame_index),
        "frame_count": int(len(frames)),
        "depth_valid_ratio": float(valid_mask.mean()),
        "depth_mean_m": float(depth[valid_mask].mean()),
        "dense_stride": int(args.dense_stride),
        "dense_point_count": int(dense_points.shape[0]),
        "dense_bounds_xyz_m": {
            "x": human_range(dense_points[:, 0]),
            "y": human_range(dense_points[:, 1]),
            "z": human_range(dense_points[:, 2]),
        },
        "mesh_stride": int(args.mesh_stride),
        "mesh_vertex_count": int(mesh_vertices.shape[0]),
        "mesh_face_count": int(mesh_faces.shape[0]),
        "fused_frame_indices": [int(i) for i in fused_frame_indices],
        "fused_point_count_after_voxel_downsample": int(fused_points.shape[0]),
        "fused_stride": int(args.fused_stride),
        "fused_voxel_size_m": float(args.fused_voxel_size_m),
        "orbit_video_written": bool(orbit_written),
        "artifacts": {
            "single_frame_dense_cloud_ply": str(out_dir / "single_frame_dense_cloud.ply"),
            "single_frame_depth_mesh_ply": str(out_dir / "single_frame_depth_mesh.ply"),
            "temporal_fused_cloud_ply": str(out_dir / "temporal_fused_cloud.ply"),
            "reconstruction_sheet_png": str(out_dir / "reconstruction_sheet.png"),
            "temporal_fused_orbit_mp4": str(out_dir / "temporal_fused_orbit.mp4"),
        },
    }
    (out_dir / "reconstruction_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
