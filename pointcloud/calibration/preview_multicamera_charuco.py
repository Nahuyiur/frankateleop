"""Live three-camera RGB-D preview for multi-camera ChArUco calibration.

This utility is deliberately read-only with respect to the robot.  It opens the
configured RealSense cameras, displays their latest frames side by side, draws
detected ArUco markers, and can save an approximately synchronized RGB-D
snapshot for offline calibration/inspection.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np

from franka_capture.cameras.realsense import (
    close_cameras,
    create_realsense_cameras,
)
from franka_capture.config.fr3_single import DEFAULT_CAMERAS


WINDOW_NAME = "Multi-camera calibration preview"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera-names",
        default="left_wrist,left,middle",
        help="Comma-separated configured camera names.",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Camera stream FPS (all three RGB-D cameras have been verified at 30).",
    )
    parser.add_argument(
        "--no-depth",
        action="store_true",
        help="Open RGB only. Depth is enabled and aligned to color by default.",
    )
    parser.add_argument(
        "--detect-scale",
        type=float,
        default=2.0,
        help="Upscale used only for marker detection; the displayed image stays native.",
    )
    parser.add_argument("--min-markers", type=int, default=12)
    parser.add_argument("--panel-width", type=int, default=480)
    parser.add_argument("--depth-max-m", type=float, default=1.5)
    parser.add_argument(
        "--snapshot-dir",
        default=str(Path.home() / "Desktop" / "multicam_calibration_preview"),
    )
    parser.add_argument("--startup-timeout-sec", type=float, default=12.0)
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="Exit after this many seconds; 0 keeps running until Q/Esc.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Do not open a window (useful with --duration-sec for a camera self-test).",
    )
    return parser.parse_args()


def _camera_names(spec: str) -> list[str]:
    names = [part.strip() for part in spec.split(",") if part.strip()]
    if not names:
        raise ValueError("--camera-names must contain at least one camera")
    if len(names) != len(set(names)):
        raise ValueError(f"Duplicate camera name in {names}")
    unknown = [name for name in names if name not in DEFAULT_CAMERAS]
    if unknown:
        raise ValueError(
            f"Unknown camera(s) {unknown}; configured names: {sorted(DEFAULT_CAMERAS)}"
        )
    return names


def _make_configs(args: argparse.Namespace, names: list[str]) -> Dict[str, Any]:
    return {
        name: replace(
            DEFAULT_CAMERAS[name],
            dim=(int(args.width), int(args.height)),
            fps=int(args.fps),
            depth=not args.no_depth,
            align_depth=not args.no_depth,
            read_timeout_ms=1500,
        )
        for name in names
    }


@dataclass
class FrameSample:
    rgb: Optional[np.ndarray] = None
    depth_m: Optional[np.ndarray] = None
    captured_monotonic_ns: Optional[int] = None
    fps: float = 0.0
    error: Optional[str] = None
    sequence: int = 0


class CameraWorker:
    def __init__(self, name: str, camera: Any) -> None:
        self.name = name
        self.camera = camera
        self.sample = FrameSample()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name=f"camera-{name}",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    def join(self, timeout: float = 3.0) -> None:
        self.thread.join(timeout=timeout)

    def latest(self) -> FrameSample:
        with self.lock:
            return FrameSample(
                rgb=None if self.sample.rgb is None else self.sample.rgb.copy(),
                depth_m=(
                    None
                    if self.sample.depth_m is None
                    else self.sample.depth_m.copy()
                ),
                captured_monotonic_ns=self.sample.captured_monotonic_ns,
                fps=self.sample.fps,
                error=self.sample.error,
                sequence=self.sample.sequence,
            )

    def _run(self) -> None:
        last_time: Optional[float] = None
        fps_ema = 0.0
        while not self.stop_event.is_set():
            try:
                rgb, depth_m = self.camera.read()
                now = time.monotonic()
                if last_time is not None and now > last_time:
                    instantaneous = 1.0 / (now - last_time)
                    fps_ema = instantaneous if fps_ema == 0.0 else 0.9 * fps_ema + 0.1 * instantaneous
                last_time = now
                with self.lock:
                    self.sample.rgb = rgb
                    self.sample.depth_m = depth_m
                    self.sample.captured_monotonic_ns = time.monotonic_ns()
                    self.sample.fps = fps_ema
                    self.sample.error = None
                    self.sample.sequence += 1
            except Exception as exc:  # keep other camera previews alive
                with self.lock:
                    self.sample.error = str(exc)
                self.stop_event.wait(0.1)


def _aruco_detector() -> Any:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.minMarkerPerimeterRate = 0.015
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 53
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def _detect_markers(
    rgb: np.ndarray,
    detector: Any,
    scale: float,
) -> tuple[list[np.ndarray], Optional[np.ndarray], int]:
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    scale = max(1.0, float(scale))
    if scale > 1.0:
        gray = cv2.resize(
            gray,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC,
        )
    corners, ids, rejected = detector.detectMarkers(gray)
    if scale > 1.0:
        corners = [corner.astype(np.float32) / scale for corner in corners]
    return corners, ids, len(rejected)


def _depth_stats(depth_m: Optional[np.ndarray]) -> tuple[float, Optional[float]]:
    if depth_m is None:
        return 0.0, None
    depth = np.asarray(depth_m).squeeze()
    valid = np.isfinite(depth) & (depth > 0.0)
    valid_percent = 100.0 * float(np.count_nonzero(valid)) / float(valid.size)
    median = float(np.median(depth[valid])) if np.any(valid) else None
    return valid_percent, median


def _text(
    image: np.ndarray,
    value: str,
    row: int,
    color: tuple[int, int, int] = (255, 255, 255),
    scale: float = 0.55,
) -> None:
    y = 25 + row * 23
    cv2.putText(image, value, (11, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, value, (11, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _depth_colormap(
    depth_m: Optional[np.ndarray],
    shape: tuple[int, int],
    depth_max_m: float,
) -> np.ndarray:
    if depth_m is None:
        return np.zeros((shape[0], shape[1], 3), dtype=np.uint8)
    depth = np.asarray(depth_m).squeeze()
    depth_max_m = max(0.01, float(depth_max_m))
    clipped = np.clip(depth, 0.0, depth_max_m)
    gray = np.asarray(255.0 * clipped / depth_max_m, dtype=np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    colored[(~np.isfinite(depth)) | (depth <= 0.0)] = 0
    return colored


def _placeholder(name: str, width: int, height: int, message: str) -> np.ndarray:
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    _text(panel, name, 0, (255, 255, 255), 0.7)
    _text(panel, message[:75], 2, (80, 80, 255), 0.48)
    return panel


def _render_panel(
    name: str,
    serial: str,
    sample: FrameSample,
    detector: Any,
    args: argparse.Namespace,
    show_depth: bool,
) -> tuple[np.ndarray, Dict[str, Any]]:
    assert sample.rgb is not None
    corners, ids, rejected_count = _detect_markers(
        sample.rgb, detector, args.detect_scale
    )
    marker_count = 0 if ids is None else int(len(ids))
    unique_ids = [] if ids is None else sorted({int(x) for x in ids.reshape(-1)})
    repeated_id_count = marker_count - len(unique_ids)
    valid_percent, median_depth_m = _depth_stats(sample.depth_m)
    frame_age_ms = None
    if sample.captured_monotonic_ns is not None:
        frame_age_ms = (time.monotonic_ns() - sample.captured_monotonic_ns) / 1e6
    stale = frame_age_ms is None or frame_age_ms > 1000.0

    if show_depth:
        panel = _depth_colormap(
            sample.depth_m,
            sample.rgb.shape[:2],
            args.depth_max_m,
        )
    else:
        panel = cv2.cvtColor(sample.rgb, cv2.COLOR_RGB2BGR)
        if ids is not None and len(ids):
            cv2.aruco.drawDetectedMarkers(panel, corners, ids)

    ready = marker_count >= int(args.min_markers)
    if args.no_depth:
        depth_ready = True
    else:
        depth_ready = sample.depth_m is not None and valid_percent >= 50.0
    quality_ok = ready and depth_ready and not stale and sample.error is None
    if sample.error is not None:
        status = "CAMERA ERROR"
        status_color = (50, 50, 255)
    elif stale:
        status = "STALE FRAME"
        status_color = (50, 50, 255)
    elif marker_count == 0:
        status = "NO TARGET"
        status_color = (50, 50, 255)
    elif repeated_id_count > 0:
        status = "MULTI-FACE: CHECK SAME FACE"
        status_color = (0, 180, 255)
    elif quality_ok:
        status = "VIEW OK"
        status_color = (60, 220, 60)
    else:
        status = "CHECK VIEW"
        status_color = (0, 180, 255)
    mode = "DEPTH" if show_depth else "RGB + ArUco"
    _text(panel, f"{name}  S/N {serial}", 0, (255, 255, 255), 0.58)
    age_text = "n/a" if frame_age_ms is None else f"{frame_age_ms:.0f} ms old"
    _text(panel, f"{mode}  {sample.fps:4.1f} FPS  {age_text}", 1, (255, 255, 255))
    _text(
        panel,
        f"markers {marker_count} / unique IDs {len(unique_ids)} / rejected {rejected_count}",
        2,
        status_color,
    )
    if args.no_depth:
        _text(panel, "depth disabled", 3, (160, 160, 160))
    else:
        median_text = "n/a" if median_depth_m is None else f"{median_depth_m:.3f} m"
        _text(panel, f"valid depth {valid_percent:5.1f}% / median {median_text}", 3)
    _text(panel, status, 4, status_color, 0.65)

    target_width = max(240, int(args.panel_width))
    target_height = max(1, round(panel.shape[0] * target_width / panel.shape[1]))
    panel = cv2.resize(panel, (target_width, target_height), interpolation=cv2.INTER_AREA)
    result = {
        "sequence": int(sample.sequence),
        "capture_monotonic_ns": sample.captured_monotonic_ns,
        "measured_fps": float(sample.fps),
        "marker_count": marker_count,
        "unique_marker_ids": unique_ids,
        "repeated_marker_id_count": repeated_id_count,
        "rejected_candidate_count": rejected_count,
        "valid_depth_percent": valid_percent,
        "median_depth_m": median_depth_m,
        "frame_age_ms": frame_age_ms,
        "ready": bool(quality_ok),
        "error": sample.error,
    }
    return panel, result


def _save_snapshot(
    root: Path,
    names: list[str],
    samples: Dict[str, FrameSample],
    cameras: Dict[str, Any],
    detections: Dict[str, Dict[str, Any]],
    args: argparse.Namespace,
) -> Path:
    now = datetime.now()
    base_name = now.strftime("snapshot_%Y%m%d_%H%M%S_%f")[:-3]
    snapshot_dir = root.expanduser() / base_name
    suffix = 1
    while snapshot_dir.exists():
        snapshot_dir = root.expanduser() / f"{base_name}_{suffix:02d}"
        suffix += 1
    snapshot_dir.mkdir(parents=True)

    timestamps = []
    for name in names:
        sample = samples[name]
        if sample.rgb is None:
            continue
        rgb_bgr = cv2.cvtColor(sample.rgb, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(snapshot_dir / f"{name}_rgb.png"), rgb_bgr):
            raise RuntimeError(f"Failed to write RGB image for {name}")
        if sample.depth_m is not None:
            depth = np.asarray(sample.depth_m).squeeze()
            depth_mm = np.zeros(depth.shape, dtype=np.uint16)
            valid = np.isfinite(depth) & (depth > 0.0)
            depth_mm[valid] = np.clip(
                np.rint(depth[valid] * 1000.0), 1, np.iinfo(np.uint16).max
            ).astype(np.uint16)
            if not cv2.imwrite(str(snapshot_dir / f"{name}_depth_mm.png"), depth_mm):
                raise RuntimeError(f"Failed to write depth image for {name}")
        if sample.captured_monotonic_ns is not None:
            timestamps.append(sample.captured_monotonic_ns)

    spread_ms = None
    if len(timestamps) >= 2:
        spread_ms = (max(timestamps) - min(timestamps)) / 1e6
    metadata = {
        "schema_version": "multicamera_preview_snapshot_v1",
        "saved_at_local": now.astimezone().isoformat(),
        "camera_names": names,
        "stream": {
            "width": int(args.width),
            "height": int(args.height),
            "fps": int(args.fps),
            "depth_enabled": not args.no_depth,
            "depth_aligned_to_color": not args.no_depth,
            "depth_encoding": "uint16 millimetres; 0 means invalid",
        },
        "target": {
            "dictionary": "DICT_4X4_50",
            "detect_scale": float(args.detect_scale),
            "minimum_markers_for_preview_ready": int(args.min_markers),
            "note": "The trihedral target repeats IDs on its faces; marker count alone does not identify a physical face.",
        },
        "approximate_capture_spread_ms": spread_ms,
        "cameras": {name: cameras[name].metadata() for name in names},
        "frames": {name: detections.get(name, {}) for name in names},
    }
    with (snapshot_dir / "snapshot.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return snapshot_dir


def main() -> None:
    args = parse_args()
    if args.detect_scale < 1.0:
        raise ValueError("--detect-scale must be >= 1")
    if args.headless and args.duration_sec <= 0.0:
        raise ValueError("--headless requires --duration-sec > 0")
    names = _camera_names(args.camera_names)
    configs = _make_configs(args, names)

    print("Opening cameras (this program never commands robot motion):")
    for name in names:
        print(f"  {name:12s} serial={configs[name].serial_number}")

    cameras: Dict[str, Any] = {}
    workers: Dict[str, CameraWorker] = {}
    started_at = time.monotonic()
    show_depth = False
    try:
        cameras = create_realsense_cameras(configs)
        workers = {name: CameraWorker(name, cameras[name]) for name in names}
        for worker in workers.values():
            worker.start()

        deadline = time.monotonic() + float(args.startup_timeout_sec)
        while time.monotonic() < deadline:
            if all(workers[name].latest().rgb is not None for name in names):
                break
            time.sleep(0.05)
        missing = [name for name in names if workers[name].latest().rgb is None]
        if missing:
            errors = {name: workers[name].latest().error for name in missing}
            raise RuntimeError(f"No startup frame from {missing}: {errors}")

        detector = _aruco_detector()
        print("All camera streams are live.")
        if args.headless:
            print(f"Headless self-test will run for {args.duration_sec:.1f} seconds.")
        else:
            print("Keys: S=save all RGB-D frames, D=toggle RGB/depth, Q or Esc=quit")
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        while True:
            samples = {name: workers[name].latest() for name in names}
            panels = []
            detections: Dict[str, Dict[str, Any]] = {}
            for name in names:
                sample = samples[name]
                if sample.rgb is None:
                    message = sample.error or "waiting for frame"
                    panel = _placeholder(name, args.width, args.height, message)
                    panel = cv2.resize(
                        panel,
                        (
                            args.panel_width,
                            round(args.height * args.panel_width / args.width),
                        ),
                    )
                    detections[name] = {"ready": False, "error": message}
                else:
                    panel, detections[name] = _render_panel(
                        name,
                        configs[name].serial_number,
                        sample,
                        detector,
                        args,
                        show_depth,
                    )
                panels.append(panel)
            mosaic = np.hstack(panels)

            capture_times = [
                sample.captured_monotonic_ns
                for sample in samples.values()
                if sample.captured_monotonic_ns is not None
            ]
            if len(capture_times) >= 2:
                spread_ms = (max(capture_times) - min(capture_times)) / 1e6
                cv2.putText(
                    mosaic,
                    f"latest-frame spread {spread_ms:.1f} ms",
                    (10, mosaic.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            key = -1
            if not args.headless:
                cv2.imshow(WINDOW_NAME, mosaic)
                key = cv2.waitKey(1) & 0xFF
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
            else:
                time.sleep(0.02)

            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("d"), ord("D")):
                show_depth = not show_depth
            if key in (ord("s"), ord("S")):
                try:
                    path = _save_snapshot(
                        Path(args.snapshot_dir),
                        names,
                        samples,
                        cameras,
                        detections,
                        args,
                    )
                    print(f"Saved RGB-D snapshot: {path}")
                except Exception as exc:
                    print(f"Snapshot was not saved: {exc}")

            if args.duration_sec > 0.0 and time.monotonic() - started_at >= args.duration_sec:
                break
        if args.headless:
            print("Camera self-test summary:")
            for name in names:
                sample = workers[name].latest()
                age_ms = None
                if sample.captured_monotonic_ns is not None:
                    age_ms = (time.monotonic_ns() - sample.captured_monotonic_ns) / 1e6
                age_text = "n/a" if age_ms is None else f"{age_ms:.0f} ms"
                print(
                    f"  {name:12s} frames={sample.sequence:4d} "
                    f"measured_fps={sample.fps:5.1f} age={age_text} "
                    f"error={sample.error or 'none'}"
                )
    finally:
        for worker in workers.values():
            worker.stop()
        for worker in workers.values():
            worker.join()
        if cameras:
            close_cameras(cameras.values())
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
