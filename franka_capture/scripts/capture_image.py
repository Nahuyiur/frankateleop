"""Preview configured RealSense cameras and save snapshots with Space."""

import argparse
import time
from pathlib import Path

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_single import DEFAULT_CAMERAS
from franka_capture.recording.preview import concatenate_rgb_images, show_rgb_preview


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--save-dir", default="./camera_checks")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    import cv2

    save_dir = Path(args.save_dir).expanduser()
    save_dir.mkdir(parents=True, exist_ok=True)

    print("Initializing configured RealSense cameras...")
    cameras = create_realsense_cameras(DEFAULT_CAMERAS)
    camera_names = list(cameras.keys())
    print(f"Connected cameras: {camera_names}")

    try:
        print("Waiting for first frame from each camera...")
        warmup_frames = {}
        for name, camera in cameras.items():
            print(f"  {name}: {camera.metadata()}")
            rgb, _ = camera.read()
            warmup_frames[name] = rgb
            print(f"  {name}: first RGB frame shape={rgb.shape}")

        preview = concatenate_rgb_images(
            [warmup_frames[name] for name in camera_names]
        )
        show_rgb_preview("Preview", preview)
        print("Preview is streaming. Click the Preview window first. Space=snapshot, q=quit.")

        while True:
            rgb_frames = {}
            for name, camera in cameras.items():
                rgb, _ = camera.read()
                rgb_frames[name] = rgb

            preview = concatenate_rgb_images([rgb_frames[name] for name in camera_names])
            key = show_rgb_preview("Preview", preview)

            if key in (ord("q"), ord("Q")):
                break
            if key == 32:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                for name, rgb in rgb_frames.items():
                    path = save_dir / f"{name}_{timestamp}.png"
                    cv2.imwrite(str(path), rgb[:, :, ::-1])
                    print(f"Saved {path}")
                if preview is not None:
                    path = save_dir / f"preview_{timestamp}.png"
                    cv2.imwrite(str(path), preview[:, :, ::-1])
                    print(f"Saved {path}")
    finally:
        cv2.destroyAllWindows()
        for camera in cameras.values():
            camera.close()
        print("Cameras closed.")


if __name__ == "__main__":
    main()
