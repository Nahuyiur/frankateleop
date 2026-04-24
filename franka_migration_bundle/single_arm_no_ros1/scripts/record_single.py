import argparse
import gzip
import json
import os
import pickle
import time

import cv2
import imageio
import numpy as np
import requests
import sys
sys.path.append("/home/ubuntu/FR3/")
from envs.camera.rs_capture import RSCapture
from fr3_single_arm_config import SingleArmConfig


class RobotClient:
    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip("/")
        self._session = requests.Session()
        self._session.trust_env = False

    def get_pose_euler(self):
        response = self._session.post(f"{self.base_url}/getpos_euler", timeout=2.0)
        response.raise_for_status()
        return response.json()["pose"]

    def get_joint(self):
        response = self._session.post(f"{self.base_url}/getq", timeout=2.0)
        response.raise_for_status()
        return response.json()["q"]

    def get_gripper(self):
        response = self._session.post(f"{self.base_url}/get_gripper", timeout=2.0)
        response.raise_for_status()
        return float(response.json()["gripper"])


def save_keyframes_json(output_dir, keyframes):
    out_path = os.path.join(output_dir, "keyframes.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({"keyframes": keyframes}, f, ensure_ascii=False, indent=2)


def save_trajectory(output_dir, index, frames, keyframes):
    out_path = os.path.join(output_dir, f"{index}.pkl.gz")
    with gzip.open(out_path, "wb", compresslevel=1) as f:
        pickle.dump({"data": frames, "keyframes": keyframes}, f, protocol=pickle.HIGHEST_PROTOCOL)


def concatenate_images(images, line_color=(255, 0, 0), line_width=4):
    if not images:
        return None

    min_height = min(img.shape[0] for img in images)
    resized = []
    for img in images:
        scale = min_height / img.shape[0]
        resized.append(cv2.resize(img, (int(img.shape[1] * scale), min_height)))

    total_width = sum(img.shape[1] for img in resized) + line_width * (len(resized) - 1)
    display = np.zeros((min_height, total_width, 3), dtype=resized[0].dtype)
    x = 0
    for idx, img in enumerate(resized):
        width = img.shape[1]
        display[:min_height, x:x + width] = img
        x += width
        if idx < len(resized) - 1:
            display[:min_height, x:x + line_width] = line_color
            x += line_width
    return display


def build_output_dir(task: str, index: int, output_root: str):
    output_dir = os.path.join(output_root, task, str(index))
    os.makedirs(output_dir, exist_ok=True)
    return output_dir


def create_video_writers(output_dir: str, camera_names: list[str]):
    writers = {}
    for name in camera_names:
        writers[name] = imageio.get_writer(os.path.join(output_dir, f"{name}.mp4"), fps=30)
    return writers


def close_video_writers(writers):
    for writer in writers.values():
        writer.close()


def create_cameras():
    cameras = {}
    for name, kwargs in SingleArmConfig.REALSENSE_CAMERAS.items():
        cameras[name] = RSCapture(name=name, **kwargs)
    return cameras


def close_cameras(cameras):
    for camera in cameras.values():
        camera.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=int, required=True)
    parser.add_argument("--task", type=str, default="test")
    parser.add_argument(
        "--output_root",
        type=str,
        default="/home/ubuntu/FR3/record_data",
    )
    args = parser.parse_args()

    output_dir = build_output_dir(args.task, args.index, args.output_root)
    robot = RobotClient(SingleArmConfig.SERVER_URL)
    cameras = create_cameras()
    camera_names = list(cameras.keys())
    video_writers = create_video_writers(output_dir, camera_names)
    
    frames = []
    keyframes = [0]
    record_flag = False

    print("[提示] 点击 [RGB] 窗口后按键：s=开始录制, w=停止, q=退出并保存, k=关键帧")
    try:
        while True:
            rgb_frames = {}
            depth_frames = {}
            for name, camera in cameras.items():
                rgb, _ = camera.read()
                rgb_frames[name] = rgb
                # depth_frames[name] = depth

            preview = concatenate_images([rgb_frames[name] for name in camera_names])
            cv2.imshow("RGB", preview)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                record_flag = True
                print("Start Recording")
            if key in (ord("w"), ord("W")):
                record_flag = False
                print("Stop Recording")
            if key in (ord("k"), ord("K")) and record_flag:
                keyframes.append(len(frames))
                print(f"Keyframe {len(frames)} added.")

            if not record_flag:
                continue

            pose = robot.get_pose_euler()
            joint = robot.get_joint()
            gripper = robot.get_gripper()
            print(f"pose: {pose}, joint: {joint}, gripper: {gripper}")

            for name in camera_names:
                video_writers[name].append_data(cv2.cvtColor(rgb_frames[name], cv2.COLOR_BGR2RGB))

            frame = {
                "pose": pose,
                "joint": joint,
                "gripper": gripper,
                "timestamp": time.time(),
            }
            for name in camera_names:
                frame[f"{name}_image"] = rgb_frames[name].copy()
                # frame[f"{name}_depth"] = depth_frames[name].copy() if depth_frames[name] is not None else None
            frames.append(frame)
    finally:
        cv2.destroyAllWindows()
        close_cameras(cameras)
        close_video_writers(video_writers)

    save_trajectory(output_dir, args.index, frames, keyframes)
    save_keyframes_json(output_dir, keyframes)
    print(f"Saved {len(frames)} frames to {output_dir}")


if __name__ == "__main__":
    main()
