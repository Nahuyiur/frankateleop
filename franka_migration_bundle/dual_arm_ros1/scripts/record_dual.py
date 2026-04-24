
import numpy as np
from scipy.spatial.transform import Rotation as R
import requests
import time
import pyrealsense2 as rs
import cv2
import imageio
import argparse
import os
from envs import RSCapture

import pickle
import gzip

import json

def save_keyframes_json(output_dir, keyframes):
    """
    将关键帧索引单独保存为 JSON 文件。
    """
    out_path = os.path.join(output_dir, "keyframes.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({"keyframes": keyframes}, f, ensure_ascii=False, indent=2)
    print(f"Saved keyframes: {out_path}")


class Robot:
    def __init__(self, url):
        self.BASE_URL = url
        self._session = requests.Session()
        self._session.trust_env = False  # 不走代理，直连本机机器人

    def get_pose_quat(self):
        url = f"{self.BASE_URL}/getpos"
        response = self._session.post(url)
        cur_pose = response.json()['pose']
        
        return cur_pose

    def get_pose_euler(self):
        url = f"{self.BASE_URL}/getpos_euler"
        response = self._session.post(url)
        cur_pose = response.json()['pose']
        
        return cur_pose

    def get_joint(self):
        url = f"{self.BASE_URL}/getq"
        response = self._session.post(url)
        cur_joint = response.json()['q']
        
        return cur_joint    

    def get_gripper(self):
        url = f"{self.BASE_URL}/get_gripper"
        response = self._session.post(url)
        cur_gripper = response.json()['gripper']
        
        return float(cur_gripper)

    def goto_pose(self, pose):
        url = f"{self.BASE_URL}/pose"
        message = {
            "arr": pose
        }
        
        self._session.post(url, json=message)

    def open_gripper(self):
        url = f"{self.BASE_URL}/open_gripper"
        response = self._session.post(url)
        return response.status_code, response.text

    def close_gripper(self):
        url = f"{self.BASE_URL}/close_gripper_slow"
        response = self._session.post(url)
        return response.status_code, response.text

    def reset_robot(self):
        reset_pose = [0.48, 0.0, 0.30, 1, 1, 0, 0.008]
        self.open_gripper()
        for i in range(40):
            self.goto_pose(reset_pose)
            time.sleep(0.1)

    def reset_joint(self):
        url = f"{self.BASE_URL}/jointreset"
        response = self._session.post(url)
        self.open_gripper()
    
def save_trajectory_gzip_pickle(output_dir, index, data, keyframes, compresslevel=6):
    """
    使用 pickle + gzip 压缩保存轨迹数据，并保存关键帧列表。
    """
    out_path = os.path.join(output_dir, f"{index}.pkl.gz")
    with gzip.open(out_path, "wb", compresslevel=compresslevel) as f:
        pickle.dump({"data": data, "keyframes": keyframes}, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"Saved: {out_path}")

def concatenate_images_with_line(images, line_color=(0, 0, 0), line_width=5):
    """
    将多个图像拼接到一起，并用分割线隔开。

    - 如果是 4 张图像，则按照 2x2 网格布局。
    - 其它数量则按原来方式横向拼接。
    """
    if len(images) == 0:
        return None

    # 4 视角时：2x2 布局
    if len(images) == 4:
        # 统一到相同大小（取最小宽高，避免越界）
        heights = [img.shape[0] for img in images]
        widths = [img.shape[1] for img in images]
        h = min(heights)
        w = min(widths)

        resized = [cv2.resize(img, (w, h)) for img in images]

        total_height = 2 * h + line_width
        total_width = 2 * w + line_width

        concatenated_image = np.zeros((total_height, total_width, 3), dtype=np.uint8)

        # 四个象限：0-左上，1-右上，2-左下，3-右下
        concatenated_image[0:h, 0:w] = resized[0]
        concatenated_image[0:h, w + line_width: w + line_width + w] = resized[1]
        concatenated_image[h + line_width: h + line_width + h, 0:w] = resized[2]
        concatenated_image[h + line_width: h + line_width + h, w + line_width: w + line_width + w] = resized[3]

        # 画分割线
        concatenated_image[h: h + line_width, :] = line_color  # 水平线
        concatenated_image[:, w: w + line_width] = line_color  # 垂直线

        return concatenated_image

    # 非 4 张图像时，保持原来的横向拼接逻辑
    heights = [img.shape[0] for img in images]
    widths = [img.shape[1] for img in images]

    max_height = max(heights)
    total_width = sum(widths) + (len(images) - 1) * line_width

    concatenated_image = np.zeros((max_height, total_width, 3), dtype=np.uint8)

    current_x = 0
    for i, img in enumerate(images):
        height, width = img.shape[:2]
        concatenated_image[:height, current_x:current_x + width] = img
        current_x += width

        if i < len(images) - 1:
            concatenated_image[:max_height, current_x:current_x + line_width] = line_color
            current_x += line_width

    return concatenated_image

def reset_realsense_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("未检测到 RealSense 设备！")
        return
        
    print(f"正在复位 {len(devices)} 个设备...")
    for dev in devices:
        dev.hardware_reset()
    
    # 复位后设备会断开重连，必须等待
    print("等待设备重新枚举 (5秒)...")
    time.sleep(5)


if __name__ == "__main__":
    
    # reset_realsense_devices()
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--index', type=int, default=0, required=True)
    parser.add_argument('--task', type=str, default='test', required=True)
    parser.add_argument('--output_root', type=str, default="/home/franka2/fqx/test0225")
    
    args = parser.parse_args()
    
    output_dir = os.path.join(args.output_root, args.task, str(args.index))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    left_poses = []
    left_joints = []
    left_grippers = []

    right_poses = []
    right_joints = []
    right_grippers = []

    keyframes = [0]  # 用于保存关键帧的索引
    record_flag = False

    front_video_filename = os.path.join(output_dir, "front.mp4")
    left_video_filename = os.path.join(output_dir, "left.mp4")
    right_video_filename = os.path.join(output_dir, "right.mp4")
    high_video_filename = os.path.join(output_dir, "high.mp4")
    
    front_video_writer = imageio.get_writer(front_video_filename, fps=60)
    left_video_writer = imageio.get_writer(left_video_filename, fps=60)
    right_video_writer = imageio.get_writer(right_video_filename, fps=60)
    high_video_writer = imageio.get_writer(high_video_filename, fps=60)

    # front_camera = RSCapture(name="front", serial_number="338622073454", fps=15, depth=True)
    # front_camera = RSCapture(name="front", serial_number="341222301576", fps=15, depth=True)
    high_camera = RSCapture(name="high", serial_number="341222301576", fps=15, depth=True)
    high_images = []
    high_depths = []
    
    # left_camera = RSCapture(name="left", serial_number="242322070237", fps=15, depth=True)
    left_camera = RSCapture(name="left", serial_number="242322070237", fps=15, depth=True)
    left_images = []
    left_depths = []

    right_camera = RSCapture(name="right", serial_number="244222074494", fps=15, depth=True)
    right_images = []
    right_depths = []
    # right_camera = RSCapture(name="right", serial_number="339222071866", fps=15, depth=True)
    front_camera = RSCapture(name="front", serial_number="338622073454", fps=15, depth=True)  # right1（banana/coke)
    # front_camera = RSCapture(name="front", serial_number="346522072397", fps=15, depth=True) 

    front_images = []
    front_depths = []

    left_robot = Robot(f"http://127.0.0.1:5000")
    right_robot = Robot(f"http://127.0.0.2:5000")

    from concurrent.futures import ThreadPoolExecutor
    pool = ThreadPoolExecutor(max_workers=2)

    print("[提示] 请用鼠标点击弹出的 [RGB] 窗口，使其获得焦点后，按键才有效：s=开始录制, w=停止, q=退出, r=双机复位, k=关键帧")

    while True: 
        # start_time = time.time()
        front_image, front_depth = front_camera.read()
        left_image, left_depth = left_camera.read()
        right_image, right_depth = right_camera.read()
        high_image, high_depth = high_camera.read()

        images_to_display = [front_image, left_image, right_image,  high_image]
        concatenated_image = concatenate_images_with_line(images_to_display, line_color=(255, 0, 0), line_width=5)
    
        # 显示拼接后的图像（按键需先点击该窗口使其获得焦点）
        cv2.imshow("RGB", concatenated_image)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:  # 255 表示无按键 (-1 & 0xFF)，仅在有按键时打印
            print(f" [键] key = {key}")

        if key == ord('r') or key == ord('R'):
            f1 = pool.submit(left_robot.reset_robot)
            f2 = pool.submit(right_robot.reset_robot)
            f1.result()
            f2.result()
        
        if key == ord('q') or key == ord('Q'):
            cv2.destroyAllWindows()
            front_camera.close()
            left_camera.close()
            right_camera.close()
            high_camera.close()
            front_video_writer.close()
            left_video_writer.close()
            right_video_writer.close()
            high_video_writer.close()

            print("End Recording")

            break
        
        if key == ord('s') or key == ord('S'):
            record_flag = True
            print("Start Recording")
            
        if key == ord('w') or key == ord('W'):
            record_flag = False
            print("Stop Recording")

        if key == ord('k') or key == ord('K'):  # 按下 k 键时标记为关键帧
            if record_flag:
                keyframes.append(len(front_images))  # 将当前帧添加到关键帧列表
                print(f"Keyframe {len(front_images)} added.")
            
        if record_flag:

            front_image, front_depth = front_camera.read()
            left_image, left_depth = left_camera.read()
            right_image, right_depth = right_camera.read()
            high_image, high_depth = high_camera.read()

            front_video_writer.append_data(cv2.cvtColor(front_image, cv2.COLOR_BGR2RGB))
            left_video_writer.append_data(cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB))
            right_video_writer.append_data(cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB))
            high_video_writer.append_data(cv2.cvtColor(high_image, cv2.COLOR_BGR2RGB))


            left_joint = left_robot.get_joint()
            left_pose = left_robot.get_pose_euler()
            left_gripper = left_robot.get_gripper()

            right_joint = right_robot.get_joint()
            right_pose = right_robot.get_pose_euler()
            right_gripper = right_robot.get_gripper()

            left_joints.append(left_joint)
            left_poses.append(left_pose)
            left_grippers.append(left_gripper)

            right_joints.append(right_joint)
            right_poses.append(right_pose)
            right_grippers.append(right_gripper)
            
            front_images.append(front_image.copy())
            front_depths.append(front_depth.copy())
            left_images.append(left_image.copy())
            left_depths.append(left_depth.copy())
            right_images.append(right_image.copy())
            right_depths.append(right_depth.copy())
            high_images.append(high_image.copy())
            high_depths.append(high_depth.copy())

            # end_time = time.time()

            # print("duration:",end_time-start_time)

            
    data = []
    
    for i in range(len(front_images)):
        frame_data = {
            'front_image': front_images[i],
            'front_depth': front_depths[i],
            'left_image': left_images[i],
            'left_depth': left_depths[i],
            'right_image': right_images[i],
            'right_depth': right_depths[i],
            'high_image': high_images[i],
            'high_depth': high_depths[i],
            'left_joint': left_joints[i],
            'left_pose': left_poses[i],
            'left_gripper': left_grippers[i],
            'right_joint': right_joints[i],
            'right_pose': right_poses[i],
            'right_gripper': right_grippers[i],
        }
        data.append(frame_data)

    # print(f"Left Gripper: {left_grippers}")
    # print(f"Right Gripper: {right_grippers}")

    save_trajectory_gzip_pickle(output_dir, args.index, data, keyframes, compresslevel=1)
    save_keyframes_json(output_dir, keyframes)
    print("Save Done!")