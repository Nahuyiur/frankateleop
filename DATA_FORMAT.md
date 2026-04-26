# FR3 数据采集保存格式说明

本文档说明当前 `franka_capture` 数据采集脚本保存的数据结构、字段含义和维度定义。

当前主要采集入口：

```bash
bash 6_record_fr3.sh <task>
```

默认保存根目录：

```text
/home/pnp/Desktop/franka_record_data
```

最终每条 episode 保存到：

```text
<output_root>/<task>/<episode_index>/
```

例如：

```text
/home/pnp/Desktop/franka_record_data/pick_block/3/
```

## Episode 目录结构

当前单相机配置下，一个 episode 目录通常包含：

```text
3/
  3.pkl.gz
  keyframes.json
  wrist.mp4
```

如果后续增加多路相机，会额外出现对应相机名的视频文件，例如：

```text
exterior_1.mp4
exterior_2.mp4
```

当前实现没有保存 `metadata.json`，也没有保存 depth 数据。

## `pkl.gz` 顶层结构

`<episode_index>.pkl.gz` 是 gzip 压缩后的 pickle 文件。

读取方式：

```python
import gzip
import pickle

path = "/home/pnp/Desktop/franka_record_data/pick_block/3/3.pkl.gz"

with gzip.open(path, "rb") as f:
    obj = pickle.load(f)

frames = obj["data"]
keyframes = obj["keyframes"]
```

顶层结构：

```python
{
    "data": [frame0, frame1, frame2, ...],
    "keyframes": [0, ...],
}
```

其中：

- `data`：逐帧数据列表。
- `keyframes`：关键帧索引列表。默认包含 `[0]`；录制时按 `k` 会追加当前帧索引。

## 单帧 `frame` 结构

当前每帧保存：

```python
{
    "pose": [x, y, z, rx, ry, rz],
    "joint": [j1, j2, j3, j4, j5, j6, j7],
    "gripper": width,
    "timestamp": unix_time,
    "wrist_image": image,
}
```

如果有多路相机，每路相机会增加一个字段：

```python
"<camera_name>_image"
```

例如：

```python
"wrist_image"
"exterior_1_image"
```

## 字段含义

### `pose`

维度：

```text
6
```

格式：

```text
[x, y, z, rx, ry, rz]
```

含义：

- `x, y, z`：机器人末端位置。
- `rx, ry, rz`：机器人末端姿态欧拉角。

单位：

- 位置：米。
- 姿态：弧度。

姿态来源：

```python
robot_observations["ee_pose_euler"]
```

欧拉角顺序：

```text
xyz
```

### `joint`

维度：

```text
7
```

格式：

```text
[j1, j2, j3, j4, j5, j6, j7]
```

含义：

- FR3 机械臂 7 个关节角。

单位：

```text
弧度
```

来源：

```python
joint_state[:7]
```

### `gripper`

维度：

```text
1
```

格式：

```text
width
```

含义：

- Franka Hand 当前夹爪开口宽度。

单位：

```text
米
```

范围大致：

```text
0.0 ~ 0.09
```

说明：

- `0.0` 表示完全闭合。
- `0.09` 表示最大张开，约 9 cm。

当前保存时的计算方式：

```python
gripper = joint_state[-1] * 0.09
```

这里的 `joint_state[-1]` 是 robot node 返回的 normalized gripper width：

```text
0.0 = 闭合
1.0 = 张开
```

所以保存进数据集的 `gripper` 已经是真实宽度，不再是归一化值。

### `timestamp`

维度：

```text
1
```

格式：

```text
unix_time
```

含义：

- 当前帧写入时的系统时间。

单位：

```text
秒
```

来源：

```python
time.time()
```

可用相邻帧 timestamp 差值估算真实采集频率。

### `<camera_name>_image`

当前默认字段：

```text
wrist_image
```

维度：

```text
H x W x 3
```

当前默认相机分辨率：

```text
480 x 640 x 3
```

注意：

- `pkl.gz` 里的图像是 OpenCV BGR 顺序。
- `mp4` 视频文件是从相同帧流写出的 RGB 视频。

如果使用 OpenCV 显示 `pkl.gz` 中的图像，可以直接：

```python
import cv2

cv2.imshow("wrist", frame["wrist_image"])
cv2.waitKey(0)
```

如果使用 matplotlib 显示，需要转成 RGB：

```python
import matplotlib.pyplot as plt

img_rgb = frame["wrist_image"][:, :, ::-1]
plt.imshow(img_rgb)
plt.show()
```

## `keyframes.json`

`keyframes.json` 内容很简单：

```json
{
  "keyframes": [0]
}
```

含义：

- 保存关键帧索引。
- 默认第一帧是关键帧，所以至少有 `[0]`。
- 录制时按 `k` 会把当前帧索引加入列表。

## `mp4` 视频文件

每路相机单独保存一个 mp4：

```text
<camera_name>.mp4
```

当前默认只有：

```text
wrist.mp4
```

说明：

- 视频只保存 RGB 图像。
- 不保存 depth。
- 视频主要用于快速查看采集是否成功。
- 训练或精确读取时，建议以 `pkl.gz` 为准，因为里面有逐帧 robot state 和图像。

## 当前默认相机配置

配置文件：

```text
franka_capture/config/fr3_single.py
```

当前默认只有一台 wrist 相机：

```python
"wrist": CameraConfig(
    name="wrist",
    serial_number="348122072222",
    fps=15,
    depth=False,
)
```

说明：

- `fps=15`：RealSense 采集目标帧率。
- `depth=False`：当前不采集深度。
- `dim=(640, 480)`：默认分辨率。

## 当前录制按键

运行 `6_record_fr3.sh` 后，点击 RGB 预览窗口，再使用键盘：

```text
s = 开始或继续录制
w = 暂停录制
e = 保存当前 episode，并等待下一次 s 开始新 episode
d = 丢弃当前 episode，并等待下一次 s 重新录制
k = 添加关键帧
q = 保存当前 episode 并退出
```

## 快速检查数据

可以用下面脚本检查一条数据：

```python
import gzip
import pickle
import numpy as np

path = "/home/pnp/Desktop/franka_record_data/pick_block/3/3.pkl.gz"

with gzip.open(path, "rb") as f:
    obj = pickle.load(f)

frames = obj["data"]

poses = np.array([x["pose"] for x in frames])
joints = np.array([x["joint"] for x in frames])
grippers = np.array([x["gripper"] for x in frames])
timestamps = np.array([x["timestamp"] for x in frames])

print("frames:", len(frames))
print("pose shape:", poses.shape)
print("joint shape:", joints.shape)
print("gripper min/max:", grippers.min(), grippers.max())
print("duration:", timestamps[-1] - timestamps[0])
print("avg fps:", (len(timestamps) - 1) / (timestamps[-1] - timestamps[0]))
print("image shape:", frames[0]["wrist_image"].shape)
print("keyframes:", obj["keyframes"])
```

期望形状：

```text
pose shape: (N, 6)
joint shape: (N, 7)
gripper shape: (N,)
wrist_image shape: (480, 640, 3)
```

## 和 replay 的关系

`7_replay_fr3.sh` 使用 `pkl.gz` 中的：

```text
joint
gripper
timestamp
```

执行轨迹复现。

其中：

- `joint` 用于复现 7 维机械臂关节轨迹。
- `gripper` 用于复现夹爪宽度。
- `timestamp` 用于按原始时间节奏 replay。

`pose` 和图像不会直接用于当前 replay；它们主要用于后续训练、分析或可视化。
