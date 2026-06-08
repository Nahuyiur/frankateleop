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

当前配置下，一个 episode 目录通常包含：

```text
3/
  3.pkl.gz
  keyframes.json
  wrist.mp4
  right.mp4
```

如果后续继续增加多路相机，会额外出现对应相机名的视频文件，例如：

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
    "gripper": command,
    "gripper_width": actual_width,
    "gripper_target_width": target_width,
    "timestamp": unix_time,
    "wrist_image": image,
    "right_image": image,
}
```

如果有多路相机，每路相机会增加一个字段：

```python
"<camera_name>_image"
```

例如：

```python
"wrist_image"
"right_image"
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
command
```

含义：

- 实际下发给 robot node 的二值夹爪闭合命令。

单位：

```text
无
```

取值：

```text
0.0 或 1.0
```

说明：

- `0.0` 表示打开。
- `1.0` 表示闭合。
- 二值阈值为闭合度 `0.5`。

当前 GUI 采集时从 robot node 的 observation 读取：

```python
gripper = robot_observations["gripper_command"]
```

每帧同时保存 `gripper_width`，表示 Franka Hand 反馈的实际开口宽度，单位米：

```text
0.0 = 闭合
0.09 = 最大张开，约 9 cm
```

`gripper_target_width` 则是该命令换算后发给夹爪的目标宽度，单位米。

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

当前配置中的字段：

```text
wrist_image
right_image
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
cv2.imshow("right", frame["right_image"])
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

当前配置里通常有：

```text
wrist.mp4
right.mp4
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

当前配置里有两台 RGB 相机：

```python
"wrist": CameraConfig(
    name="wrist",
    serial_number="348122072222",
    fps=15,
    depth=False,
),
"right": CameraConfig(
    name="right",
    serial_number="332522072275",
    fps=15,
    depth=False,
)
```

说明：

- `fr3_single.py` 里的 `fps=15` 是相机配置文件中的默认目标帧率。
- 通过 `6_record_fr3.sh` 或 `0_record_fr3_pipeline.sh` 录制时，脚本级全局 FPS 默认是 `10`，并且会同时覆盖相机采集 FPS 和 mp4 保存 FPS。
- `depth=False`：当前不采集深度。
- `dim=(640, 480)`：默认分辨率。

## 当前录制 FPS

脚本入口只有一个全局 FPS 参数：

```bash
bash 6_record_fr3.sh pick_block --fps 10
bash 0_record_fr3_pipeline.sh pick_block --fps 10
```

也可以用环境变量设置默认值：

```bash
DEFAULT_FPS=10 bash 6_record_fr3.sh pick_block
DEFAULT_FPS=10 bash 0_record_fr3_pipeline.sh pick_block
```

含义：

- `--fps` 会同时设置 RealSense 相机 stream FPS 和保存出来的 mp4 FPS。
- 如果不传，当前脚本默认是 `10`。
- `franka_capture/config/fr3_single.py` 里的 `CameraConfig.fps` 仍然保留，主要作为配置文件默认值；通过脚本录制时以脚本传入的全局 FPS 为准。

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
if "right_image" in frames[0]:
    print("right image shape:", frames[0]["right_image"].shape)
print("keyframes:", obj["keyframes"])
```

期望形状：

```text
pose shape: (N, 6)
joint shape: (N, 7)
gripper shape: (N,)
wrist_image shape: (480, 640, 3)
right_image shape: (480, 640, 3)
```

## 转成 LeRobot v2.1 后的字段

`franka_lerobot` 转换器会读取 `pkl.gz` 里的原始字段，然后生成 LeRobot v2.1 episode-based 数据集。

当前映射是：

```text
observation.state:   8 维 [j1, j2, j3, j4, j5, j6, j7, gripper_command]
observation.ee_pose: 6 维 [x, y, z, rx, ry, rz]
action:             14 维 下一帧 [x, y, z, rx, ry, rz, j1, ..., j7, gripper_command]
observation.images.<camera>: 每个 <camera>_image 自动转换成一路视频
```

这里的 `action` 是“下一帧绝对目标”，最后一帧重复自身：

```text
action[i] = [pose[i+1], joint[i+1], gripper[i+1]]
action[-1] = [pose[-1], joint[-1], gripper[-1]]
```

为什么之前 8 维 action 也能转换？

- LeRobot v2.1 是按 `meta/info.json` 里的 feature schema 解释数据的格式。
- 它不强制 `action` 必须是固定维度；只要 parquet 里的列和 `meta/info.json` 声明一致，8 维或 14 维都可以是格式合法的 LeRobot v2.1。
- 之前的 8 维 action 表示“下一帧关节 + 夹爪”，所以格式上没问题。
- 当前改成 14 维，是因为我们希望 action 明确包含 `abs ee pose + abs joint + abs gripper`，这是训练语义选择，不是因为加了相机才必须改 action 维度。

多相机只影响图像字段：

- 新增相机会多出新的 `<camera>_image`、`<camera>.mp4` 和 LeRobot 的 `observation.images.<camera>`。
- 新增相机不会改变 `pose`、`joint`、`gripper`、`observation.state` 或 `action` 的维度。
- 转换整个 task 时，同一个输出 LeRobot 数据集内所有 episode 必须有相同的相机字段；不要把旧的单相机 episode 和新的双相机 episode 混到同一次 task 转换里。

## 转成 HDF5 后的字段

`franka_hdf5` 转换器会把每条 episode 写成一个 `.hdf5` 文件。

单条转换：

```bash
bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3
```

整个 task 转换：

```bash
bash 11_convert_task_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block
```

HDF5 的核心字段是：

```text
observations/state:   float32 (N, 8)   [j1..j7, gripper_command]
observations/ee_pose: float32 (N, 6)   [x, y, z, rx, ry, rz]
observations/joint:   float32 (N, 7)   [j1..j7]
observations/gripper: float32 (N,)     gripper_command, 0=open, 1=closed
action:               float32 (N, 14)  下一帧 [ee_pose(6), state(8)]
timestamp:            float32 (N,)     i / fps
source_timestamp:     float64 (N,)     原始 pkl.gz 里的 unix timestamp
keyframes:            int64   (K,)
observations/images/<camera>: uint8 (N, H, W, 3), RGB
```

HDF5 里的 `action` 和 LeRobot 转换保持同一套语义：下一帧绝对 `abs ee + abs joint + abs gripper`。相机数量只影响 `observations/images/<camera>`，不影响 action 维度。

如果只需要 right 视角：

```bash
bash 11_convert_task_to_hdf5.sh \
  /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer \
  /home/pnp/Desktop/franka_hdf5_data/put_eraser_into_drawer_10hz_right \
  --camera right \
  --fps 10 \
  --overwrite
```

## 30Hz 转 10Hz Right-Only 数据

如果原始数据是 30Hz，并且只想保留 right 视角，先运行：

```bash
bash 12_downsample_task_to_10hz.sh \
  /home/pnp/Desktop/franka_record_data/put_eraser_into_drawer \
  /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer \
  --camera right \
  --source-fps 30 \
  --target-fps 10 \
  --overwrite
```

输出仍然是当前 capture 格式：

```text
/home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer/
  0/
    0.pkl.gz
    keyframes.json
    right.mp4
  1/
    1.pkl.gz
    keyframes.json
    right.mp4
  downsample_metadata.json
```

降采样规则：

```text
selected_source_indices = [0, 3, 6, 9, ...]
```

每帧只保留：

```text
pose
joint
gripper
gripper_width
gripper_target_width
timestamp
right_image
```

不会写入 `wrist_image` 或 `wrist.mp4`。

详细说明见：

```text
franka_hdf5/README.md
franka_downsample/README.md
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
- `gripper` 用于复现夹爪命令；新数据是 `0=open, 1=closed`。
  如果 episode 里有 `gripper_target_width`，replay 会用它发实际宽度事件。
- `timestamp` 用于按原始时间节奏 replay。

`pose` 和图像不会直接用于当前 replay；它们主要用于后续训练、分析或可视化。
