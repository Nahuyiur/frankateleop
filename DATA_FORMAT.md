# FR3 数据采集保存格式说明

本文档说明当前 `franka_capture` 数据采集脚本保存的数据结构、字段含义和维度定义。

当前主要采集入口：

```bash
bash 6_record_fr3.sh <task>
bash A_run_left_arm_capture_gui.sh
bash B_run_right_arm_capture_gui.sh
bash C_run_dual_arm_capture_gui.sh
```

CLI 默认保存根目录：

```text
/home/pnp/Desktop/franka_record_data
```

A/B/C GUI 的最终保存根目录是：

```text
/home/pnp/Desktop/Muka_NAS
```

A/B/C GUI 默认直接在 NAS 的 `.recording/<uuid>/episode/` 隐藏 staging 目录录制；
视频、pkl 和 metadata 完整写入后才原子发布为最终 episode 目录。NAS 未挂载时会拒绝
开始录制，避免误写入本地同名目录。旧的本地 outbox 同步模式可通过
`FRANKA_GUI_STORAGE_MODE=local-outbox` 显式启用。

最终每条 episode 保存到：

```text
<output_root>/<task>/<episode_index>/
```

GUI 入口会在保存前要求质量分层，保存路径为：

```text
<output_root>/<task>/<quality>/<episode_index>/
```

其中 `<quality>` 为 `High_Quality`、`Low_Quality` 或 `Failure`。三个目录
独立计数，分别从 `0` 开始编号。`Failure` 用于值得保留的失败轨迹；明显误操作、
无训练价值的数据应继续使用 `d` 丢弃，不会写入任何质量目录。

例如：

```text
/home/pnp/Desktop/franka_record_data/pick_block/3/
/home/pnp/Desktop/Muka_NAS/pick_block/High_Quality/3/
/home/pnp/Desktop/Muka_NAS/pick_block/Failure/0/
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

当前 CLI 双臂和 GUI 会保存 `metadata.json`；新版 CLI 单臂也会保存
`metadata.json`。GUI 还会保存 `instruction.txt`，并在 `metadata.json` 中写入
`text_instruction`、`quality` 和 `relative_episode_dir`。当前没有保存 depth 数据。

## `pkl.gz` 顶层结构

`<episode_index>.pkl.gz` 是 gzip 压缩后的 pickle 文件。

读取方式：

```python
import gzip
import pickle

path = "/home/pnp/Desktop/franka_record_data/pick_block/3/3.pkl.gz"
# GUI 质量分层数据示例：
# path = "/home/pnp/Desktop/franka_record_data/pick_block/High_Quality/3/3.pkl.gz"

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
    "schema_version": "franka_single_v3",
    "image_storage": {...},
}
```

其中：

- `data`：逐帧数据列表。
- `keyframes`：关键帧索引列表。默认包含 `[0]`；录制时按 `k` 会追加当前帧索引。

## 单帧 `frame` 结构

当前维护的 GUI 与 CLI 录制入口都使用 v3：每帧只保存机器人状态、动作对齐字段和
时间戳，画面通过同目录 MP4 的相同 `frame_index` 读取。RGB 不会重复写入 pkl。
旧 v1/v2 历史数据仍可能包含内嵌图像字段。单臂 v3 示例：

```python
{
    "schema_version": "franka_single_v3",
    "frame_index": i,
    "pose": [x, y, z, rx, ry, rz],
    "joint": [j1, j2, j3, j4, j5, j6, j7],
    "gripper_closedness": continuous_closedness,
    "gripper_01closedness": binary_closedness,
    "gripper_width": actual_width,
    "gripper_target_width": target_width,
    "timestamp": unix_time,
}
```

旧 v2 数据仍会为每路相机保存内嵌字段：

```python
"<camera_name>_image"
```

例如：

```python
"wrist_image"
"right_image"
"exterior_1_image"
```

`schema_version` 缺失的数据视为历史单臂格式；当前 GUI 与 CLI 都使用
`franka_single_v3` / `franka_dual_v3`。replay 和 validator 兼容单/双臂旧 v1/v2 及新 v3；现有
LeRobot/HDF5 转换和 downsample 对单臂 v3 兼容，双臂转换能力没有因此扩展。

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

### Gripper 字段

新数据每帧只保存四个夹爪字段：

```text
gripper_closedness
gripper_01closedness
gripper_width
gripper_target_width
```

`gripper_closedness`：

```text
连续闭合度，0=open, 1=closed，无单位
```

`gripper_01closedness`：

```text
二值闭合状态，gripper_closedness >= 0.5 -> 1，否则为 0，无单位
```

`gripper_width`：

```text
实际反馈开口宽度，单位米；0.0=闭合，0.09≈最大张开
```

`gripper_target_width`：

```text
命令目标开口宽度，单位米；0.0=闭合，0.09≈最大张开
```

旧数据里的 `gripper`、`gripper_closed`、`gripper_command_raw` 仍可由转换工具读取作为 fallback。
Replay 的旧字段 fallback 以可推导目标宽度为准，详见下方 replay 优先级。新采集不再写这些旧字段。


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

旧 v2 内嵌图像注意事项：

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
- v2 中视频主要用于快速查看，图像也内嵌在 `pkl.gz`。
- v3 中 MP4 是唯一 RGB 数据源，`pkl.gz` 保存逐帧 robot state/action/timestamp。
- v3 相机 MP4 保持采集输入的完整 `640x480 @ 30 FPS`，不做缩放。
- `metadata.json.image_storage` 记录每路视频的宽、高、通道数和帧数。

## 当前默认相机配置

配置文件：

```text
franka_capture/config/fr3_single.py
```

当前配置里的 RGB 相机会按 `franka_capture/config/fr3_single.py` 的
`DEFAULT_CAMERAS` 动态决定。典型配置如下：

```python
"left_wrist": CameraConfig(
    name="left_wrist",
    serial_number="348122072222",
    fps=30,
    depth=False,
),
"left": CameraConfig(..., fps=30, depth=False),
"middle": CameraConfig(..., fps=30, depth=False),
"right": CameraConfig(..., fps=30, depth=False),
"right_wrist": CameraConfig(
    name="right_wrist",
    serial_number="347622075798",
    fps=30,
    depth=False,
),
```

说明：

- 录制频率固定为 `30 Hz`。
- RealSense 相机 stream FPS 和 mp4 保存 FPS 都使用 `30 Hz`。
- 脚本入口不再提供 `--fps`、`DEFAULT_FPS`、`--camera-fps` 或 `--video-fps` 覆盖。
- `depth=False`：当前不采集深度。
- `dim=(640, 480)`：默认分辨率。

## 当前录制 FPS

当前只有一种录制 FPS：`30 Hz`。GUI 和命令行录制入口都会固定使用这个频率。

## 双臂采集格式

双臂采集入口：

```bash
bash 15_record_bi_arm_pipeline.sh <task>
```

CLI 与 GUI 双臂 episode 都使用 `franka_dual_v3`。两者的
机器人字段都使用显式前缀，v3 不再在 frame 内重复保存图像：

```python
{
    "schema_version": "franka_dual_v3",
    "frame_index": i,
    "timestamp": unix_time,
    "left_pose": [x, y, z, rx, ry, rz],
    "left_joint": [j1, j2, j3, j4, j5, j6, j7],
    "left_gripper_closedness": left_continuous_closedness,
    "left_gripper_01closedness": left_binary_closedness,
    "left_gripper_width": left_actual_width,
    "left_gripper_target_width": left_target_width,
    "right_pose": [x, y, z, rx, ry, rz],
    "right_joint": [j1, j2, j3, j4, j5, j6, j7],
    "right_gripper_closedness": right_continuous_closedness,
    "right_gripper_01closedness": right_binary_closedness,
    "right_gripper_width": right_actual_width,
    "right_gripper_target_width": right_target_width,
}
```

双臂数据不会写单臂兼容的 `pose/joint/gripper` 顶层字段，避免旧转换器误把双臂 episode 当单臂数据处理。后续转换到 LeRobot/HDF5 时应使用双臂转换逻辑。

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

运行 A/B/C GUI 时，`Text instruction` 必填。按 `e`、`q` 或点击 `保存当前`
会先进入蓝色 `JUDGING` 状态，不会立刻落盘；随后必须选择质量分层：

```text
h = 保存到 High_Quality
l = 保存到 Low_Quality
f = 保存到 Failure
d = 丢弃当前 episode，不写入任何质量目录
```

## 快速检查数据

可以用下面脚本检查一条数据：

```python
import gzip
import pickle
import numpy as np

path = "/home/pnp/Desktop/franka_record_data/pick_block/3/3.pkl.gz"
# GUI 质量分层数据示例：
# path = "/home/pnp/Desktop/franka_record_data/pick_block/High_Quality/3/3.pkl.gz"

with gzip.open(path, "rb") as f:
    obj = pickle.load(f)

frames = obj["data"]

poses = np.array([x["pose"] for x in frames])
joints = np.array([x["joint"] for x in frames])
grippers = np.array([x["gripper_closedness"] for x in frames])
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
observation.state:   8 维 [j1, j2, j3, j4, j5, j6, j7, gripper_closedness]
observation.ee_pose: 6 维 [x, y, z, rx, ry, rz]
action:             14 维 下一帧 [x, y, z, rx, ry, rz, j1, ..., j7, gripper_closedness]
observation.images.<camera>: 每个 <camera>_image 自动转换成一路视频
```

这里的 `action` 是“下一帧绝对目标”，最后一帧重复自身：

```text
action[i] = [pose[i+1], joint[i+1], gripper_closedness[i+1]]
action[-1] = [pose[-1], joint[-1], gripper_closedness[-1]]
```

为什么之前 8 维 action 也能转换？

- LeRobot v2.1 是按 `meta/info.json` 里的 feature schema 解释数据的格式。
- 它不强制 `action` 必须是固定维度；只要 parquet 里的列和 `meta/info.json` 声明一致，8 维或 14 维都可以是格式合法的 LeRobot v2.1。
- 之前的 8 维 action 表示“下一帧关节 + 夹爪”，所以格式上没问题。
- 当前改成 14 维，是因为我们希望 action 明确包含 `abs ee pose + abs joint + abs gripper_closedness`，这是训练语义选择，不是因为加了相机才必须改 action 维度。

多相机只影响图像字段：

- 新增相机会多出新的 `<camera>_image`、`<camera>.mp4` 和 LeRobot 的 `observation.images.<camera>`。
- 新增相机不会改变 `pose`、`joint`、`gripper_closedness`、`observation.state` 或 `action` 的维度。
- 转换整个 task 时，同一个输出 LeRobot 数据集内所有 episode 必须有相同的相机字段；不要把旧的单相机 episode 和新的双相机 episode 混到同一次 task 转换里。

## 转成 HDF5 后的字段

`franka_hdf5` 转换器会把每条 episode 写成一个 `.hdf5` 文件。

单条转换：

```bash
bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3
# GUI 质量分层数据示例：
bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/3
```

整个 task 转换：

```bash
bash 11_convert_task_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block
```

如果 task 目录下有 `High_Quality`、`Low_Quality` 或 `Failure` 子目录，转换器会
按质量目录展开读取，并在输出 metadata 中保留 quality 信息。

HDF5 的核心字段是：

```text
observations/state:   float32 (N, 8)   [j1..j7, gripper_closedness]
observations/ee_pose: float32 (N, 6)   [x, y, z, rx, ry, rz]
observations/joint:   float32 (N, 7)   [j1..j7]
observations/gripper_closedness: float32 (N,)  continuous closedness
observations/gripper_01closedness: float32 (N,) binary closedness
observations/gripper_width: float32 (N,) actual width in meters
observations/gripper_target_width: float32 (N,) target width in meters
action:               float32 (N, 14)  下一帧 [ee_pose(6), state(8)]
timestamp:            float32 (N,)     i / fps
source_timestamp:     float64 (N,)     原始 pkl.gz 里的 unix timestamp
keyframes:            int64   (K,)
observations/images/<camera>: uint8 (N, H, W, 3), RGB
```

HDF5 里的 `action` 和 LeRobot 转换保持同一套语义：下一帧绝对 `abs ee + abs joint + abs gripper_closedness`。相机数量只影响 `observations/images/<camera>`，不影响 action 维度。

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
gripper_closedness
gripper_01closedness
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
gripper_target_width
gripper_closedness
timestamp
```

执行轨迹复现。

其中：

- `joint` 用于复现 7 维机械臂关节轨迹。
- `gripper_target_width` 优先用于复现夹爪目标宽度。
- 如果没有 `gripper_target_width`，replay 会按 `gripper_closedness` 推导目标宽度；旧数据还会依次 fallback 到 `gripper_command_raw`、`gripper_01closedness`、legacy `gripper`，最后可用 `gripper_width` 作为目标宽度。
- `timestamp` 用于按原始时间节奏 replay。

`pose` 和图像不会直接用于当前 replay；它们主要用于后续训练、分析或可视化。
