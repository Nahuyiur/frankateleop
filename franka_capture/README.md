# franka_capture 使用说明

`franka_capture` 是一个放在仓库顶层的独立数据采集模块，只负责两件事：

- 读取 RealSense 相机的 RGB / depth 数据
- 从已经启动的 robot node 读取机器人状态，并保存成 episode 数据

它是刻意非侵入式设计的：不修改、不导入 `teleop` 或 `polymetis`，也不会向机器人发送运动命令。机器人控制仍然使用仓库原来的四个脚本启动；本模块只在旁边读取状态和相机画面。

## 目录结构

```text
franka_capture/
├── README.md
├── config/
│   └── fr3_single.py          # 默认机器人端口、相机序列号、输出目录
├── cameras/
│   └── realsense.py           # RealSense RGB/depth 读取
├── core/
│   └── robot_zmq_client.py    # 只读 ZMQ robot node 客户端
├── recording/
│   ├── episode_writer.py      # 保存 mp4 / pkl.gz / json
│   └── preview.py             # OpenCV 预览拼接
└── scripts/
    ├── capture_image.py       # 相机自检和截图
    └── record_fr3.py          # 单臂 FR3 主采集入口
```

## 依赖

推荐使用独立 conda 环境：

```bash
conda activate franka_capture
```

这个环境已经安装了数采需要的 Python 包：

```text
numpy
pyzmq
pyrealsense2
opencv-python
imageio
```

视频保存优先使用 `imageio + ffmpeg`。如果环境里没有 ffmpeg，会自动尝试用 OpenCV 的 `VideoWriter` 保存 mp4。

如果运行时提示缺 `cv2`，安装：

```bash
conda activate franka_capture
pip install opencv-python
```

如果运行时提示找不到 ffmpeg，可以安装：

```bash
conda activate franka_capture
pip install imageio-ffmpeg
```

也可以通过系统包安装 ffmpeg。

## 使用前配置

默认配置在：

```text
franka_capture/config/fr3_single.py
```

最常需要检查的是：

```python
DEFAULT_ROBOT = RobotEndpointConfig(
    host="127.0.0.1",
    port=6001,
    timeout_ms=2000,
)
```

这里对应 `3_launch_node.sh` 启动的 robot node。当前默认读取：

```text
tcp://127.0.0.1:6001
```

相机配置也在同一个文件里：

```python
DEFAULT_CAMERAS = {
    "exterior_1": CameraConfig(...),
    "exterior_2": CameraConfig(...),
    "wrist": CameraConfig(...),
}
```

每个相机需要确认：

- `name`：保存数据时使用的相机名字
- `serial_number`：RealSense 设备序列号
- `dim`：分辨率，默认 `(640, 480)`
- `fps`：相机 stream 帧率；录制入口固定使用 `30 Hz`
- `depth`：是否采集 depth
- `align_depth`：depth 是否对齐到 color
- `flip`：是否旋转 180 度

默认关闭 depth，是为了减少 USB 带宽压力。如果只接一两个相机，或者确认带宽足够，可以把对应相机的 `depth=True`。

## 推荐启动顺序

先按原来的方式启动机器人和遥操作。四个脚本保持不变：

```bash
cd /home/pnp/frankateleop
bash 1_launch_robot.sh
bash 2_launch_gripper.sh
bash 3_launch_node.sh
bash 4_run_env.sh
```

确认机器人可以正常 teleop 后，再另开一个终端进入同一个仓库目录：

```bash
cd /home/pnp/frankateleop
conda activate franka_capture
```

然后运行相机检查或数据采集。

## 相机自检

正式采集前建议先跑相机自检：

```bash
conda activate franka_capture
python -m franka_capture.scripts.capture_image
```

程序会按 `franka_capture/config/fr3_single.py` 里的配置初始化所有相机，并打开一个 `Preview` 窗口。

按键说明：

- `空格`：保存每个相机当前画面，并保存一张拼接预览图
- `q`：退出

默认截图保存到：

```text
./camera_checks/
```

也可以指定保存目录：

```bash
conda activate franka_capture
python -m franka_capture.scripts.capture_image --save-dir ./my_camera_checks
```

如果这里报错说某个 serial 没连接，先运行：

```bash
conda activate franka_capture
python - <<'PY'
import pyrealsense2 as rs
ctx = rs.context()
for dev in ctx.query_devices():
    print(dev.get_info(rs.camera_info.serial_number))
PY
```

把打印出的真实序列号写回 `franka_capture/config/fr3_single.py`。

## 开始采集

在确认四个原始脚本已经启动、机器人可以正常 teleop、相机自检正常后，运行：

```bash
bash 6_record_fr3.sh test
```

这个脚本会自动激活 `franka_capture` 环境。默认保存根目录是：

```text
/home/pnp/Desktop/franka_record_data
```

封装脚本参数说明：

- 第 1 个参数：任务名，会成为输出目录的一层，例如 `test`
- 第 2 个参数：可选，数据保存根目录，默认 `/home/pnp/Desktop/franka_record_data`
- 后续参数：可选，会继续传给 `record_fr3.py`，例如 `--port 6001`

episode 编号会自动从 `{保存根目录}/{任务名}/` 下已有的最大数字目录继续。
例如已经有 `0/`、`1/`、`4/`，下一次会从 `5/` 开始。

例子：

```bash
bash 6_record_fr3.sh pick_block
```

如果通过总控脚本启动：

```bash
bash 0_record_fr3_pipeline.sh pick_block
```

总控脚本默认会给录制入口追加 `--enable-depth --depth-cameras all`，
也就是开启所有配置相机的 aligned depth。若 USB 带宽不够，建议只开一两路：

```bash
bash 0_record_fr3_pipeline.sh pick_block --depth-cameras middle,left_wrist
```

临时关闭 depth：

```bash
bash 0_record_fr3_pipeline.sh pick_block --no-depth
```

如果要手动指定保存根目录：

```bash
bash 6_record_fr3.sh pick_block /home/pnp/Desktop/franka_record_data
```

如果确实想指定起始编号：

```bash
bash 6_record_fr3.sh pick_block --index 10
```

打开 `RGB` 窗口后，需要先用鼠标点一下这个窗口，让键盘输入生效。

按键说明：

- `s`：开始录制；保存当前 episode 后再按 `s` 会开始下一个自动编号 episode
- `w`：暂停录制
- `e`：保存当前 episode，但不退出程序
- `d`：丢弃当前 episode，并等待下一次 `s` 重新录制
- `k`：记录一个关键帧
- `q`：保存当前 episode 并退出

注意：这个脚本只读取 robot node 和相机，不会控制机器人。机器人动作仍然来自已经运行的 `4_run_env.sh`。

## 输出格式

默认保存路径：

```text
/home/pnp/Desktop/franka_record_data/{task}/{index}/
```

例如：

```text
/home/pnp/Desktop/franka_record_data/pick_block/3/
```

每个 episode 会包含：

```text
exterior_1.mp4
exterior_2.mp4
wrist.mp4
3.pkl.gz
keyframes.json
```

其中：

- `{camera_name}.mp4`：每路相机的 RGB 视频
- `{index}.pkl.gz`：完整 episode 数据，格式对齐迁移包 `record_single.py`
- `keyframes.json`：关键帧索引

每一帧数据包含：

```text
schema_version
frame_index
pose
joint
gripper_closedness
gripper_01closedness
gripper_width
gripper_target_width
timestamp
{camera_name}_depth    # 仅当 depth 录制开启
```

`gripper_closedness` 是连续闭合度，`0=open, 1=closed`。
`gripper_01closedness` 是二值闭合状态，`closedness >= 0.5 -> 1`。
`gripper_width` 和 `gripper_target_width` 是米单位开口宽度。

当前录制入口将 RGB 仅写入 `{camera_name}.mp4`；`pkl.gz` 保存状态、动作、时间戳和 `frame_index`。
同目录 `metadata.json.image_storage` 记录视频文件、尺寸和帧数，RGB 通过相同 `frame_index` 对齐读取。
旧 v1/v2 episode 才会包含 `{camera_name}_image`（OpenCV BGR）。
如果开启 depth，`{camera_name}_depth` 保存完整 aligned depth 图，shape 为 `(H, W)`，
dtype 为 `float32`，单位是米，并且已经对齐到 color 图。
`pose` 来自 robot node 的真实末端位姿，保存为 `[x, y, z, rx, ry, rz]`。
夹爪字段使用 `gripper_closedness` / `gripper_01closedness` / `gripper_width` / `gripper_target_width`。

点云不作为主数据逐帧保存，因为它可以从 depth、RGB 和 `metadata.json`
里的相机内参还原，而且逐帧保存稠密点云会非常占空间。开启 depth 后，每次保存
episode 会额外生成：

```text
depth_proof/
  summary.json
  {camera_name}_frame000000_depth.png
  {camera_name}_frame000000_cloud.ply
```

其中 `depth.png` 是深度伪彩色图，`cloud.ply` 是从当前帧的完整 depth
按 stride 抽样还原的点云 proof。也可以对已经录好的 episode 重新生成：

```bash
conda activate franka_capture
python -m franka_capture.scripts.verify_depth_episode \
  /home/pnp/Desktop/franka_record_data/pick_block/3
```

录制入口固定使用 `30 Hz`，同时作为 RealSense 相机 stream FPS 和 mp4 保存 FPS。
脚本不再提供 `--fps`、`--camera-fps` 或 `--video-fps` 覆盖。

## 常见问题

### 1. 连接不上 robot node

确认 `3_launch_node.sh` 已经启动，并且端口是 `6001`。

可以先看启动命令：

```bash
sed -n '1,120p' 3_launch_node.sh
```

如果 robot node 端口不是 `6001`，采集时手动传入：

```bash
conda activate franka_capture
python -m franka_capture.scripts.record_fr3 --task test --port 你的端口
```

### 2. OpenCV 窗口按键没反应

先用鼠标点击 `RGB` 或 `Preview` 窗口，再按键。

### 3. 多相机很卡

优先关闭外部相机 depth：

```python
depth=False
```

如果 30Hz 多相机仍然卡顿，优先检查 USB 拓扑、换 USB 3.x 端口或减少同时启用的相机数量。

### 4. 保存视频失败

确认环境里有 ffmpeg 或 OpenCV 视频后端。推荐先安装：

```bash
conda activate franka_capture
pip install imageio-ffmpeg opencv-python
```

### 5. 数据里 pose 是 0

当前 `pose` 来自 robot node 返回的 `ee_pose_euler`。如果录制脚本提示缺少 `ee_pose_euler`，说明 `3_launch_node.sh` 还在运行旧的 robot node，需要先停止旧进程并重新启动。

已经录好的旧数据不会自动变好；需要重新录制新的 episode。

## TODO: 现场检查

- 确认 RealSense serial 和相机命名。
- 确认哪些相机开启 depth。
- 确认 USB 带宽；多路 depth 如果卡顿，先关闭 exterior depth。
- 确认 `3_launch_node.sh` 当前 robot node port 是 `6001`。
- 确认输出目录空间和写权限。
- 确认当前 `fr3Robot.get_observations()` 里的 `ee_pos_quat` 和 `ee_pose_euler` 是否为真实末端位姿。
- 确认 gripper 数值方向是否符合数据集约定。

## 设计边界

- 本模块不发送任何机器人运动命令。
- 本模块不 import `teleop`。
- 本模块不 import `polymetis`。
- 本模块不修改现有四个启动脚本。
- RGB 保存为 mp4。
- `pkl.gz` 的保存逻辑对齐迁移包单臂 `record_single.py`：`{"data": frames, "keyframes": keyframes}`。
- 每个 episode 的状态数据先放在内存里，按 `e` 或 `q` 保存该 episode；长时间采集前请确认内存足够。
