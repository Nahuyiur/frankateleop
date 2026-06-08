# franka_hdf5

这个模块负责把 `franka_capture` 采集到的 `pkl.gz` 数据转换成 HDF5 格式。

它只读取已经保存好的数据，不连接机器人、不连接相机，也不修改录制、回放或 LeRobot 转换逻辑。

## 单条 Episode 转换

```bash
bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3
```

默认输出：

```text
/home/pnp/Desktop/franka_hdf5_data/pick_block_episode_3.hdf5
```

如果文件已经存在，需要显式覆盖：

```bash
bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --overwrite
```

如果这条数据是之前按 15fps 录的，转换时显式指定：

```bash
bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --fps 15 --overwrite
```

也可以指定输出文件：

```bash
bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3 /home/pnp/Desktop/debug.hdf5 --overwrite
```

## 整个 Task 转换

```bash
bash 11_convert_task_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block
```

默认输出：

```text
/home/pnp/Desktop/franka_hdf5_data/pick_block/
```

目录结构：

```text
pick_block/
  metadata.json
  episodes/
    episode_000000.hdf5
    episode_000001.hdf5
    ...
```

如果输出目录已经存在，需要显式覆盖：

```bash
bash 11_convert_task_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block --overwrite
```

如果只保留单个相机，例如只保留 right 视角：

```bash
bash 11_convert_task_to_hdf5.sh \
  /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer \
  /home/pnp/Desktop/franka_hdf5_data/put_eraser_into_drawer_10hz_right \
  --camera right \
  --fps 10 \
  --overwrite
```

`--camera right` 会只写入：

```text
observations/images/right
```

不会写入 wrist。

## HDF5 单文件结构

每个 `.hdf5` 文件对应一条 episode。

```text
/
  attrs:
    format_version
    task
    robot_type
    fps
    episode_index
    source_episode_index
    source_pkl
    num_frames
    state_names
    pose_names
    action_names
    camera_names
    image_color_order
    source_image_color_order
    action_semantics

  frame_index              int64    (N,)
  timestamp                float32  (N,)
  source_timestamp         float64  (N,)
  keyframes                int64    (K,)

  observations/
    state                  float32  (N, 8)
    ee_pose                float32  (N, 6)
    joint                  float32  (N, 7)
    gripper                float32  (N,)
    images/
      <camera_name>        uint8    (N, H, W, 3)

  action                   float32  (N, 14)
```

## 字段语义

`observations/state`：

```text
[j1, j2, j3, j4, j5, j6, j7, gripper_command]
```

- 维度：8
- 关节角单位：弧度
- `gripper_command`：二值闭合命令，`0=open, 1=closed`

`observations/ee_pose`：

```text
[x, y, z, rx, ry, rz]
```

- 维度：6
- 位置单位：米
- 姿态单位：弧度
- 欧拉角顺序：xyz

`action`：

```text
[next_x, next_y, next_z, next_rx, next_ry, next_rz,
 next_j1, next_j2, next_j3, next_j4, next_j5, next_j6, next_j7,
 next_gripper_command]
```

- 维度：14
- 语义：下一帧绝对目标
- 最后一帧：重复最后一帧自身

也就是：

```text
action[i] = [ee_pose[i+1], state[i+1]]
action[-1] = [ee_pose[-1], state[-1]]
```

`observations/images/<camera_name>`：

- shape：`(N, H, W, 3)`
- dtype：`uint8`
- 颜色顺序：RGB
- 原始 `pkl.gz` 里的图像是 OpenCV BGR，转换时会写成 RGB。

## Timestamp

HDF5 中保存两种时间：

- `timestamp`：按转换时指定的 `fps` 生成的相对时间，单位秒，`timestamp[i] = i / fps`。
- `source_timestamp`：原始录制时 `pkl.gz` 中保存的系统时间戳，单位秒。

如果要和 LeRobot 转换保持一致，训练时通常使用 `timestamp`；如果要分析真实采集抖动，可以看 `source_timestamp`。

## 多相机要求

单条 episode 会自动检测所有 `*_image` 字段，例如：

```text
wrist_image  -> observations/images/wrist
right_image  -> observations/images/right
```

转换整个 task 时，所有 episode 必须有相同的相机字段和图像 shape。不要把旧的单相机数据和新的双相机数据直接混在同一个 task 转换里。

如果传入 `--camera right`，一致性检查只针对 right 视角。

## 30Hz 到 10Hz Right-Only 推荐流程

对于已经采好的 30Hz 双相机数据，推荐先生成一份 10Hz right-only capture-format 数据，再转 HDF5：

```bash
bash 12_downsample_task_to_10hz.sh \
  /home/pnp/Desktop/franka_record_data/put_eraser_into_drawer \
  /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer \
  --camera right \
  --source-fps 30 \
  --target-fps 10 \
  --overwrite

bash 11_convert_task_to_hdf5.sh \
  /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer \
  /home/pnp/Desktop/franka_hdf5_data/put_eraser_into_drawer_10hz_right \
  --camera right \
  --fps 10 \
  --overwrite
```

第一步输出仍然有 `pkl.gz`、`keyframes.json`、`right.mp4`；第二步输出通用 HDF5。

## 依赖

转换脚本默认使用 `data_convert` 环境：

```bash
conda activate data_convert
python -m pip install h5py
```

当前环境已经安装过 `h5py`。

## 快速读取示例

```python
import h5py

path = "/home/pnp/Desktop/franka_hdf5_data/pick_block_episode_3.hdf5"

with h5py.File(path, "r") as f:
    state = f["observations/state"][:]
    ee_pose = f["observations/ee_pose"][:]
    action = f["action"][:]
    wrist = f["observations/images/wrist"][0]

print(state.shape)    # (N, 8)
print(ee_pose.shape)  # (N, 6)
print(action.shape)   # (N, 14)
print(wrist.shape)    # (H, W, 3), RGB
```
