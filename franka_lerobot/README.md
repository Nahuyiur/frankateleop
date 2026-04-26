# franka_lerobot

这个模块负责把 `franka_capture` 采集到的离线数据转换成 LeRobot v2.1 格式。

它只读取已经保存好的 `pkl.gz` 和图像帧，不连接机器人、不连接相机，也不修改录制或回放逻辑。

## 单条数据转换

```bash
bash 8_convert_episode_to_lerobot.sh /home/pnp/Desktop/franka_record_data/pick_block/3
```

默认输出：

```text
/home/pnp/Desktop/franka_lerobot_data/pick_block_episode_3
```

如果目录已经存在，需要显式覆盖：

```bash
bash 8_convert_episode_to_lerobot.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --overwrite
```

## 整个任务转换

```bash
bash 9_convert_task_to_lerobot.sh /home/pnp/Desktop/franka_record_data/pick_block
```

默认输出：

```text
/home/pnp/Desktop/franka_lerobot_data/pick_block
```

## 字段映射

- `observation.state`：8 维，`[j1, j2, j3, j4, j5, j6, j7, gripper_width]`
- `observation.ee_pose`：6 维，`[x, y, z, rx, ry, rz]`
- `action`：8 维，下一帧的 `[j1..j7, gripper_width]`，最后一帧重复自身
- `observation.images.<camera>`：从 `*_image` 字段自动检测并重新编码为 mp4

其中：

- 关节角单位是弧度。
- 夹爪宽度单位是米。
- 末端位置单位是米，姿态是 xyz 欧拉角弧度。

## 输出结构

```text
dataset/
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.wrist/episode_000000.mp4
  meta/info.json
  meta/tasks.jsonl
  meta/episodes.jsonl
  meta/episodes_stats.jsonl
  meta/stats.json
```

## 依赖

转换脚本默认使用 `franka_capture` 环境。如果提示缺少依赖：

```bash
conda activate franka_capture
python -m pip install pandas==2.0.3 pyarrow==14.0.2
```

