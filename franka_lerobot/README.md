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

转换默认写入 `--fps 10`。如果这条数据是之前按 15fps 录的，转换时显式指定：

```bash
bash 8_convert_episode_to_lerobot.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --fps 15 --overwrite
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

- `observation.state`：8 维绝对状态，`[j1, j2, j3, j4, j5, j6, j7, gripper_command]`
- `observation.ee_pose`：6 维绝对末端位姿，`[x, y, z, rx, ry, rz]`
- `action`：14 维下一帧绝对目标，`[abs_ee_pose(6), abs_joint(7), abs_gripper_command(1)]`，最后一帧重复自身
- `observation.images.<camera>`：从 `*_image` 字段自动检测并重新编码为 mp4

其中：

- 关节角单位是弧度。
- `gripper_command` 是二值闭合命令，`0=open, 1=closed`。
- 末端位置单位是米，姿态是 xyz 欧拉角弧度。

`action` 的维度和相机数量没有关系。新增相机只会新增 `observation.images.<camera>` 视频，不会改变 `observation.state`、`observation.ee_pose` 或 `action` 的维度。

之前 8 维 action 也能转换成 LeRobot v2.1，是因为 v2.1 数据格式按 `meta/info.json` 中的 feature schema 解释数据，不强制 action 必须是固定维度。只要 parquet 列和 schema 一致，8 维“下一帧关节+夹爪”和 14 维“下一帧末端+关节+夹爪”都可以是格式合法的数据。现在采用 14 维，是为了符合当前希望保存 `abs ee + abs joint + abs gripper` action 的训练语义。

转换整个 task 时，所有 episode 的相机字段必须一致。例如不要把旧的 `wrist_image` 单相机数据和新的 `wrist_image + right_image` 双相机数据直接混成同一个 LeRobot 数据集；需要分开转换，或者后续增加补齐/筛选逻辑。

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

转换脚本默认使用 `data_convert` 环境。这个环境专门用于离线数据转换和格式检查，后续也可以继续扩展 RobotWin 等格式转换。

```bash
conda activate data_convert
python -m pip install --extra-index-url https://download.pytorch.org/whl/cpu -e '/home/pnp/lerobot[dataset]'
python -m pip install 'imageio[ffmpeg]'
```
