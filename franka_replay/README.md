# franka_replay

离线读取 `franka_capture` 录制的 episode，并可选择通过现有 robot node 复现 joint 轨迹。

默认模式只检查数据和当前机器人是否接近轨迹第一帧，不会控制机械臂。如果起点
偏差超过普通阈值、但仍在 `--approach-start-max-delta` 内，dry-run 会通过并提示
`--execute --approach-start` 会先慢速靠近 frame 0。

当前数采格式支持：

```text
/home/pnp/Desktop/franka_record_data/<task_name>/<High_Quality|Low_Quality|Failure>/<episode_index>/
  metadata.json
  instruction.txt
  keyframes.json
  <episode_index>.pkl.gz
  preview_all.mp4
```

旧格式 `task_name/episode_index/*.pkl.gz` 仍然可以读取。

## 用法

### 单臂 replay

检查轨迹。单臂默认 `--arm auto`，优先读取 `metadata.json` 的 `arm_side`，所以左臂/右臂数据都可以直接传 episode 目录：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0
```

也可以直接传 `metadata.json`：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0/metadata.json
```

如果传 task 或 quality 目录，需要显式选择最新一条，避免误播：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality --latest
```

`--latest` 按找到的 `pkl.gz` 文件 mtime 选择最新文件。如果直接传 `<task_name>/`，它会在 `High_Quality`、`Low_Quality`、`Failure` 之间一起选最新；如果只想 replay 某一类质量数据，应传具体质量目录或 episode 目录。

真正执行：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --execute
```

慢速执行：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --execute --speed 0.5
```

调整夹爪力度：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --execute --gripper-force 20
```

只检查文件、不连接 robot node：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --skip-robot-check
```

### 双臂 replay

双臂使用 pipeline 脚本，它会启动本机左臂 stack、远程右臂 stack、右臂 ZMQ/夹爪 SSH tunnel，然后调用 `franka_replay.replay_fr3_dual`：

```bash
bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/dual_task/High_Quality/0
```

真正执行：

```bash
bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/dual_task/High_Quality/0 --execute
```

双臂也支持 `metadata.json` 和 `--latest`：

```bash
bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/dual_task/Failure --latest
```

## 安全提醒

- 执行前先启动 `1_launch_robot.sh`、`2_launch_gripper.sh`、`3_launch_node.sh`。
- 单臂执行前先启动对应 robot/gripper/node stack；双臂建议直接用 `16_replay_bi_arm_pipeline.sh`。
- 执行 replay 时不要同时跑 `4_run_env.sh` 或 GUI teleop run env。
- 默认 `--max-start-delta 0.25`，超过后需要自动靠近或手动靠近第一帧。
- 默认脚本开启自动靠近检查；若偏差小于 `--approach-start-max-delta 0.75`，
  dry-run 只提示，`--execute` 时才会真的慢速靠近 frame 0。超过 0.75 会退出。
- `--execute` 才会发送机器人命令；不加时不会动机械臂。
- `7_replay_fr3.sh` 顶部可以修改默认 `DEFAULT_REPLAY_SPEED`、`DEFAULT_GRIPPER_SPEED`、`DEFAULT_GRIPPER_FORCE`。
- 命令行传入的 `--speed`、`--gripper-speed`、`--gripper-force` 会覆盖脚本默认值。
- 单臂夹爪 endpoint 默认自动推断：左臂使用本机默认夹爪 server，右臂优先跟随 metadata 的 robot host 并使用右臂默认端口；需要时用 `--gripper-host/--gripper-port` 覆盖。
- 如果输入目录下面有多条 episode，必须传精确 episode 目录或加 `--latest`，避免误播错误数据。
- 夹爪 replay 默认 `--gripper-replay-mode event`，只在目标宽度变化事件上发 direct gripper 命令。
- 连续夹爪 replay 可显式使用 `--gripper-replay-mode continuous --gripper-command-hz 15`。
- 字段优先级是 `gripper_target_width > gripper_closedness > gripper_command_raw > gripper_01closedness > legacy gripper`。
