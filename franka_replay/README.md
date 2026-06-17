# franka_replay

离线读取 `franka_capture` 录制的 `pkl.gz`，并可选择通过现有 robot node 复现 joint 轨迹。

默认模式只检查数据和当前机器人是否接近轨迹第一帧，不会控制机械臂。

## 用法

检查轨迹：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3
```

真正执行：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --execute
```

慢速执行：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --execute --speed 0.5
```

调整夹爪力度：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --execute --gripper-force 20
```

只检查文件、不连接 robot node：

```bash
bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --skip-robot-check
```

## 安全提醒

- 执行前先启动 `1_launch_robot.sh`、`2_launch_gripper.sh`、`3_launch_node.sh`。
- 执行 replay 时不要同时跑 `4_run_env.sh`。
- 默认 `--max-start-delta 0.25`，当前机械臂离第一帧太远会直接退出。
- `--execute` 才会发送机器人命令；不加时不会动机械臂。
- `7_replay_fr3.sh` 顶部可以修改默认 `DEFAULT_REPLAY_SPEED`、`DEFAULT_GRIPPER_SPEED`、`DEFAULT_GRIPPER_FORCE`。
- 命令行传入的 `--speed`、`--gripper-speed`、`--gripper-force` 会覆盖脚本默认值。
- 夹爪 replay 默认 `--gripper-replay-mode event`，只在目标宽度变化事件上发 direct gripper 命令。
- 连续夹爪 replay 可显式使用 `--gripper-replay-mode continuous --gripper-command-hz 15`。
- 字段优先级是 `gripper_target_width > gripper_closedness > gripper_command_raw > gripper_01closedness > legacy gripper`。
