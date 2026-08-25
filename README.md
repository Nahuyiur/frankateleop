# Franka Teleop Integration

本仓库是 Franka 左臂、右臂和双臂数据采集工作台。生产拓扑固定为：

- `192.168.100.67`：左机，账号 `muka`，仓库 `/home/muka/frankateleop`；运行 GUI、相机、左臂栈和双臂编排。
- `192.168.1.131`：右机，账号 `pnp`，仓库 `/home/pnp/frankateleop`；只运行右臂 robot/gripper/node/teleop 子栈。

不要把 131 当作 GUI 登录机，也不要把 100.67 的账号、仓库或 Xpra socket 套到 131。

## 日常启动

在 100.67 上进入仓库，只选择一个入口：

```bash
cd /home/muka/frankateleop
bash A_run_left_arm_capture_gui.sh   # A：左臂
bash B_run_right_arm_capture_gui.sh  # B：右臂，右臂子栈由 100.67 编排到 131
bash C_run_dual_arm_capture_gui.sh   # C：双臂
bash run_data_collection_hub.sh      # Hub：先选模式再进入采集
```

A/B/C 都进入同一个 Hub，并预选模式。日常使用时不要再手工并行启动 `1-4`、`15` 或另一个 A/B/C 入口。

从 macOS 使用 Xpra：

```bash
bash remote/open_franka_gui_xpra.sh A
bash remote/open_franka_gui_xpra.sh B
bash remote/open_franka_gui_xpra.sh C
```

默认 Xpra 目标是 `muka@192.168.100.67`、`/home/muka/frankateleop`、`/tmp/codex-franka-67.sock`。机器差异通过运行时配置覆盖，见 [运行时配置](docs/runtime-config.md)。

## 常用辅助命令

```bash
# 工时监控：只读本地账本，不扫描 NAS，不启动机械臂
bash M_run_worktime_monitor.sh

# 数据校验：按任务检查完整性，默认根目录为 ~/Desktop/Muka_NAS
bash V_validate_task_data.sh <task_name>

# 单臂 Replay：先文件检查，再硬件 dry-run；只有 --execute 才执行
bash 7_replay_fr3.sh <episode_dir> --skip-robot-check
bash 7_replay_fr3.sh <episode_dir>

# 双臂 Replay：同样默认不执行
bash 16_replay_bi_arm_pipeline.sh <episode_dir> --skip-robot-check
```

Replay 不是运动规划或碰撞检查。执行前必须确认臂侧、标定、起始姿态、控制栈互斥和急停可达。

## 角色速查

| 入口 | 职责 |
| --- | --- |
| A | 左臂采集，预选 `left_wrist,left,middle` |
| B | 右臂采集，预选 `middle,right,right_wrist`，右臂服务在 131 |
| C | 双臂采集，使用全部配置相机，100.67 编排两机 |
| Hub | 身份确认、模式选择、采集窗口、工时监控和数据复核的统一入口 |
| Replay | 读取已保存 episode；默认检查，显式 `--execute` 才发送运动命令 |
| Validator | 只读检查 schema、文件、视频帧数、时间、相机和机器人状态 |
| Monitor | 只读展示本地 SQLite 工时账本，不读取 NAS，不控制硬件 |

## 稳定约定

- 录制固定为 30 Hz；默认保存根目录是 `$HOME/Desktop/Muka_NAS`。
- GUI 默认 `direct-nas`，NAS 必须是真实挂载；`local-outbox` 只用于显式启用的延迟同步流程。
- 当前数据为 v3 video-first：RGB 写入各相机 MP4，pkl 保存机器人、动作、时间戳、schema 和对齐元数据。历史 v2 数据仍可能把图像放在 pkl 中。
- `teleop/` 和 `polymetis/` 是仓库内置运行依赖，不需要另行 clone；业务功能不要在其中重复实现。
- 配置、端口、目录和数据格式的完整说明见 [架构](docs/architecture.md)、[运行时配置](docs/runtime-config.md) 和 [脚本索引](docs/script-index.md)。

## 开发验证

```bash
python -m pytest -q
bash -n A_run_left_arm_capture_gui.sh B_run_right_arm_capture_gui.sh C_run_dual_arm_capture_gui.sh
```

根目录 pytest 只收集 `tests/` 中的集成层测试；`teleop/` 和 `polymetis/` 自带的上游测试需要各自完整的仿真/构建环境，不属于本仓库默认回归集。
