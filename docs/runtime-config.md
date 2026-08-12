# 运行时配置

## 配置入口与优先级

共享默认值由 `scripts/franka_runtime_config.sh` 提供。优先级从高到低为：

1. 当前 shell 中显式设置的环境变量。
2. `${FRANKA_RUNTIME_CONFIG_FILE}`；未设置时读取 `$HOME/.config/frankateleop/runtime.env`。
3. 仓库内置生产默认值。

模板见 `config/runtime.env.example`。配置文件只接受 `KEY=VALUE`，键名必须以 `FRANKA_`、`BI_ARM_`、`LEFT_` 或 `RIGHT_` 开头；不要写 shell 命令，也不要保存密码。

## 生产主机

| 变量 | 默认值 | 含义 |
| --- | --- | --- |
| `FRANKA_LEFT_HOST` | `192.168.1.170` | 左机地址 |
| `FRANKA_LEFT_SSH` | `muka@192.168.1.170` | Xpra/左机 SSH 身份 |
| `FRANKA_LEFT_REPO` | `/home/muka/frankateleop` | 左机仓库 |
| `BI_ARM_RIGHT_HOST` | `192.168.1.131` | 右机地址 |
| `BI_ARM_RIGHT_SSH` | `pnp@192.168.1.131` | 右机编排身份 |
| `BI_ARM_RIGHT_REPO` | `/home/pnp/frankateleop` | 右机仓库 |
| `FRANKA_XPRA_SSH_SOCKET` | `/tmp/codex-franka-170.sock` | macOS 到 170 的复用 socket |

Xpra 的 host、repo、socket 必须作为同一组修改；这些值不能用于覆盖 131 的右臂目标。只有默认的 `muka@192.168.1.170` 会自动尝试 `/tmp/codex-franka-170.sock`；自定义 host 未显式配置 socket 时会使用普通 SSH，避免误走旧的 ControlMaster 连接。

## 端口契约

| 场景 | 位置 | robot gRPC | gripper gRPC | robot ZMQ |
| --- | --- | ---: | ---: | ---: |
| A 单臂 | 170 | 50051 | 50052 | 6001 |
| C 左臂 | 170 | 50052 | 50054 | 6002 |
| B/C 右臂 | 131 | 50051 | 50053 | 6001 |

跨机辅助端口：

- `BI_ARM_RIGHT_LOCAL_ZMQ_PORT=16001`：170 本地到 131 `6001` 的 SSH 隧道。
- `BI_ARM_RIGHT_LOCAL_GRIPPER_PORT=15053`：双臂 Replay 使用的右夹爪本地转发端口。
- `FRANKA_RIGHT_ZMQ_HOST/PORT`：GUI/录制读取右臂状态的 endpoint，默认 `192.168.1.131:6001`。

端口属于具体模式。A 的本地 `50051/50052/6001` 与 C 左臂的 `50052/50054/6002` 不是同一套栈，不能并行启动。

## 存储与数据格式

- GUI 录制固定 `30 Hz`，默认根目录 `$HOME/Desktop/Muka_NAS`。
- 默认 `FRANKA_GUI_STORAGE_MODE=direct-nas`：先写隐藏 staging，再原子提交到 `<task>/<High_Quality|Low_Quality|Failure>/<index>/`。
- 写入默认 NAS 路径前必须通过真实 mount 检查，不能退化为本地同名目录。
- `local-outbox` 使用 `$HOME/Desktop/franka_record_cache/outbox/<uuid>/`，由 `franka_sync` 延迟发布；只有显式设置时启用。
- v3 schema 为 `franka_single_v3` / `franka_dual_v3`。RGB 在 `<camera>.mp4`，pkl 保存机器人/动作/时间戳/schema/对齐元数据。
- 历史 v2 可继续读取，但不要声称它们已转换为 v3。

## 相机与状态入口

| 模式 | 默认相机 | 状态 endpoint |
| --- | --- | --- |
| A | `left_wrist,left,middle` | `127.0.0.1:6001` |
| B | `middle,right,right_wrist` | `192.168.1.131:6001` |
| C | 全部配置相机 | 左 `127.0.0.1:6002`，右 `192.168.1.131:6001` |

相机序列号的源码入口是 `franka_capture/config/fr3_single.py`；双臂 endpoint 默认值在 `franka_capture/config/fr3_dual.py`。日常机器差异优先走运行时配置，不要把机器路径散落到业务代码。

## 环境与凭据

- `franka_capture` conda 环境：Hub、GUI、相机、Validator、Monitor。
- `polymetis` conda 环境：robot/gripper server、teleop node/env、Replay。
- `data_convert` conda 环境：LeRobot、HDF5 和降采样工具。
- `teleop/` 与 `polymetis/` 已内置在仓库，启动脚本按 `$REPO_ROOT` 引用；不要额外 clone 到其他路径。
- sudo 凭据文件和 SSH 私钥不放进 `runtime.env`。凭据文件必须归当前用户所有，权限为 `600` 或 `400`。

## 操作前检查

```bash
bash A_run_left_arm_capture_gui.sh --help
bash remote/open_franka_gui_xpra.sh --help
bash M_run_worktime_monitor.sh --help
```

真实硬件启动前还需现场确认：170/131 可达、NAS 已挂载、串口唯一、机器人处于可控状态、目标端口无残留进程。文档中的默认值不代表当前服务已经 ready。
