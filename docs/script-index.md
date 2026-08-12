# 脚本索引

## 日常入口

| 脚本 | 职责 | 是否控制硬件 |
| --- | --- | --- |
| `A_run_left_arm_capture_gui.sh` | 启动 Hub 并预选 A 左臂模式 | 可，由 GUI 显式启动栈 |
| `B_run_right_arm_capture_gui.sh` | 启动 Hub 并预选 B 右臂模式；170 编排 131 | 可 |
| `C_run_dual_arm_capture_gui.sh` | 启动 Hub 并预选 C 双臂模式 | 可 |
| `run_data_collection_hub.sh` | 不预选模式的统一入口；身份、采集、Monitor、Review | 可 |
| `D_review_capture_data.sh` | 打开数据人工复核窗口 | 否 |
| `M_run_worktime_monitor.sh` | 只读本地 SQLite 工时账本，每秒刷新 | 否 |
| `V_validate_task_data.sh` | 校验任务下 episode，输出 PASS/WARN/FAIL，可写 JSON 报告 | 否 |
| `S_sync_nas.sh` | 低优先级同步 `local-outbox` 到 NAS | 否 |

Hub 内部职责：`franka_gui.hub` 负责统一入口；`ProcessManager` 启停并检查控制栈；`CaptureController` 负责录制状态机；Monitor 和 Review 是独立辅助窗口。

## Replay

| 脚本 | 适用数据 | 默认行为 |
| --- | --- | --- |
| `7_replay_fr3.sh` | 左/右单臂 episode；拒绝双臂数据 | dry-run；`--skip-robot-check` 只查文件，`--execute` 才运动 |
| `16_replay_bi_arm_pipeline.sh` | 双臂 episode；拒绝单臂数据 | dry-run；负责两机 replay 栈、转发和清理 |

优先传 episode 目录或 `metadata.json`，让 Replay 读取 `schema_version` 和 `arm_side`。裸右臂 pkl 必须显式指定 `--arm right`。执行 Replay 时不得同时运行 teleop env、GUI recording 或其他控制客户端。

## 低层采集与控制

| 脚本 | 职责 |
| --- | --- |
| `0_record_fr3_pipeline.sh` | 旧式单臂全栈编排，依次管理 `1-6`；日常 GUI 不优先使用 |
| `1_launch_robot.sh` | A 单臂 Polymetis robot server，端口 50051 |
| `2_launch_gripper.sh` | A 单臂 gripper server，端口 50052 |
| `3_launch_node.sh` | A 单臂 teleop robot node，ZMQ 6001 |
| `4_run_env.sh` | A 单臂同构臂 teleop env |
| `5_capture_image.sh` | 相机静态截图自检 |
| `6_record_fr3.sh` | CLI 单臂 30 Hz 录制 |
| `14_run_capture_gui.sh` | A 入口的兼容转发别名 |
| `15_record_bi_arm_pipeline.sh` | 从 170 编排 left/right `1-4`、SSH/隧道及双臂录制；GUI 以 stack-only 调用 |
| `left_franka/1-4` | C 模式下 170 左臂低层子栈，端口 50052/50054/6002 |
| `right_franka/1-4` | B/C 模式下 131 右臂低层子栈，端口 50051/50053/6001 |

这些低层脚本由 GUI/双臂编排器拥有生命周期。除故障定位外，不要与 A/B/C 并行手工启动。

## 离线处理

| 脚本 | 职责 |
| --- | --- |
| `8_convert_episode_to_lerobot.sh` | 单 episode 转 LeRobot |
| `9_convert_task_to_lerobot.sh` | 整个 task 转 LeRobot |
| `10_convert_episode_to_hdf5.sh` | 单 episode 转 HDF5 |
| `11_convert_task_to_hdf5.sh` | 整个 task 转 HDF5 |
| `12_downsample_task_to_10hz.sh` | task 级 30 Hz 到 10 Hz 降采样 |

离线工具不启动机器人，但会读取或创建数据目录；覆盖已有输出必须显式传对应参数。

## 专项工具

| 脚本 | 职责 |
| --- | --- |
| `13_run_eraser_closed_loop.sh` | 橡皮擦闭环策略客户端及单臂栈编排 |
| `17_monitor_left_gripper_signal.sh` | 只读监控左夹爪反馈/命令信号，不同于工时 Monitor |
| `18_record_rgb_pointclouds.sh` | 仅相机 RGB-D/点云录制，不启动机械臂 |
| `19_view_recorded_rgb_pointclouds.sh` | 导出 RGB、深度预览和相机坐标系 PLY |
| `20_pointcloud_calibration.sh` | ChArUco 生成、采样、求解、验证、变换和融合 |
| `20_calibrate_pointcloud_world.sh` | `20_pointcloud_calibration.sh` 的兼容转发入口 |
| `21_calibrate_initial_joints.sh` | 只读当前关节并保存/校验初始姿态配置 |

## 远程与配置脚本

| 脚本 | 职责 |
| --- | --- |
| `remote/open_franka_gui_xpra.sh` | macOS Xpra 客户端，默认登录 170 并选择 A/B/C |
| `remote/remote_open_gui.sh` | 170 端稳定映射 A/B/C 到当前 GUI 脚本 |
| `scripts/franka_gui_bootstrap.sh` | conda、GUI 依赖、dialout 和共享默认值装配 |
| `scripts/franka_runtime_config.sh` | Shell 启动层的 170/131、仓库、端口和 Xpra 默认值入口 |
| `scripts/resolve_left_teleop_port.sh` | 解析左侧同构臂串口并拒绝歧义 |

`teleop/` 和 `polymetis/` 是上述启动脚本直接引用的仓库内置依赖，不是独立部署步骤。各 Python 模块的更细参数以对应模块 README 和 `--help` 为准。
