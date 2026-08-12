# 架构与所有权

## 运行链路

```text
A/B/C 或 Hub
  -> franka_gui.hub：身份确认、模式选择、辅助窗口
  -> MainWindow + ProcessManager：控制栈生命周期和 ready 检查
  -> CaptureController：相机/机器人状态采集、episode 状态机
  -> EpisodeWriter / AsyncSaver：v3 MP4+pkl 写入及原子提交
  -> ~/Desktop/Muka_NAS/<task>/<quality>/<index>/
```

Replay、Validator、Monitor 是旁路工具，不属于录制写入链路：

- Replay 复用 `franka_replay`，单臂走 `7_replay_fr3.sh`，双臂走 `16_replay_bi_arm_pipeline.sh`。
- Validator 走 `validate.validate_task`，只读 episode 并输出 PASS/WARN/FAIL。
- Monitor 走 `work_monitor.dashboard`，只读本地工时 SQLite 账本。

## 模式职责

| 模式 | GUI 所在机 | 控制栈 | 相机集合 | 机器人状态入口 |
| --- | --- | --- | --- | --- |
| A / `single` | 170 | 170 上单臂 `1-4` | `left_wrist,left,middle` | `127.0.0.1:6001` |
| B / `right` | 170 | 170 通过 `15` 在 131 启动 right `1-4` | `middle,right,right_wrist` | 默认 `192.168.1.131:6001` |
| C / `dual` | 170 | 170 本地 left `1-4`，SSH 启动 131 right `1-4` | 全部配置相机 | 左 `127.0.0.1:6002`，右默认 `192.168.1.131:6001` |

`15_record_bi_arm_pipeline.sh` 还建立 `127.0.0.1:16001 -> 131:6001` 的 SSH 隧道，用于编排和兼容路径。直连右臂 endpoint 与本地隧道是两个不同地址，不应互换配置含义。

## 两机边界

### 170 左机拥有

- GUI、数采员身份、相机预览与录制。
- A 的完整单臂栈，以及 C 的左臂栈。
- B/C 对 131 的 SSH 编排、脚本同步、ready 检查和清理。
- NAS 提交、本地 outbox、工时账本、数据复核和 Validator。

### 131 右机拥有

- `right_franka/1-4` 对应的右臂 robot、gripper、ZMQ node 和 teleop env。
- 右臂硬件端口、串口和远端日志。
- 不拥有 Hub、Xpra 会话、相机数据目录或左臂服务。

### 禁止跨界

- Xpra 只连 `muka@192.168.1.170`；右臂 SSH 固定为 `pnp@192.168.1.131`。
- B/C 必须从 170 发起，不在 131 再启动一套 Hub。
- 同一机械臂不能同时由 teleop、recording replay 或闭环客户端占用。
- 旧的 `192.168.1.114` 不是当前生产边界，不应出现在新配置中。

## 目录所有权

| 目录/文件组 | 所有权与职责 |
| --- | --- |
| 根目录 `*.sh` | 面向操作员的入口和跨模块编排；不承载数据格式实现 |
| `franka_gui/` | Hub、采集 UI、进程生命周期、异步保存和 Replay 启动协调 |
| `franka_capture/` | 相机、机器人 ZMQ 读取、单/双臂录制和 v3 episode 写入 |
| `franka_replay/` | episode 解析、臂侧推断和单/双臂 Replay |
| `validate/` | 只读数据完整性与质量阈值检查 |
| `work_monitor/` | 工时事件账本、聚合和只读仪表盘 |
| `data_review/` | 已采数据人工复核界面 |
| `franka_sync/` | `local-outbox` 到 NAS 的校验、锁、发布和恢复 |
| `left_franka/`、`right_franka/` | 双臂编排调用的单机低层启动脚本 |
| `franka_lerobot/`、`franka_hdf5/`、`franka_downsample/` | 离线格式转换与降采样，不参与在线控制 |
| `pointcloud/` | RGB-D 录制、查看和外参标定专项工具 |
| `teleop/`、`polymetis/` | 仓库内置依赖；由底层遥操作/控制运行时拥有，业务改动应留在集成模块 |

## 数据边界

当前 GUI/CLI 产出 `franka_single_v3` 或 `franka_dual_v3`。每个 episode 的目录包含轨迹 pkl、`metadata.json`、每相机 MP4、预览/关键帧等附属文件。v3 的 RGB 不嵌入 pkl；读取器和 Replay 仍兼容部分历史格式，但兼容不等于历史数据已迁移。
