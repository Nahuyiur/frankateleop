# Single-Arm No-ROS1 Setup

这个版本面向：

- Ubuntu 22.04
- 不安装 ROS1
- 单臂 FR3
- `franky_servers/* + Python`
- SpaceMouse 实时控制
- RealSense 数据采集

## 当前本地配置

- Python: `/home/ubuntu/robot_env/bin/python`
- Robot IP: `172.16.0.2`
- SpaceMouse: `SpaceMouse Compact`
- Data Root: `/home/ubuntu/robot_env/data`
- Cameras:
  - `exterior_1`: `332322072361`
  - `exterior_2`: `336222070587`
  - `wrist`: `247122071645`

## 现在可用的命令

启动 franky server:

```bash
bash franky_servers/start_server.sh
```

记录当前机械臂位姿为 `RESET_POSE`:

```bash
/home/ubuntu/robot_env/bin/python -m scripts.capture_reset_pose --write-config
```

单臂采集:

```bash
/home/ubuntu/robot_env/bin/python -m scripts.record_single --task test --index 0
```

## teleop 当前状态

`python -m teleop.run` 现在会先检查安全配置。

由于你还没有提供下面三项，它会拒绝启动，这是故意做的安全保护：

- `RESET_POSE`
- `ABS_POSE_LIMIT_LOW`
- `ABS_POSE_LIMIT_HIGH`

这些值位于：

`fr3_single_arm_config.py`

## 你下一步需要补的安全参数

格式都是欧拉角版本：

```python
RESET_POSE = np.array([x, y, z, rx, ry, rz])
ABS_POSE_LIMIT_LOW = np.array([x, y, z, rx, ry, rz])
ABS_POSE_LIMIT_HIGH = np.array([x, y, z, rx, ry, rz])
```

填完后就可以启动单臂 SpaceMouse teleop:

```bash
/home/ubuntu/robot_env/bin/python -m teleop.run
```
