# Franka Migration Bundle

这个目录是从当前仓库里抽出来的一份“迁移参考包”，专门给你拿去另一台机器上配置 Franka。

它刻意排除了 `demo_maze` 相关内容，只保留下面三类东西：

- 单臂、无 ROS1 的 `franky` 路线
- 双臂、ROS1 的 `robot_servers` 路线
- 这两条路线共用的相机和环境代码

如果你现在的目标是先把一台新机器上的单臂 FR3 跑通，优先看 `single_arm_no_ros1/`。

如果你后面要迁移双臂采集，再看 `dual_arm_ros1/`。

## 目录结构

```text
franka_migration_bundle/
├── README.md
├── single_arm_no_ros1/
│   ├── SINGLE_ARM_NO_ROS1.md
│   ├── fr3_single_arm_config.py
│   ├── record.sh
│   ├── franky_servers/
│   ├── teleop/
│   └── scripts/
├── dual_arm_ros1/
│   ├── README.md
│   ├── bashes/
│   ├── robot_servers/
│   ├── teleop_dual/
│   └── scripts/
└── shared/
    └── envs/
```

## 推荐阅读顺序

### 如果你迁移单臂

1. `single_arm_no_ros1/SINGLE_ARM_NO_ROS1.md`
2. `single_arm_no_ros1/fr3_single_arm_config.py`
3. `single_arm_no_ros1/franky_servers/start_server.sh`
4. `single_arm_no_ros1/scripts/capture_image.py`
5. `single_arm_no_ros1/scripts/capture_reset_pose.py`
6. `single_arm_no_ros1/teleop/config.py`
7. `single_arm_no_ros1/scripts/record_single.py`

### 如果你迁移双臂

1. `dual_arm_ros1/README.md`
2. `dual_arm_ros1/bashes/launch_left_server.sh`
3. `dual_arm_ros1/bashes/launch_right_server.sh`
4. `dual_arm_ros1/teleop_dual/config.py`
5. `dual_arm_ros1/scripts/record_dual.py`

## Single Arm: 每个文件是干什么的

### `single_arm_no_ros1/SINGLE_ARM_NO_ROS1.md`

这是单臂迁移说明入口。

它告诉你：

- 目标环境是什么
- 依赖哪条控制链路
- 启动 `franky server` 的命令
- 怎样记录 `RESET_POSE`
- 怎样开始单臂采集

适用场景：

- 新机器第一次搭环境
- 想快速知道单臂最小可用流程

### `single_arm_no_ros1/fr3_single_arm_config.py`

这是单臂最重要的配置文件。你迁移到新机器时，优先改这个。

这里集中定义了：

- `ROBOT_IP`: 机械臂控制箱 IP
- `SERVER_URL`: 本地 HTTP 服务地址
- `SPACEMOUSE_DEVICE`: SpaceMouse 设备名
- `DATA_OUTPUT_ROOT`: 数据保存根目录
- `PYTHON_BIN`: Python 解释器路径
- `RELATIVE_DYNAMICS_FACTOR`: 动力学比例
- `REALSENSE_CAMERAS`: 每个相机的序列号、帧率、是否启用深度
- `RESET_POSE`: 单臂复位姿态
- `ABS_POSE_LIMIT_LOW` / `ABS_POSE_LIMIT_HIGH`: 安全工作空间边界
- `ACTION_SCALE`: 遥操作动作缩放

迁移到新机器时最常改的就是：

- 机器人 IP
- 相机序列号
- 数据目录
- SpaceMouse 名字
- 安全边界

### `single_arm_no_ros1/franky_servers/start_server.sh`

这是单臂 Franka HTTP 服务的启动脚本。

它会做这些事：

- 杀掉旧的 `server.py`
- 用指定 Python 启动 `franky_servers/server.py`
- 检查 `http://127.0.0.1:5000/health`
- 调用 `/connect` 自动连接机器人

你迁移时通常要改：

- `PYTHON_BIN`
- `ROBOT_IP`

什么时候用：

- 每次正式开始 teleop / 采集前
- 新机器第一次验证服务能否正常启动时

### `single_arm_no_ros1/franky_servers/server.py`

这是单臂底层控制服务本体，负责把上层 Python 调用转成机器人动作。

它提供的核心接口包括：

- `/health`: 查看服务状态
- `/connect`: 连接机器人
- `/recover`: 清错
- `/state`, `/getpos_euler`, `/getq`, `/get_gripper`: 读取状态
- `/motion/goto_pose`, `/motion/goto_joint`: 发送运动
- `/gripper/open`, `/gripper/close`: 控制夹爪

什么时候看它：

- 想知道 teleop 和采集脚本到底在调哪些接口
- 想改运动模式、同步异步行为、碰撞阈值
- 新机器上服务能起但行为不对时

### `single_arm_no_ros1/franky_servers/TROUBLESHOOTING.md`

这是单臂排障文档。

适合处理这些问题：

- 5000 端口没起来
- `server.py` 崩了
- reset 失败
- 客户端连不上服务器

建议你在新机器上把它当作“启动失败应急手册”。

### `single_arm_no_ros1/teleop/config.py`

这是单臂遥操作配置层。

它把 `fr3_single_arm_config.py` 里的内容喂给 `FrankaEnv`，包括：

- 服务地址
- 机器人 IP
- SpaceMouse 设备名
- 复位位姿
- 安全边界
- 是否自动连接
- 是否异步运动

什么时候改它：

- 你想固定 `MOTION_ASYNC`
- 你想改遥操作层行为，但不想改底层服务

### `single_arm_no_ros1/teleop/run.py`

这是单臂 teleop 启动入口。

作用很简单：

- 建环境
- reset 到初始位姿
- 进入持续 step 的循环

什么时候用：

- `franky server` 已启动后，开始 SpaceMouse 遥操作

### `single_arm_no_ros1/teleop/wrapper.py`

这是单臂 teleop 的 reset 行为定义。

重点是：

- reset 前先清错
- 先抬高再回 `RESET_POSE`
- 支持可选 joint reset

什么时候看：

- 机械臂回初始位姿的动作不符合预期
- 你想改 reset 轨迹

### `single_arm_no_ros1/scripts/capture_reset_pose.py`

这是“读取当前末端位姿并写回配置”的工具脚本。

用途：

- 把机械臂手动摆到一个你认可的 home / reset 位姿
- 运行它，自动把当前位姿写进 `fr3_single_arm_config.py`

典型用法：

```bash
/path/to/python -m scripts.capture_reset_pose --write-config
```

非常适合新机器第一次标定。

### `single_arm_no_ros1/scripts/capture_image.py`

这是相机自检脚本。

用途：

- 按 `SingleArmConfig.REALSENSE_CAMERAS` 初始化相机
- 预览一个选中的相机画面
- 按空格保存截图

建议在正式采集前先跑它，用来验证：

- 序列号对不对
- 相机能不能出图
- USB 带宽和权限有没有问题

### `single_arm_no_ros1/scripts/record_single.py`

这是单臂主采集脚本。

它会同时做三件事：

- 从机器人服务读取 `pose/joint/gripper`
- 从 RealSense 读取 RGB
- 保存为 `mp4 + pkl.gz + keyframes.json`

键盘操作：

- `s`: 开始录
- `w`: 停止录
- `k`: 记关键帧
- `q`: 退出并保存

什么时候用：

- teleop 已经跑起来以后，正式录演示数据

### `single_arm_no_ros1/record.sh`

这是单臂一键串联脚本。

它会按顺序：

1. 启动 `franky server`
2. 启动 `teleop/run.py`
3. 前台运行 `record_single.py`
4. 结束后清理后台进程

适合：

- 你想减少手工开多个终端
- 想做一个最小自动化入口

## Dual Arm: 每个文件是干什么的

### `dual_arm_ros1/README.md`

这是原仓库里的双臂使用说明。

重点是：

- 需要 `catkin_ws`
- 左右臂分别起一个服务
- 再开 `teleop_dual.run`
- 最后用 `scripts.record_dual` 录数据

### `dual_arm_ros1/bashes/launch_left_server.sh`

左臂 ROS 服务启动脚本。

你通常要改：

- `robot_ip`
- `ROS_MASTER_URI`
- 左臂监听的 Flask 地址

### `dual_arm_ros1/bashes/launch_right_server.sh`

右臂 ROS 服务启动脚本。

你通常要改：

- `robot_ip`
- `ROS_MASTER_URI`
- 右臂监听的 Flask 地址

### `dual_arm_ros1/robot_servers/franka_server.py`

双臂 ROS 控制服务的核心代码。

它会：

- 起 `roscore`
- `roslaunch` 对应控制器
- 开 Flask 接口给上层访问

什么时候看：

- 新机器 ROS1 栈不通
- `serl_franka_controllers` 起不来
- 需要查 joint reset / impedance 切换逻辑

### `dual_arm_ros1/robot_servers/franka_gripper_server.py`

双臂路线里的夹爪服务封装。

通常不需要先改，除非夹爪行为异常。

### `dual_arm_ros1/teleop_dual/config.py`

双臂 teleop 配置文件。

这里定义了：

- 左右服务 URL
- 左右 SpaceMouse 设备名
- 左右复位位姿
- 安全边界
- 动作缩放

双臂迁移时这是最常改的配置文件之一。

### `dual_arm_ros1/teleop_dual/run.py`

双臂 teleop 主入口，同时拉起左右臂环境。

### `dual_arm_ros1/teleop_dual/run_left.py`

左臂单独调试入口。

适合先只验证左臂，不把双臂一起拉起来。

### `dual_arm_ros1/teleop_dual/wrapper.py`

双臂 teleop 包装逻辑，包含 reset 等行为。

### `dual_arm_ros1/scripts/record_dual.py`

双臂主采集脚本。

要特别注意：

- 它里面写死了很多相机序列号
- 左右机器人地址也直接写在脚本里
- 输出目录默认值也是旧机器路径

所以迁移双臂时，这个脚本几乎一定要改。

## Shared: 共用代码是干什么的

### `shared/envs/camera/rs_capture.py`

RealSense 采集封装。

作用：

- 按序列号绑定设备
- 开彩色流
- 可选开深度流
- 自动做 depth 对齐到 color

这是排查相机问题时最该看的文件。

### `shared/envs/franka/franka_env.py`

共享的 Franka 环境层。

它负责：

- 自动检查服务是否连接
- 通过 HTTP 跟机器人通信
- 定义动作空间 / 观测空间
- 做安全边界裁剪

单臂和双臂很多行为都依赖它。

### `shared/envs/franka/relative_env.py`

相对位姿控制包装层。

用途是把原始环境包装成更适合遥操作的相对控制形式。

### `shared/envs/franka/wrappers.py`

一组通用 wrapper。

里面最重要的是：

- `Quat2EulerWrapper`
- `SpacemouseIntervention`

也就是把观测转欧拉角、以及接入 SpaceMouse 的地方。

### `shared/envs/utils/rotations.py`

欧拉角 / 四元数等旋转工具函数。

通常不用单独运行，但很多 reset 和控制逻辑依赖它。

## 新机器迁移时先改什么

### 单臂路线

- `single_arm_no_ros1/fr3_single_arm_config.py`
  改机器人 IP、相机序列号、SpaceMouse 名字、输出目录、安全边界
- `single_arm_no_ros1/franky_servers/start_server.sh`
  改 Python 路径和默认 `ROBOT_IP`
- `single_arm_no_ros1/record.sh`
  改 Python 路径、输出目录

### 双臂路线

- `dual_arm_ros1/bashes/launch_left_server.sh`
- `dual_arm_ros1/bashes/launch_right_server.sh`
- `dual_arm_ros1/teleop_dual/config.py`
- `dual_arm_ros1/scripts/record_dual.py`

## 最小验证流程

### 单臂最小验证

1. 改 `fr3_single_arm_config.py`
2. 启动 `franky_servers/start_server.sh`
3. 跑 `scripts/capture_image.py` 检查相机
4. 跑 `scripts/capture_reset_pose.py --write-config`
5. 跑 `teleop/run.py`
6. 跑 `scripts/record_single.py`

### 双臂最小验证

1. 改左右服务脚本和 `teleop_dual/config.py`
2. 分别启动左右服务
3. 跑 `teleop_dual/run.py`
4. 跑 `scripts/record_dual.py`

## 备注

- 这个目录是“迁移参考包”，不是一个已经完全解耦、可直接独立运行的小仓库。
- 里面很多文件仍然保留了原始 import 路径和旧机器路径，迁移时需要按你的新机器环境改。
- 如果你下一步要，我可以继续帮你再做一版：
  `MIGRATION_CHECKLIST.md`
  把每个文件里“必须改的字段”逐项列出来。
