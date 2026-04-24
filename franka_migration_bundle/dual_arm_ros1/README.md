# FR3
Franka Research 3 Use Instruction By Chenyang Gu

## Table of Contents

- [FR3](#fr3)
  - [Table of Contents](#table-of-contents)
  - [⚙️Overview](#️overview)
  - [🧩Project Structure](#project-structure)
  - [📦Usage](#usage)
    - [Record](#record)
- [结束所有 roscore](#结束所有-roscore)
- [结束所有 roslaunch（会顺带关掉由它起的节点）](#结束所有-roslaunch会顺带关掉由它起的节点)
- [若上面不够，再结束 ros master（按需使用）](#若上面不够再结束-ros-master按需使用)
---

## ⚙️Overview

FR3 is a versatile developer tool designed to streamline interactions with cameras, robotic systems, and spacemouse devices.

## 🧩Project Structure

```sh
└── FR3.git/
    ├── LICENSE
    ├── README.md
    ├── data
    │   └── readme.md
    ├── envs
    │   ├── __init__.py
    │   ├── camera
    │   ├── franka
    │   ├── spacemouse
    │   └── utils
    ├── policies
    │   └── readme.md
    ├── robot_servers
    │   ├── __init__.py
    │   ├── franka_gripper_server.py
    │   ├── franka_server.py
    │   ├── gripper_server.py
    │   └── robotiq_gripper_server.py
    ├── scripts
    │   ├── eval.py
    │   ├── record.py
    │   └── test
    ├── teleop
    │   ├── __init__.py
    │   ├── config.py
    │   ├── run.py
    │   └── wrapper.py
    └── third_party
```

## 📦Usage

### Record

先导入环境变量 'source ~/catkin_ws/devel/setup.bash'

source /home/franka/catkin_ws/devel/setup.bash
1. 在一个终端激活conda环境: `conda activate hilserl`
2. 在第一步的终端拆分成两个，分别启动左右臂服务: `source /home/franka/Code/FR3/bashes/launch_left_server.sh` `source /home/franka/Code/FR3/bashes/launch_right_server.sh` 
   jobs -l    kill -9 

3. 在一个终端激活conda环境: `conda activate hilserl`
4. 新建一个终端激活conda环境挂载spacemosue: `python -m teleop_dual.run` 
   

5. 在一个终端激活conda环境: `conda activate hilserl`
6. 开始采集脚本: `python -m scripts.record_dual --index {index} --task {task_name} --camera_num {camera_number} --output_root {data_save_root}`
7. 采集数据的操作注意事项：1.空间鼠标夹爪开合不灵敏，需要稍微使劲，完全按压按钮。2.有时候开合动作会导致机械臂持续性卡顿，此时，如果机械臂夹爪张开，按压张开夹爪按钮，并观察机械臂抖动突然停止，则恢复正常；夹爪闭合时则按压闭合按钮，做相同观察。


# 结束所有 roscore
pkill -f roscore

# 结束所有 roslaunch（会顺带关掉由它起的节点）
pkill -f roslaunch

# 若上面不够，再结束 ros master（按需使用）
pkill -f "ros master"
