import numpy as np


def require_safety_config(name: str, value):
    if value is None:
        raise RuntimeError(
            f"{name} 还没有配置。请先在 fr3_single_arm_config.py 中填写安全的单臂参数后再启动。"
        )
    return value

class SingleArmConfig:
    ROBOT_IP = "172.16.0.2"
    SERVER_URL = "http://127.0.0.1:5000/"
    SPACEMOUSE_DEVICE = "SpaceMouse Compact"
    DATA_OUTPUT_ROOT = "/home/ubuntu/robot_env/data"
    PYTHON_BIN = "/home/ubuntu/robot_env/bin/python"
    RELATIVE_DYNAMICS_FACTOR = 0.04

    # 这里先写入你的 RealSense 序列号；采集脚本会按这个顺序读取并保存。
    REALSENSE_CAMERAS = {
        "exterior_1": {"serial_number": "336222070587", "fps": 15, "depth": False},
        "exterior_2": {"serial_number": "332322072361", "fps": 15, "depth": False},
        "wrist": {"serial_number": "250122075514", "fps": 15, "depth": False},
    }

    # 你还没有提供安全位姿和工作空间边界；这里故意留空，防止真机误动作。
    RESET_POSE = np.array([0.3, 0.0, 0.48, 3.13, 0.0, 0.0])
    # 记得加上 np.array()
    
    LIMIT_DELTA = np.array([1.0, 1.0, 1.0, 1, 1, 1])
    
    ABS_POSE_LIMIT_LOW = RESET_POSE - LIMIT_DELTA
    ABS_POSE_LIMIT_HIGH = RESET_POSE + LIMIT_DELTA

    ACTION_SCALE = (0.04, 0.1, 1.0)
