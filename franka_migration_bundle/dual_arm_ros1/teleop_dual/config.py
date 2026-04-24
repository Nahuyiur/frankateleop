import os
import jax
import jax.numpy as jnp
import numpy as np

import sys
sys.path.append('/home/franka/Code/FR3')

from envs.franka.wrappers import (
    Quat2EulerWrapper,
    SpacemouseIntervention,
)
from envs.franka.relative_env import RelativeFrame
from envs.franka.franka_env import DefaultEnvConfig
from .wrapper import TELEOPEnv

class LeftSpaceMouseTeleopConfig(DefaultEnvConfig):
    TELEOP_HZ = 20  # 提高控制频率使遥操作更连续

    SERVER_URL = "http://127.0.0.1:5000/"

    DEVICE = "SpaceMouse Wireless"

    RESET_POSE = np.array([0.48049184958385605,-0.00022334620358853869,0.30009448728042004,3.140056888974983,0.007291706929760888,0.006764377533135413]) 
    # RESET_POSE = np.array([ 0.5832, -0.2472,  0.2656,-3.1148, -0.0827 , 1.518 ])   # pour_coke
    # RESET_POSE = np.array([0.48, 0.0, 0.30, 3.13, 0, 0]) 
    RESET_TIMEOUT = 5.0          # 复位插值时间（秒），越大越慢
    MAX_STEP_TRANSLATION = 0.01 # 单步最大位移 (m)
    MAX_STEP_ROTATION = np.deg2rad(10)  # 单步最大旋转 (rad) ≈ 30°
    ACTION_SCALE = (0.04, 0.1, 1) 
    
    ABS_POSE_LIMIT_LOW = RESET_POSE - np.array([2.0, 1.0, 1.0, 3.14, 3.14, 3.14]) 
    ABS_POSE_LIMIT_HIGH = RESET_POSE + np.array([2.0, 1.0, 1.0, 3.14, 3.14, 3.14])


    def get_environment(self):
        print("Creating TELEOPEnv...")
        env = TELEOPEnv(
            config=LeftSpaceMouseTeleopConfig(),
            hz=self.TELEOP_HZ,
        )
        print("TELEOPEnv created, adding wrappers...")
        env = SpacemouseIntervention(env, device=self.DEVICE)
        print("SpacemouseIntervention added")
        env = RelativeFrame(env)
        env = Quat2EulerWrapper(env)
        return env
    
class RightSpaceMouseTeleopConfig(DefaultEnvConfig):
    TELEOP_HZ = 20

    SERVER_URL = "http://127.0.0.2:5000/"
    DEVICE = "SpaceMouse Wireless [NEW]"

    RESET_POSE = np.array([0.48, 0.0, 0.30, 3.13, 0, 0]) 
    RESET_TIMEOUT = 5.0          # 右臂复位插值时间
    MAX_STEP_TRANSLATION = 0.05  # 单步最大位移 (m)
    MAX_STEP_ROTATION = np.deg2rad(30.0)  # 单步最大旋转 (rad) ≈ 30°
    ACTION_SCALE = (0.04, 0.1, 1) 
    
    # Don't Change LIMIT 
    ABS_POSE_LIMIT_LOW = RESET_POSE - np.array([1.0, 1.0, 1.0, 3.14, 3.14, 3.14]) 
    ABS_POSE_LIMIT_HIGH = RESET_POSE + np.array([1.0, 1.0, 1.0, 3.14, 3.14, 3.14])

    def get_environment(self):
        env = TELEOPEnv(
            config=RightSpaceMouseTeleopConfig(),
            hz=self.TELEOP_HZ,
        )
        env = SpacemouseIntervention(env, device=self.DEVICE)
        env = RelativeFrame(env)
        env = Quat2EulerWrapper(env)
        return env
