import numpy as np

from envs.franka.wrappers import Quat2EulerWrapper, SpacemouseIntervention
from envs.franka.relative_env import RelativeFrame
from envs.franka.franka_env import DefaultEnvConfig
from fr3_single_arm_config import SingleArmConfig, require_safety_config
from .wrapper import TELEOPEnv


class SpaceMouseTeleopConfig(DefaultEnvConfig):
    SERVER_URL = SingleArmConfig.SERVER_URL
    ROBOT_IP = SingleArmConfig.ROBOT_IP
    DEVICE = SingleArmConfig.SPACEMOUSE_DEVICE
    RESET_POSE = require_safety_config("RESET_POSE", SingleArmConfig.RESET_POSE)
    ACTION_SCALE = SingleArmConfig.ACTION_SCALE
    ABS_POSE_LIMIT_LOW = require_safety_config(
        "ABS_POSE_LIMIT_LOW", SingleArmConfig.ABS_POSE_LIMIT_LOW
    )
    ABS_POSE_LIMIT_HIGH = require_safety_config(
        "ABS_POSE_LIMIT_HIGH", SingleArmConfig.ABS_POSE_LIMIT_HIGH
    )
    AUTO_CONNECT = True
    MOTION_ASYNC = True
    RELATIVE_DYNAMICS_FACTOR = SingleArmConfig.RELATIVE_DYNAMICS_FACTOR
    DISPLAY_IMAGE = False
    REALSENSE_CAMERAS = {}

    def get_environment(self):
        env = TELEOPEnv(config=SpaceMouseTeleopConfig())
        env = SpacemouseIntervention(env, device=self.DEVICE)
        env = RelativeFrame(env)
        env = Quat2EulerWrapper(env)
        return env
