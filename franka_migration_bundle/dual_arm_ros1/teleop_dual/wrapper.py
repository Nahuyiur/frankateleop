import copy
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

from envs.franka.franka_env import FrankaEnv
from envs.utils.rotations import euler_2_quat

PRECISION_PARAM = {
    "translational_stiffness": 2000,
    "translational_damping": 89,
    "rotational_stiffness": 150,
    "rotational_damping": 7,
    "translational_Ki": 0.0,
    "translational_clip_x": 0.01,
    "translational_clip_y": 0.01,
    "translational_clip_z": 0.01,
    "translational_clip_neg_x": 0.01,
    "translational_clip_neg_y": 0.01,
    "translational_clip_neg_z": 0.01,
    "rotational_clip_x": 0.03,
    "rotational_clip_y": 0.03,
    "rotational_clip_z": 0.03,
    "rotational_clip_neg_x": 0.03,
    "rotational_clip_neg_y": 0.03,
    "rotational_clip_neg_z": 0.03,
    "rotational_Ki": 0.0,
}

class TELEOPEnv(FrankaEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.should_reset = False
        self.joint_reset = False

        self._session.post(self.url + "update_param", json=self.config.PRECISION_PARAM)

    def go_to_reset(self):
        """
        Move to the rest position defined in base class.
        Add a small z offset before going to rest to avoid collision with object.
        """        
        # use compliance mode for coupled reset
        self._update_currpos()
        self._send_gripper_command(1.0)
        self._send_pos_command(self.currpos)
        time.sleep(0.1)
        # requests.post(self.url + "update_param", json=self.config.PRECISION_PARAM)

        # pull up
        self._update_currpos()
        reset_pose = copy.deepcopy(self.currpos)
        reset_pose[2] = self.resetpos[2] + 0.10
        # 复位过程采用逐步确认模式：每一步按下回车再执行
        self.interpolate_move(reset_pose, step_by_step=True)

        time.sleep(0.5)

        # perform joint reset if needed
        if self.joint_reset:
            print("JOINT RESET")
            self._session.post(self.url + "jointreset")
            time.sleep(0.5)
        reset_pose = self.resetpos.copy()
        print("RESETTING TO", reset_pose)
        # 再次回到 RESET_POSE，同样逐步确认
        self.interpolate_move(reset_pose, step_by_step=True)

        # Change to compliance mode
        # requests.post(self.url + "update_param", json=self.config.COMPLIANCE_PARAM)
        time.sleep(1)
    
    def reset(self, **kwargs):

        self._recover()
        self.go_to_reset()
        self._recover()
        self.curr_path_length = 0

        self._update_currpos()
        obs = self._get_obs()
        # requests.post(self.url + "update_param", json=self.config.COMPLIANCE_PARAM)
        self.terminate = False
        return obs, {}
