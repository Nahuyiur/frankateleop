import time
import torch
from typing import Dict
from typing import Optional
import numpy as np
from teleop.robots.robot import Robot

MAX_OPEN = 0.09


class fr3Robot(Robot):
    """A class representing a UR robot."""

    def __init__(
            self, 
            robot_ip: str = "192.168.1.100", 
            franka_port: int=50051, 
            frankahand_port: int = 50053,
            joint_positions_desired: Optional[torch.Tensor] = None,
            control_mode: str = "joint",
            home_on_init: bool = True,
            open_gripper_on_init: bool = True,
            ):
            
        from polymetis import GripperInterface, RobotInterface
        print(f"Connecting to robot at IP: {robot_ip}")

        self.robot = RobotInterface(
            ip_address=robot_ip,
            port=franka_port,
        )
        self.gripper = GripperInterface(
            ip_address=robot_ip,
            port=frankahand_port,
        )
        if joint_positions_desired is None and home_on_init:
            self.robot.go_home()
        elif joint_positions_desired is None:
            print("Skipping robot.go_home() on init.")
        else:
            if joint_positions_desired.shape != (7,):
                raise ValueError(f"Franka requires 7 joints params, current input is: {joint_positions_desired.shape}")
            print("init robot")
            self.joint_positions_desired = joint_positions_desired
            self.robot.move_to_joint_positions(self.joint_positions_desired)
            
        self.control_mode = control_mode
        self._start_control_mode(control_mode)
        if open_gripper_on_init:
            self.gripper.goto(width=MAX_OPEN, speed=255, force=255)
            time.sleep(1)
        else:
            print("Skipping gripper open on init.")

    def _start_control_mode(self, control_mode: str) -> None:
        if control_mode == "joint":
            self.robot.start_joint_impedance()
        elif control_mode == "ee":
            self.robot.start_cartesian_impedance()
        else:
            raise ValueError(f"Unsupported fr3 control_mode: {control_mode}")

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        return 8

    def get_control_mode(self) -> str:
        return self.control_mode

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.robot.get_joint_positions()
        gripper_pos = self.gripper.get_state()
        pos = np.append(robot_joints, gripper_pos.width / MAX_OPEN)
        return pos

    def command_joint_state(
            self,
            joint_state: np.ndarray,
            gripper_speed: float = 1,
            gripper_force: float = 1,
            update_gripper: bool = True,
            ) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        import torch

        self.robot.update_desired_joint_positions(torch.tensor(joint_state[:-1]))
        if update_gripper:
            self.gripper.goto(
                width=(MAX_OPEN * (1 - joint_state[-1])),
                speed=gripper_speed,
                force=gripper_force,
            )

    def command_ee_pose(
            self,
            pose_6d: np.ndarray,
            gripper_width: float,
            gripper_speed: float = 0.05,
            gripper_force: float = 40.0,
            update_gripper: bool = True,
            ) -> None:
        """Command an absolute EE pose [x, y, z, rx, ry, rz] and gripper width in meters."""
        pose = np.asarray(pose_6d, dtype=float).reshape(-1)
        if pose.shape != (6,):
            raise ValueError(f"Expected pose_6d shape (6,), got {pose.shape}")

        from scipy.spatial.transform import Rotation

        position = torch.tensor(pose[:3], dtype=torch.float32)
        quat = Rotation.from_euler("xyz", pose[3:], degrees=False).as_quat()
        orientation = torch.tensor(quat, dtype=torch.float32)
        update_idx = self.robot.update_desired_ee_pose(position=position, orientation=orientation)
        if update_idx == -1:
            raise RuntimeError(f"Franka IK failed for pose_6d={pose.tolist()}")

        if update_gripper:
            width = float(np.clip(gripper_width, 0.0, MAX_OPEN))
            self.gripper.goto(
                width=width,
                speed=gripper_speed,
                force=gripper_force,
            )

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        ee_pos, ee_quat = self.robot.get_ee_pose()
        ee_pos = ee_pos.detach().cpu().numpy()
        ee_quat = ee_quat.detach().cpu().numpy()
        pos_quat = np.concatenate([ee_pos, ee_quat])

        from scipy.spatial.transform import Rotation

        ee_euler = Rotation.from_quat(ee_quat).as_euler("xyz", degrees=False)
        pos_euler = np.concatenate([ee_pos, ee_euler])
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "ee_pose_euler": pos_euler,
            "gripper_position": gripper_pos,
        }


def main():
    robot = fr3Robot()
    current_joints = robot.get_joint_state()
    # move a small delta 0.1 rad
    move_joints = current_joints + 0.05
    # make last joint (gripper) closed
    move_joints[-1] = 0.5
    time.sleep(1)
    m = 0.09
    robot.gripper.goto(1 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.05 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.1 * m, speed=255, force=255)
    time.sleep(1)


if __name__ == "__main__":
    main()
