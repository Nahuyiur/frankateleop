import datetime
import glob
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tyro

from teleop.agents.agent import BimanualAgent, DummyAgent
from teleop.agents.teleop_agent import TeleopAgent
from teleop.data_utils.format_obs import save_frame
from teleop.env import RobotEnv
from teleop.robots.robot import PrintRobot
from teleop.zmq_core.robot_node import ZMQClientRobot


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


def arm_joint_indices(num_dofs, bimanual=False):
    if num_dofs < 2:
        raise ValueError(f"Expected arm joints plus a gripper, got {num_dofs} DOFs")
    if not bimanual:
        return np.arange(num_dofs - 1)
    if num_dofs % 2 != 0:
        raise ValueError(f"Bimanual state must have even size, got {num_dofs}")
    arm_size = num_dofs // 2
    if arm_size < 2:
        raise ValueError(f"Invalid bimanual state size: {num_dofs}")
    return np.concatenate(
        [np.arange(arm_size - 1), np.arange(arm_size, num_dofs - 1)]
    )


def joint_vectors(action, reference):
    action = np.asarray(action, dtype=float).copy()
    reference = np.asarray(reference, dtype=float)
    if action.ndim != 1 or reference.ndim != 1:
        raise ValueError(
            f"Joint states must be 1-D, got {action.shape} and {reference.shape}"
        )
    if action.shape != reference.shape:
        raise ValueError(
            f"Agent output shape {action.shape} does not match robot state "
            f"{reference.shape}"
        )
    return action, reference


def wrap_arm_action_to_nearest(action, reference, bimanual=False):
    action, reference = joint_vectors(action, reference)
    arm_indices = arm_joint_indices(action.size, bimanual=bimanual)
    action[arm_indices] = reference[arm_indices] + (
        (action[arm_indices] - reference[arm_indices] + np.pi) % (2 * np.pi)
        - np.pi
    )
    return action


def limit_arm_step(command, current, max_delta, bimanual=False):
    command, current = joint_vectors(command, current)
    arm_indices = arm_joint_indices(command.size, bimanual=bimanual)
    delta = command[arm_indices] - current[arm_indices]
    largest_delta = float(np.max(np.abs(delta)))
    if largest_delta > max_delta:
        delta = delta / largest_delta * max_delta
    limited = command.copy()
    limited[arm_indices] = current[arm_indices] + delta
    return limited


def env_flag(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in ("1", "true", "yes", "on")


@dataclass
class Args:
    agent: str = "none"
    tele_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: Optional[str] = None  # only needed for quest or spacemouse agents
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    teleop_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False
    check_only: bool = False


def main(args):
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.tele_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    if args.bimanual:
        if args.agent == "teleop":
            # dynamixel control box port map (to distinguish left and right teleop)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0"
            left_agent = TeleopAgent(port=left)
            right_agent = TeleopAgent(port=right)
            agent = BimanualAgent(left_agent, right_agent)
        elif args.agent == "quest":
            from teleop.agents.quest_agent import SingleArmQuestAgent

            left_agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
            right_agent = SingleArmQuestAgent(
                robot_type=args.robot_type, which_hand="r"
            )
            agent = BimanualAgent(left_agent, right_agent)
            # raise NotImplementedError
        elif args.agent == "spacemouse":
            from teleop.agents.spacemouse_agent import SpacemouseAgent

            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            left_agent = SpacemouseAgent(
                robot_type=args.robot_type, device_path=left_path, verbose=args.verbose
            )
            right_agent = SpacemouseAgent(
                robot_type=args.robot_type,
                device_path=right_path,
                verbose=args.verbose,
                invert_button=True,
            )
            agent = BimanualAgent(left_agent, right_agent)
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([0, -90, -90, -90, 90, 0, 0])
        reset_joints_right = np.deg2rad([0, -90, 90, -90, -90, 0, 0])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)
    else:
        if args.agent == "teleop":
            teleop_port = args.teleop_port
            if teleop_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    teleop_port = usb_ports[0]
                    print(f"using port {teleop_port}")
                else:
                    raise ValueError(
                        "No teleop port found, please specify one or plug in teleop"
                    )
            if args.start_joints is None:
                reset_joints = np.deg2rad(
                    [0, -90, 90, -90, -90, 0, 0]
                )  # Change this to your own reset joints
            else:
                reset_joints = args.start_joints
                reset_joints = np.array(reset_joints)
            agent = TeleopAgent(port=teleop_port, start_joints=args.start_joints)
            curr_joints = env.get_obs()["joint_positions"]
            curr_joints = np.array(curr_joints)
            if args.check_only:
                print("Skipping reset_joints in check-only mode")
            elif reset_joints.shape == curr_joints.shape:
                max_delta = (np.abs(curr_joints - reset_joints)).max()
                steps = min(int(max_delta / 0.01), 100)

                for jnt in np.linspace(curr_joints, reset_joints, steps):
                    env.step(jnt)
                    time.sleep(0.001)
            elif reset_joints.size + 1 == curr_joints.size:
                print(
                    "Keeping robot-node initial pose: legacy reset target contains "
                    f"{reset_joints.size} arm joints while robot state also includes "
                    "a gripper value"
                )
            else:
                print(
                    "Skipping reset_joints because shape does not match robot state: "
                    f"reset_joints={reset_joints.shape}, robot_joints={curr_joints.shape}"
                )
        elif args.agent == "quest":
            from teleop.agents.quest_agent import SingleArmQuestAgent

            agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
        elif args.agent == "spacemouse":
            from teleop.agents.spacemouse_agent import SpacemouseAgent

            agent = SpacemouseAgent(robot_type=args.robot_type, verbose=args.verbose)
        elif args.agent == "dummy" or args.agent == "none":
            agent = DummyAgent(num_dofs=robot_client.num_dofs())
        elif args.agent == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent name")

    # going to start position
    print("Going to start position")
    obs = env.get_obs()
    joints = obs["joint_positions"]
    joints = np.array(joints)
    start_pos = wrap_arm_action_to_nearest(
        agent.act(obs), joints, bimanual=args.bimanual
    )
    arm_indices = arm_joint_indices(joints.size, bimanual=args.bimanual)
    abs_deltas = np.abs(start_pos[arm_indices] - joints[arm_indices])

    max_joint_delta = 0.8
    if np.max(abs_deltas) > max_joint_delta:
        unsafe_indices = arm_indices[abs_deltas > max_joint_delta]
        print()
        for i, delta, joint, current_j in zip(
            unsafe_indices,
            abs_deltas[abs_deltas > max_joint_delta],
            start_pos[unsafe_indices],
            joints[unsafe_indices],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        raise SystemExit(1)

    print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    assert len(start_pos) == len(
        joints
    ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

    if args.check_only:
        print("Teleop alignment check passed.")
        return

    max_delta = 0.05
    for _ in range(25):
        obs = env.get_obs()
        current_joints = obs["joint_positions"]
        command_joints = wrap_arm_action_to_nearest(
            agent.act(obs), current_joints, bimanual=args.bimanual
        )
        env.step(
            limit_arm_step(
                command_joints,
                current_joints,
                max_delta,
                bimanual=args.bimanual,
            )
        )

    obs = env.get_obs()
    joints = obs["joint_positions"]
    action = wrap_arm_action_to_nearest(
        agent.act(obs), joints, bimanual=args.bimanual
    )
    arm_indices = arm_joint_indices(len(joints), bimanual=args.bimanual)
    arm_delta = action[arm_indices] - joints[arm_indices]
    if (np.abs(arm_delta) > 0.5).any():
        print("Action is too big")

        # print which joints are too big
        joint_indices = arm_indices[np.abs(arm_delta) > 0.5]
        for j in joint_indices:
            print(
                f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
            )
        exit()

    if args.use_save_interface:
        from teleop.data_utils.keyboard_interface import KBReset

        kb_interface = KBReset()

    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))

    save_path = None
    start_time = time.time()
    debug_action = env_flag("TELEOP_DEBUG_ACTION")
    debug_interval = float(os.environ.get("TELEOP_DEBUG_INTERVAL_SEC", "1.0"))
    debug_next_time = 0.0
    debug_prev_action = None
    if debug_action:
        print(
            "TELEOP_DEBUG_ACTION enabled "
            f"(interval={debug_interval:.2f}s)"
        )
    while True:
        num = time.time() - start_time
        message = f"\rTime passed: {round(num, 2)}          "
        print_color(
            message,
            color="white",
            attrs=("bold",),
            end="",
            flush=True,
        )
        action = wrap_arm_action_to_nearest(
            agent.act(obs), obs["joint_positions"], bimanual=args.bimanual
        )
        now = time.time()
        if debug_action and now >= debug_next_time:
            current = np.asarray(obs["joint_positions"], dtype=float)
            action_array = np.asarray(action, dtype=float)
            delta = action_array - current
            if debug_prev_action is None:
                leader_step = 0.0
            else:
                leader_step = float(np.max(np.abs(action_array - debug_prev_action)))
            debug_prev_action = action_array.copy()
            print()
            print(
                "TELEOP_DEBUG_ACTION "
                f"max_cmd_delta={np.max(np.abs(delta)):.4f} "
                f"max_leader_step={leader_step:.4f} "
                "current[:7]="
                f"{np.array2string(current[:7], precision=3, suppress_small=True)} "
                "action[:7]="
                f"{np.array2string(action_array[:7], precision=3, suppress_small=True)}",
                flush=True,
            )
            debug_next_time = now + debug_interval
        dt = datetime.datetime.now()
        if args.use_save_interface:
            state = kb_interface.update()
            if state == "start":
                dt_time = datetime.datetime.now()
                save_path = (
                    Path(args.data_dir).expanduser()
                    / args.agent
                    / dt_time.strftime("%m%d_%H%M%S")
                )
                save_path.mkdir(parents=True, exist_ok=True)
                print(f"Saving to {save_path}")
            elif state == "save":
                assert save_path is not None, "something went wrong"
                save_frame(save_path, dt, obs, action)
            elif state == "normal":
                save_path = None
            else:
                raise ValueError(f"Invalid state {state}")
        obs = env.step(action)


if __name__ == "__main__":
    main(tyro.cli(Args))
