"""Bounded dual-arm command transport and trajectory validation for Host98."""
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import zmq

from contract import ARMS

ROUTES = {"left": "tcp://127.0.0.1:6002", "right": "tcp://172.16.0.3:6001"}


class MotionError(RuntimeError):
    pass


def finite_vector(value, size, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise MotionError(label + " must be a finite vector of length " + str(size))
    return a


def pose_rotation(pose):
    q = finite_vector(pose, 7, "pose")[3:]
    if abs(np.linalg.norm(q)-1) > 1e-3:
        raise MotionError("Non-unit target quaternion")
    return Rotation.from_quat(q)


def rotation_distance(a, b):
    return float((pose_rotation(a).inv() * pose_rotation(b)).magnitude())


def observed_state(raw, profile):
    joints = finite_vector(raw["joint_positions"][:7], 7, "joint positions")
    velocities = finite_vector(raw["joint_velocities"], 7, "joint velocities")
    eef = finite_vector(raw["ee_pose_euler"], 6, "EEF")
    width = float(finite_vector(raw["gripper_width"], 1, "gripper width")[0])
    maximum = profile["gripper_state_max_width_m"]
    if not 0 <= width <= maximum:
        raise MotionError("Invalid measured gripper width")
    return {"joint_position": joints.tolist(), "joint_velocity": velocities.tolist(), "eef_pose": eef.tolist(),
            "gripper_width_m": width, "gripper_closedness": 1.0-width/maximum}


def state_target(state):
    eef = np.asarray(state["eef_pose"], dtype=float)
    return {"target": [*eef[:3], *Rotation.from_euler("xyz", eef[3:]).as_quat()],
            "target_layout": "xyz_quaternion_xyzw", "gripper_target_width_m": state["gripper_width_m"]}


def validate_joints(states, profile):
    lo = np.asarray(profile["joint_limits_lower"]) + profile["joint_soft_margin_rad"]
    hi = np.asarray(profile["joint_limits_upper"]) - profile["joint_soft_margin_rad"]
    for arm in ARMS:
        q = finite_vector(states[arm]["joint_position"], 7, arm + " joints")
        if np.any(q < lo) or np.any(q > hi):
            raise MotionError(arm + " is near a joint limit; move away using the existing robot controls")


def validate_stationary(states):
    for arm in ARMS:
        velocities = finite_vector(states[arm]["joint_velocity"], 7, arm + " velocity")
        if np.max(np.abs(velocities)) > 0.05:
            raise MotionError("Stop teleoperation and wait for both arms to settle before taking control")


def world_tcp(pose, arm, profile):
    tool = profile["tool_transforms"][arm]
    tcp = np.asarray(pose[:3]) + pose_rotation(pose).apply(tool["translation_m"])
    base = profile["base_frames"][arm]
    return Rotation.from_euler("xyz", base["rpy"]).apply(tcp) + base["xyz"]


def validate_pair(targets, anchors, profile):
    if set(targets) != set(ARMS):
        raise MotionError("Both arms must be present in every command")
    tips = {}
    for arm in ARMS:
        item = targets[arm]
        if item.get("target_layout") != "xyz_quaternion_xyzw":
            raise MotionError("Unexpected target layout")
        p = finite_vector(item["target"], 7, arm + " target")
        pose_rotation(p)
        width = float(item["gripper_target_width_m"])
        if not math.isfinite(width) or not 0 <= width <= profile["gripper_state_max_width_m"]:
            raise MotionError("Invalid gripper target")
        if np.linalg.norm(p[:3]-np.asarray(anchors[arm]["target"][:3])) > profile["max_session_translation_m"]:
            raise MotionError(arm + " target exceeds the session translation envelope")
        if rotation_distance(p, anchors[arm]["target"]) > profile["max_session_rotation_rad"]:
            raise MotionError(arm + " target exceeds the session rotation envelope")
        tips[arm] = world_tcp(p, arm, profile)
        bounds = np.asarray(profile["workspace_xyz"])
        if np.any(tips[arm] < bounds[:, 0]) or np.any(tips[arm] > bounds[:, 1]):
            raise MotionError(arm + " TCP is outside the configured shared workspace")
    if np.linalg.norm(tips["left"]-tips["right"]) < profile["min_tcp_separation_m"]:
        raise MotionError("Predicted TCP separation is too small")


def validate_tracking(states, commanded, profile):
    validate_joints(states, profile)
    for arm in ARMS:
        actual = state_target(states[arm])["target"]
        target = commanded[arm]["target"]
        if np.linalg.norm(np.asarray(actual[:3])-target[:3]) > profile["max_tracking_translation_m"]:
            raise MotionError(arm + " position tracking error is too large")
        if rotation_distance(actual, target) > profile["max_tracking_rotation_rad"]:
            raise MotionError(arm + " orientation tracking error is too large")


def trajectory(response, observed, session_anchors, profile, profile_hash, *, steps=5, speed_scale=0.2, commanded_start=None):
    if response.get("format_version") != "robot_action_v2" or response.get("robot_mode") != "dual":
        raise MotionError("Expected robot_action_v2 / dual")
    metadata = response.get("server_metadata", {})
    if metadata.get("policy_id") != "pi05_umi_dual_franka_stack_cubes_10k":
        raise MotionError("Wrong policy ID")
    backend = metadata.get("backend_metadata", {})
    if backend.get("hardware_profile_sha256") != profile_hash:
        raise MotionError("106 and 98 hardware profiles differ; refusing motion")
    if backend.get("output_target_frame") != "per_arm_robot_base_to_panda_link8":
        raise MotionError("Wrong output reference frame")
    arms = response.get("arm_actions")
    if not isinstance(arms, dict) or set(arms) != set(ARMS):
        raise MotionError("Missing one arm in action response")
    counts = [len(arms[a].get("actions", [])) for a in ARMS]
    if counts != [50, 50] or any(arms[a].get("control_type") != "absolute_eef_pose" for a in ARMS):
        raise MotionError("Expected equal 50-step absolute-EEF chunks")
    if not 1 <= steps <= 50 or not 0.05 <= speed_scale <= 1.0:
        raise MotionError("Invalid execution horizon or speed scale")
    starts = {a: state_target(observed[a]) for a in ARMS}
    previous = starts if commanded_start is None else commanded_start
    frames = []
    clipped_grippers = 0
    for i in range(steps):
        targets = {}
        duration = 1.0/(profile["nominal_policy_hz"]*speed_scale)
        rotations = {}
        for arm in ARMS:
            item = arms[arm]["actions"][i]
            if item.get("action_index") != i:
                raise MotionError("Unexpected action ordering")
            target = {k: item[k] for k in ("target", "target_layout", "gripper_target_width_m")}
            p = finite_vector(target["target"], 7, arm + " predicted target")
            if np.linalg.norm(p[:3]-np.asarray(starts[arm]["target"][:3])) > profile["max_chunk_translation_m"]:
                raise MotionError("Large predicted translation in the execution prefix")
            if rotation_distance(p, starts[arm]["target"]) > profile["max_chunk_rotation_rad"]:
                raise MotionError("Large predicted rotation in the execution prefix")
            width = float(target["gripper_target_width_m"])
            if not math.isfinite(width) or not 0 <= width <= profile["gripper_state_max_width_m"]:
                raise MotionError("Invalid predicted gripper width")
            capped = min(width, profile["gripper_command_max_width_m"])
            clipped_grippers += int(capped != width)
            target["gripper_target_width_m"] = capped
            targets[arm] = target
            duration = max(duration,
                           np.linalg.norm(p[:3]-np.asarray(previous[arm]["target"][:3]))/profile["max_translation_speed_m_s"],
                           rotation_distance(p, previous[arm]["target"])/profile["max_rotation_speed_rad_s"],
                           abs(capped-previous[arm]["gripper_target_width_m"])/profile["max_gripper_speed_m_s"])
            rotations[arm] = Slerp([0, 1], Rotation.from_quat([previous[arm]["target"][3:], p[3:]]))
        validate_pair(targets, session_anchors, profile)
        count = max(1, math.ceil(duration*profile["output_hz"]))
        for tick in range(1, count+1):
            fraction = tick/count
            pair = {}
            for arm in ARMS:
                old, new = previous[arm], targets[arm]
                p = (1-fraction)*np.asarray(old["target"][:3]) + fraction*np.asarray(new["target"][:3])
                pair[arm] = {"target": [*p, *rotations[arm]([fraction]).as_quat()[0]],
                             "target_layout": "xyz_quaternion_xyzw",
                             "gripper_target_width_m": (1-fraction)*old["gripper_target_width_m"] + fraction*new["gripper_target_width_m"]}
            validate_pair(pair, session_anchors, profile)
            frames.append(pair)
        previous = targets
    return frames, clipped_grippers


class DualRPC:
    """Send each pair before waiting for either reply. Never retry a motion RPC."""
    def __init__(self, endpoints=None, timeout_s=0.25):
        self.endpoints = endpoints or ROUTES
        self.timeout_s = timeout_s
        self.context = zmq.Context()
        self.sockets = {}
        self.sent_counts = {a: 0 for a in ARMS}
        self.baseline = None
        self.open()

    def open(self):
        for arm in ARMS:
            s = self.context.socket(zmq.REQ)
            s.setsockopt(zmq.LINGER, 0)
            s.setsockopt(zmq.SNDTIMEO, max(1, int(self.timeout_s*1000)))
            s.connect(self.endpoints[arm])
            self.sockets[arm] = s

    def reset(self):
        for s in self.sockets.values():
            s.close(0)
        self.sockets.clear()
        self.open()

    def call(self, method, per_arm=None, timeout_s=None):
        started = time.monotonic()
        budget = self.timeout_s if timeout_s is None else timeout_s
        pending = set(ARMS)
        results = {}
        poller = zmq.Poller()
        try:
            for arm in ARMS:
                self.sockets[arm].send_pyobj({"method": method, "args": {} if per_arm is None else per_arm[arm]})
                if method == "command_ee_pose":
                    self.sent_counts[arm] += 1
                poller.register(self.sockets[arm], zmq.POLLIN)
            while pending:
                remaining = budget-(time.monotonic()-started)
                if remaining <= 0:
                    raise MotionError("RPC deadline exceeded: " + method + ", arms=" + str(sorted(pending)))
                events = dict(poller.poll(max(1, int(remaining*1000))))
                for arm in list(pending):
                    if self.sockets[arm] in events:
                        results[arm] = self.sockets[arm].recv_pyobj(zmq.NOBLOCK)
                        pending.remove(arm)
            for arm, value in results.items():
                if isinstance(value, dict) and ("error" in value or value.get("status") == "hold"):
                    raise MotionError(arm + " robot rejected " + method + ": " + str(value))
                if method == "command_ee_pose" and value is not None:
                    raise MotionError("Unexpected motion acknowledgement from " + arm + ": " + str(value))
            return results
        except Exception:
            self.reset()
            raise

    def states(self, profile, timeout_s=None):
        raw = self.call("get_observations", timeout_s=profile["max_command_latency_s"] if timeout_s is None else timeout_s)
        return {arm: observed_state(raw[arm], profile) for arm in ARMS}

    def acquire(self):
        if any(mode != "ee" for mode in self.call("get_control_mode", timeout_s=1.0).values()):
            raise MotionError("Both robot nodes must already run in ee mode")
        first = self.call("get_metrics", timeout_s=1.0)
        time.sleep(0.12)
        second = self.call("get_metrics", timeout_s=1.0)
        if any(first[a]["command_ee_pose_total"] != second[a]["command_ee_pose_total"] for a in ARMS):
            raise MotionError("Another controller is sending commands")
        self.baseline = {a: (second[a]["pid"], second[a]["command_ee_pose_total"]-self.sent_counts[a]) for a in ARMS}

    def verify_ownership(self):
        current = self.call("get_metrics")
        for arm in ARMS:
            pid, initial = self.baseline[arm]
            if current[arm]["pid"] != pid or current[arm]["command_ee_pose_total"] != initial+self.sent_counts[arm]:
                raise MotionError("Robot node restarted or another controller sent a command: " + arm)

    def command(self, pair, profile, update_gripper=True, timeout_s=None):
        args = {}
        for arm in ARMS:
            target = pair[arm]
            pose = finite_vector(target["target"], 7, arm + " target")
            args[arm] = {"pose_6d": np.r_[pose[:3], pose_rotation(pose).as_euler("xyz")],
                         "gripper_width": float(target["gripper_target_width_m"]),
                         "gripper_speed": profile["gripper_speed_m_s"], "gripper_force": profile["gripper_force_n"],
                         "update_gripper": bool(update_gripper)}
        return self.call("command_ee_pose", args, timeout_s=timeout_s)

    def close(self):
        for s in self.sockets.values():
            s.close(0)
        self.context.term()
