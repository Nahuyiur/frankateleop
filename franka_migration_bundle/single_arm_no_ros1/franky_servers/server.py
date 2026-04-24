# franky_server.py
# -*- coding: utf-8 -*-
"""
基于 franky 的机器人/夹爪控制 Flask 服务
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation

# ===== franky SDK =====
# 假设这些类/函数均来自 franky，与你示例一致
from franky import Robot, Gripper, JointMotion, CartesianMotion, Affine

# ======================
# 工具函数（取自你的示例，略作封装/修正）
# ======================

def get_pose(robot: Robot) -> np.ndarray:
    cartesian_state = robot.current_cartesian_state
    robot_pose = cartesian_state.pose
    ee_pose = robot_pose.end_effector_pose
    # elbow_pos = robot_pose.elbow_state  # 如需肘部可扩展返回
    pose = np.concatenate([ee_pose.translation, ee_pose.quaternion]).flatten()
    return pose  # [x, y, z, qx, qy, qz, qw]

def get_pose_euler(robot: Robot) -> np.ndarray:
    pose_quat = get_pose(robot)
    quat = pose_quat[3:]
    rotation = Rotation.from_quat(quat)
    euler = rotation.as_euler('xyz', degrees=False)
    pose_euler = np.concatenate([pose_quat[:3], euler])
    return pose_euler  # [x, y, z, rx, ry, rz] (r为弧度)

def get_joint(robot: Robot):
    joint_state = robot.current_joint_state
    return np.array(joint_state.position)

def get_gripper(gripper: Gripper) -> float:
    return float(gripper.width)


def get_state_snapshot(robot: Robot, gripper: Gripper) -> dict:
    return {
        "pose": np_to_list(get_pose(robot)),
        "joint": np_to_list(get_joint(robot)),
        "gripper_width": get_gripper(gripper),
    }

def goto_joint(robot: Robot, joint, asynchronous: bool = True):
    motion = JointMotion(joint)
    robot.move(motion, asynchronous=asynchronous)

def goto_pose(robot: Robot, pose, asynchronous: bool = True):
    pos = pose[:3]
    quat = pose[3:]
    motion = CartesianMotion(Affine(pos, quat))
    robot.move(motion, asynchronous=asynchronous)
    STATE["motion_count"] += 1
    STATE["last_motion_time"] = time.time()

def goto_pose_euler(robot: Robot, pose, asynchronous: bool = True):
    pos = pose[:3]
    euler = pose[3:]
    rotation = Rotation.from_euler('xyz', euler)
    quat = rotation.as_quat()
    pose_quat = np.concatenate([pos, quat])
    goto_pose(robot, pose_quat, asynchronous=asynchronous)

def move_gripper(gripper: Gripper, width: float, speed: float = 0.05):
    success = gripper.move(width, speed)
    return bool(success)

def close_gripper(
    gripper: Gripper,
    width: float = 0.0,
    speed: float = 0.1,
    force: float = 2.0,
    epsilon_outer: float = 1.0,
):
    return bool(
        gripper.grasp(width, speed, force, epsilon_outer=epsilon_outer)
    )

def open_gripper(gripper: Gripper, speed: float = 0.05):
    gripper.open(speed)
    return True

def reset_robot(robot: Robot, gripper: Gripper):
    reset_pose = np.array([0.30414, 0.000157113, 0.481851, 0.999985, -0.00211438, -0.00509529, -0.000415816])
    goto_pose(robot, reset_pose)
    reset_joint = np.array([0.0, -0.79, 0.0, -2.37, 0.0, 1.57, 0.79])
    goto_joint(robot, reset_joint)
    open_gripper(gripper)
    return True

# ======================
# Flask 应用
# ======================

app = Flask(__name__)
CORS(app)

DEFAULT_COLLISION_TAU_THRESHOLDS = [25, 25, 25, 25, 25, 25, 25]
DEFAULT_COLLISION_FORCE_THRESHOLDS = [40, 40, 80, 25, 25, 25]

# 全局单例（简单服务场景足够用）
STATE = {
    "robot": None,
    "gripper": None,
    "ip": None,
    "relative_dynamics_factor": 0.035,  # twin-rl 默认动力学比例
    "motion_async": False,
    "stop_before_move": False,
    "collision_behavior_enabled": True,
    "collision_tau_thresholds": DEFAULT_COLLISION_TAU_THRESHOLDS.copy(),
    "collision_force_thresholds": DEFAULT_COLLISION_FORCE_THRESHOLDS.copy(),
    "last_motion_time": 0,  # 上次运动命令的时间戳
    "motion_count": 0,  # 运动命令计数器
    "last_pose": None,
    "last_joint": None,
    "last_state_ts": None,
}
LOCK = threading.Lock()

# --------- 工具：统一响应 / 错误处理 ---------
def ok(data=None, msg="ok", code=0, http=200):
    return jsonify({"code": code, "msg": msg, "data": data}), http

def fail(msg="error", code=1, http=400, data=None):
    return jsonify({"code": code, "msg": msg, "data": data}), http

@app.errorhandler(Exception)
def on_exception(e):
    return fail(msg=f"{type(e).__name__}: {str(e)}", http=500)

# --------- 工具：参数读取 ---------
def get_json():
    if request.is_json:
        return request.get_json(silent=True) or {}
    return {}

def ensure_connected():
    if STATE["robot"] is None or STATE["gripper"] is None:
        raise RuntimeError("机器人或夹爪未连接，请先调用 /connect")
    return STATE["robot"], STATE["gripper"]

def maybe_stop_robot(robot: Robot):
    if not STATE["stop_before_move"]:
        return
    try:
        robot.stop()
    except Exception:
        # Ignore stop failures when no motion is active.
        pass

def np_to_list(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (list, tuple)):
        return list(x)
    return x


def parse_threshold_list(value, *, expected_len: int, name: str, default: list[float]):
    if value is None:
        values = default
    elif isinstance(value, (list, tuple)):
        values = value
    else:
        raise ValueError(f"{name} 必须是长度为 {expected_len} 的数组")

    if len(values) != expected_len:
        raise ValueError(f"{name} 必须是长度为 {expected_len} 的数组")
    return [float(v) for v in values]


def is_preempted_exception(exc: Exception) -> bool:
    return "preempted" in str(exc).lower()


def estimate_velocity(current: np.ndarray, previous: np.ndarray | None, dt: float) -> np.ndarray:
    if previous is None or dt <= 1e-6:
        return np.zeros_like(current, dtype=float)
    return (current - previous) / dt


def refresh_state_cache(robot: Robot, gripper: Gripper) -> dict:
    now = time.time()
    pose = np.array(get_pose(robot), dtype=float)
    joint = np.array(get_joint(robot), dtype=float)
    gripper_width = float(get_gripper(gripper))

    prev_pose = STATE["last_pose"]
    prev_joint = STATE["last_joint"]
    prev_ts = STATE["last_state_ts"]
    dt = now - prev_ts if prev_ts is not None else 0.0

    cartesian_velocity = estimate_velocity(
        pose[:6],
        prev_pose[:6] if prev_pose is not None else None,
        dt,
    )
    joint_velocity = estimate_velocity(joint, prev_joint, dt)

    STATE["last_pose"] = pose
    STATE["last_joint"] = joint
    STATE["last_state_ts"] = now

    return {
        "pose": pose.tolist(),
        "joint": joint.tolist(),
        "gripper_width": gripper_width,
        "vel": cartesian_velocity.tolist(),
        "force": [0.0, 0.0, 0.0],
        "torque": [0.0, 0.0, 0.0],
        "jacobian": [0.0] * 42,
        "dq": joint_velocity.tolist(),
        "gripper_pos": [gripper_width],
    }


def legacy_state_payload(robot: Robot, gripper: Gripper) -> dict:
    snapshot = refresh_state_cache(robot, gripper)
    return {
        "pose": snapshot["pose"],
        "vel": snapshot["vel"],
        "force": snapshot["force"],
        "torque": snapshot["torque"],
        "jacobian": snapshot["jacobian"],
        "q": snapshot["joint"],
        "dq": snapshot["dq"],
        "gripper_pos": snapshot["gripper_pos"],
    }

# ======================
# 路由
# ======================

@app.route("/connect", methods=["POST"])
def connect():
    """
    连接机器人与夹爪
    body:
      {
        "ip": "172.16.0.2",
        "relative_dynamics_factor": 0.015,  # 可选
        "recover": true,                    # 可选，自动清错
        "motion_async": false,              # 可选，默认同步阻塞，贴近 twin-rl
        "stop_before_move": false,          # 可选，默认关闭，避免流式控制被强制 stop
        "collision_behavior_enabled": true, # 可选，默认开启 twin-rl 同款碰撞阈值
        "collision_tau_thresholds": [25,25,25,25,25,25,25],
        "collision_force_thresholds": [40,40,80,25,25,25]
      }
    """
    body = get_json()
    ip = body.get("ip", STATE["ip"] or "172.16.0.2")
    rdf = float(body.get("relative_dynamics_factor", STATE["relative_dynamics_factor"]))
    recover = bool(body.get("recover", True))
    motion_async = bool(body.get("motion_async", STATE["motion_async"]))
    stop_before_move = bool(body.get("stop_before_move", STATE["stop_before_move"]))
    collision_behavior_enabled = bool(
        body.get("collision_behavior_enabled", STATE["collision_behavior_enabled"])
    )
    collision_tau_thresholds = parse_threshold_list(
        body.get("collision_tau_thresholds"),
        expected_len=7,
        name="collision_tau_thresholds",
        default=STATE["collision_tau_thresholds"],
    )
    collision_force_thresholds = parse_threshold_list(
        body.get("collision_force_thresholds"),
        expected_len=6,
        name="collision_force_thresholds",
        default=STATE["collision_force_thresholds"],
    )

    with LOCK:
        # 如已有连接，先释放（可按需保留）
        STATE["robot"] = Robot(ip)
        STATE["gripper"] = Gripper(ip)
        STATE["ip"] = ip
        STATE["relative_dynamics_factor"] = rdf
        STATE["motion_async"] = motion_async
        STATE["stop_before_move"] = stop_before_move
        STATE["collision_behavior_enabled"] = collision_behavior_enabled
        STATE["collision_tau_thresholds"] = collision_tau_thresholds
        STATE["collision_force_thresholds"] = collision_force_thresholds

        # 推荐：启动先做一次清错与参数设置
        STATE["robot"].recover_from_errors()
        STATE["robot"].relative_dynamics_factor = rdf
        if collision_behavior_enabled:
            STATE["robot"].set_collision_behavior(
                collision_tau_thresholds, collision_force_thresholds
            )

        if recover:
            STATE["robot"].recover_from_errors()

        STATE["last_pose"] = None
        STATE["last_joint"] = None
        STATE["last_state_ts"] = None

    return ok(
        {
            "ip": ip,
            "relative_dynamics_factor": rdf,
            "recover": recover,
            "motion_async": motion_async,
            "stop_before_move": stop_before_move,
            "collision_behavior_enabled": collision_behavior_enabled,
            "collision_tau_thresholds": collision_tau_thresholds,
            "collision_force_thresholds": collision_force_thresholds,
        },
        msg="connected"
    )

@app.route("/recover", methods=["POST"])
def recover():
    """恢复错误"""
    robot, _ = ensure_connected()
    with LOCK:
        robot.recover_from_errors()
    return ok(msg="recovered")

@app.route("/state/pose", methods=["GET"])
def state_pose():
    """获取末端位姿（含四元数）"""
    robot, _ = ensure_connected()
    pose = get_pose(robot)
    return ok({"pose": np_to_list(pose)})


@app.route("/state", methods=["GET"])
def state():
    """一次性获取末端位姿、关节位置和夹爪状态。"""
    robot, gripper = ensure_connected()
    snapshot = get_state_snapshot(robot, gripper)
    return ok(snapshot)

@app.route("/state/pose_euler", methods=["GET"])
def state_pose_euler():
    """获取末端位姿（欧拉角，弧度）"""
    robot, _ = ensure_connected()
    pose_e = get_pose_euler(robot)
    return ok({"pose_euler": np_to_list(pose_e)})

@app.route("/state/joint", methods=["GET"])
def state_joint():
    """获取关节位置"""
    robot, _ = ensure_connected()
    joint = get_joint(robot)
    return ok({"joint": np_to_list(joint)})

@app.route("/state/gripper", methods=["GET"])
def state_gripper():
    """获取夹爪开口宽度（m）"""
    _, gripper = ensure_connected()
    width = get_gripper(gripper)
    return ok({"width": width})

@app.route("/motion/goto_joint", methods=["POST"])
def motion_goto_joint():
    """
    关节空间运动
    body:
      {"joint": [j1, j2, j3, j4, j5, j6, j7]}
    """
    body = get_json()
    joint = body.get("joint")
    if not (isinstance(joint, (list, tuple)) and len(joint) in (6, 7)):
        return fail("joint 必须是长度为 6/7 的数组")
    joint = np.array(joint, dtype=float)

    robot, _ = ensure_connected()
    maybe_stop_robot(robot)
    try:
        goto_joint(robot, joint, asynchronous=STATE["motion_async"])
    except Exception as exc:
        if is_preempted_exception(exc):
            return ok({"preempted": True}, msg="goto_joint preempted")
        raise
    return ok(msg="goto_joint done")

@app.route("/motion/goto_pose", methods=["POST"])
def motion_goto_pose():
    """
    笛卡尔运动（四元数）
    body:
      {"pose": [x, y, z, qx, qy, qz, qw]}
    """
    body = get_json()
    pose = body.get("pose")
    if not (isinstance(pose, (list, tuple)) and len(pose) == 7):
        return fail("pose 必须是长度为 7 的数组 [x,y,z,qx,qy,qz,qw]")
    pose = np.array(pose, dtype=float)

    robot, _ = ensure_connected()
    maybe_stop_robot(robot)
    try:
        goto_pose(robot, pose, asynchronous=STATE["motion_async"])
    except Exception as exc:
        if is_preempted_exception(exc):
            return ok({"preempted": True}, msg="goto_pose preempted")
        raise
    return ok(msg="goto_pose done")

@app.route("/motion/goto_pose_euler", methods=["POST"])
def motion_goto_pose_euler():
    """
    笛卡尔运动（欧拉角，弧度）
    body:
      {"pose": [x, y, z, rx, ry, rz]}
    """
    body = get_json()
    pose = body.get("pose")
    if not (isinstance(pose, (list, tuple)) and len(pose) == 6):
        return fail("pose 必须是长度为 6 的数组 [x,y,z,rx,ry,rz]")
    pose = np.array(pose, dtype=float)

    robot, _ = ensure_connected()
    maybe_stop_robot(robot)
    try:
        goto_pose_euler(robot, pose, asynchronous=STATE["motion_async"])
    except Exception as exc:
        if is_preempted_exception(exc):
            return ok({"preempted": True}, msg="goto_pose_euler preempted")
        raise
    return ok(msg="goto_pose_euler done")

@app.route("/gripper/move", methods=["POST"])
def gripper_move():
    """
    移动夹爪到指定宽度
    body:
      {"width": 0.04, "speed": 0.05}  # speed 可选
    """
    body = get_json()
    width = body.get("width", None)
    speed = float(body.get("speed", 0.05))
    if width is None:
        return fail("缺少参数 width")
    width = float(width)

    _, gripper = ensure_connected()
    success = move_gripper(gripper, width, speed)
    return ok({"success": success, "width": width, "speed": speed})

@app.route("/gripper/open", methods=["POST"])
def gripper_open():
    """打开夹爪"""
    body = get_json()
    speed = float(body.get("speed", 0.1))
    _, gripper = ensure_connected()
    success = open_gripper(gripper, speed)
    return ok({"success": success, "speed": speed})

@app.route("/gripper/close", methods=["POST"])
def gripper_close():
    """关闭夹爪（使用 grasp 语义，贴近 twin-rl）。"""
    body = get_json()
    width = float(body.get("width", 0.0))
    speed = float(body.get("speed", 0.1))
    force = float(body.get("force", 2.0))
    epsilon_outer = float(body.get("epsilon_outer", 1.0))
    _, gripper = ensure_connected()
    success = close_gripper(
        gripper,
        width=width,
        speed=speed,
        force=force,
        epsilon_outer=epsilon_outer,
    )
    return ok(
        {
            "success": success,
            "width": width,
            "speed": speed,
            "force": force,
            "epsilon_outer": epsilon_outer,
        }
    )


@app.route("/clearerr", methods=["POST"])
def legacy_clearerr():
    return recover()


@app.route("/getstate", methods=["POST"])
def legacy_getstate():
    robot, gripper = ensure_connected()
    return jsonify(legacy_state_payload(robot, gripper))


@app.route("/getpos", methods=["POST"])
def legacy_getpos():
    robot, _ = ensure_connected()
    return jsonify({"pose": np_to_list(get_pose(robot))})


@app.route("/getpos_euler", methods=["POST"])
def legacy_getpos_euler():
    robot, _ = ensure_connected()
    return jsonify({"pose": np_to_list(get_pose_euler(robot))})


@app.route("/getq", methods=["POST"])
def legacy_getq():
    robot, _ = ensure_connected()
    return jsonify({"q": np_to_list(get_joint(robot))})


@app.route("/get_gripper", methods=["POST"])
def legacy_get_gripper():
    _, gripper = ensure_connected()
    return jsonify({"gripper": float(get_gripper(gripper))})


@app.route("/pose", methods=["POST"])
def legacy_pose():
    body = get_json()
    pose = body.get("arr")
    if not (isinstance(pose, (list, tuple)) and len(pose) == 7):
        return fail("arr 必须是长度为 7 的数组 [x,y,z,qx,qy,qz,qw]")
    pose = np.array(pose, dtype=float)

    robot, _ = ensure_connected()
    maybe_stop_robot(robot)
    try:
        goto_pose(robot, pose, asynchronous=STATE["motion_async"])
    except Exception as exc:
        if is_preempted_exception(exc):
            return jsonify({"preempted": True})
        raise
    return jsonify({"success": True})


@app.route("/open_gripper", methods=["POST"])
def legacy_open_gripper():
    body = get_json()
    _, gripper = ensure_connected()
    speed = float(body.get("speed", 0.1))
    success = open_gripper(gripper, speed=speed)
    return jsonify({"success": bool(success), "speed": speed})


@app.route("/close_gripper", methods=["POST"])
def legacy_close_gripper():
    body = get_json()
    _, gripper = ensure_connected()
    width = float(body.get("width", 0.0))
    speed = float(body.get("speed", 0.1))
    force = float(body.get("force", 2.0))
    epsilon_outer = float(body.get("epsilon_outer", 1.0))
    success = close_gripper(
        gripper,
        width=width,
        speed=speed,
        force=force,
        epsilon_outer=epsilon_outer,
    )
    return jsonify(
        {
            "success": bool(success),
            "width": width,
            "speed": speed,
            "force": force,
            "epsilon_outer": epsilon_outer,
        }
    )


@app.route("/close_gripper_slow", methods=["POST"])
def legacy_close_gripper_slow():
    body = get_json()
    _, gripper = ensure_connected()
    width = float(body.get("width", 0.0))
    speed = float(body.get("speed", 0.05))
    force = float(body.get("force", 2.0))
    epsilon_outer = float(body.get("epsilon_outer", 1.0))
    success = close_gripper(
        gripper,
        width=width,
        speed=speed,
        force=force,
        epsilon_outer=epsilon_outer,
    )
    return jsonify(
        {
            "success": bool(success),
            "width": width,
            "speed": speed,
            "force": force,
            "epsilon_outer": epsilon_outer,
        }
    )


@app.route("/jointreset", methods=["POST"])
def legacy_jointreset():
    robot, gripper = ensure_connected()
    reset_joint = np.array([0.0, -0.79, 0.0, -2.37, 0.0, 1.57, 0.79], dtype=float)
    maybe_stop_robot(robot)
    goto_joint(robot, reset_joint, asynchronous=False)
    open_gripper(gripper)
    return jsonify({"success": True, "joint": reset_joint.tolist()})


@app.route("/update_param", methods=["POST"])
def legacy_update_param():
    body = get_json()
    robot, _ = ensure_connected()

    if "relative_dynamics_factor" in body:
        rdf = float(body["relative_dynamics_factor"])
        robot.relative_dynamics_factor = rdf
        STATE["relative_dynamics_factor"] = rdf

    return jsonify({"success": True, "applied_keys": sorted(body.keys())})

@app.route("/reset", methods=["POST"])
def api_reset():
    """复位机器人到预设姿态并打开夹爪"""
    robot, gripper = ensure_connected()
    with LOCK:
        success = reset_robot(robot, gripper)
    return ok({"success": success})

@app.route("/config/dynamics", methods=["POST"])
def config_dynamics():
    """
    设置机器人动力学比例（相对动态因子）
    body: {"relative_dynamics_factor": 0.015}
    """
    body = get_json()
    rdf = body.get("relative_dynamics_factor", None)
    if rdf is None:
        return fail("缺少参数 relative_dynamics_factor")
    rdf = float(rdf)

    robot, _ = ensure_connected()
    with LOCK:
        robot.relative_dynamics_factor = rdf
        STATE["relative_dynamics_factor"] = rdf
    return ok({"relative_dynamics_factor": rdf})

@app.route("/config/motion_mode", methods=["POST"])
def config_motion_mode():
    """
    设置运动模式（同步/异步）
    body: {"motion_async": true/false}

    注意：这只改变 motion_async 标志，不会重新连接机器人
    """
    body = get_json()
    motion_async = body.get("motion_async", None)
    if motion_async is None:
        return fail("缺少参数 motion_async")
    motion_async = bool(motion_async)

    ensure_connected()  # 确保已连接
    with LOCK:
        STATE["motion_async"] = motion_async
    return ok({"motion_async": motion_async})

@app.route("/health", methods=["GET"])
def health():
    """健康检查"""
    return ok({
        "connected": STATE["robot"] is not None and STATE["gripper"] is not None,
        "ip": STATE["ip"],
        "relative_dynamics_factor": STATE["relative_dynamics_factor"],
        "motion_async": STATE["motion_async"],
        "stop_before_move": STATE["stop_before_move"],
        "collision_behavior_enabled": STATE["collision_behavior_enabled"],
        "collision_tau_thresholds": STATE["collision_tau_thresholds"],
        "collision_force_thresholds": STATE["collision_force_thresholds"],
    })

# ======================
# 启动
# ======================

if __name__ == "__main__":
    # 生产环境建议使用 gunicorn/uvicorn 等启动
    # 例如：gunicorn -w 1 -b 0.0.0.0:5000 franky_server:app
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
