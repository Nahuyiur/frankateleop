import copy
import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import zmq

from contract import load_profile
from motion_control import (ARMS, DualRPC, MotionError, observed_state, state_target,
                            trajectory, validate_joints, validate_pair)
import pico_ownership
import run_policy


@pytest.fixture
def profile():
    p, _ = load_profile()
    p = copy.deepcopy(p)
    p["base_frames"] = {a: {"xyz": [0, 0, 0], "rpy": [0, 0, 0]} for a in ARMS}
    p["workspace_xyz"] = [[-2, 2]]*3
    return p


@pytest.fixture
def states():
    return {a: {"joint_position": [0, 0, 0, -1.5, 0, 2, 0], "joint_velocity": [0]*7, "eef_pose": [0.4, y, 0.3, 0, 0, 0],
                "gripper_width_m": 0.08, "gripper_closedness": 0.6}
            for a, y in (("left", 0.2), ("right", -0.2))}


def response(states, width=0.08):
    result = {"format_version": "robot_action_v2", "robot_mode": "dual",
              "server_metadata": {"policy_id": run_policy.POLICY, "backend_metadata": {
                  "hardware_profile_sha256": "test_hash", "output_target_frame": "per_arm_robot_base_to_panda_link8"}},
              "arm_actions": {}}
    for arm in ARMS:
        start = state_target(states[arm])
        actions = []
        for i in range(50):
            item = copy.deepcopy(start)
            item["target"][0] += 0.001*(i+1)
            item["action_index"] = i
            item["gripper_target_width_m"] = width
            actions.append(item)
        result["arm_actions"][arm] = {"control_type": "absolute_eef_pose", "actions": actions}
    return result


def test_measured_gripper_width_overrides_last_command(profile):
    raw = {"joint_positions": [0]*8, "joint_velocities": [0]*7, "ee_pose_euler": [0]*6, "gripper_width": [0.05],
           "gripper_closedness": [0.0]}
    state = observed_state(raw, profile)
    assert state["gripper_closedness"] == pytest.approx(0.75)


def test_trajectory_preserves_absolute_targets_and_applies_rate_limits(profile, states):
    anchors = {a: state_target(states[a]) for a in ARMS}
    frames, clipped = trajectory(response(states, width=0.12), states, anchors, profile, "test_hash", steps=5)
    assert clipped == 10
    assert frames[-1]["left"]["target"][0] == pytest.approx(0.405)
    assert frames[-1]["right"]["target"][1] == pytest.approx(-0.2)
    assert frames[-1]["left"]["gripper_target_width_m"] == pytest.approx(0.09)
    previous = anchors
    for pair in frames:
        for arm in ARMS:
            delta = np.linalg.norm(np.asarray(pair[arm]["target"][:3])-previous[arm]["target"][:3])
            assert delta*profile["output_hz"] <= profile["max_translation_speed_m_s"]+1e-8
        previous = pair


def test_replanning_interpolates_from_last_command_instead_of_jumping_to_lagged_measurement(profile, states):
    anchors = {a: state_target(states[a]) for a in ARMS}
    commanded = copy.deepcopy(anchors)
    commanded["left"]["target"][0] += 0.02
    frames, _ = trajectory(response(states), states, anchors, profile, "test_hash", steps=1, commanded_start=commanded)
    assert abs(frames[0]["left"]["target"][0]-commanded["left"]["target"][0]) <= profile["max_translation_speed_m_s"]/profile["output_hz"]+1e-8


@pytest.mark.parametrize("failure", ["missing_arm", "unequal", "nan", "quaternion", "frame", "profile", "large_jump", "index"])
def test_reject_malformed_or_incompatible_chunk(failure, profile, states):
    r = response(states)
    if failure == "missing_arm": del r["arm_actions"]["right"]
    if failure == "unequal": r["arm_actions"]["right"]["actions"].pop()
    if failure == "nan": r["arm_actions"]["left"]["actions"][0]["target"][0] = float("nan")
    if failure == "quaternion": r["arm_actions"]["left"]["actions"][0]["target"][3:] = [0, 0, 0, 0]
    if failure == "frame": r["server_metadata"]["backend_metadata"]["output_target_frame"] = "world"
    if failure == "profile": r["server_metadata"]["backend_metadata"]["hardware_profile_sha256"] = "old"
    if failure == "large_jump": r["arm_actions"]["left"]["actions"][0]["target"][0] += 0.2
    if failure == "index": r["arm_actions"]["left"]["actions"][0]["action_index"] = 10
    with pytest.raises((MotionError, ValueError)):
        trajectory(r, states, {a: state_target(states[a]) for a in ARMS}, profile, "test_hash")


def test_joint_margin_and_tcp_collision_checks(profile, states):
    states["left"]["joint_position"][0] = 2.62
    with pytest.raises(MotionError): validate_joints(states, profile)
    targets = {a: state_target(states[a]) for a in ARMS}
    targets["right"]["target"] = targets["left"]["target"].copy()
    with pytest.raises(MotionError): validate_pair(targets, targets, profile)


def test_reject_stale_unsynchronized_observations(profile):
    with pytest.raises(MotionError):
        run_policy.validate_freshness({a: {"captured_at": 10} for a in ARMS}, 10, profile, now=11)
    with pytest.raises(MotionError):
        run_policy.validate_freshness({"left": {"captured_at": 10}, "right": {"captured_at": 10.4}}, 10.4, profile, now=10.5)


class FakeServers:
    def __init__(self, replies=None, silent_right=False):
        self.context = zmq.Context()
        self.stop = threading.Event()
        self.received = {a: threading.Event() for a in ARMS}
        self.commands = {a: [] for a in ARMS}
        self.endpoints = {}
        self.threads = []
        self.replies = replies or {a: None for a in ARMS}
        self.silent_right = silent_right
        for arm in ARMS:
            ready = threading.Event()
            thread = threading.Thread(target=self.serve, args=(arm, ready), daemon=True)
            thread.start()
            assert ready.wait(2)
            self.threads.append(thread)

    def serve(self, arm, ready):
        s = self.context.socket(zmq.REP)
        s.setsockopt(zmq.LINGER, 0)
        port = s.bind_to_random_port("tcp://127.0.0.1")
        self.endpoints[arm] = "tcp://127.0.0.1:"+str(port)
        ready.set()
        while not self.stop.is_set():
            if not s.poll(30): continue
            request = s.recv_pyobj()
            self.commands[arm].append(request)
            self.received[arm].set()
            # Neither side replies until BOTH commands have arrived.
            for other in ARMS:
                if not self.received[other].wait(1): break
            if self.silent_right and arm == "right":
                self.stop.wait(1)
                break
            s.send_pyobj(self.replies[arm])
        s.close(0)

    def close(self):
        self.stop.set()
        for t in self.threads: t.join(2)
        self.context.term()


def test_dual_rpc_sends_both_before_waiting_and_converts_quaternion(profile, states):
    servers = FakeServers()
    rpc = DualRPC(servers.endpoints, timeout_s=0.5)
    try:
        targets = {a: state_target(states[a]) for a in ARMS}
        targets["right"]["target"][3:] = Rotation.from_euler("xyz", [0.2, 0.1, -0.3]).as_quat().tolist()
        rpc.command(targets, profile)
        assert rpc.sent_counts == {"left": 1, "right": 1}
        request = servers.commands["right"][0]
        assert request["method"] == "command_ee_pose"
        np.testing.assert_allclose(request["args"]["pose_6d"][3:], [0.2, 0.1, -0.3])
        assert request["args"]["gripper_width"] == 0.08
    finally:
        rpc.close(); servers.close()


@pytest.mark.parametrize("reply", [{"error": "IK failed"}, {"status": "hold", "reason": "joint_soft_limit"}])
def test_one_arm_rejection_is_fatal_and_not_retried(reply, profile, states):
    servers = FakeServers({"left": None, "right": reply})
    rpc = DualRPC(servers.endpoints, timeout_s=0.5)
    try:
        with pytest.raises(MotionError): rpc.command({a: state_target(states[a]) for a in ARMS}, profile)
        assert all(len(servers.commands[a]) == 1 for a in ARMS)
    finally:
        rpc.close(); servers.close()


def test_motion_timeout_is_not_retried(profile, states):
    servers = FakeServers(silent_right=True)
    rpc = DualRPC(servers.endpoints, timeout_s=0.08)
    try:
        with pytest.raises(MotionError): rpc.command({a: state_target(states[a]) for a in ARMS}, profile)
        assert all(len(servers.commands[a]) == 1 for a in ARMS)
    finally:
        rpc.close(); servers.close()


def test_owner_change_rejected():
    rpc = object.__new__(DualRPC)
    rpc.baseline = {a: (10, 100) for a in ARMS}
    rpc.sent_counts = {a: 5 for a in ARMS}
    rpc.call = lambda *a, **k: {arm: {"pid": 10, "command_ee_pose_total": 106} for arm in ARMS}
    with pytest.raises(MotionError): rpc.verify_ownership()


def test_pico_identity_ignores_reused_unrelated_process(tmp_path):
    p = tmp_path / "123"
    p.mkdir()
    (p / "cmdline").write_bytes(b"python\0unrelated.py\0--execute\0")
    (p / "stat").write_text("123 (python) S " + " ".join(["0"]*18+["999"]+["0"]*10))
    assert pico_ownership.identity(123, tmp_path) is None
    (p / "cmdline").write_bytes(("python\0"+pico_ownership.PICO_SCRIPT+"\0--execute\0").encode())
    assert pico_ownership.identity(123, tmp_path)["start_ticks"] == "999"


@pytest.fixture
def fake_main(monkeypatch, tmp_path, profile, states):
    home = tmp_path / "home.json"
    home.write_text(json.dumps({a: {"joints": states[a]["joint_position"]} for a in ARMS}))
    profile["home_file"] = str(home)
    monkeypatch.setattr(run_policy, "ROOT", tmp_path)
    monkeypatch.setattr(run_policy, "load_profile", lambda: (profile, "test_hash"))
    state = {"paused": False, "commands": [], "holds": [], "command_timeouts": []}

    class RPC:
        def __init__(self, **kw):
            self.sent_counts = {a: 0 for a in ARMS}
            self.endpoints = {a: "fake://"+a for a in ARMS}
        def call(self, method, **kw):
            return {a: "ee" for a in ARMS} if method == "get_control_mode" else {a: {"pid": 123} for a in ARMS}
        def states(self, p, **kwargs): return copy.deepcopy(states)
        def acquire(self): assert state["paused"]
        def verify_ownership(self): pass
        def command(self, pair, p, update_gripper=True, timeout_s=None):
            assert state["paused"]
            state["commands"].append(copy.deepcopy(pair))
            state["command_timeouts"].append(timeout_s)
            for a in ARMS: self.sent_counts[a] += 1
        def close(self): pass
    monkeypatch.setattr(run_policy, "DualRPC", RPC)
    monkeypatch.setattr(run_policy.pico_ownership, "controllers", lambda: [{"pid": 123, "start_ticks": "1", "state": "T" if state["paused"] else "S", "args": []}])
    def pause(*args): state["paused"] = True; return [123]
    monkeypatch.setattr(run_policy.pico_ownership, "pause", pause)
    monkeypatch.setattr(run_policy, "capture_images", lambda: {a: {"captured_at": time.time(), "image": {}} for a in ARMS})
    def http(url, payload=None, timeout=120):
        if url.endswith("/sessions"):
            return {"session_id": "fake", "server_readiness": {"backend_metadata": {"hardware_profile_sha256": "test_hash"}}}
        if url.endswith("/infer"): return response(payload["robot_states"])
        return {}
    monkeypatch.setattr(run_policy, "http", http)
    monkeypatch.setattr(run_policy, "best_effort_hold", lambda *args: state["holds"].append(True) or {a: "held" for a in ARMS})
    return state, tmp_path


def test_preview_never_pauses_or_sends_motion(fake_main):
    state, root = fake_main
    assert run_policy.main(["--output", str(root / "out.json")]) == 0
    assert not state["paused"] and not state["commands"] and not state["holds"]


def test_execute_refuses_active_pico_without_takeover(fake_main):
    state, root = fake_main
    assert run_policy.main(["--execute", "--output", str(root / "out.json")]) == 2
    assert not state["paused"] and not state["commands"]


def test_execute_dispatches_only_after_takeover_and_holds_on_exit(fake_main):
    state, root = fake_main
    assert run_policy.main(["--execute", "--pause-pico", "--steps-per-chunk", "1", "--output", str(root / "out.json")]) == 0
    assert state["paused"] and state["commands"] and state["holds"]
    assert state["command_timeouts"][0] == 2.0
    assert all(value is None for value in state["command_timeouts"][1:])
    report = json.loads((root / "out.json").read_text())
    assert report["status"] == "completed" and report["robot_commands_sent"]
    assert len(report["cycles"]) == 1
