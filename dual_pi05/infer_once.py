#!/usr/bin/env python3
"""Host98 coordinator: collect both arms, infer on106, save routing preview.

This entry point only calls get_observations and inference HTTP endpoints.
It has no actuation, homing or robot-start method.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time
import urllib.request

import zmq

POLICY = "pi05_umi_dual_franka_stack_cubes_10k"
ROUTES = {"left": "tcp://127.0.0.1:6002", "right": "tcp://172.16.0.3:6001"}
CAMERAS = {"left": "http://127.0.0.1:9140/snapshot", "right": "http://172.16.0.3:9141/snapshot"}
HOME = Path("/home/pnp/frankateleop/config/initial_joints_v0.5.json")


def http(url, payload=None, timeout=120):
    request = urllib.request.Request(url, data=None if payload is None else json.dumps(payload).encode(),
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def state(arm):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDTIMEO, 2000)
    socket.setsockopt(zmq.RCVTIMEO, 5000)
    socket.connect(ROUTES[arm])
    try:
        socket.send_pyobj({"method": "get_observations", "args": {}})
        value = socket.recv_pyobj()
        # Use measured width, not the controller's last commanded closedness.
        width = float(value["gripper_width"][0])
        result = {"joint_position": [float(x) for x in value["joint_positions"][:7]],
                  "eef_pose": [float(x) for x in value["ee_pose_euler"]],
                  "gripper_width_m": width, "gripper_closedness": 1.0-width/0.2}
        if len(result["joint_position"]) != 7 or len(result["eef_pose"]) != 6:
            raise ValueError("Unexpected state layout for " + arm)
        if not all(math.isfinite(x) for x in result["joint_position"] + result["eef_pose"] + [width]):
            raise ValueError("Non-finite state for " + arm)
        if not 0 <= width <= 0.2:
            raise ValueError("Gripper width outside the configured 0.2m range")
        return {"robot_state": result, "received_at": time.time()}
    finally:
        socket.close()
        context.term()


def routing_preview(result):
    if result.get("format_version") != "robot_action_v2" or result.get("robot_mode") != "dual":
        raise ValueError("Expected canonical dual-arm action response")
    arms = result.get("arm_actions")
    if not isinstance(arms, dict) or set(arms) != {"left", "right"}:
        raise ValueError("Missing left/right actions")
    if len(arms["left"]["actions"]) != len(arms["right"]["actions"]):
        raise ValueError("Unequal left/right action horizons")
    return {arm: {"destination": ROUTES[arm], "action_count": len(arms[arm]["actions"]),
                  "first_target": arms[arm]["actions"][0], "sent": False} for arm in ROUTES}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gateway", default="http://172.16.0.5:8010")
    parser.add_argument("--prompt", default="Stack the cubes into a tower")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "runtime/last_inference.json")
    args = parser.parse_args()
    gateway = args.gateway.rstrip("/")
    session = http(gateway + "/api/v2/sessions", {"policy_id": POLICY, "instruction": args.prompt,
                   "robot_mode": "dual", "client": {"host": "98", "mode": "inference_only"}})
    session_url = gateway + "/api/v2/sessions/" + session["session_id"]
    try:
        with ThreadPoolExecutor(max_workers=4) as pool:
            cameras = {arm: pool.submit(http, CAMERAS[arm], timeout=10) for arm in ROUTES}
            cameras = {arm: future.result() for arm, future in cameras.items()}
            # Read state after camera startup, close to the delivered frames.
            states = {arm: pool.submit(state, arm) for arm in ROUTES}
            states = {arm: future.result() for arm, future in states.items()}
        observation = {"format_version": "frankaeval_observation_v2", "robot_mode": "dual",
                       "observation_seq": 0, "timestamp": time.time(),
                       "images": {arm+"_wrist": cameras[arm]["image"] for arm in ROUTES},
                       "robot_states": {arm: states[arm]["robot_state"] for arm in ROUTES}}
        started = time.monotonic()
        response = http(session_url + "/infer", observation)
        elapsed = time.monotonic() - started
        home = json.loads(HOME.read_text())
        home_delta = {arm: max(abs(x-y) for x, y in zip(observation["robot_states"][arm]["joint_position"],
                                                        home[arm]["joints"])) for arm in ROUTES}
        result = {"gateway": gateway, "prompt": args.prompt, "robot_commands_sent": False,
                  "execution_ready": False, "network_inference_seconds": elapsed,
                  "home_pose_source": str(HOME), "max_joint_distance_to_home_rad": home_delta,
                  "capture_times": {arm: cameras[arm]["captured_at"] for arm in ROUTES},
                  "state_received_times": {arm: states[arm]["received_at"] for arm in ROUTES},
                  "robot_states": observation["robot_states"], "routing_preview": routing_preview(response),
                  "response": response}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False))
        print(json.dumps({k: v for k, v in result.items() if k != "response"}, indent=2))
        print("Saved", args.output)
    finally:
        http(session_url + "/end", {"reason": "inference_only_validation_complete"})


if __name__ == "__main__":
    main()
