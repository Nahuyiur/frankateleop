#!/usr/bin/env python3
"""Host98: observe both Frankas, infer on106, optionally execute a short prefix."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import time

import numpy as np
import zmq

from contract import ARMS, load_profile
from infer_once import CAMERAS, POLICY, http
from motion_control import (DualRPC, MotionError, observed_state, pose_rotation, state_target,
                            rotation_distance, trajectory, validate_joints, validate_pair,
                            validate_stationary, validate_tracking)
import pico_ownership

ROOT = Path(__file__).resolve().parent


def capture_images():
    with ThreadPoolExecutor(max_workers=2) as pool:
        jobs = {arm: pool.submit(http, CAMERAS[arm], timeout=5) for arm in ARMS}
        return {arm: jobs[arm].result() for arm in ARMS}


def observation(cameras, states, sequence):
    return {"format_version": "frankaeval_observation_v2", "robot_mode": "dual",
            "observation_seq": sequence, "timestamp": time.time(),
            "images": {arm+"_wrist": cameras[arm]["image"] for arm in ARMS},
            "robot_states": states}


def validate_freshness(cameras, state_time, profile, now=None):
    now = time.time() if now is None else now
    times = [float(cameras[a]["captured_at"]) for a in ARMS]
    if not all(math.isfinite(t) for t in times+[state_time]):
        raise MotionError("Invalid observation timestamps")
    if any(t > now+0.05 or now-t > profile["max_observation_age_s"] for t in times+[state_time]):
        raise MotionError("Observation is stale or host clocks differ")
    if max(times+[state_time])-min(times+[state_time]) > profile["max_image_state_skew_s"]:
        raise MotionError("Image/state capture skew is too large")


def best_effort_hold(endpoints, profile):
    """Read and hold each reachable arm independently; never replay failed targets."""
    result = {}
    for arm in ARMS:
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDTIMEO, 250)
        socket.setsockopt(zmq.RCVTIMEO, 250)
        socket.connect(endpoints[arm])
        try:
            socket.send_pyobj({"method": "get_observations", "args": {}})
            state = observed_state(socket.recv_pyobj(), profile)
            socket.send_pyobj({"method": "command_ee_pose", "args": {
                "pose_6d": np.asarray(state["eef_pose"]), "gripper_width": state["gripper_width_m"],
                "update_gripper": False}})
            reply = socket.recv_pyobj()
            result[arm] = "measured_pose_hold_sent" if reply is None else str(reply)
        except Exception as error:
            result[arm] = "hold_unconfirmed: " + str(error)
        finally:
            socket.close(0)
            context.term()
    return result


class KeepAlive:
    def __init__(self, rpc, profile, anchors):
        self.rpc, self.profile, self.anchors = rpc, profile, anchors
        self.current = anchors
        self.next_tick = time.monotonic()
        self.last_check = 0.0
        self.motion_started = False

    def tick(self, target=None, update_gripper=False):
        period = 1.0/self.profile["output_hz"]
        now = time.monotonic()
        if now < self.next_tick:
            time.sleep(self.next_tick-now)
        # Do not replay a backlog after a slow RPC or observation.
        self.next_tick = time.monotonic()+period
        if time.monotonic()-self.last_check >= 0.1:
            self.rpc.verify_ownership()
            states = self.rpc.states(self.profile)
            validate_tracking(states, self.current, self.profile)
            self.last_check = time.monotonic()
        candidate = self.current if target is None else target
        validate_pair(candidate, self.anchors, self.profile)
        for arm in ARMS:
            previous, following = self.current[arm], candidate[arm]
            distance = np.linalg.norm(np.asarray(previous["target"][:3])-following["target"][:3])
            if distance > self.profile["max_translation_speed_m_s"]*period+1e-7:
                raise MotionError("Commanded position step exceeds the configured speed limit")
            if rotation_distance(previous["target"], following["target"]) > self.profile["max_rotation_speed_rad_s"]*period+1e-7:
                raise MotionError("Commanded rotation step exceeds the configured speed limit")
            if abs(previous["gripper_target_width_m"]-following["gripper_target_width_m"]) > self.profile["max_gripper_speed_m_s"]*period+1e-7:
                raise MotionError("Commanded gripper step exceeds the configured speed limit")
        self.motion_started = True
        self.rpc.command(candidate, self.profile, update_gripper=update_gripper)
        self.current = candidate

    def wait(self, future, timeout):
        deadline = time.monotonic()+timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise MotionError("Observation/inference deadline exceeded")
            self.tick()
        return future.result()


def preflight(rpc, profile):
    # Setup reads may wait behind an existing Pico request or node startup.
    # The short deadlines remain enforced after motion control is acquired.
    modes = rpc.call("get_control_mode", timeout_s=5.0)
    metrics = rpc.call("get_metrics", timeout_s=5.0)
    states = rpc.states(profile, timeout_s=5.0)
    home = json.loads(Path(profile["home_file"]).read_text())
    home_distance = {}
    for arm in ARMS:
        joints = home[arm]["joints"]
        if len(joints) != 7 or not all(math.isfinite(float(q)) for q in joints):
            raise MotionError("Invalid home joints")
        home_distance[arm] = max(abs(x-y) for x, y in zip(joints, states[arm]["joint_position"]))
    return {"control_modes": modes, "robot_node_pids": {a: metrics[a]["pid"] for a in ARMS},
            "max_joint_distance_to_home_rad": home_distance, "home_file": profile["home_file"]}, states


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="send robot motions; omitted means preview only")
    parser.add_argument("--pause-pico", action="store_true", help="with --execute, suspend only Pico's command process")
    parser.add_argument("--resume-pico", action="store_true", help="resume Pico explicitly; sends no Pi0.5 actions")
    parser.add_argument("--prompt", default="Stack the cubes into a tower")
    parser.add_argument("--gateway", default="http://172.16.0.5:8010")
    parser.add_argument("--cycles", type=int, default=1, help="number of observation/inference/action cycles")
    parser.add_argument("--steps-per-chunk", type=int, default=5)
    parser.add_argument("--speed-scale", type=float, default=0.2)
    parser.add_argument("--output", type=Path, default=ROOT / "runtime/policy_execution.json")
    args = parser.parse_args(argv)
    if args.resume_pico and (args.execute or args.pause_pico):
        parser.error("--resume-pico must be used alone")
    if not 1 <= args.cycles <= 1000 or not 1 <= args.steps_per_chunk <= 50 or not 0.05 <= args.speed_scale <= 1:
        parser.error("Invalid cycle count, prefix length or speed scale")
    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    lock = (runtime / "motion.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise MotionError("Another Pi0.5 coordinator is running")
    receipt = runtime / "paused_pico.json"
    if args.resume_pico:
        print(json.dumps({"resumed_pico_pids": pico_ownership.resume(receipt)}))
        lock.close()
        return 0
    profile, profile_hash = load_profile()
    rpc = DualRPC(timeout_s=profile["max_command_latency_s"])
    report = {"mode": "execute" if args.execute else "preview", "robot_commands_sent": False,
              "hardware_profile_id": profile["profile_id"], "hardware_profile_sha256": profile_hash,
              "physical_execution_validated": False, "cycles": [], "started_at": time.time()}
    session_url, keeper = None, None
    pool = ThreadPoolExecutor(max_workers=1)
    exit_code = 0
    try:
        check, states = preflight(rpc, profile)
        report["preflight"] = check
        pico = pico_ownership.controllers()
        pico_ownership.verify_tool_config(pico, profile)
        report["pico_controllers"] = [{k: x[k] for k in ("pid", "state", "start_ticks")} for x in pico]
        active = [x for x in pico if x["state"] not in ("T", "t")]
        if args.execute and active and not args.pause_pico:
            raise MotionError("Pico owns motion control; use --pause-pico with --execute, or stop its command process first")
        if args.execute and any(v != "ee" for v in check["control_modes"].values()):
            raise MotionError("Both existing robot nodes must use Cartesian ee mode")
        session = http(args.gateway.rstrip("/") + "/api/v2/sessions", {
            "policy_id": POLICY, "instruction": args.prompt, "robot_mode": "dual",
            "client": {"host": "98", "mode": report["mode"]}})
        session_url = args.gateway.rstrip("/")+"/api/v2/sessions/"+session["session_id"]
        meta = session["server_readiness"]["backend_metadata"]
        if meta.get("hardware_profile_sha256") != profile_hash:
            raise MotionError("106 hardware profile does not match this executor")
        # Compile/load before taking control; this warm-up never dispatches actions.
        cameras = capture_images()
        states = rpc.states(profile, timeout_s=5.0)
        report["preview_observation"] = {
            "capture_times": {a: cameras[a]["captured_at"] for a in ARMS},
            "state_received_at": time.time(), "robot_states": states,
        }
        warm = http(session_url+"/infer", observation(cameras, states, 0))
        report["model_metadata"] = warm["server_metadata"]["backend_metadata"]
        anchors = {a: state_target(states[a]) for a in ARMS}
        frames, clipped = trajectory(warm, states, anchors, profile, profile_hash,
                                    steps=args.steps_per_chunk, speed_scale=args.speed_scale)
        report["preview"] = {"waypoints_per_arm": 50, "selected_prefix": args.steps_per_chunk,
                             "interpolated_command_pairs": len(frames), "limited_gripper_targets": clipped,
                             "first_pair": frames[0], "last_pair": frames[-1]}
        validate_joints(states, profile)
        if not args.execute:
            report["status"] = "preview_passed"
            print("PREVIEW PASSED: paired motion plan validated; no robot command was sent.")
            return 0
        validate_stationary(states)
        if args.pause_pico:
            report["paused_pico"] = pico_ownership.pause(pico, receipt)
        if any(x["state"] not in ("T", "t") for x in pico_ownership.controllers()):
            raise MotionError("A Pico command process is still running")
        rpc.acquire()
        states = rpc.states(profile)
        validate_joints(states, profile)
        validate_stationary(states)
        anchors = {a: state_target(states[a]) for a in ARMS}
        validate_pair(anchors, anchors, profile)
        # Existing nodes lazily start Cartesian impedance. Allow that initial
        # measured-pose hold to initialize; policy targets keep the short deadline.
        rpc.command(anchors, profile, update_gripper=False, timeout_s=2.0)
        keeper = KeepAlive(rpc, profile, anchors)
        keeper.tick()
        for cycle in range(args.cycles):
            cameras = keeper.wait(pool.submit(capture_images), 6.0)
            states = rpc.states(profile)
            state_time = time.time()
            validate_freshness(cameras, state_time, profile)
            started = time.monotonic()
            response = keeper.wait(pool.submit(http, session_url+"/infer", observation(cameras, states, cycle+1), 3), 3.5)
            validate_freshness(cameras, state_time, profile)
            measured = rpc.states(profile)
            validate_tracking(measured, {a: state_target(states[a]) for a in ARMS}, profile)
            cycle_record = {"cycle": cycle+1, "observed_states": states,
                            "state_received_at": state_time,
                            "capture_times": {a: cameras[a]["captured_at"] for a in ARMS},
                            "response": response, "policy_command_pairs_acknowledged": 0}
            report.setdefault("inference_cycles", []).append(cycle_record)
            frames, clipped = trajectory(response, states, anchors, profile, profile_hash,
                                        steps=args.steps_per_chunk, speed_scale=args.speed_scale,
                                        commanded_start=keeper.current)
            # Validate the whole interpolated prefix before dispatching its first pair.
            cycle_record["planned_command_pairs"] = len(frames)
            cycle_record["max_waypoint_translation_m"] = {
                a: max(float(np.linalg.norm(np.asarray(item["target"][:3])-states[a]["eef_pose"][:3]))
                       for item in response["arm_actions"][a]["actions"][:args.steps_per_chunk])
                for a in ARMS}
            cycle_record["last_pair"] = frames[-1]
            for pair in frames:
                keeper.tick(pair, update_gripper=True)
                cycle_record["policy_command_pairs_acknowledged"] += 1
            report["cycles"].append({"cycle": cycle+1, "command_pairs": len(frames),
                                     "limited_gripper_targets": clipped, "completed_at": time.time(),
                                     "inference_and_execution_s": time.monotonic()-started})
            print("Completed cycle", cycle+1, "command pairs", len(frames), flush=True)
        for _ in range(10):
            keeper.tick()
        report["status"] = "completed"
        print("Execution prefix completed. Pico remains paused; --resume-pico restores it explicitly.")
    except KeyboardInterrupt:
        report["status"], report["error"] = "interrupted", "Stopped by operator"
        exit_code = 130
    except Exception as error:
        report["status"], report["error"] = "failed", str(error)
        print("STOPPED:", error, file=sys.stderr)
        exit_code = 2
    finally:
        sent = any(rpc.sent_counts.values())
        report["robot_commands_sent"] = sent
        report["command_attempts_per_arm"] = rpc.sent_counts.copy()
        if sent:
            report["final_hold"] = best_effort_hold(rpc.endpoints, profile)
        rpc.close()
        if session_url:
            try:
                http(session_url+"/end", {"reason": report.get("status", "stopped")}, timeout=5)
            except Exception as error:
                report["session_end_error"] = str(error)
        pool.shutdown(wait=True)
        report["finished_at"] = time.time()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
        print("Report:", args.output)
        lock.close()
    return exit_code


if __name__ == "__main__":
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    sys.exit(main())
