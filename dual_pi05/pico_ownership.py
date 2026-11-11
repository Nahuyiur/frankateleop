"""Pause only the identified Pico command process, preserving the robot nodes.

SIGSTOP leaves its shell supervisor alive; killing the Pico launcher instead
would run that launcher's cleanup and stop both robot stacks. Resumption is
always a separate explicit CLI action, never an automatic error cleanup.
"""
import json
import os
from pathlib import Path
import signal
import time

from motion_control import MotionError

PICO_SCRIPT = "/home/pnp/Desktop/pico_franka_bimanual_teleop/run_controller.py"


def identity(pid, proc_root=Path("/proc")):
    try:
        p = proc_root / str(pid)
        args = (p / "cmdline").read_bytes().decode().strip("\0").split("\0")
        stat = (p / "stat").read_text().rsplit(")", 1)[1].split()
        if PICO_SCRIPT not in args or "--execute" not in args:
            return None
        return {"pid": int(pid), "start_ticks": stat[19], "state": stat[0], "args": args}
    except (OSError, ValueError, IndexError):
        return None


def controllers(proc_root=Path("/proc")):
    result = []
    for p in proc_root.iterdir():
        if p.name.isdigit():
            found = identity(p.name, proc_root)
            if found:
                result.append(found)
    return result


def verify_tool_config(items, profile):
    expected = profile["tool_transforms"]["left"]["translation_m"]
    for item in items:
        args = item["args"]
        if "--tool-tip-offset-eef-m" in args:
            start = args.index("--tool-tip-offset-eef-m") + 1
            actual = [float(x) for x in args[start:start+3]]
            if len(actual) != 3 or any(abs(x-y) > 1e-8 for x, y in zip(actual, expected)):
                raise MotionError("Running Pico tool offset differs from the Pi0.5 hardware profile")


def pause(items, receipt_path):
    receipt_path = Path(receipt_path)
    receipt = json.loads(receipt_path.read_text()) if receipt_path.exists() else []
    existing = {(x["pid"], x["start_ticks"]) for x in receipt}
    for item in items:
        current = identity(item["pid"])
        if current is None or current["start_ticks"] != item["start_ticks"]:
            raise MotionError("Pico process changed before pause")
        if current["state"] in ("T", "t"):
            continue
        # Persist identity before pausing so an interrupted invocation remains recoverable.
        key = (item["pid"], item["start_ticks"])
        if key not in existing:
            receipt.append({"pid": item["pid"], "start_ticks": item["start_ticks"]})
            receipt_path.parent.mkdir(parents=True, exist_ok=True)
            receipt_path.write_text(json.dumps(receipt, indent=2))
            existing.add(key)
        os.kill(item["pid"], signal.SIGSTOP)
        deadline = time.monotonic()+1.0
        while time.monotonic() < deadline:
            current = identity(item["pid"])
            if current and current["start_ticks"] == item["start_ticks"] and current["state"] in ("T", "t"):
                break
            time.sleep(0.01)
        else:
            raise MotionError("Could not verify Pico pause")
    return receipt


def resume(receipt_path):
    path = Path(receipt_path)
    if not path.exists():
        return []
    receipt = json.loads(path.read_text())
    validated = []
    for item in receipt:
        current = identity(item["pid"])
        if current is None:
            continue
        if current["start_ticks"] != item["start_ticks"]:
            raise MotionError("Recorded PID was reused; refusing to resume it")
        if current["state"] in ("T", "t"):
            validated.append(item["pid"])
    for pid in validated:
        os.kill(pid, signal.SIGCONT)
    path.unlink()
    return validated
