#!/usr/bin/env python3
"""Standalone, guarded GUI for tuning GELLO Dynamixel damping profiles.

The GUI never writes while a slider is moving. Hardware changes happen only
after an explicit Apply click. The teleoperation process must be stopped because
only one process may own the serial bus.
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
TELEOP_ROOT = REPO_ROOT / "teleop"
SDK_ROOT = TELEOP_ROOT / "third_party" / "DynamixelSDK" / "python" / "src"
sys.path.insert(0, str(TELEOP_ROOT))
sys.path.insert(0, str(SDK_ROOT))

from teleop.dynamixel.tuning_profile import (  # noqa: E402
    BinarySearchBounds,
    ProfileError,
    load_profile,
    nearest_safe_goals,
    save_profile,
    validate_profile,
)


ADDR_MODEL_NUMBER = 0
ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_TORQUE_ENABLE = 64
ADDR_KP_D = 80
ADDR_KP_I = 82
ADDR_KP_P = 84
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
OPERATING_MODE_CURRENT_POSITION = 5
COMM_SUCCESS = 0
SUPPORTED_MODEL_NUMBERS = {1190, 1200}

SIDE_PORT_HINTS = {
    "left": ("FTBZIB2R", "FTBJKECV", "FT7WBEIA"),
    "right": ("FTBJTKP2", "FTAUMOPA", "FT7WBG6A"),
}


def _signed32(value: int) -> int:
    return value - (1 << 32) if value & (1 << 31) else value


def _pulse_to_rad(value: int) -> float:
    return float(_signed32(value)) / 2048.0 * math.pi


def _rad_to_pulse(value: float) -> int:
    return int(round(float(value) * 2048.0 / math.pi))


def _serial_owner_pids(port: str) -> List[int]:
    executable = shutil.which("lsof")
    if executable is None:
        return []
    resolved = os.path.realpath(port)
    result = subprocess.run(
        [executable, "-t", resolved],
        check=False,
        capture_output=True,
        text=True,
    )
    return [int(line) for line in result.stdout.splitlines() if line.strip().isdigit()]


class DynamixelBus:
    """Small exclusive bus client used only by this standalone tuner."""

    def __init__(self, port: str, ids: Sequence[int], baudrate: int = 57600):
        owners = [pid for pid in _serial_owner_pids(port) if pid != os.getpid()]
        if owners:
            raise RuntimeError(
                f"Serial port {port} is already owned by PID(s) {owners}. "
                "Stop run_env/teleop before opening the tuner."
            )
        from dynamixel_sdk import GroupSyncWrite, PacketHandler, PortHandler

        self.ids = tuple(int(item) for item in ids)
        self._GroupSyncWrite = GroupSyncWrite
        self.port = PortHandler(port)
        self.packet = PacketHandler(2.0)
        self.closed = False
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open {port}")
        if not self.port.setBaudRate(baudrate):
            self.port.closePort()
            raise RuntimeError(f"Failed to set {port} to {baudrate} baud")
        try:
            models = self.read_vector(ADDR_MODEL_NUMBER, 2)
            unsupported = sorted(set(models) - SUPPORTED_MODEL_NUMBERS)
            if unsupported:
                raise RuntimeError(
                    "Refusing Current-based Position mode for unverified model "
                    f"number(s): {unsupported}"
                )
        except Exception:
            self.port.closePort()
            self.closed = True
            raise

    def _check(self, result: int, error: int, action: str, dxl_id: int) -> None:
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"{action} failed for Dynamixel ID {dxl_id}: "
                f"comm={result}, device_error={error}"
            )

    def read(self, dxl_id: int, address: int, length: int) -> int:
        reader = {
            1: self.packet.read1ByteTxRx,
            2: self.packet.read2ByteTxRx,
            4: self.packet.read4ByteTxRx,
        }[length]
        value, result, error = reader(self.port, dxl_id, address)
        self._check(result, error, f"read address {address}", dxl_id)
        return int(value)

    def read_vector(self, address: int, length: int) -> List[int]:
        return [self.read(dxl_id, address, length) for dxl_id in self.ids]

    def sync_write(self, address: int, length: int, values: Sequence[int]) -> None:
        if len(values) != len(self.ids):
            raise ValueError("sync-write value count does not match IDs")
        writer = self._GroupSyncWrite(self.port, self.packet, address, length)
        try:
            for dxl_id, value in zip(self.ids, values):
                unsigned = int(value) & ((1 << (8 * length)) - 1)
                payload = [
                    (unsigned >> (8 * index)) & 0xFF for index in range(length)
                ]
                if not writer.addParam(dxl_id, payload):
                    raise RuntimeError(
                        f"Could not queue address {address} for ID {dxl_id}"
                    )
            result = writer.txPacket()
            if result != COMM_SUCCESS:
                raise RuntimeError(
                    f"Sync-write address {address} failed: comm={result}"
                )
        finally:
            writer.clearParam()

    def set_torque_mask(self, mask: Sequence[int]) -> None:
        if len(mask) != len(self.ids):
            raise ValueError("torque mask count does not match IDs")
        for dxl_id, enabled in zip(self.ids, mask):
            result, error = self.packet.write1ByteTxRx(
                self.port, dxl_id, ADDR_TORQUE_ENABLE, int(enabled)
            )
            self._check(result, error, "set torque", dxl_id)

    def disable_all(self) -> None:
        self.set_torque_mask([0] * len(self.ids))

    def snapshot(self) -> Dict[str, List[int]]:
        return {
            "model_number": self.read_vector(ADDR_MODEL_NUMBER, 2),
            "operating_mode": self.read_vector(ADDR_OPERATING_MODE, 1),
            "current_limit_ma": self.read_vector(ADDR_CURRENT_LIMIT, 2),
            "torque_enable": self.read_vector(ADDR_TORQUE_ENABLE, 1),
            "kp_p": self.read_vector(ADDR_KP_P, 2),
            "kp_i": self.read_vector(ADDR_KP_I, 2),
            "kp_d": self.read_vector(ADDR_KP_D, 2),
            "goal_current_ma": self.read_vector(ADDR_GOAL_CURRENT, 2),
            "present_current_ma": self.read_vector(ADDR_PRESENT_CURRENT, 2),
            "present_position_raw": self.read_vector(ADDR_PRESENT_POSITION, 4),
        }

    def apply(
        self,
        profile: Dict[str, object],
        joint_offsets: Sequence[float],
        joint_signs: Sequence[int],
        max_goal_delta_rad: float,
    ) -> List[float]:
        snapshot = self.snapshot()
        profile = validate_profile(
            profile,
            joint_count=len(self.ids),
            max_current_ma=snapshot["current_limit_ma"],
        )
        current_raw = [_pulse_to_rad(value) for value in snapshot["present_position_raw"]]
        current_robot = [
            (raw - float(offset)) * int(sign)
            for raw, offset, sign in zip(current_raw, joint_offsets, joint_signs)
        ]
        safe_goal_robot = nearest_safe_goals(
            current_robot,
            profile["goal_position"],
            max_delta_rad=max_goal_delta_rad,
        )
        final_raw = [
            goal * int(sign) + float(offset)
            for goal, offset, sign in zip(
                safe_goal_robot, joint_offsets, joint_signs
            )
        ]
        try:
            # No joint is energized while mode/gains/current are being changed.
            self.disable_all()
            self.sync_write(
                ADDR_OPERATING_MODE,
                1,
                [OPERATING_MODE_CURRENT_POSITION] * len(self.ids),
            )
            self.sync_write(ADDR_KP_P, 2, profile["kp_p"])
            self.sync_write(ADDR_KP_I, 2, profile["kp_i"])
            self.sync_write(ADDR_KP_D, 2, profile["kp_d"])
            self.sync_write(ADDR_GOAL_CURRENT, 2, profile["goal_current_ma"])

            # Bootstrap at the measured pose. This removes the stale/edge-pose
            # error before torque is enabled. The requested nearby goal is sent
            # only after the torque mask has been applied.
            self.sync_write(
                ADDR_GOAL_POSITION,
                4,
                [_rad_to_pulse(value) for value in current_raw],
            )
            self.set_torque_mask(profile["torque_enable"])
            self.sync_write(
                ADDR_GOAL_POSITION,
                4,
                [_rad_to_pulse(value) for value in final_raw],
            )
        except Exception:
            try:
                self.disable_all()
            finally:
                raise
        return safe_goal_robot

    def close(self) -> None:
        if self.closed:
            return
        try:
            self.disable_all()
        finally:
            self.port.closePort()
            self.closed = True


def _default_profile(count: int) -> Dict[str, object]:
    return {
        "version": 1,
        "torque_enable": [0] * count,
        "goal_current_ma": [0] * count,
        "kp_p": [0] * count,
        "kp_i": [0] * count,
        "kp_d": [0] * count,
        "goal_position": [0.0] * count,
        "tuning_history": [],
    }


def _resolve_port(side: str, explicit_port: Optional[str]) -> Optional[str]:
    if explicit_port:
        return explicit_port
    candidates = sorted(Path("/dev/serial/by-id").glob("*"))
    hints = SIDE_PORT_HINTS[side]
    for hint in hints:
        for candidate in candidates:
            if hint in candidate.name:
                return str(candidate)
    return None


class TunerGui:
    def __init__(self, args: argparse.Namespace):
        import tkinter as tk
        from tkinter import messagebox, ttk

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.args = args
        self.ids = list(range(1, args.joints + 1))
        self.profile_path = str(Path(args.profile).expanduser())
        self.port = _resolve_port(args.side, args.port)
        self.bus = None  # type: Optional[DynamixelBus]
        self.offsets = [0.0] * args.joints
        self.signs = [1] * args.joints
        if self.port:
            from teleop.agents.teleop_agent import PORT_CONFIG_MAP

            config = PORT_CONFIG_MAP.get(self.port)
            if config is not None and len(config.joint_ids) == args.joints:
                self.ids = list(config.joint_ids)
                self.offsets = list(config.joint_offsets)
                self.signs = list(config.joint_signs)

        if Path(self.profile_path).exists():
            self.profile = load_profile(self.profile_path, joint_count=args.joints)
        else:
            self.profile = _default_profile(args.joints)

        self.root = tk.Tk()
        self.root.title("GELLO Dynamixel damping tuner")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.status = tk.StringVar(value="Disconnected; torque not controlled by tuner")
        self.variables = []
        self._build()
        self._profile_to_widgets(self.profile)

    def _build(self) -> None:
        tk, ttk = self.tk, self.ttk
        top = ttk.Frame(self.root, padding=10)
        top.grid(sticky="nsew")
        ttk.Label(
            top,
            text=(
                "Sliders edit a pending profile only. Apply is explicit. "
                "Stop teleop/run_env before connecting."
            ),
            foreground="#8a4b00",
        ).grid(row=0, column=0, columnspan=8, sticky="w", pady=(0, 8))
        ttk.Label(top, text=f"Side: {self.args.side}").grid(row=1, column=0, sticky="w")
        ttk.Label(top, text=f"Port: {self.port or 'not detected'}").grid(
            row=1, column=1, columnspan=4, sticky="w"
        )
        ttk.Button(top, text="Connect (torque OFF)", command=self.connect).grid(
            row=1, column=5, padx=4
        )
        ttk.Button(top, text="CAPTURE CURRENT GOALS", command=self.capture_goals).grid(
            row=1, column=6, padx=4
        )
        ttk.Button(top, text="TORQUE OFF", command=self.torque_off).grid(
            row=1, column=7, padx=4
        )

        headers = ("Joint", "Enable", "P", "I", "D", "Current mA", "Goal rad")
        for column, label in enumerate(headers):
            ttk.Label(top, text=label).grid(row=2, column=column, pady=(10, 2))
        for row_index in range(self.args.joints):
            row = row_index + 3
            enabled = tk.IntVar()
            p_value = tk.IntVar()
            i_value = tk.IntVar()
            d_value = tk.IntVar()
            current = tk.IntVar()
            goal = tk.StringVar()
            self.variables.append(
                {
                    "torque_enable": enabled,
                    "kp_p": p_value,
                    "kp_i": i_value,
                    "kp_d": d_value,
                    "goal_current_ma": current,
                    "goal_position": goal,
                }
            )
            ttk.Label(top, text=f"J{row_index + 1} / ID {self.ids[row_index]}").grid(
                row=row, column=0, sticky="w"
            )
            ttk.Checkbutton(top, variable=enabled).grid(row=row, column=1)
            for column, variable, maximum in (
                (2, p_value, self.args.max_gain),
                (3, i_value, self.args.max_gain),
                (4, d_value, self.args.max_gain),
                (5, current, self.args.max_current_ma),
            ):
                tk.Scale(
                    top,
                    variable=variable,
                    from_=0,
                    to=maximum,
                    orient="horizontal",
                    resolution=1,
                    length=175,
                ).grid(row=row, column=column, padx=2)
            ttk.Entry(top, textvariable=goal, width=10).grid(row=row, column=6)

        action_row = self.args.joints + 3
        ttk.Button(top, text="Save profile", command=self.save).grid(
            row=action_row, column=0, columnspan=2, sticky="ew", pady=8
        )
        ttk.Button(top, text="APPLY SAFELY", command=self.apply).grid(
            row=action_row, column=2, columnspan=3, sticky="ew", padx=4, pady=8
        )
        ttk.Label(top, textvariable=self.status).grid(
            row=action_row + 1, column=0, columnspan=8, sticky="w"
        )

        binary = ttk.LabelFrame(top, text="Guided binary search", padding=8)
        binary.grid(row=action_row + 2, column=0, columnspan=8, sticky="ew", pady=8)
        self.binary_joint = tk.IntVar(value=1)
        self.binary_field = tk.StringVar(value="goal_current_ma")
        self.binary_low = tk.IntVar(value=0)
        self.binary_high = tk.IntVar(value=self.args.max_current_ma)
        self.binary_mid = tk.StringVar(value="mid: -")
        ttk.Label(binary, text="Joint").grid(row=0, column=0)
        ttk.Spinbox(binary, from_=1, to=self.args.joints, textvariable=self.binary_joint, width=5).grid(row=0, column=1)
        ttk.Combobox(
            binary,
            textvariable=self.binary_field,
            values=("goal_current_ma", "kp_p", "kp_i", "kp_d"),
            state="readonly",
            width=18,
        ).grid(row=0, column=2, padx=4)
        ttk.Label(binary, text="Low").grid(row=0, column=3)
        ttk.Entry(binary, textvariable=self.binary_low, width=7).grid(row=0, column=4)
        ttk.Label(binary, text="High").grid(row=0, column=5)
        ttk.Entry(binary, textvariable=self.binary_high, width=7).grid(row=0, column=6)
        ttk.Button(binary, text="Set midpoint", command=self.binary_set_mid).grid(row=0, column=7, padx=4)
        ttk.Button(binary, text="Too weak", command=lambda: self.binary_feedback(True)).grid(row=0, column=8, padx=4)
        ttk.Button(binary, text="Too strong", command=lambda: self.binary_feedback(False)).grid(row=0, column=9, padx=4)
        ttk.Label(binary, textvariable=self.binary_mid).grid(row=0, column=10, padx=4)

    def _profile_to_widgets(self, profile: Dict[str, object]) -> None:
        for index, variables in enumerate(self.variables):
            for field, variable in variables.items():
                value = profile[field][index]
                variable.set(f"{value:.6f}" if field == "goal_position" else int(value))

    def _widgets_to_profile(self) -> Dict[str, object]:
        profile = dict(self.profile)
        for field in (
            "torque_enable",
            "goal_current_ma",
            "kp_p",
            "kp_i",
            "kp_d",
        ):
            profile[field] = [int(row[field].get()) for row in self.variables]
        try:
            profile["goal_position"] = [
                float(row["goal_position"].get()) for row in self.variables
            ]
        except ValueError as exc:
            raise ProfileError("goal_position entries must be numbers") from exc
        return validate_profile(profile, joint_count=self.args.joints)

    def connect(self) -> None:
        if self.bus is not None:
            self.status.set("Already connected")
            return
        if not self.port:
            self.messagebox.showerror("No port", "Specify --port or attach a known GELLO adapter.")
            return
        try:
            self.bus = DynamixelBus(self.port, self.ids, self.args.baudrate)
            self.bus.disable_all()
            snapshot = self.bus.snapshot()
            current_raw = [_pulse_to_rad(item) for item in snapshot["present_position_raw"]]
            current_robot = [
                (raw - offset) * sign
                for raw, offset, sign in zip(current_raw, self.offsets, self.signs)
            ]
            loaded_from_file = Path(self.profile_path).exists()
            if not loaded_from_file:
                profile = _default_profile(self.args.joints)
                for field in ("goal_current_ma", "kp_p", "kp_i", "kp_d"):
                    profile[field] = snapshot[field]
                profile["goal_position"] = current_robot
                self.profile = validate_profile(profile, self.args.joints)
                self._profile_to_widgets(self.profile)
            models = sorted(set(snapshot["model_number"]))
            self.status.set(
                f"Connected, models={models}; all tuner-managed torque is OFF"
            )
        except Exception as exc:
            if self.bus is not None:
                try:
                    self.bus.close()
                except Exception:
                    pass
                self.bus = None
            self.messagebox.showerror("Connect failed", str(exc))

    def capture_goals(self) -> None:
        if self.bus is None:
            self.messagebox.showerror("Not connected", "Connect before capturing goals.")
            return
        try:
            raw = self.bus.read_vector(ADDR_PRESENT_POSITION, 4)
            robot = [
                (_pulse_to_rad(value) - offset) * sign
                for value, offset, sign in zip(raw, self.offsets, self.signs)
            ]
            for variables, value in zip(self.variables, robot):
                variables["goal_position"].set(f"{value:.6f}")
            self.status.set("Captured current GELLO pose as the pending damping goal")
        except Exception as exc:
            self.messagebox.showerror("Capture failed", str(exc))

    def torque_off(self) -> None:
        if self.bus is None:
            self.status.set("Disconnected; no tuner-owned bus to disable")
            return
        try:
            self.bus.disable_all()
            for row in self.variables:
                row["torque_enable"].set(0)
            self.status.set("All GELLO arm-joint torque disabled")
        except Exception as exc:
            self.messagebox.showerror("Torque OFF failed", str(exc))

    def save(self) -> None:
        try:
            self.profile = self._widgets_to_profile()
            save_profile(self.profile_path, self.profile, self.args.joints)
            self.status.set(f"Saved {self.profile_path}")
        except Exception as exc:
            self.messagebox.showerror("Save failed", str(exc))

    def apply(self) -> None:
        if self.bus is None:
            self.messagebox.showerror("Not connected", "Connect before applying to hardware.")
            return
        try:
            profile = self._widgets_to_profile()
            if any(profile["torque_enable"]) and not self.messagebox.askyesno(
                "Enable GELLO torque?",
                "The selected joints will be energized. Keep hands ready on "
                "TORQUE OFF and confirm the captured goals are correct.",
            ):
                return
            safe_goal = self.bus.apply(
                profile,
                self.offsets,
                self.signs,
                self.args.max_goal_delta_rad,
            )
            profile["goal_position"] = safe_goal
            profile.setdefault("tuning_history", []).append(
                {
                    "event": "apply",
                    "torque_enable": list(profile["torque_enable"]),
                    "goal_current_ma": list(profile["goal_current_ma"]),
                    "kp_p": list(profile["kp_p"]),
                    "kp_i": list(profile["kp_i"]),
                    "kp_d": list(profile["kp_d"]),
                }
            )
            self.profile = profile
            self._profile_to_widgets(profile)
            save_profile(self.profile_path, profile, self.args.joints)
            self.status.set(
                "Applied once and saved; slider movement remains pending until next Apply"
            )
        except Exception as exc:
            try:
                self.bus.disable_all()
            except Exception:
                pass
            self.messagebox.showerror("Apply rejected; torque forced OFF", str(exc))

    def binary_set_mid(self) -> None:
        try:
            bounds = BinarySearchBounds(self.binary_low.get(), self.binary_high.get())
            value = bounds.midpoint
            joint_index = self.binary_joint.get() - 1
            field = self.binary_field.get()
            self.variables[joint_index][field].set(value)
            self.binary_mid.set(f"mid: {value} (pending)")
        except Exception as exc:
            self.messagebox.showerror("Binary search", str(exc))

    def binary_feedback(self, weak: bool) -> None:
        try:
            bounds = BinarySearchBounds(self.binary_low.get(), self.binary_high.get())
            tested = bounds.midpoint
            suggested = bounds.too_weak() if weak else bounds.too_strong()
            self.binary_low.set(bounds.low)
            self.binary_high.set(bounds.high)
            joint_index = self.binary_joint.get() - 1
            field = self.binary_field.get()
            self.variables[joint_index][field].set(suggested)
            result = "too_weak" if weak else "too_strong"
            self.profile.setdefault("tuning_history", []).append(
                {
                    "event": "binary_feedback",
                    "joint": joint_index + 1,
                    "field": field,
                    "tested": tested,
                    "result": result,
                    "next_low": bounds.low,
                    "next_high": bounds.high,
                    "next_mid": suggested,
                }
            )
            self.binary_mid.set(f"mid: {suggested} (pending)")
        except Exception as exc:
            self.messagebox.showerror("Binary search", str(exc))

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception as exc:
                self.messagebox.showwarning(
                    "Close warning", f"Could not confirm torque OFF: {exc}"
                )
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--joints", type=int, default=7)
    parser.add_argument("--profile")
    parser.add_argument("--max-current-ma", type=int, default=600)
    parser.add_argument("--max-gain", type=int, default=1000)
    parser.add_argument("--max-goal-delta-rad", type=float, default=0.20)
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate --profile and exit without opening a serial port or GUI.",
    )
    args = parser.parse_args()
    if args.profile is None:
        args.profile = str(
            REPO_ROOT / "config" / f"dynamixel_damping_{args.side}.json"
        )
    return args


def main() -> int:
    args = parse_args()
    if args.validate_only:
        profile = load_profile(args.profile, joint_count=args.joints)
        print(json.dumps(profile, indent=2, sort_keys=True))
        print("Profile valid; hardware was not opened.")
        return 0
    TunerGui(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
