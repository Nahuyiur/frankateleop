"""Process management for the GUI pipeline controls."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Optional

from PyQt6 import QtCore


@dataclass
class ManagedProcess:
    label: str
    script: str
    process: subprocess.Popen
    log_path: Path


class ProcessManager(QtCore.QObject):
    status_changed = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    log_line = QtCore.pyqtSignal(str)
    stack_state_changed = QtCore.pyqtSignal(str)

    def __init__(
        self,
        repo_root: Path,
        mode: str = "single",
        right_robot_host: str | None = None,
        right_robot_port: int | None = None,
        parent: QtCore.QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.repo_root = repo_root
        if mode not in {"single", "right", "dual"}:
            raise ValueError(f"Unsupported process manager mode: {mode}")
        self.mode = mode
        self.right_robot_host = right_robot_host
        self.right_robot_port = right_robot_port
        log_roots = {
            "single": "fr3_gui",
            "right": "fr3_gui_right",
            "dual": "fr3_gui_dual",
        }
        self.log_root = repo_root / "logs" / log_roots[mode]
        self.ready_timeout = int(os.environ.get("GUI_READY_TIMEOUT", "90"))
        self._processes: Dict[str, ManagedProcess] = {}
        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._run_log_dir: Path | None = None

    @property
    def run_log_dir(self) -> Path | None:
        return self._run_log_dir

    def start_stack(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            self.error.emit("启动流程正在运行中")
            return
        self._worker = threading.Thread(target=self._start_stack_worker, daemon=True)
        self._worker.start()

    def stop_stack(self) -> None:
        threading.Thread(target=self._stop_stack_worker, args=(True,), daemon=True).start()

    def stop_stack_blocking(self) -> None:
        self._stop_stack_worker(False)

    def stop_teleop_env(self) -> None:
        if self.mode in {"right", "dual"}:
            label = "右臂" if self.mode == "right" else "双臂"
            self.status_changed.emit(f"{label} GUI 模式下请使用“停止全部”清理遥操作栈。")
            return
        threading.Thread(target=self._stop_one_worker, args=("4_run_env",), daemon=True).start()

    def tail_logs(self, lines: int = 80) -> str:
        chunks = []
        with self._lock:
            processes = list(self._processes.values())
        for proc in processes:
            chunks.append(f"===== {proc.label}: {proc.log_path} =====")
            chunks.append(_tail_file(proc.log_path, lines))
        return "\n".join(chunks).strip()

    def _start_stack_worker(self) -> None:
        try:
            if self.mode == "dual":
                self._start_dual_stack_worker()
                return
            if self.mode == "right":
                self._start_right_stack_worker()
                return
            self.stack_state_changed.emit("starting")
            self._run_log_dir = self.log_root / time.strftime("%Y%m%d_%H%M%S")
            self._run_log_dir.mkdir(parents=True, exist_ok=True)
            self._emit_log(f"日志目录: {self._run_log_dir}")
            self._prepare_sudo()
            self._cleanup_stale_pipeline_processes()
            self._start_script("1_launch_robot", "1_launch_robot.sh", self._check_robot_grpc_ready)
            self._wait_for_stable_ready(
                "1_launch_robot",
                self._check_robot_grpc_ready,
                seconds=4,
            )
            self._start_script("2_launch_gripper", "2_launch_gripper.sh", self._check_gripper_grpc_ready)
            self._require_managed_alive("1_launch_robot")
            if not self._check_robot_grpc_ready(Path()):
                raise RuntimeError(
                    "1_launch_robot 曾经 ready，但启动夹爪后 robot gRPC 断开。"
                    "请检查 1_launch_robot.log；常见原因是机器人仍处于 Idle 模式或上一次控制未完全释放。"
                )
            self._start_script("3_launch_node", "3_launch_node.sh", self._check_robot_zmq_ready)
            self._start_script("4_run_env", "4_run_env.sh", self._check_env_loop_ready)
            self.stack_state_changed.emit("running")
            self.status_changed.emit("机器人栈已启动: 1-4 ready")
        except Exception as exc:
            self.error.emit(str(exc))
            self._stop_stack_worker(False)
            self.stack_state_changed.emit("error")

    def _start_dual_stack_worker(self) -> None:
        try:
            self.stack_state_changed.emit("starting")
            self._run_log_dir = self.log_root / time.strftime("%Y%m%d_%H%M%S")
            self._run_log_dir.mkdir(parents=True, exist_ok=True)
            self._emit_log(f"日志目录: {self._run_log_dir}")
            self._prepare_sudo()
            self._cleanup_stale_pipeline_processes()
            self._start_dual_stack_script()
            self.stack_state_changed.emit("running")
            self.status_changed.emit("双臂机器人栈已启动: left/right 1-4 ready")
        except Exception as exc:
            self.error.emit(str(exc))
            self._stop_stack_worker(False)
            self.stack_state_changed.emit("error")

    def _start_right_stack_worker(self) -> None:
        try:
            self.stack_state_changed.emit("starting")
            self._run_log_dir = self.log_root / time.strftime("%Y%m%d_%H%M%S")
            self._run_log_dir.mkdir(parents=True, exist_ok=True)
            self._emit_log(f"日志目录: {self._run_log_dir}")
            self._prepare_sudo()
            self._cleanup_stale_pipeline_processes()
            self._start_right_stack_script()
            self.stack_state_changed.emit("running")
            self.status_changed.emit("右臂机器人栈已启动: right 1-4 ready")
        except Exception as exc:
            self.error.emit(str(exc))
            self._stop_stack_worker(False)
            self.stack_state_changed.emit("error")

    def _stop_stack_worker(self, emit_done: bool) -> None:
        if self.mode == "dual":
            self._stop_one("15_bi_arm_stack")
            if emit_done:
                self.stack_state_changed.emit("stopped")
                self.status_changed.emit("双臂机器人栈已停止")
            return
        if self.mode == "right":
            self._stop_one("15_right_arm_stack")
            if emit_done:
                self.stack_state_changed.emit("stopped")
                self.status_changed.emit("右臂机器人栈已停止")
            return
        for label in ("4_run_env", "3_launch_node", "2_launch_gripper", "1_launch_robot"):
            self._stop_one(label)
        if emit_done:
            self.stack_state_changed.emit("stopped")
            self.status_changed.emit("机器人栈已停止")

    def _stop_one_worker(self, label: str) -> None:
        self._stop_one(label)
        self.status_changed.emit(f"{label} 已停止")

    def _start_script(
        self,
        label: str,
        script_name: str,
        ready_check: Callable[[Path], bool],
    ) -> None:
        assert self._run_log_dir is not None
        log_path = self._run_log_dir / f"{label}.log"
        script_path = self.repo_root / script_name
        self._emit_log(f"启动 {script_name} ...")
        log = log_path.open("ab")
        try:
            proc = subprocess.Popen(
                ["bash", str(script_path)],
                cwd=self.repo_root,
                stdout=log,
                stderr=subprocess.STDOUT,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
            )
        finally:
            log.close()
        managed = ManagedProcess(label, script_name, proc, log_path)
        with self._lock:
            self._processes[label] = managed
        self._wait_until_ready(label, managed, ready_check)
        self._emit_log(f"{label} ready")

    def _start_dual_stack_script(self) -> None:
        assert self._run_log_dir is not None
        label = "15_bi_arm_stack"
        log_path = self._run_log_dir / f"{label}.log"
        script_path = self.repo_root / "15_record_bi_arm_pipeline.sh"
        if not script_path.exists():
            raise FileNotFoundError(f"未找到双臂启动脚本: {script_path}")
        self._emit_log("启动 15_record_bi_arm_pipeline.sh stack-only ...")
        log = log_path.open("ab")
        env = {
            **os.environ,
            "PYTHONUNBUFFERED": "1",
            "BI_ARM_STACK_ONLY": "1",
            "BI_ARM_RIGHT_ONLY": "0",
        }
        gui_password = _read_sudo_password()
        if gui_password:
            for key in (
                "BI_ARM_LOCAL_SUDO_PASSWORD",
                "BI_ARM_REMOTE_SUDO_PASSWORD",
                "BI_ARM_SSH_PASSWORD",
            ):
                if not env.get(key):
                    env[key] = gui_password
            self._emit_log("双臂启动已使用 GUI 私有密码配置传递 sudo/SSH 凭据")
        try:
            proc = subprocess.Popen(
                ["bash", str(script_path), "gui_stack"],
                cwd=self.repo_root,
                stdout=log,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
        finally:
            log.close()
        managed = ManagedProcess(label, script_path.name, proc, log_path)
        with self._lock:
            self._processes[label] = managed
        self._wait_until_ready(label, managed, self._check_bi_arm_stack_ready)
        self._emit_log(f"{label} ready")

    def _start_right_stack_script(self) -> None:
        assert self._run_log_dir is not None
        label = "15_right_arm_stack"
        log_path = self._run_log_dir / f"{label}.log"
        script_path = self.repo_root / "15_record_bi_arm_pipeline.sh"
        if not script_path.exists():
            raise FileNotFoundError(f"未找到右臂启动脚本: {script_path}")
        self._emit_log("启动 15_record_bi_arm_pipeline.sh right-only stack-only ...")
        log = log_path.open("ab")
        env = {
            **os.environ,
            "PYTHONUNBUFFERED": "1",
            "BI_ARM_STACK_ONLY": "1",
            "BI_ARM_RIGHT_ONLY": "1",
        }
        right_host = self.right_robot_host or env.get("FRANKA_RIGHT_ZMQ_HOST", "192.168.1.131")
        right_port = str(self.right_robot_port or env.get("FRANKA_RIGHT_ZMQ_PORT", "6001"))
        camera_only = env.get("BI_ARM_CAMERA_ONLY", "0") == "1"
        env.update(
            {
                "FRANKA_RIGHT_ZMQ_HOST": right_host,
                "FRANKA_RIGHT_ZMQ_PORT": right_port,
                "BI_ARM_RIGHT_RECORD_ZMQ_HOST": right_host,
                "BI_ARM_RIGHT_RECORD_ZMQ_PORT": right_port,
                "BI_ARM_RIGHT_TELEOP_ZMQ_HOST": right_host,
                "BI_ARM_RIGHT_TELEOP_ZMQ_PORT": right_port,
                "BI_ARM_CAMERA_ONLY": "1" if camera_only else "0",
                "BI_ARM_START_RUN_ENV": "0" if camera_only else "1",
            }
        )
        self._emit_log(
            "右臂 GUI stack endpoint: "
            f"{right_host}:{right_port}; teleop={'off' if camera_only else 'on'}"
        )
        gui_password = _read_sudo_password()
        if gui_password:
            for key in (
                "BI_ARM_LOCAL_SUDO_PASSWORD",
                "BI_ARM_REMOTE_SUDO_PASSWORD",
                "BI_ARM_SSH_PASSWORD",
            ):
                if not env.get(key):
                    env[key] = gui_password
            self._emit_log("右臂启动已使用 GUI 私有密码配置传递 sudo/SSH 凭据")
        try:
            proc = subprocess.Popen(
                ["bash", str(script_path), "gui_right_stack"],
                cwd=self.repo_root,
                stdout=log,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
        finally:
            log.close()
        managed = ManagedProcess(label, script_path.name, proc, log_path)
        with self._lock:
            self._processes[label] = managed
        self._wait_until_ready(label, managed, self._check_bi_arm_stack_ready)
        self._emit_log(f"{label} ready")

    def _wait_for_stable_ready(
        self,
        label: str,
        ready_check: Callable[[Path], bool],
        seconds: int,
    ) -> None:
        self._emit_log(f"稳定性检查 {label} ({seconds}s) ...")
        for _ in range(max(1, seconds)):
            self._require_managed_alive(label)
            if not ready_check(Path()):
                managed = self._get_managed(label)
                log_path = managed.log_path if managed else Path()
                raise RuntimeError(
                    f"{label} ready 后又变为不可用，日志: {log_path}\n"
                    f"{_tail_file(log_path, 100)}"
                )
            time.sleep(1.0)

    def _require_managed_alive(self, label: str) -> None:
        managed = self._get_managed(label)
        if managed is None:
            raise RuntimeError(f"{label} 未被 GUI 管理，无法继续启动。")
        if managed.process.poll() is not None:
            raise RuntimeError(
                f"{label} 已退出，日志: {managed.log_path}\n"
                f"{_tail_file(managed.log_path, 100)}"
            )

    def _get_managed(self, label: str) -> Optional[ManagedProcess]:
        with self._lock:
            return self._processes.get(label)

    def _wait_until_ready(
        self,
        label: str,
        managed: ManagedProcess,
        ready_check: Callable[[Path], bool],
    ) -> None:
        started = time.time()
        while True:
            if managed.process.poll() is not None:
                raise RuntimeError(
                    f"{label} 在 ready 前退出，日志: {managed.log_path}\n{_tail_file(managed.log_path, 80)}"
                )
            if ready_check(managed.log_path):
                return
            if time.time() - started > self.ready_timeout:
                raise TimeoutError(
                    f"{label} 在 {self.ready_timeout}s 内没有 ready，日志: {managed.log_path}\n"
                    f"{_tail_file(managed.log_path, 80)}"
                )
            time.sleep(1.0)

    def _stop_one(self, label: str) -> None:
        with self._lock:
            managed = self._processes.pop(label, None)
        if managed is None:
            return
        proc = managed.process
        if proc.poll() is not None:
            return
        self._emit_log(f"停止 {label} ...")
        pids = _collect_pid_tree(proc.pid)
        if not pids:
            return
        _kill_pids(pids, signal.SIGTERM)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            _kill_pids(_collect_pid_tree(proc.pid), signal.SIGKILL)

    def _prepare_sudo(self) -> None:
        if not _command_exists("sudo"):
            return
        self._emit_log("检查 sudo 权限 ...")
        if (
            subprocess.run(
                ["sudo", "-n", "true"],
                cwd=self.repo_root,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ).returncode
            == 0
        ):
            return

        password = _read_sudo_password()
        if password is not None:
            result = subprocess.run(
                ["sudo", "-S", "-p", "", "-v"],
                cwd=self.repo_root,
                input=f"{password}\n",
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            if result.returncode == 0:
                self._emit_log("sudo 权限已通过本机私有密码文件缓存")
                return
            raise RuntimeError(
                "sudo 密码验证失败。请检查 FRANKA_GUI_SUDO_PASSWORD_FILE "
                "或 ~/.franka_gui_sudo_password。"
            )

        self._emit_log("需要 sudo 密码；请回到启动 GUI 的终端输入密码。")
        result = subprocess.run(["sudo", "-v"], cwd=self.repo_root)
        if result.returncode != 0:
            raise RuntimeError("sudo -v 失败。请在启动 GUI 的终端中确认 sudo 密码。")

    def _check_robot_grpc_ready(self, _log_path: Path) -> bool:
        code = """
from polymetis import RobotInterface
robot = RobotInterface(ip_address="127.0.0.1", port=50051, enforce_version=False)
robot.get_joint_positions()
"""
        return _conda_python("polymetis", code, timeout=10).returncode == 0

    def _check_gripper_grpc_ready(self, _log_path: Path) -> bool:
        code = """
import grpc
import polymetis_pb2
import polymetis_pb2_grpc
channel = grpc.insecure_channel("127.0.0.1:50052")
grpc.channel_ready_future(channel).result(timeout=2)
stub = polymetis_pb2_grpc.GripperServerStub(channel)
stub.GetRobotClientMetadata(polymetis_pb2.Empty(), timeout=2)
"""
        return _conda_python("polymetis", code, timeout=10).returncode == 0

    def _check_robot_zmq_ready(self, _log_path: Path) -> bool:
        code = """
import pickle
import zmq
ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.setsockopt(zmq.LINGER, 0)
sock.connect("tcp://127.0.0.1:6001")
for method in ("num_dofs", "get_observations"):
    sock.send(pickle.dumps({"method": method, "args": {}}))
    result = pickle.loads(sock.recv())
    if isinstance(result, dict) and "error" in result:
        raise RuntimeError(result["error"])
    if method == "num_dofs" and int(result) <= 0:
        raise RuntimeError(f"bad num_dofs: {result}")
    if method == "get_observations":
        if "ee_pose_euler" not in result:
            raise RuntimeError("robot node is missing ee_pose_euler")
        missing_gripper = [
            key
            for key in ("gripper_closedness", "gripper_01closedness", "gripper_target_width", "gripper_width")
            if key not in result
        ]
        if missing_gripper:
            raise RuntimeError("robot node is missing continuous gripper observation fields")
sock.close(0)
ctx.term()
"""
        return _conda_python("polymetis", code, timeout=10).returncode == 0

    def _check_env_loop_ready(self, log_path: Path) -> bool:
        if not log_path.exists():
            return False
        text = _tail_file(log_path, 200)
        return "Time passed:" in text

    def _check_bi_arm_stack_ready(self, log_path: Path) -> bool:
        if not log_path.exists():
            return False
        text = _tail_file(log_path, 240)
        return "BI_ARM_STACK_READY_FOR_GUI" in text

    def _emit_log(self, line: str) -> None:
        self.log_line.emit(line)
        self.status_changed.emit(line)

    def _cleanup_stale_pipeline_processes(self) -> None:
        self._emit_log("清理旧的 1-7 / Polymetis / teleop 残留进程 ...")
        patterns = [
            ("script 1", r"(^|[ /])1_launch_robot\.sh($| )"),
            ("script 2", r"(^|[ /])2_launch_gripper\.sh($| )"),
            ("script 3", r"(^|[ /])3_launch_node\.sh($| )"),
            ("script 4", r"(^|[ /])4_run_env\.sh($| )"),
            ("script 5", r"(^|[ /])5_capture_image\.sh($| )"),
            ("script 6", r"(^|[ /])6_record_fr3\.sh($| )"),
            ("script 7", r"(^|[ /])7_replay_fr3\.sh($| )"),
            ("script 15", r"(^|[ /])15_record_bi_arm_pipeline\.sh($| )"),
            ("Polymetis robot launcher", r"scripts/launch_robot\.py"),
            ("Polymetis gripper launcher", r"scripts/launch_gripper\.py"),
            ("Teleop robot node", r"experiments/launch_nodes\.py"),
            ("Teleop env", r"experiments/run_env\.py"),
            ("Capture image module", r"franka_capture\.scripts\.capture_image"),
            ("Record FR3 module", r"franka_capture\.scripts\.record_fr3"),
            ("Record dual FR3 module", r"franka_capture\.scripts\.record_fr3_dual"),
            ("Replay FR3 module", r"franka_replay\.replay_fr3"),
            ("run_server", r"(^|[ /])run_server($| )"),
            ("franka_hand_client", r"(^|[ /])franka_hand_client($| )"),
        ]
        current_pid = os.getpid()
        for label, pattern in patterns:
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
            )
            pids = []
            for line in result.stdout.splitlines():
                try:
                    pid = int(line.strip())
                except ValueError:
                    continue
                if pid in {current_pid, os.getppid()}:
                    continue
                pids.extend(_collect_pid_tree(pid))
            unique_pids = sorted(set(pids), reverse=True)
            if unique_pids:
                self._emit_log(f"清理旧 {label}: {unique_pids}")
                _kill_pids(unique_pids, signal.SIGTERM)
                time.sleep(0.2)
                _kill_pids(unique_pids, signal.SIGKILL)
        self._cleanup_stale_local_ports()

    def _cleanup_stale_local_ports(self) -> None:
        ports = [
            ("robot server 50051", int(os.environ.get("BI_ARM_RIGHT_ROBOT_PORT", "50051"))),
            ("left robot/single gripper 50052", int(os.environ.get("BI_ARM_LEFT_ROBOT_PORT", "50052"))),
            ("right gripper 50053", int(os.environ.get("BI_ARM_RIGHT_GRIPPER_PORT", "50053"))),
            ("left gripper 50054", int(os.environ.get("BI_ARM_LEFT_GRIPPER_PORT", "50054"))),
            ("single/right ZMQ 6001", int(os.environ.get("BI_ARM_RIGHT_REMOTE_ZMQ_PORT", "6001"))),
            ("left ZMQ 6002", int(os.environ.get("BI_ARM_LEFT_ZMQ_PORT", "6002"))),
            ("right ZMQ tunnel 16001", int(os.environ.get("BI_ARM_RIGHT_LOCAL_ZMQ_PORT", "16001"))),
            ("right gripper tunnel 15053", int(os.environ.get("BI_ARM_RIGHT_LOCAL_GRIPPER_PORT", "15053"))),
        ]
        current_pids = {os.getpid(), os.getppid()}
        seen_ports: set[int] = set()
        for label, port in ports:
            if port in seen_ports:
                continue
            seen_ports.add(port)
            pids = []
            for pid in _tcp_port_pids(port):
                if pid in current_pids:
                    continue
                pids.extend(_collect_pid_tree(pid))
            unique_pids = sorted(set(pids), reverse=True)
            if unique_pids:
                self._emit_log(f"清理旧本地端口 {label}: {unique_pids}")
                _kill_pids(unique_pids, signal.SIGTERM)
                time.sleep(0.2)
                _kill_pids(unique_pids, signal.SIGKILL)


def _conda_python(env_name: str, code: str, timeout: int) -> subprocess.CompletedProcess:
    script = (
        'set -e\n'
        'source "$(conda info --base)/etc/profile.d/conda.sh"\n'
        f'conda activate "{env_name}"\n'
        'python - <<\'PY\'\n'
        f'{code.strip()}\n'
        'PY\n'
    )
    return subprocess.run(
        ["bash", "-lc", script],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=timeout,
        check=False,
    )


def _tail_file(path: Path, lines: int) -> str:
    if not path.exists():
        return ""
    try:
        data = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception as exc:
        return f"<failed to read {path}: {exc}>"
    return "\n".join(data[-lines:])


def _collect_pid_tree(root_pid: int) -> list[int]:
    pids: list[int] = []

    def visit(pid: int) -> None:
        result = subprocess.run(
            ["pgrep", "-P", str(pid)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
        for line in result.stdout.splitlines():
            try:
                child_pid = int(line.strip())
            except ValueError:
                continue
            visit(child_pid)
        pids.append(pid)

    visit(root_pid)
    return pids


def _kill_pids(pids: list[int], sig: signal.Signals) -> None:
    if not pids:
        return
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            continue
        except PermissionError:
            _sudo_kill_pid(pid, sig)


def _sudo_kill_pid(pid: int, sig: signal.Signals) -> None:
    if not _command_exists("sudo"):
        return
    result = subprocess.run(
        ["sudo", "-n", "kill", f"-{int(sig)}", str(pid)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode == 0:
        return
    password = _read_sudo_password()
    if not password:
        return
    subprocess.run(
        ["sudo", "-S", "-p", "", "kill", f"-{int(sig)}", str(pid)],
        input=f"{password}\n",
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def _tcp_port_pids(port: int) -> list[int]:
    pids: list[int] = []
    if _command_exists("lsof"):
        for args in (
            ["lsof", "-t", f"-iTCP:{port}", "-sTCP:LISTEN"],
            ["lsof", "-t", f"-i:{port}"],
        ):
            result = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
            )
            pids.extend(_parse_pid_tokens(result.stdout))
    if _command_exists("fuser"):
        result = subprocess.run(
            ["fuser", "-n", "tcp", str(port)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
        pids.extend(_parse_pid_tokens(result.stdout))
    return sorted(set(pids))


def _parse_pid_tokens(text: str) -> list[int]:
    pids: list[int] = []
    for token in text.replace("\n", " ").split():
        try:
            pids.append(int(token))
        except ValueError:
            continue
    return pids


def _read_sudo_password() -> Optional[str]:
    password = os.environ.get("FRANKA_GUI_SUDO_PASSWORD")
    if password:
        return password

    password_file = os.environ.get("FRANKA_GUI_SUDO_PASSWORD_FILE")
    if not password_file:
        return None

    path = Path(password_file).expanduser()
    if not path.exists():
        return None
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _command_exists(name: str) -> bool:
    return subprocess.run(["bash", "-lc", f"command -v {name}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
