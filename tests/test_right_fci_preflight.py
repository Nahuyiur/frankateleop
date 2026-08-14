from __future__ import annotations

import os
from pathlib import Path
import shlex
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "15_record_bi_arm_pipeline.sh"


def _run_sourced(body: str, tmp_path: Path) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["HOME"] = str(tmp_path)
    return subprocess.run(
        ["bash", "-c", f"source {shlex.quote(str(SCRIPT))}\n{body}"],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def test_fci_preflight_accepts_ready_port(tmp_path: Path) -> None:
    result = _run_sourced(
        """
remote_bash() { echo FCI_READY; return 0; }
check_remote_right_fci_ready
""",
        tmp_path,
    )
    assert result.returncode == 0, result.stdout
    assert "Right robot FCI is ready" in result.stdout


def test_fci_preflight_reports_inactive_desk(tmp_path: Path) -> None:
    result = _run_sourced(
        """
abort() { echo "ABORT: $*"; return 79; }
remote_bash() { echo FCI_INACTIVE; return 42; }
check_remote_right_fci_ready
""",
        tmp_path,
    )
    assert result.returncode == 79, result.stdout
    assert "FCI is not active" in result.stdout
    assert "Activate FCI" in result.stdout


def test_fci_preflight_reports_unreachable_robot(tmp_path: Path) -> None:
    result = _run_sourced(
        """
abort() { echo "ABORT: $*"; return 79; }
remote_bash() { echo ROBOT_UNREACHABLE; return 43; }
check_remote_right_fci_ready
""",
        tmp_path,
    )
    assert result.returncode == 79, result.stdout
    assert "control cabinet is not reachable" in result.stdout


def test_fci_preflight_can_be_explicitly_disabled(tmp_path: Path) -> None:
    result = _run_sourced(
        """
REQUIRE_RIGHT_FCI_READY=0
remote_bash() { echo unexpected-call; return 90; }
check_remote_right_fci_ready
""",
        tmp_path,
    )
    assert result.returncode == 0, result.stdout
    assert "preflight is disabled" in result.stdout
    assert "unexpected-call" not in result.stdout


def test_fci_preflight_runs_before_left_robot_start() -> None:
    script = SCRIPT.read_text(encoding="utf-8")
    main = script[script.index("main() {") :]
    assert main.index("check_remote_right_fci_ready") < main.index(
        'start_local_script "left 1_launch_robot"'
    )


def test_early_failure_cleans_the_sudo_keepalive_process_tree() -> None:
    script = SCRIPT.read_text(encoding="utf-8")
    assert 'terminate_pid_tree "$SUDO_KEEPALIVE_PID" "sudo keepalive"' in script
