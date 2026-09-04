from __future__ import annotations

import os
from pathlib import Path
import shlex
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]


def _run_sourced(script_name: str, body: str, tmp_path: Path) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["HOME"] = str(tmp_path)
    return subprocess.run(
        ["bash", "-c", f"source {shlex.quote(str(REPO_ROOT / script_name))}\n{body}"],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


@pytest.mark.parametrize(
    "script_name", ("15_record_bi_arm_pipeline.sh", "16_replay_bi_arm_pipeline.sh")
)
def test_remote_bash_retries_transport_failures(script_name: str, tmp_path: Path) -> None:
    result = _run_sourced(
        script_name,
        r"""
SSH_COMMAND_RETRIES=4
SSH_RETRY_DELAY=0
calls=0
ssh_cmd() {
    calls=$((calls + 1))
    ((calls < 3)) && return 255
    return 0
}
remote_bash true
[[ "$calls" == 3 ]]
""",
        tmp_path,
    )
    assert result.returncode == 0, result.stdout


@pytest.mark.parametrize(
    "script_name", ("15_record_bi_arm_pipeline.sh", "16_replay_bi_arm_pipeline.sh")
)
def test_remote_bash_does_not_retry_remote_command_errors(
    script_name: str, tmp_path: Path
) -> None:
    result = _run_sourced(
        script_name,
        r"""
SSH_COMMAND_RETRIES=4
SSH_RETRY_DELAY=0
calls=0
ssh_cmd() {
    calls=$((calls + 1))
    return 23
}
if remote_bash false; then
    exit 90
else
    rc=$?
fi
[[ "$rc" == 23 && "$calls" == 1 ]]
""",
        tmp_path,
    )
    assert result.returncode == 0, result.stdout


@pytest.mark.parametrize(
    "script_name", ("15_record_bi_arm_pipeline.sh", "16_replay_bi_arm_pipeline.sh")
)
def test_remote_start_is_idempotent_for_ssh_retries(script_name: str) -> None:
    text = (REPO_ROOT / script_name).read_text(encoding="utf-8")
    assert 'existing_pid="\\$(cat "\\$pid_file"' in text
    assert "already running as process group" in text
    assert 'mv -f "\\${pid_file}.tmp.\\$\\$" "\\$pid_file"' in text
