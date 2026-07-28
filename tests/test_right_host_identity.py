from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
RIGHT_ENV_KEYS = (
    "BI_ARM_RIGHT_HOST",
    "BI_ARM_RIGHT_SSH",
    "FRANKA_RIGHT_ZMQ_HOST",
)


def _help_output(script_name: str, *, trace: bool = False, **overrides: str) -> str:
    env = os.environ.copy()
    for key in RIGHT_ENV_KEYS:
        env.pop(key, None)
    env.update(overrides)
    command = ["bash"]
    if trace:
        command.append("-x")
    command.extend((str(REPO_ROOT / script_name), "--help"))
    result = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=True,
    )
    return result.stdout


@pytest.mark.parametrize(
    "script_name",
    (
        "B_run_right_arm_capture_gui.sh",
        "C_run_dual_arm_capture_gui.sh",
        "15_record_bi_arm_pipeline.sh",
        "16_replay_bi_arm_pipeline.sh",
    ),
)
def test_right_ssh_defaults_to_explicit_pnp_identity(script_name: str) -> None:
    output = _help_output(script_name, trace=True)
    assert (
        "+ BI_ARM_RIGHT_SSH=pnp@192.168.1.131" in output
        or "+ RIGHT_SSH=pnp@192.168.1.131" in output
    )


@pytest.mark.parametrize(
    "script_name",
    ("B_run_right_arm_capture_gui.sh", "C_run_dual_arm_capture_gui.sh"),
)
def test_gui_keeps_zmq_host_separate_from_ssh_identity(script_name: str) -> None:
    output = _help_output(script_name, BI_ARM_RIGHT_HOST="10.20.30.40")
    assert "右机=pnp@10.20.30.40" in output
    assert "右臂直连 ZMQ=10.20.30.40:6001" in output
    assert "右臂直连 ZMQ=pnp@" not in output


@pytest.mark.parametrize(
    "script_name",
    (
        "B_run_right_arm_capture_gui.sh",
        "C_run_dual_arm_capture_gui.sh",
        "15_record_bi_arm_pipeline.sh",
        "16_replay_bi_arm_pipeline.sh",
    ),
)
def test_legacy_bare_ssh_host_is_normalized_to_pnp(script_name: str) -> None:
    output = _help_output(
        script_name,
        trace=True,
        BI_ARM_RIGHT_SSH="192.168.1.131",
    )
    assert (
        "+ BI_ARM_RIGHT_SSH=pnp@192.168.1.131" in output
        or "+ RIGHT_SSH=pnp@192.168.1.131" in output
    )
