from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_SCRIPT = REPO_ROOT / "scripts" / "franka_runtime_config.sh"


def _load_config(*, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    command = f"""
source {CONFIG_SCRIPT!s}
franka_runtime_export_xpra_defaults
printf '%s\n' \\
  "$FRANKA_LEFT_SSH" \\
  "$FRANKA_LEFT_REPO" \\
  "$BI_ARM_RIGHT_SSH" \\
  "$BI_ARM_RIGHT_REPO" \\
  "$BI_ARM_LEFT_ZMQ_PORT" \\
  "$BI_ARM_RIGHT_REMOTE_ZMQ_PORT" \\
  "$FRANKA_XPRA_HOST" \\
  "$FRANKA_XPRA_REPO" \\
  "$FRANKA_XPRA_SSH_SOCKET"
"""
    clean_env = os.environ.copy()
    for key in (
        "FRANKA_RUNTIME_CONFIG_FILE",
        "FRANKA_LEFT_HOST",
        "FRANKA_LEFT_SSH",
        "FRANKA_LEFT_REPO",
        "BI_ARM_RIGHT_HOST",
        "BI_ARM_RIGHT_SSH",
        "BI_ARM_RIGHT_REPO",
        "BI_ARM_LEFT_ZMQ_PORT",
        "BI_ARM_RIGHT_REMOTE_ZMQ_PORT",
        "FRANKA_XPRA_HOST",
        "FRANKA_XPRA_REPO",
        "FRANKA_XPRA_SSH_SOCKET",
    ):
        clean_env.pop(key, None)
    clean_env.update(env or {})
    return subprocess.run(
        ["bash", "-euo", "pipefail", "-c", command],
        cwd=REPO_ROOT,
        env=clean_env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def test_runtime_defaults_preserve_production_topology(tmp_path: Path) -> None:
    result = _load_config(
        env={
            "HOME": str(tmp_path),
            "FRANKA_RUNTIME_CONFIG_FILE": str(tmp_path / "missing.env"),
        }
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        "muka@192.168.1.170",
        "/home/muka/frankateleop",
        "pnp@192.168.1.131",
        "/home/pnp/frankateleop",
        "6002",
        "6001",
        "muka@192.168.1.170",
        "/home/muka/frankateleop",
        "/tmp/codex-franka-170.sock",
    ]


def test_runtime_file_is_lower_priority_than_explicit_environment(tmp_path: Path) -> None:
    config_file = tmp_path / "runtime.env"
    config_file.write_text(
        "BI_ARM_RIGHT_HOST=10.0.0.31\n"
        "BI_ARM_RIGHT_SSH=robot@10.0.0.31\n"
        "BI_ARM_LEFT_ZMQ_PORT=7002\n",
        encoding="utf-8",
    )
    result = _load_config(
        env={
            "HOME": str(tmp_path),
            "FRANKA_RUNTIME_CONFIG_FILE": str(config_file),
            "BI_ARM_RIGHT_SSH": "override@10.0.0.32",
        }
    )
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    assert lines[2] == "override@10.0.0.32"
    assert lines[4:6] == ["7002", "6001"]


def test_runtime_file_rejects_shell_syntax(tmp_path: Path) -> None:
    marker = tmp_path / "must-not-exist"
    config_file = tmp_path / "runtime.env"
    config_file.write_text(f"touch {marker}\n", encoding="utf-8")
    result = _load_config(
        env={
            "HOME": str(tmp_path),
            "FRANKA_RUNTIME_CONFIG_FILE": str(config_file),
        }
    )
    assert result.returncode != 0
    assert "expected KEY=VALUE" in result.stderr
    assert not marker.exists()


def test_legacy_right_record_endpoint_override_remains_supported(tmp_path: Path) -> None:
    command = f"""
source {CONFIG_SCRIPT!s}
franka_runtime_export_defaults
printf '%s:%s\n' "$FRANKA_RIGHT_ZMQ_HOST" "$FRANKA_RIGHT_ZMQ_PORT"
"""
    result = subprocess.run(
        ["bash", "-euo", "pipefail", "-c", command],
        cwd=REPO_ROOT,
        env={
            **os.environ,
            "HOME": str(tmp_path),
            "FRANKA_RUNTIME_CONFIG_FILE": str(tmp_path / "missing.env"),
            "BI_ARM_RIGHT_RECORD_ZMQ_HOST": "10.0.0.131",
            "BI_ARM_RIGHT_RECORD_ZMQ_PORT": "7001",
        },
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    assert result.stdout == "10.0.0.131:7001\n"


def test_xpra_help_uses_current_left_machine() -> None:
    result = subprocess.run(
        ["bash", str(REPO_ROOT / "remote" / "open_franka_gui_xpra.sh"), "--help"],
        cwd=REPO_ROOT,
        env={**os.environ, "FRANKA_RUNTIME_CONFIG_FILE": "/dev/null"},
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=True,
    )
    assert "FRANKA_XPRA_HOST=muka@192.168.1.170" in result.stdout
    assert "FRANKA_XPRA_REPO=/home/muka/frankateleop" in result.stdout
    assert "192.168.1.114" not in result.stdout


def test_custom_xpra_host_does_not_reuse_default_control_socket(tmp_path: Path) -> None:
    command = f"""
source {CONFIG_SCRIPT!s}
franka_runtime_export_xpra_defaults
printf '<%s>\n' "$FRANKA_XPRA_SSH_SOCKET"
"""
    result = subprocess.run(
        ["bash", "-euo", "pipefail", "-c", command],
        cwd=REPO_ROOT,
        env={
            **os.environ,
            "HOME": str(tmp_path),
            "FRANKA_RUNTIME_CONFIG_FILE": str(tmp_path / "missing.env"),
            "FRANKA_XPRA_HOST": "operator@10.0.0.70",
        },
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    assert result.stdout == "<>\n"


@pytest.mark.parametrize(
    "script_name",
    (
        "B_run_right_arm_capture_gui.sh",
        "C_run_dual_arm_capture_gui.sh",
        "15_record_bi_arm_pipeline.sh",
        "16_replay_bi_arm_pipeline.sh",
    ),
)
def test_runtime_file_reaches_gui_record_and_replay_entries(
    tmp_path: Path,
    script_name: str,
) -> None:
    config_file = tmp_path / "runtime.env"
    config_file.write_text(
        "BI_ARM_RIGHT_HOST=10.20.30.131\n"
        "BI_ARM_RIGHT_SSH=robot@10.20.30.131\n"
        "BI_ARM_RIGHT_REPO=/srv/frankateleop\n"
        "BI_ARM_LEFT_ZMQ_PORT=7002\n"
        "BI_ARM_RIGHT_REMOTE_ZMQ_PORT=7001\n",
        encoding="utf-8",
    )
    env = os.environ.copy()
    for key in (
        "BI_ARM_RIGHT_HOST",
        "BI_ARM_RIGHT_SSH",
        "BI_ARM_RIGHT_REPO",
        "BI_ARM_LEFT_ZMQ_PORT",
        "BI_ARM_RIGHT_REMOTE_ZMQ_PORT",
        "FRANKA_RIGHT_ZMQ_HOST",
        "FRANKA_RIGHT_ZMQ_PORT",
    ):
        env.pop(key, None)
    env["FRANKA_RUNTIME_CONFIG_FILE"] = str(config_file)
    result = subprocess.run(
        ["bash", "-x", str(REPO_ROOT / script_name), "--help"],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=True,
    )
    assert "10.20.30.131" in result.stdout
    assert "robot@10.20.30.131" in result.stdout
    assert "/srv/frankateleop" in result.stdout
    assert "7002" in result.stdout
    assert "7001" in result.stdout


def test_runtime_file_reaches_xpra_entry(tmp_path: Path) -> None:
    config_file = tmp_path / "runtime.env"
    config_file.write_text(
        "FRANKA_XPRA_HOST=operator@10.20.30.170\n"
        "FRANKA_XPRA_REPO=/srv/frankateleop\n",
        encoding="utf-8",
    )
    fake_xpra = tmp_path / "xpra"
    fake_xpra.write_text("#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n", encoding="utf-8")
    fake_xpra.chmod(0o755)
    env = os.environ.copy()
    for key in (
        "FRANKA_XPRA_HOST",
        "FRANKA_XPRA_REPO",
        "FRANKA_XPRA_SSH_SOCKET",
        "FRANKA_XPRA_SSH",
    ):
        env.pop(key, None)
    env.update(
        {
            "FRANKA_RUNTIME_CONFIG_FILE": str(config_file),
            "FRANKA_XPRA_BIN": str(fake_xpra),
        }
    )
    result = subprocess.run(
        ["bash", str(REPO_ROOT / "remote" / "open_franka_gui_xpra.sh"), "A"],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=True,
    )
    assert "ssh://operator@10.20.30.170/124" in result.stdout
    assert "Remote repo: /srv/frankateleop" in result.stdout
    assert "cd\\ /srv/frankateleop" in result.stdout
    assert "--ssh=ssh" in result.stdout
    assert "codex-franka-170.sock" not in result.stdout
