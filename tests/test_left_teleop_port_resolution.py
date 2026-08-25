from __future__ import annotations

import os
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
RESOLVER = REPO_ROOT / "scripts" / "resolve_left_teleop_port.sh"


def test_default_primary_matches_current_left_teleop_adapter() -> None:
    text = RESOLVER.read_text(encoding="utf-8")
    assert "usb-FTDI_USB__-__Serial_Converter_FTBJKECV-if00-port0" in text
    assert "usb-FTDI_USB_TO_RS-485_DAAQM7QD-if00-port0" not in text


def _run_resolver(
    tmp_path: Path,
    *,
    primary_exists: bool = False,
    fallback_exists: bool = False,
    explicit: Path | None = None,
) -> subprocess.CompletedProcess[str]:
    primary = tmp_path / "primary"
    fallback = tmp_path / "fallback"
    if primary_exists:
        primary.touch(mode=0o600)
    if fallback_exists:
        fallback.touch(mode=0o600)

    env = os.environ.copy()
    for key in ("LEFT_TELEOP_PORT", "FRANKA_TELEOP_PORT", "TELEOP_PORT"):
        env.pop(key, None)
    env.update(
        {
            "LEFT_TELEOP_PRIMARY_PORT": str(primary),
            "LEFT_TELEOP_FALLBACK_PORT": str(fallback),
        }
    )
    if explicit is not None:
        env["LEFT_TELEOP_PORT"] = str(explicit)

    return subprocess.run(
        ["bash", str(RESOLVER)],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def test_primary_left_teleop_port_is_preferred(tmp_path: Path) -> None:
    result = _run_resolver(tmp_path, primary_exists=True, fallback_exists=True)

    assert result.returncode == 0
    assert result.stdout.strip() == str(tmp_path / "primary")


def test_registered_fallback_is_used_when_primary_is_missing(tmp_path: Path) -> None:
    result = _run_resolver(tmp_path, fallback_exists=True)

    assert result.returncode == 0
    assert result.stdout.strip() == str(tmp_path / "fallback")
    assert "使用已登记的备用左臂串口" in result.stderr


def test_explicit_missing_port_fails_closed(tmp_path: Path) -> None:
    missing = tmp_path / "explicit-missing"
    result = _run_resolver(tmp_path, fallback_exists=True, explicit=missing)

    assert result.returncode != 0
    assert result.stdout == ""
    assert str(missing) in result.stderr
    assert "拒绝自动改选其他设备" in result.stderr


def test_missing_known_ports_reports_both_candidates(tmp_path: Path) -> None:
    result = _run_resolver(tmp_path)

    assert result.returncode != 0
    assert result.stdout == ""
    assert str(tmp_path / "primary") in result.stderr
    assert str(tmp_path / "fallback") in result.stderr
