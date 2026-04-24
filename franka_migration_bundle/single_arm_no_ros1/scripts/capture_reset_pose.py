import argparse
import re
from pathlib import Path

import requests

from fr3_single_arm_config import SingleArmConfig


CONFIG_PATH = Path(__file__).resolve().parents[1] / "fr3_single_arm_config.py"


def fetch_current_pose(server_url: str) -> list[float]:
    session = requests.Session()
    session.trust_env = False

    health = session.get(f"{server_url.rstrip('/')}/health", timeout=2.0)
    health.raise_for_status()
    health_data = health.json().get("data", {})
    if not health_data.get("connected"):
        raise RuntimeError("franky server 已启动，但机器人尚未连接。请先启动并连接机器人。")

    response = session.post(f"{server_url.rstrip('/')}/getpos_euler", timeout=2.0)
    response.raise_for_status()
    pose = response.json()["pose"]
    if not (isinstance(pose, list) and len(pose) == 6):
        raise RuntimeError(f"读取到的 pose 格式不正确: {pose}")
    return [float(x) for x in pose]


def format_pose_array(pose: list[float]) -> str:
    values = ", ".join(f"{x:.9f}" for x in pose)
    return f"np.array([{values}])"


def write_reset_pose_to_config(pose: list[float], config_path: Path) -> None:
    content = config_path.read_text(encoding="utf-8")
    replacement = f"    RESET_POSE = {format_pose_array(pose)}"
    updated, count = re.subn(
        r"^    RESET_POSE = .*$",
        replacement,
        content,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise RuntimeError("没有在 fr3_single_arm_config.py 中找到 RESET_POSE 配置项。")
    config_path.write_text(updated, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(
        description="读取当前 FR3 位姿，并可选写入 fr3_single_arm_config.py 作为 RESET_POSE"
    )
    parser.add_argument(
        "--server-url",
        default=SingleArmConfig.SERVER_URL,
        help="franky server 地址，默认读取 fr3_single_arm_config.py",
    )
    parser.add_argument(
        "--write-config",
        action="store_true",
        help="将当前位姿写入 fr3_single_arm_config.py 的 RESET_POSE",
    )
    args = parser.parse_args()

    pose = fetch_current_pose(args.server_url)
    formatted = format_pose_array(pose)

    print("当前末端位姿（欧拉角）:")
    print(formatted)
    print()
    print("建议下一步：基于这个 RESET_POSE 再设置工作空间上下界。")

    if args.write_config:
        write_reset_pose_to_config(pose, CONFIG_PATH)
        print()
        print(f"已写入 {CONFIG_PATH}")


if __name__ == "__main__":
    main()
