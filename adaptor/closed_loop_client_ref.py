#!/usr/bin/env python3
import argparse
import asyncio
import base64
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import requests
import websockets
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
for candidate in (PROJECT_ROOT, SCRIPT_DIR):
    candidate_str = str(candidate)
    if candidate_str not in sys.path:
        sys.path.append(candidate_str)

from envs.camera.rs_capture import RSCapture
from fr3_single_arm_config import SingleArmConfig
from project_paths import DEMO_DATA_ROOT


START_FRANKA_SCRIPT = SCRIPT_DIR / "start_franka_if_needed.sh"
GO_RESET_SCRIPT = SCRIPT_DIR.parent / "core" / "go_reset_pose.py"
PICK_SCRIPT = SCRIPT_DIR / "pick.py"
DEFAULT_RUN_PREFIX = "cloud_policy"
LEGACY_OVERLAY_X_RATIO = 0.14
LEGACY_OVERLAY_Y_RATIO = 0.29
LEGACY_OVERLAY_RADIUS_AT_224 = 4.0


def euler_to_quat_7d(pose_6d: np.ndarray | list[float]) -> list[float]:
    pose = np.asarray(pose_6d, dtype=float).reshape(-1)
    rot = R.from_euler("xyz", pose[3:], degrees=False)
    quat = rot.as_quat()
    return np.concatenate([pose[:3], quat]).tolist()


def _encode_frame(frame: np.ndarray, *, route: str, seed: int | None = None) -> str:
    return json.dumps(
        {
            "route": route,
            "shape": list(frame.shape),
            "dtype": str(frame.dtype),
            "frame_b64": base64.b64encode(np.ascontiguousarray(frame).tobytes()).decode("ascii"),
            "seed": seed,
        }
    )


def safe_camera_read(camera, *, max_retries: int = 5, retry_sleep: float = 0.15):
    last_error = None
    for _ in range(max_retries):
        try:
            return camera.read()
        except RuntimeError as exc:
            last_error = exc
            time.sleep(retry_sleep)
    raise RuntimeError(f"相机连续读帧失败: {last_error}")


def load_first_frame_from_video(path: Path) -> np.ndarray:
    cap = cv2.VideoCapture(str(path))
    try:
        ok, frame = cap.read()
    finally:
        cap.release()
    if not ok or frame is None:
        raise RuntimeError(f"无法从视频读取第一帧: {path}")
    return frame


def load_prime_frame(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix in {".mp4", ".mov", ".avi", ".mkv"}:
        frame_bgr = load_first_frame_from_video(path)
    else:
        frame_bgr = cv2.imread(str(path))
        if frame_bgr is None:
            raise RuntimeError(f"无法读取 priming 图像: {path}")
    return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)


def normalize_ws_timeout(value: float | None) -> float | None:
    if value is None:
        return None
    return None if value <= 0 else float(value)


def run_step(command: list[str], *, label: str, cwd: Path | None = None) -> None:
    print(f"\n🚀 {label}")
    subprocess.run(command, cwd=str(cwd) if cwd else None, check=True)


def ensure_franka_service() -> None:
    run_step(["bash", str(START_FRANKA_SCRIPT)], label="检查或启动 Franka 底层服务", cwd=SCRIPT_DIR)


def reset_to_home_pose() -> None:
    run_step(
        [SingleArmConfig.PYTHON_BIN, str(GO_RESET_SCRIPT), "--open-gripper"],
        label="回到 home pose",
        cwd=SCRIPT_DIR,
    )


def wait_for_pick_setup(delay_seconds: float) -> None:
    if delay_seconds <= 0:
        return
    print(f"⏳ 已完成云端取图，等待 {delay_seconds:.1f}s 让你放置红色物体...")
    time.sleep(delay_seconds)


def run_pick_pipeline(run_id: str) -> None:
    run_step(
        [
            SingleArmConfig.PYTHON_BIN,
            str(PICK_SCRIPT),
            "--run_id",
            run_id,
            "--auto_confirm",
            "--skip_task_reset",
            "--post_pick_pose",
            "none",
        ],
        label="执行本地 pick 流程",
        cwd=SCRIPT_DIR,
    )


def capture_single_frame(camera_name: str, *, warmup_seconds: float, flush_frames: int) -> np.ndarray:
    camera_kwargs = SingleArmConfig.REALSENSE_CAMERAS.get(camera_name)
    if not camera_kwargs:
        raise RuntimeError(f"未找到相机配置: {camera_name}")

    camera = RSCapture(name=camera_name, **camera_kwargs)
    try:
        print(f"📸 打开相机 [{camera_name}]，预热 {warmup_seconds:.1f}s...")
        time.sleep(warmup_seconds)
        for _ in range(max(0, flush_frames)):
            safe_camera_read(camera, max_retries=2, retry_sleep=0.05)
        frame_bgr, _ = safe_camera_read(camera, max_retries=5, retry_sleep=0.15)
        return frame_bgr
    finally:
        camera.close()
        print("🔌 相机资源已释放。")


def resize_without_crop(image_bgr: np.ndarray, size: int) -> np.ndarray:
    return cv2.resize(image_bgr, (size, size), interpolation=cv2.INTER_AREA)


def scale_point(point: tuple[int, int], src_shape: tuple[int, int, int], dst_size: int) -> tuple[int, int]:
    src_h, src_w = src_shape[:2]
    scale_x = dst_size / float(src_w)
    scale_y = dst_size / float(src_h)
    return int(round(point[0] * scale_x)), int(round(point[1] * scale_y))


def draw_legacy_overlay_frame(raw_bgr: np.ndarray, resize_to: int) -> tuple[np.ndarray, tuple[int, int], tuple[int, int]]:
    raw_h, raw_w = raw_bgr.shape[:2]
    raw_x = int(round(LEGACY_OVERLAY_X_RATIO * raw_w))
    raw_y = int(round(LEGACY_OVERLAY_Y_RATIO * raw_h))
    raw_x = int(np.clip(raw_x, 0, raw_w - 1))
    raw_y = int(np.clip(raw_y, 0, raw_h - 1))
    raw_radius = max(1, int(round((LEGACY_OVERLAY_RADIUS_AT_224 / 224.0) * min(raw_w, raw_h))))

    raw_rgb = cv2.cvtColor(raw_bgr, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(raw_rgb)
    draw = ImageDraw.Draw(pil_image)
    draw.ellipse((raw_x - raw_radius, raw_y - raw_radius, raw_x + raw_radius, raw_y + raw_radius), fill=(255, 0, 0))
    resized_rgb = pil_image.resize((resize_to, resize_to), Image.BICUBIC)
    resized_bgr = cv2.cvtColor(np.asarray(resized_rgb), cv2.COLOR_RGB2BGR)
    resized_xy = scale_point((raw_x, raw_y), raw_bgr.shape, resize_to)
    return resized_bgr, (raw_x, raw_y), resized_xy


def prepare_policy_frame_from_bgr(
    raw_bgr: np.ndarray,
    *,
    resize_to: int,
    overlay_radius: int,
    overlay_x: int | None,
    overlay_y: int | None,
    overlay_raw_x: int | None,
    overlay_raw_y: int | None,
) -> np.ndarray:
    raw_h, raw_w = raw_bgr.shape[:2]

    if (overlay_raw_x is None) ^ (overlay_raw_y is None):
        raise RuntimeError("overlay_raw_x 和 overlay_raw_y 必须同时提供。")
    if (overlay_x is None) ^ (overlay_y is None):
        raise RuntimeError("overlay_x 和 overlay_y 必须同时提供。")

    if overlay_raw_x is not None and overlay_raw_y is not None:
        raw_overlay_xy = (
            int(np.clip(overlay_raw_x, 0, raw_w - 1)),
            int(np.clip(overlay_raw_y, 0, raw_h - 1)),
        )
        raw_with_ball = raw_bgr.copy()
        raw_overlay_radius = max(2, int(round(overlay_radius * max(raw_w, raw_h) / float(resize_to))))
        cv2.circle(raw_with_ball, raw_overlay_xy, raw_overlay_radius + 2, (255, 255, 255), -1)
        cv2.circle(raw_with_ball, raw_overlay_xy, raw_overlay_radius, (0, 0, 255), -1)
        cloud_bgr = resize_without_crop(raw_with_ball, resize_to)
    elif overlay_x is not None and overlay_y is not None:
        cloud_bgr = resize_without_crop(raw_bgr, resize_to)
        overlay_xy = (
            int(np.clip(overlay_x, 0, resize_to - 1)),
            int(np.clip(overlay_y, 0, resize_to - 1)),
        )
        cv2.circle(cloud_bgr, overlay_xy, overlay_radius + 2, (255, 255, 255), -1)
        cv2.circle(cloud_bgr, overlay_xy, overlay_radius, (0, 0, 255), -1)
    else:
        cloud_bgr, _, _ = draw_legacy_overlay_frame(raw_bgr, resize_to)

    return cv2.cvtColor(cloud_bgr, cv2.COLOR_BGR2RGB)


def prepare_policy_input(
    raw_image_path: Path,
    output_dir: Path,
    *,
    resize_to: int,
    overlay_radius: int,
    overlay_x: int,
    overlay_y: int,
    overlay_raw_x: int | None,
    overlay_raw_y: int | None,
    seed: int,
) -> dict[str, Any]:
    raw_bgr = cv2.imread(str(raw_image_path))
    if raw_bgr is None:
        raise RuntimeError(f"无法读取原始图片: {raw_image_path}")
    raw_h, raw_w = raw_bgr.shape[:2]

    if (overlay_raw_x is None) ^ (overlay_raw_y is None):
        raise RuntimeError("overlay_raw_x 和 overlay_raw_y 必须同时提供。")
    if (overlay_x is None) ^ (overlay_y is None):
        raise RuntimeError("overlay_x 和 overlay_y 必须同时提供。")

    raw_overlay_path = output_dir / "camera_overlay_raw.png"
    resized_path = output_dir / f"camera_{resize_to}.png"
    overlay_path = output_dir / f"camera_{resize_to}_overlay.png"
    legacy_overlay_path = output_dir / "overlay_frame.png"

    if overlay_raw_x is not None and overlay_raw_y is not None:
        raw_overlay_xy = (
            int(np.clip(overlay_raw_x, 0, raw_w - 1)),
            int(np.clip(overlay_raw_y, 0, raw_h - 1)),
        )
        raw_with_ball = raw_bgr.copy()
        raw_overlay_radius = max(2, int(round(overlay_radius * max(raw_w, raw_h) / float(resize_to))))
        cv2.circle(raw_with_ball, raw_overlay_xy, raw_overlay_radius + 2, (255, 255, 255), -1)
        cv2.circle(raw_with_ball, raw_overlay_xy, raw_overlay_radius, (0, 0, 255), -1)
        cv2.imwrite(str(raw_overlay_path), raw_with_ball)

        cloud_bgr = resize_without_crop(raw_with_ball, resize_to)
        resized_bgr = resize_without_crop(raw_bgr, resize_to)
        overlay_xy = scale_point(raw_overlay_xy, raw_bgr.shape, resize_to)
        overlay_mode = "raw_then_resize"
    elif overlay_x is not None and overlay_y is not None:
        resized_bgr = resize_without_crop(raw_bgr, resize_to)
        cloud_bgr = resized_bgr.copy()
        overlay_xy = (
            int(np.clip(overlay_x, 0, resize_to - 1)),
            int(np.clip(overlay_y, 0, resize_to - 1)),
        )
        cv2.circle(cloud_bgr, overlay_xy, overlay_radius + 2, (255, 255, 255), -1)
        cv2.circle(cloud_bgr, overlay_xy, overlay_radius, (0, 0, 255), -1)
        raw_overlay_xy = None
        overlay_mode = "resize_then_overlay"
    else:
        resized_bgr = resize_without_crop(raw_bgr, resize_to)
        cloud_bgr, raw_overlay_xy, overlay_xy = draw_legacy_overlay_frame(raw_bgr, resize_to)
        overlay_mode = "legacy_local_model_overlay"

    cv2.imwrite(str(resized_path), resized_bgr)
    cv2.imwrite(str(overlay_path), cloud_bgr)
    cv2.imwrite(str(legacy_overlay_path), cloud_bgr)

    metadata = {
        "seed": seed,
        "raw_image_path": str(raw_image_path),
        "raw_overlay_path": str(raw_overlay_path) if raw_overlay_path.exists() else None,
        "resized_path": str(resized_path),
        "overlay_path": str(overlay_path),
        "legacy_overlay_path": str(legacy_overlay_path),
        "overlay_mode": overlay_mode,
        "overlay_raw_xy": list(raw_overlay_xy) if raw_overlay_xy is not None else None,
        "overlay_resized_xy": list(overlay_xy),
        "resize_to": resize_to,
    }
    with open(output_dir / "cloud_request_metadata.json", "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, ensure_ascii=False)

    cloud_rgb = cv2.cvtColor(cloud_bgr, cv2.COLOR_BGR2RGB)
    return {
        "frame_rgb": cloud_rgb,
        "metadata": metadata,
    }


class RobotExecutionClient:
    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip("/")
        self._session = requests.Session()
        self._session.trust_env = False
        self.gripper_state = "open"

    def set_motion_mode(self, motion_async: bool) -> None:
        response = self._session.post(
            f"{self.base_url}/config/motion_mode",
            json={"motion_async": bool(motion_async)},
            timeout=2.0,
        )
        response.raise_for_status()

    def set_dynamics_factor(self, factor: float) -> None:
        response = self._session.post(
            f"{self.base_url}/config/dynamics",
            json={"relative_dynamics_factor": float(factor)},
            timeout=2.0,
        )
        response.raise_for_status()

    def get_gripper_width(self) -> float:
        response = self._session.post(f"{self.base_url}/get_gripper", timeout=2.0)
        response.raise_for_status()
        return float(response.json()["gripper"])

    def sync_gripper_state(self, *, close_threshold: float = 0.04) -> None:
        width = self.get_gripper_width()
        self.gripper_state = "close" if width < close_threshold else "open"
        print(f"🤏 当前夹爪宽度: {width:.4f} m，执行前状态同步为: {self.gripper_state}")

    def goto_pose_7d(self, pose_7d: list[float]) -> None:
        try:
            self._session.post(f"{self.base_url}/pose", json={"arr": pose_7d}, timeout=0.1)
        except requests.exceptions.ReadTimeout:
            pass

    def execute_gripper(self, gripper_width: float, *, close_threshold: float = 0.04) -> None:
        target_state = "close" if gripper_width < close_threshold else "open"
        if target_state == self.gripper_state:
            return

        if target_state == "close":
            print(f"🤏 [动作] 闭合夹爪 (预测宽度: {gripper_width:.4f})")
            response = self._session.post(
                f"{self.base_url}/close_gripper_slow",
                json={"width": 0.0, "speed": 0.05, "force": 40.0, "epsilon_outer": 1.0},
                timeout=2.0,
            )
            response.raise_for_status()
        else:
            print(f"🫱 [动作] 打开夹爪 (预测宽度: {gripper_width:.4f})")
            response = self._session.post(f"{self.base_url}/open_gripper", timeout=2.0)
            response.raise_for_status()

        self.gripper_state = target_state


def normalize_action_chunk(payload: Any) -> np.ndarray:
    array = np.asarray(payload, dtype=np.float32)
    if array.ndim == 1:
        if array.shape[0] < 7:
            raise RuntimeError(f"收到的 action 维度不足 7: {array.shape}")
        array = array.reshape(1, -1)
    if array.ndim != 2 or array.shape[1] < 7:
        raise RuntimeError(f"收到的 action shape 非法: {array.shape}")
    return array[:, :7]


def extract_action_chunk(response: dict[str, Any]) -> np.ndarray | None:
    if "action" in response:
        return normalize_action_chunk(response["action"])
    if "actions" in response:
        return normalize_action_chunk(response["actions"])
    return None


def response_indicates_done(response: dict[str, Any]) -> bool:
    status = str(response.get("status", "")).lower()
    if response.get("done") is True:
        return True
    if response.get("has_more") is False or response.get("more") is False:
        return True
    return status in {"done", "completed", "complete"}


async def maybe_reset_remote_policy(ws, *, enabled: bool) -> None:
    if not enabled:
        return
    await ws.send(json.dumps({"route": "/reset"}))
    try:
        message = await asyncio.wait_for(ws.recv(), timeout=10.0)
        print(f"♻️ 远程策略 reset 响应: {message[:160]}")
    except asyncio.TimeoutError:
        print("⚠️ 远程策略 reset 超时，继续发送图片。")


async def send_priming_frame(
    ws,
    frame_rgb: np.ndarray,
    *,
    route: str,
    seed: int,
    output_dir: Path,
) -> None:
    await ws.send(_encode_frame(frame_rgb, route=route, seed=seed))
    print("🧭 已发送 priming frame，忽略这次返回的 action。")
    message = await ws.recv()
    try:
        response = json.loads(message)
    except json.JSONDecodeError:
        response = {"raw": message}
    with open(output_dir / "cloud_policy_prime_response.json", "w", encoding="utf-8") as handle:
        json.dump(response, handle, indent=2, ensure_ascii=False)


async def receive_policy_chunk(
    ws,
    *,
    recv_timeout: float,
    response_log: list[dict[str, Any]],
) -> tuple[dict[str, Any], np.ndarray | None]:
    while True:
        message = await asyncio.wait_for(ws.recv(), timeout=recv_timeout)
        try:
            response = json.loads(message)
        except json.JSONDecodeError:
            response = {"raw": message}
        response_log.append(response)

        chunk = extract_action_chunk(response) if isinstance(response, dict) else None
        if chunk is not None or (isinstance(response, dict) and response_indicates_done(response)):
            return response, chunk


def execute_action_chunk(
    robot: RobotExecutionClient,
    chunk: np.ndarray,
    *,
    fps: int,
    chunk_index: int,
    total_chunks_hint: int | None = None,
) -> None:
    actions = np.asarray(chunk, dtype=np.float32)
    if actions.ndim != 2 or actions.shape[1] < 7:
        raise RuntimeError(f"action chunk shape 非法: {actions.shape}")

    chunk_label = f"{chunk_index}" if total_chunks_hint is None else f"{chunk_index}/{total_chunks_hint}"
    print(f"📚 开始执行 chunk {chunk_label}，共 {actions.shape[0]} 条 action。")

    robot.set_motion_mode(True)
    robot.set_dynamics_factor(SingleArmConfig.RELATIVE_DYNAMICS_FACTOR)
    robot.sync_gripper_state()

    for action_in_chunk_index, target_action in enumerate(actions, start=1):
        loop_start = time.time()
        target_pose_6d = target_action[:6]
        target_gripper = float(target_action[6])

        pose_7d = euler_to_quat_7d(target_pose_6d)
        robot.goto_pose_7d(pose_7d)
        robot.execute_gripper(target_gripper)

        elapsed = time.time() - loop_start
        sleep_t = max(0.0, (1.0 / fps) - elapsed)
        time.sleep(sleep_t)
        print(
            f"🔄 已执行 chunk {chunk_label} 内 action {action_in_chunk_index}/{actions.shape[0]}",
            end="\r",
        )
    print("")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Single-shot cloud policy pipeline for FR3 maze demo.")
    parser.add_argument("--uri", default="ws://127.0.0.1:8765", help="云端策略 WebSocket 地址")
    parser.add_argument("--camera-name", default="exterior_2", help="拍 maze 全景图的相机名称")
    parser.add_argument("--route", default="predict", help="发送给云端的推理 route")
    parser.add_argument("--prime-frame-path", default=None, help="可选：先发送一张 priming 图像或视频第一帧，并忽略其 action")
    parser.add_argument("--prime-route", default=None, help="priming frame 使用的 route；默认与 --route 相同")
    parser.add_argument("--run-id", default=None, help="输出目录名；默认自动生成")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--fps", type=int, default=10, help="执行 action 的控制频率")
    parser.add_argument("--resize-to", type=int, default=224, help="发送云端前统一 resize 到的尺寸")
    parser.add_argument("--overlay-radius", type=int, default=6, help="叠加小红球的半径（resize 后像素）")
    parser.add_argument(
        "--overlay-x",
        type=int,
        default=None,
        help="resize 后图像上的红球 X 像素坐标；不传则默认复用旧本地模型 overlay_frame 的位置逻辑",
    )
    parser.add_argument(
        "--overlay-y",
        type=int,
        default=None,
        help="resize 后图像上的红球 Y 像素坐标；不传则默认复用旧本地模型 overlay_frame 的位置逻辑",
    )
    parser.add_argument("--overlay-raw-x", type=int, default=None, help="原始相机图像上的红球 X 像素坐标")
    parser.add_argument("--overlay-raw-y", type=int, default=None, help="原始相机图像上的红球 Y 像素坐标")
    parser.add_argument("--camera-warmup", type=float, default=1.5)
    parser.add_argument("--camera-flush-frames", type=int, default=10)
    parser.add_argument("--pick-setup-wait", type=float, default=8.0, help="拍完 maze 图并发送云端后，等待放置红色物体的秒数")
    parser.add_argument("--recv-timeout", type=float, default=180.0, help="等待云端返回动作的超时时间")
    parser.add_argument("--ws-ping-interval", type=float, default=0.0, help="WebSocket keepalive ping 间隔；<=0 表示关闭，避免长推理时被误判断连")
    parser.add_argument("--ws-ping-timeout", type=float, default=0.0, help="WebSocket ping 超时；<=0 表示关闭")
    parser.add_argument("--ws-close-timeout", type=float, default=10.0, help="WebSocket close 超时；<=0 表示关闭")
    parser.add_argument("--skip-policy-reset", action="store_true", help="不发送 WebSocket /reset 指令")
    parser.add_argument("--skip-first-chunk", action="store_true", default=True, help="跳过第一整个无效 chunk")
    parser.add_argument("--no-skip-first-chunk", dest="skip_first_chunk", action="store_false")
    return parser


async def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    run_id = args.run_id or f"{DEFAULT_RUN_PREFIX}_{time.strftime('%Y%m%d_%H%M%S')}"
    output_dir = Path(DEMO_DATA_ROOT) / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_image_path = output_dir / "camera.png"

    ensure_franka_service()
    reset_to_home_pose()

    raw_frame_bgr = capture_single_frame(
        args.camera_name,
        warmup_seconds=args.camera_warmup,
        flush_frames=args.camera_flush_frames,
    )
    cv2.imwrite(str(raw_image_path), raw_frame_bgr)
    print(f"✅ 已保存原始 maze 图像: {raw_image_path}")

    request_bundle = prepare_policy_input(
        raw_image_path,
        output_dir,
        resize_to=args.resize_to,
        overlay_radius=args.overlay_radius,
        overlay_x=args.overlay_x,
        overlay_y=args.overlay_y,
        overlay_raw_x=args.overlay_raw_x,
        overlay_raw_y=args.overlay_raw_y,
        seed=args.seed,
    )

    ws_ping_interval = normalize_ws_timeout(args.ws_ping_interval)
    ws_ping_timeout = normalize_ws_timeout(args.ws_ping_timeout)
    ws_close_timeout = normalize_ws_timeout(args.ws_close_timeout)
    print(
        "🔌 WebSocket 连接参数: "
        f"ping_interval={ws_ping_interval} ping_timeout={ws_ping_timeout} close_timeout={ws_close_timeout}"
    )

    received_chunks: list[np.ndarray] = []
    raw_responses: list[dict[str, Any]] = []
    robot = RobotExecutionClient(SingleArmConfig.SERVER_URL)

    async with websockets.connect(
        args.uri,
        max_size=None,
        ping_interval=ws_ping_interval,
        ping_timeout=ws_ping_timeout,
        close_timeout=ws_close_timeout,
    ) as ws:
        await maybe_reset_remote_policy(ws, enabled=not args.skip_policy_reset)

        if args.prime_frame_path:
            prime_path = Path(args.prime_frame_path)
            prime_route = args.prime_route or args.route
            prime_rgb = load_prime_frame(prime_path)
            await send_priming_frame(
                ws,
                prime_rgb,
                route=prime_route,
                seed=args.seed,
                output_dir=output_dir,
            )

        await ws.send(_encode_frame(request_bundle["frame_rgb"], route=args.route, seed=args.seed))
        print("☁️ 已发送首帧到云端，等待第一个 action chunk ...")

        try:
            await asyncio.to_thread(wait_for_pick_setup, args.pick_setup_wait)
            await asyncio.to_thread(run_pick_pipeline, run_id)
        except Exception:
            raise

        camera_kwargs = SingleArmConfig.REALSENSE_CAMERAS.get(args.camera_name)
        if not camera_kwargs:
            raise RuntimeError(f"未找到相机配置: {args.camera_name}")

        loop_camera = RSCapture(name=args.camera_name, **camera_kwargs)
        try:
            print(f"📸 重新打开相机 [{args.camera_name}] 用于闭环观测...")
            time.sleep(max(0.6, args.camera_warmup * 0.5))
            for _ in range(max(2, args.camera_flush_frames // 2)):
                safe_camera_read(loop_camera, max_retries=2, retry_sleep=0.05)

            chunk_index = 0
            while True:
                response, chunk = await receive_policy_chunk(
                    ws,
                    recv_timeout=args.recv_timeout,
                    response_log=raw_responses,
                )

                if chunk is not None:
                    chunk_index += 1
                    received_chunks.append(chunk)
                    print(f"📦 收到 action chunk #{chunk_index}: shape={tuple(chunk.shape)}")

                    if args.skip_first_chunk and chunk_index == 1:
                        print("⏭️ 按配置跳过第一个 chunk，不执行。")
                    else:
                        execute_action_chunk(
                            robot,
                            chunk,
                            fps=args.fps,
                            chunk_index=chunk_index,
                        )

                if response_indicates_done(response):
                    print("✅ 云端已显式结束 chunk 流。")
                    break

                live_bgr, _ = safe_camera_read(loop_camera, max_retries=5, retry_sleep=0.1)
                live_rgb = prepare_policy_frame_from_bgr(
                    live_bgr,
                    resize_to=args.resize_to,
                    overlay_radius=args.overlay_radius,
                    overlay_x=args.overlay_x,
                    overlay_y=args.overlay_y,
                    overlay_raw_x=args.overlay_raw_x,
                    overlay_raw_y=args.overlay_raw_y,
                )
                next_seed = args.seed + chunk_index
                await ws.send(_encode_frame(live_rgb, route=args.route, seed=next_seed))
                print(f"📨 已回传当前相机帧，等待下一个 chunk (seed={next_seed}) ...")
        finally:
            loop_camera.close()
            print("🔌 闭环观测相机已释放。")

    with open(output_dir / "cloud_policy_responses.json", "w", encoding="utf-8") as handle:
        json.dump(raw_responses, handle, indent=2, ensure_ascii=False)

    if not received_chunks:
        raise RuntimeError("云端没有返回任何 action chunk。")

    actions = np.stack(received_chunks, axis=0)
    np.save(output_dir / "cloud_actions.npy", actions)
    print(f"💾 云端 action chunks 已保存: {output_dir / 'cloud_actions.npy'}")

    reset_to_home_pose()
    print(f"🎉 云端 pipeline 执行完成，输出目录: {output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("\n🛑 收到中断信号，云端 pipeline 已停止。")
