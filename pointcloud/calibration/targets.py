"""ChArUco target helpers."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import cv2
import numpy as np


DEFAULT_BOARD_CONFIG: Dict[str, Any] = {
    "type": "charuco",
    "dictionary": "DICT_5X5_250",
    "squares_x": 7,
    "squares_y": 5,
    "square_length_m": 0.035,
    "marker_length_m": 0.026,
}


def dictionary_from_name(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("This OpenCV build does not include cv2.aruco")
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary {name!r}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def normalize_board_config(config: Optional[Dict[str, Any]] = None, **overrides: Any) -> Dict[str, Any]:
    merged = dict(DEFAULT_BOARD_CONFIG)
    if config:
        merged.update(config)
    merged.update({key: value for key, value in overrides.items() if value is not None})
    merged["squares_x"] = int(merged["squares_x"])
    merged["squares_y"] = int(merged["squares_y"])
    merged["square_length_m"] = float(merged["square_length_m"])
    merged["marker_length_m"] = float(merged["marker_length_m"])
    if merged["squares_x"] < 2 or merged["squares_y"] < 2:
        raise ValueError("ChArUco board needs at least 2 squares in each direction")
    if merged["marker_length_m"] >= merged["square_length_m"]:
        raise ValueError("marker_length_m must be smaller than square_length_m")
    return merged


def create_charuco_board(config: Optional[Dict[str, Any]] = None, **overrides: Any):
    board_config = normalize_board_config(config, **overrides)
    dictionary = dictionary_from_name(board_config["dictionary"])
    return cv2.aruco.CharucoBoard(
        (board_config["squares_x"], board_config["squares_y"]),
        board_config["square_length_m"],
        board_config["marker_length_m"],
        dictionary,
    )


def render_charuco_board(config: Optional[Dict[str, Any]] = None, *, image_size: Sequence[int] = (1800, 1200), margin_px: int = 40) -> np.ndarray:
    board = create_charuco_board(config)
    width, height = int(image_size[0]), int(image_size[1])
    return board.generateImage((width, height), marginSize=int(margin_px))


def camera_matrix_from_intrinsics(intrinsics: Dict[str, Any]) -> np.ndarray:
    return np.array(
        [
            [float(intrinsics["fx"]), 0.0, float(intrinsics["ppx"])],
            [0.0, float(intrinsics["fy"]), float(intrinsics["ppy"])],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def dist_coeffs_from_intrinsics(intrinsics: Dict[str, Any]) -> np.ndarray:
    coeffs = intrinsics.get("coeffs") or intrinsics.get("distortion_coeffs") or intrinsics.get("distortion")
    if coeffs is None:
        return np.zeros((5, 1), dtype=np.float64)
    array = np.asarray(coeffs, dtype=np.float64).reshape(-1, 1)
    if array.size == 0:
        return np.zeros((5, 1), dtype=np.float64)
    return array


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a printable ChArUco calibration target.")
    parser.add_argument("--output", default="charuco_board.png")
    parser.add_argument("--dictionary", default=DEFAULT_BOARD_CONFIG["dictionary"])
    parser.add_argument("--squares-x", type=int, default=DEFAULT_BOARD_CONFIG["squares_x"])
    parser.add_argument("--squares-y", type=int, default=DEFAULT_BOARD_CONFIG["squares_y"])
    parser.add_argument("--square-length-m", type=float, default=DEFAULT_BOARD_CONFIG["square_length_m"])
    parser.add_argument("--marker-length-m", type=float, default=DEFAULT_BOARD_CONFIG["marker_length_m"])
    parser.add_argument("--image-width", type=int, default=1800)
    parser.add_argument("--image-height", type=int, default=1200)
    parser.add_argument("--margin-px", type=int, default=40)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = normalize_board_config(
        None,
        dictionary=args.dictionary,
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length_m=args.square_length_m,
        marker_length_m=args.marker_length_m,
    )
    image = render_charuco_board(
        config,
        image_size=(args.image_width, args.image_height),
        margin_px=args.margin_px,
    )
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise RuntimeError(f"Failed to write {output}")
    print(f"Wrote {output}")
    print(config)


if __name__ == "__main__":
    main()
