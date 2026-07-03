"""JSON and sample loading helpers for calibration sessions."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from .geometry import matrix_from_list, matrix_to_list


SCHEMA_VERSION = "frankateleop_calibration_v1"


def read_json(path: Path) -> Dict[str, Any]:
    with Path(path).expanduser().open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")


def timestamp_slug() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def create_session_dir(output_root: Path, camera_name: str) -> Path:
    output_root = Path(output_root).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    base = output_root / f"{camera_name}_eye_to_hand_{timestamp_slug()}"
    if not base.exists():
        base.mkdir(parents=True)
        return base
    suffix = 1
    while True:
        candidate = output_root / f"{base.name}_{suffix:02d}"
        if not candidate.exists():
            candidate.mkdir(parents=True)
            return candidate
        suffix += 1


def next_sample_dir(session_dir: Path) -> Path:
    samples_dir = Path(session_dir).expanduser() / "samples"
    samples_dir.mkdir(parents=True, exist_ok=True)
    existing = []
    for child in samples_dir.iterdir():
        if child.is_dir() and child.name.startswith("sample_"):
            try:
                existing.append(int(child.name.split("_", 1)[1]))
            except ValueError:
                pass
    index = max(existing, default=-1) + 1
    path = samples_dir / f"sample_{index:06d}"
    path.mkdir(parents=True)
    return path


def sample_id_from_dir(sample_dir: Path) -> str:
    return Path(sample_dir).name


def load_sample_metadata(session_dir: Path) -> List[Dict[str, Any]]:
    samples = []
    samples_dir = Path(session_dir).expanduser() / "samples"
    if not samples_dir.exists():
        return samples
    for metadata_path in sorted(samples_dir.glob("sample_*/metadata.json")):
        metadata = read_json(metadata_path)
        metadata["_sample_dir"] = str(metadata_path.parent)
        metadata["_sample_id"] = metadata_path.parent.name
        samples.append(metadata)
    return samples


def transform_payload(transform, frame_id: str, child_frame_id: str) -> Dict[str, Any]:
    return {
        "frame_id": frame_id,
        "child_frame_id": child_frame_id,
        "matrix": matrix_to_list(transform),
    }


def load_transform_payload(payload: Dict[str, Any], key: Optional[str] = None):
    value = payload[key] if key else payload
    if isinstance(value, dict) and "matrix" in value:
        value = value["matrix"]
    return matrix_from_list(value, name=key or "transform")
