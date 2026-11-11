"""Shared, explicit hardware configuration for inference and execution."""
import hashlib
import json
from pathlib import Path

import numpy as np

ARMS = ("left", "right")


def load_profile(path=None):
    path = Path(path) if path else Path(__file__).resolve().parent / "hardware_profile.json"
    profile = json.loads(path.read_text())
    tools = profile.get("tool_transforms")
    if not isinstance(tools, dict) or set(tools) != set(ARMS):
        raise ValueError("Exactly left/right tool transforms are required")
    for arm in ARMS:
        p = np.asarray(tools[arm]["translation_m"], dtype=float)
        q = np.asarray(tools[arm]["quaternion_xyzw"], dtype=float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise ValueError("Invalid tool transform for " + arm)
        if np.linalg.norm(p) > 0.3 or not np.isclose(np.linalg.norm(q), 1.0, atol=1e-6):
            raise ValueError("Invalid tool translation or quaternion for " + arm)
    encoded = json.dumps(profile, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return profile, hashlib.sha256(encoded).hexdigest()
