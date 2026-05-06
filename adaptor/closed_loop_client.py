#!/usr/bin/env python3
from __future__ import annotations

import asyncio

try:
    from .franka_eraser_closed_loop_client import main
except ImportError:
    from franka_eraser_closed_loop_client import main


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
