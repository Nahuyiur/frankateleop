"""Run the delayed NAS publisher with ``python -m franka_sync``."""

from .nas_sync import main


if __name__ == "__main__":
    raise SystemExit(main())
