from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .config import V3Config
from .contracts import RunMode
from .task_manager import RehabTaskManager


def build_parser():
    p = argparse.ArgumentParser(
        description="康复机械臂 V3 formal task entrypoint"
    )
    p.add_argument(
        "--mode",
        choices=[m.value for m in RunMode],
        required=True,
        help="check: read-only baseline; dry-run: plan/audit full task without motion; real: hard-gated real task",
    )
    p.add_argument(
        "--workspace",
        default=str(Path.home() / "rehab_robot_ws"),
    )
    return p


def main() -> int:
    args = build_parser().parse_args()
    cfg = V3Config.load(Path(args.workspace))
    mode = RunMode(args.mode)
    print("=" * 72)
    print("REHAB ROBOT V3 FORMAL ENGINEERING ENTRYPOINT")
    print(f"mode={mode.value}")
    print("policy=no-auto-retry / no-auto-return / no-auto-release")
    print("=" * 72)
    result = RehabTaskManager(cfg, mode).run()
    return 0 if result.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
