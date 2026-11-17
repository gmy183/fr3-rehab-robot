from __future__ import annotations

from .config import V3Config


class XHandExecutionError(RuntimeError):
    pass


class XHandExecutor:
    def __init__(self, config: V3Config, dry_run: bool):
        self.config = config
        self.dry_run = dry_run

    def grasp(self) -> str:
        if self.dry_run:
            return "DRY-RUN grasp simulated; no XHand command sent"
        xh = self.config.hardware.get("xhand", {})
        if not bool(xh.get("combined_motion_enabled", False)):
            raise XHandExecutionError("XHand combined-motion gate is closed")
        if not bool(xh.get("real_grasp_enabled", False)):
            raise XHandExecutionError("XHand real grasp gate is closed")
        raise XHandExecutionError("real XHand task binding not commissioned in V3 release")

    def release(self) -> str:
        if self.dry_run:
            return "DRY-RUN release simulated; no XHand command sent"
        xh = self.config.hardware.get("xhand", {})
        if not bool(xh.get("combined_motion_enabled", False)):
            raise XHandExecutionError("XHand combined-motion gate is closed")
        if not bool(xh.get("real_grasp_enabled", False)):
            raise XHandExecutionError("XHand real grasp gate is closed")
        raise XHandExecutionError("real XHand task binding not commissioned in V3 release")
