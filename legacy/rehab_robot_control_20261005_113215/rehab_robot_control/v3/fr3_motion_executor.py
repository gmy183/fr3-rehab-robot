from __future__ import annotations

from typing import Any, Dict

from .config import V3Config


class FR3ExecutionError(RuntimeError):
    pass


class FR3MotionExecutor:
    """Execution boundary for V3.

    The current production build intentionally leaves real execution hard-locked
    until the effort-JTC tracking backend is validated. Planning and task logic do
    not depend on the concrete executor implementation.
    """

    def __init__(self, config: V3Config):
        self.config = config

    def execute(self, stage_plan: Dict[str, Any]) -> None:
        fr3 = self.config.hardware.get("fr3", {})
        if not bool(fr3.get("real_execution_enabled", False)):
            raise FR3ExecutionError("FR3 real execution gate is closed")
        if not bool(fr3.get("tracking_backend_validated", False)):
            raise FR3ExecutionError("FR3 tracking backend is not validated")
        raise FR3ExecutionError(
            "V3 real FR3 backend intentionally not commissioned in this release; "
            "wire the validated executor here after tracking validation"
        )
