from __future__ import annotations

from dataclasses import dataclass, asdict
from enum import Enum
from typing import Any, Dict, List, Optional


class RunMode(str, Enum):
    CHECK = "check"
    DRY_RUN = "dry-run"
    FR3_TEST = "fr3-test"
    REAL = "real"


class TaskState(str, Enum):
    READY = "READY"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    REHAB_MOTION = "REHAB_MOTION"
    RETURN = "RETURN"
    RELEASE = "RELEASE"
    FINISH = "FINISH"
    FAULT = "FAULT"


@dataclass
class StepRecord:
    state: str
    ok: bool
    message: str
    data: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class TaskResult:
    mode: str
    ok: bool
    final_state: str
    fault_latched: bool
    steps: List[StepRecord]
    artifacts: Dict[str, str]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "ok": self.ok,
            "final_state": self.final_state,
            "fault_latched": self.fault_latched,
            "steps": [s.to_dict() for s in self.steps],
            "artifacts": dict(self.artifacts),
        }
