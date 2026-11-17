from __future__ import annotations

from .contracts import TaskState


class StateTransitionError(RuntimeError):
    pass


_ALLOWED = {
    TaskState.READY: {TaskState.APPROACH, TaskState.FAULT},
    TaskState.APPROACH: {TaskState.GRASP, TaskState.FAULT},
    TaskState.GRASP: {TaskState.REHAB_MOTION, TaskState.FAULT},
    TaskState.REHAB_MOTION: {TaskState.RETURN, TaskState.FAULT},
    TaskState.RETURN: {TaskState.RELEASE, TaskState.FAULT},
    TaskState.RELEASE: {TaskState.FINISH, TaskState.FAULT},
    TaskState.FINISH: set(),
    TaskState.FAULT: set(),
}


class RehabStateMachine:
    def __init__(self):
        self.state = TaskState.READY
        self.fault_latched = False

    def transition(self, target: TaskState) -> None:
        if self.fault_latched:
            raise StateTransitionError("FAULT is latched; no automatic continuation is allowed")
        if target not in _ALLOWED[self.state]:
            raise StateTransitionError(f"illegal transition: {self.state.value} -> {target.value}")
        self.state = target

    def fault(self) -> None:
        self.state = TaskState.FAULT
        self.fault_latched = True
