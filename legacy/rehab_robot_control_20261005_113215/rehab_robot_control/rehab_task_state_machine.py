from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

from rehab_robot_control.rehab_task_contract import (
    RehabTaskConfig,
    validate_task_config,
)


class RehabTaskState(str, Enum):
    READY = "READY"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    REHAB_MOTION = "REHAB_MOTION"
    RETURN = "RETURN"
    RELEASE = "RELEASE"
    FINISH = "FINISH"
    FAULT = "FAULT"


class RehabTaskExecutionError(RuntimeError):
    pass


@dataclass
class RehabTaskActions:
    """
    Hardware-independent action interface.

    The state machine owns sequencing.
    The future hardware layer owns FR3/XHand execution.
    """

    approach: Callable[[], None]
    grasp: Callable[[], None]
    rehab_motion: Callable[[], None]
    return_motion: Callable[[], None]
    release: Callable[[], None]
    on_fault: Callable[
        [RehabTaskState, Exception],
        None,
    ]


NORMAL_SEQUENCE = (
    RehabTaskState.READY,
    RehabTaskState.APPROACH,
    RehabTaskState.GRASP,
    RehabTaskState.REHAB_MOTION,
    RehabTaskState.RETURN,
    RehabTaskState.RELEASE,
    RehabTaskState.FINISH,
)


class RehabTaskStateMachine:

    def __init__(
        self,
        config: RehabTaskConfig,
        actions: RehabTaskActions,
    ):
        self.config = validate_task_config(
            config
        )

        self.actions = actions

        self.state = RehabTaskState.READY

        self.history = [
            RehabTaskState.READY
        ]

        self.last_error: Optional[
            Exception
        ] = None

    def _enter(
        self,
        state: RehabTaskState,
    ):
        self.state = state
        self.history.append(
            state
        )

        print(
            f"[STATE] {state.value}"
        )

    def _run_stage(
        self,
        target_state: RehabTaskState,
        action: Callable[[], None],
    ):
        try:
            action()

        except Exception as exc:
            failed_from = self.state

            self.last_error = exc

            try:
                self.actions.on_fault(
                    failed_from,
                    exc,
                )

            finally:
                self._enter(
                    RehabTaskState.FAULT
                )

            raise RehabTaskExecutionError(
                f"rehab task fault during "
                f"{target_state.value}: {exc}"
            ) from exc

        self._enter(
            target_state
        )

    def run(self):
        if self.state != RehabTaskState.READY:
            raise RehabTaskExecutionError(
                "task may only start from READY"
            )

        print(
            "[STATE] READY"
        )

        self._run_stage(
            RehabTaskState.APPROACH,
            self.actions.approach,
        )

        self._run_stage(
            RehabTaskState.GRASP,
            self.actions.grasp,
        )

        self._run_stage(
            RehabTaskState.REHAB_MOTION,
            self.actions.rehab_motion,
        )

        self._run_stage(
            RehabTaskState.RETURN,
            self.actions.return_motion,
        )

        self._run_stage(
            RehabTaskState.RELEASE,
            self.actions.release,
        )

        self._enter(
            RehabTaskState.FINISH
        )

        return RehabTaskState.FINISH
