import pytest

from rehab_robot_tasks.rehab_task_contract import (
    default_task_config,
)

from rehab_robot_tasks.rehab_task_state_machine import (
    NORMAL_SEQUENCE,
    RehabTaskActions,
    RehabTaskExecutionError,
    RehabTaskState,
    RehabTaskStateMachine,
)


def build_machine(
    *,
    fail_stage=None,
):
    calls = []
    faults = []

    def action(
        name,
    ):
        def run():
            calls.append(
                name
            )

            if name == fail_stage:
                raise RuntimeError(
                    f"synthetic {name} failure"
                )

        return run

    actions = RehabTaskActions(
        approach=action(
            "approach"
        ),
        grasp=action(
            "grasp"
        ),
        rehab_motion=action(
            "rehab_motion"
        ),
        return_motion=action(
            "return"
        ),
        release=action(
            "release"
        ),
        on_fault=lambda state, exc: (
            faults.append(
                (
                    state,
                    str(exc),
                )
            )
        ),
    )

    machine = RehabTaskStateMachine(
        default_task_config(),
        actions,
    )

    return (
        machine,
        calls,
        faults,
    )


def test_normal_sequence_reaches_finish():
    machine, _, _ = build_machine()

    result = machine.run()

    assert result == RehabTaskState.FINISH

    assert tuple(
        machine.history
    ) == NORMAL_SEQUENCE


def test_normal_action_order():
    machine, calls, _ = (
        build_machine()
    )

    machine.run()

    assert calls == [
        "approach",
        "grasp",
        "rehab_motion",
        "return",
        "release",
    ]


def test_approach_fault_stops_task():
    machine, calls, faults = (
        build_machine(
            fail_stage="approach"
        )
    )

    with pytest.raises(
        RehabTaskExecutionError
    ):
        machine.run()

    assert calls == [
        "approach"
    ]

    assert (
        machine.state
        == RehabTaskState.FAULT
    )

    assert len(faults) == 1


def test_grasp_fault_prevents_rehab_motion():
    machine, calls, _ = (
        build_machine(
            fail_stage="grasp"
        )
    )

    with pytest.raises(
        RehabTaskExecutionError
    ):
        machine.run()

    assert calls == [
        "approach",
        "grasp",
    ]


def test_rehab_fault_does_not_auto_return():
    machine, calls, _ = (
        build_machine(
            fail_stage="rehab_motion"
        )
    )

    with pytest.raises(
        RehabTaskExecutionError
    ):
        machine.run()

    assert calls == [
        "approach",
        "grasp",
        "rehab_motion",
    ]

    assert "return" not in calls
    assert "release" not in calls


def test_return_fault_does_not_continue_to_release():
    machine, calls, _ = (
        build_machine(
            fail_stage="return"
        )
    )

    with pytest.raises(
        RehabTaskExecutionError
    ):
        machine.run()

    assert calls == [
        "approach",
        "grasp",
        "rehab_motion",
        "return",
    ]

    assert "release" not in calls
