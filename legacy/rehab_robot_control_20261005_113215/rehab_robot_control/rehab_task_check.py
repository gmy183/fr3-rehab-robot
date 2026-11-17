from __future__ import annotations

from rehab_robot_control.rehab_task_contract import (
    default_task_config,
)

from rehab_robot_control.rehab_task_state_machine import (
    RehabTaskActions,
    RehabTaskStateMachine,
)


def make_action(
    name,
):
    def action():
        print(
            f"[DRY ACTION] {name}"
        )

    return action


def main():

    config = default_task_config()

    print("=" * 60)
    print("FR3 + XHand REHAB TASK V1")
    print("HIGH-LEVEL TASK CHECK / NO HARDWARE")
    print("=" * 60)

    print(
        f"task_name      = {config.task_name}"
    )

    print(
        f"motion_profile = {config.motion_profile}"
    )

    print(
        f"repetitions    = {config.repetitions}"
    )

    print(
        f"speed_scale    = {config.speed_scale}"
    )

    print(
        f"grasp_profile  = {config.grasp_profile}"
    )

    print(
        f"record_data    = {config.record_data}"
    )

    print()

    actions = RehabTaskActions(
        approach=make_action(
            "APPROACH"
        ),
        grasp=make_action(
            "GRASP"
        ),
        rehab_motion=make_action(
            "REHAB_MOTION"
        ),
        return_motion=make_action(
            "RETURN"
        ),
        release=make_action(
            "RELEASE"
        ),
        on_fault=lambda state, exc: print(
            "[DRY FAULT] "
            f"state={state.value} "
            f"error={exc}"
        ),
    )

    machine = RehabTaskStateMachine(
        config,
        actions,
    )

    result = machine.run()

    print()
    print(
        "history =",
        [
            state.value
            for state in machine.history
        ],
    )

    print()
    print(
        "[PASS] REHAB TASK STATE MACHINE"
    )

    print(
        f"final_state={result.value}"
    )

    print()
    print("No FR3 ActionClient created.")
    print("No FR3 PTP goal sent.")
    print("No XHand EtherCAT opened.")
    print("No XHand command sent.")

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
