#!/usr/bin/env python3

from pathlib import Path
import json

from rehab_robot_control.empty_grasp_execution_state import (
    load_state,
    next_stage,
)


STATE = (
    Path.home()
    / "rehab_robot_ws"
    / "runtime_state"
    / "empty_grasp_execution.json"
)


def main():

    print(
        "state_file =",
        STATE,
    )

    if not STATE.exists():

        print(
            "status = NO ACTIVE RUN"
        )

        return 0

    state = load_state(
        STATE
    )

    print(
        json.dumps(
            state,
            indent=2,
            sort_keys=True,
        )
    )

    print()

    print(
        "next_stage =",
        next_stage(
            state
        ),
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
