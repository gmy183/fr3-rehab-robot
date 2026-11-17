from __future__ import annotations

from moveit_msgs.srv import (
    GetCartesianPath,
)

from control_msgs.action import (
    FollowJointTrajectory,
)

from rehab_robot_control.rehab_cartesian_contract import (
    RehabCartesianContractError,
    RelativeCartesianMove,
    load_cartesian_config,
    require_real_cartesian_gate,
    validate_relative_move,
)

from rehab_robot_control.rehab_moveit_topology import (
    discover_fr3_moveit_topology,
)


def main():

    config = load_cartesian_config()

    topology = (
        discover_fr3_moveit_topology()
    )

    print("=" * 60)
    print(
        "FR3 CARTESIAN REHAB MOTION LAYER V1"
    )
    print(
        "SOFTWARE / TOPOLOGY CHECK"
    )
    print("=" * 60)

    print()
    print(
        "MoveIt package =",
        topology.package_share,
    )

    print(
        "planning group =",
        topology.planning_group,
    )

    print(
        "base link      =",
        topology.base_link,
    )

    print(
        "MoveIt tip     =",
        topology.tip_link,
    )

    print(
        "controller     =",
        topology.trajectory_controller,
    )

    print(
        "action ns      =",
        topology.trajectory_action_ns,
    )

    print()
    print(
        "rehab TCP      =",
        config[
            "robot"
        ][
            "task_tool_frame"
        ],
    )

    print(
        "TCP calibrated =",
        config[
            "robot"
        ][
            "tool_frame_calibrated"
        ],
    )

    print()
    print(
        "[PASS] moveit_msgs/GetCartesianPath "
        "available"
    )

    assert (
        GetCartesianPath
        is not None
    )

    print(
        "[PASS] control_msgs/"
        "FollowJointTrajectory available"
    )

    assert (
        FollowJointTrajectory
        is not None
    )

    # Planning-contract examples only.
    # No physical motion is implied.
    examples = {
        "translation_5mm": (
            RelativeCartesianMove(
                dz=0.005,
            )
        ),
        "rotation_5deg": (
            RelativeCartesianMove(
                dpitch=(
                    5.0
                    * 3.141592653589793
                    / 180.0
                ),
            )
        ),
    }

    print()
    print(
        "planning-contract examples:"
    )

    for name, move in (
        examples.items()
    ):
        validate_relative_move(
            move,
            config=config,
        )

        print(
            f"  {name}: PASS"
        )

    print()
    print(
        "Real execution gate:"
    )

    try:
        require_real_cartesian_gate(
            config=config,
        )

    except RehabCartesianContractError as exc:
        print(
            "[PASS] CLOSED:",
            exc,
        )

    else:
        raise RuntimeError(
            "real Cartesian execution "
            "must still be closed"
        )

    print()
    print(
        "[PASS] CARTESIAN REHAB "
        "ARCHITECTURE READY"
    )

    print()
    print(
        "NO MoveIt plan was executed."
    )
    print(
        "NO trajectory controller "
        "was activated."
    )
    print(
        "NO FR3 command was sent."
    )
    print(
        "NO XHand command was sent."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
