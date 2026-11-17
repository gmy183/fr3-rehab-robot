from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from .fr3_ptp_contract import (
    build_goal,
    load_joint_limits,
    relative_target,
)
from .xhand_motion_contract import (
    XHandMotionGateError,
    load_active_gains,
    load_profile,
    plan_relative_motion,
    require_verified_gains,
)


class EmptyGraspExecutionBlocked(RuntimeError):
    pass


def _safe_fr3_midpoint():
    limits = load_joint_limits()

    values = []

    for i in range(1, 8):
        item = limits[
            f"fr3_joint{i}"
        ]

        values.append(
            (
                float(item["safe_lower_rad"])
                + float(item["safe_upper_rad"])
            )
            / 2.0
        )

    return tuple(values)


def load_sequence():
    path = (
        Path.home()
        / "rehab_robot_ws/config/"
          "empty_grasp_sequence.yaml"
    )

    return yaml.safe_load(
        path.read_text(encoding="utf-8")
    )


def build_dry_run_plan():
    seq = load_sequence()
    profile = load_profile()

    fr3_cfg = seq["fr3"]

    fr3_start = _safe_fr3_midpoint()

    fr3_pregrasp = relative_target(
        fr3_start,
        joint_index=0,
        delta_rad=float(
            fr3_cfg[
                "pregrasp_delta_joint1_rad"
            ]
        ),
    )

    fr3_grasp = relative_target(
        fr3_pregrasp,
        joint_index=0,
        delta_rad=float(
            fr3_cfg[
                "grasp_delta_joint1_rad"
            ]
        ),
    )

    fr3_rehab = relative_target(
        fr3_grasp,
        joint_index=0,
        delta_rad=float(
            fr3_cfg[
                "rehab_stage_1"
            ][
                "delta_joint1_rad"
            ]
        ),
    )

    fr3_goals = [
        build_goal(fr3_pregrasp),
        build_goal(fr3_grasp),
        build_goal(fr3_rehab),
    ]

    xhand_start = [0.0] * 12

    single_cfg = profile[
        "single_joint"
    ]

    single = plan_relative_motion(
        xhand_start,
        active_joints=[
            single_cfg["joint"]
        ],
        delta_rad=single_cfg[
            "delta_rad"
        ],
        effort_limit_ma=single_cfg[
            "effort_limit_ma"
        ],
        profile=profile,
    )

    whole_cfg = profile[
        "safe_whole_hand_group"
    ]

    whole = plan_relative_motion(
        xhand_start,
        active_joints=whole_cfg[
            "active_joints"
        ],
        delta_rad=whole_cfg[
            "delta_rad"
        ],
        effort_limit_ma=whole_cfg[
            "effort_limit_ma"
        ],
        profile=profile,
    )

    return {
        "sequence": seq["sequence"],
        "fr3_start": fr3_start,
        "fr3_goals": fr3_goals,
        "xhand_single": single,
        "xhand_whole": whole,
    }


def require_real_execution_gate():
    profile = load_profile()
    gains = load_active_gains(
        profile
    )

    try:
        require_verified_gains(
            gains
        )
    except XHandMotionGateError as exc:
        raise EmptyGraspExecutionBlocked(
            str(exc)
        ) from exc


def main(args=None):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    ns = parser.parse_args(args)

    plan = build_dry_run_plan()

    print("=" * 60)
    print("FR3 + XHand Empty-Grasp software plan")
    print("=" * 60)

    print()
    print("sequence:")

    for index, stage in enumerate(
        plan["sequence"],
        1,
    ):
        print(
            f"  {index:02d}. {stage}"
        )

    print()
    print("FR3:")
    print(
        "  software limits: loaded"
    )
    print(
        "  max velocity: 0.020 rad/s"
    )
    print(
        "  relative PTP goals: 3"
    )

    for i, goal in enumerate(
        plan["fr3_goals"],
        1,
    ):
        print(
            f"  goal{i}: "
            f"{list(goal.goal_joint_configuration)}"
        )

    print()
    print("XHand single:")
    print(
        "  joints =",
        list(
            plan[
                "xhand_single"
            ][
                "active_joints"
            ]
        ),
    )
    print(
        "  delta =",
        plan[
            "xhand_single"
        ][
            "delta_rad"
        ],
    )
    print(
        "  limit =",
        plan[
            "xhand_single"
        ][
            "effort_limit_ma"
        ],
        "mA",
    )

    print()
    print("XHand safe whole-hand group:")
    print(
        "  joints =",
        list(
            plan[
                "xhand_whole"
            ][
                "active_joints"
            ]
        ),
    )
    print(
        "  delta =",
        plan[
            "xhand_whole"
        ][
            "delta_rad"
        ],
    )
    print(
        "  limit =",
        plan[
            "xhand_whole"
        ][
            "effort_limit_ma"
        ],
        "mA",
    )

    if ns.execute:
        print()
        print(
            "[CHECK] real execution gate"
        )

        require_real_execution_gate()

        # No real execution implementation is reached
        # in the current stage.
        raise EmptyGraspExecutionBlocked(
            "real execution backend is not enabled"
        )

    print()
    print(
        "[PASS] DRY-RUN ONLY"
    )
    print(
        "No ROS Action goal sent."
    )
    print(
        "No XHand command sent."
    )
    print(
        "No hardware opened."
    )


if __name__ == "__main__":
    main()
