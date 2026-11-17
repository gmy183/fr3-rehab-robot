from __future__ import annotations

from pathlib import Path
import math

import yaml

from franka_msgs.action import PTPMotion


PTP_ACTION_TYPE = "franka_msgs/action/PTPMotion"

JOINT_NAMES = tuple(
    f"fr3_joint{i}"
    for i in range(1, 8)
)


class FR3PTPContractError(ValueError):
    pass


def load_joint_limits(
    path: Path | None = None,
):
    if path is None:
        path = (
            Path.home()
            / "rehab_robot_ws/config/"
              "fr3_joint_limits_resolved.yaml"
        )

    data = yaml.safe_load(
        path.read_text(
            encoding="utf-8"
        )
    )

    joints = data["joints"]

    for name in JOINT_NAMES:
        if name not in joints:
            raise FR3PTPContractError(
                f"missing joint limit: {name}"
            )

    return joints


def validate_target(
    target,
    *,
    limits=None,
):
    values = tuple(
        float(v)
        for v in target
    )

    if len(values) != 7:
        raise FR3PTPContractError(
            "FR3 target must contain exactly 7 joints"
        )

    if not all(
        math.isfinite(v)
        for v in values
    ):
        raise FR3PTPContractError(
            "FR3 target contains non-finite value"
        )

    if limits is None:
        limits = load_joint_limits()

    for index, (
        name,
        value,
    ) in enumerate(
        zip(
            JOINT_NAMES,
            values,
        )
    ):
        item = limits[name]

        lower = float(
            item["safe_lower_rad"]
        )
        upper = float(
            item["safe_upper_rad"]
        )

        if not lower <= value <= upper:
            raise FR3PTPContractError(
                f"{name} target {value:.6f} "
                f"outside safe "
                f"[{lower:.6f}, {upper:.6f}]"
            )

    return values


def relative_target(
    current,
    *,
    joint_index: int,
    delta_rad: float,
    limits=None,
):
    base = tuple(
        float(v)
        for v in current
    )

    if len(base) != 7:
        raise FR3PTPContractError(
            "current FR3 state must contain 7 joints"
        )

    if not 0 <= joint_index < 7:
        raise FR3PTPContractError(
            "joint_index must be in [0, 6]"
        )

    target = list(base)

    target[joint_index] += float(
        delta_rad
    )

    return validate_target(
        target,
        limits=limits,
    )


def build_goal(
    target,
    *,
    maximum_velocity_rad_s: float = 0.020,
    goal_tolerance: float = 0.001,
    limits=None,
):
    target = validate_target(
        target,
        limits=limits,
    )

    velocity = float(
        maximum_velocity_rad_s
    )

    if not 0.0 < velocity <= 0.020:
        raise FR3PTPContractError(
            "maximum velocity must be in (0, 0.020]"
        )

    tolerance = float(
        goal_tolerance
    )

    if tolerance <= 0.0:
        raise FR3PTPContractError(
            "goal_tolerance must be > 0"
        )

    goal = PTPMotion.Goal()

    goal.goal_joint_configuration = list(
        target
    )

    goal.maximum_joint_velocities = [
        velocity
    ] * 7

    goal.goal_tolerance = tolerance

    return goal


def discover_unique_ptp_action(
    action_names_and_types,
):
    matches = []

    for name, types in action_names_and_types:
        if PTP_ACTION_TYPE in types:
            matches.append(name)

    if len(matches) == 0:
        raise FR3PTPContractError(
            "no PTPMotion action server discovered"
        )

    if len(matches) > 1:
        raise FR3PTPContractError(
            "multiple PTPMotion action servers discovered: "
            + ", ".join(matches)
        )

    return matches[0]
