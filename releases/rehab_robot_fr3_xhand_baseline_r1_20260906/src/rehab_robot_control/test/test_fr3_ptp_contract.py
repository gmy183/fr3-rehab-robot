import math

import pytest

from rehab_robot_control.fr3_ptp_contract import (
    FR3PTPContractError,
    JOINT_NAMES,
    PTP_ACTION_TYPE,
    build_goal,
    discover_unique_ptp_action,
    load_joint_limits,
    relative_target,
    validate_target,
)


def safe_midpoint():
    limits = load_joint_limits()

    return tuple(
        (
            float(limits[name]["safe_lower_rad"])
            + float(limits[name]["safe_upper_rad"])
        )
        / 2.0
        for name in JOINT_NAMES
    )


def test_joint_limit_file_has_all_fr3_joints():
    limits = load_joint_limits()

    assert tuple(JOINT_NAMES) == tuple(
        f"fr3_joint{i}"
        for i in range(1, 8)
    )

    for name in JOINT_NAMES:
        assert name in limits

        lower = float(
            limits[name]["safe_lower_rad"]
        )
        upper = float(
            limits[name]["safe_upper_rad"]
        )

        assert math.isfinite(lower)
        assert math.isfinite(upper)
        assert lower < upper


def test_validate_target_accepts_safe_midpoint():
    q = safe_midpoint()

    assert validate_target(q) == q


def test_validate_target_rejects_wrong_length():
    with pytest.raises(FR3PTPContractError):
        validate_target([0.0] * 6)


def test_validate_target_rejects_nonfinite():
    q = list(safe_midpoint())
    q[2] = float("nan")

    with pytest.raises(FR3PTPContractError):
        validate_target(q)


def test_validate_target_rejects_outside_safe_limit():
    limits = load_joint_limits()
    q = list(safe_midpoint())

    q[0] = (
        float(
            limits["fr3_joint1"]["safe_upper_rad"]
        )
        + 0.001
    )

    with pytest.raises(FR3PTPContractError):
        validate_target(q)


def test_relative_target_changes_only_requested_joint():
    q0 = safe_midpoint()

    q1 = relative_target(
        q0,
        joint_index=0,
        delta_rad=0.005,
    )

    assert q1[0] == pytest.approx(
        q0[0] + 0.005
    )

    assert q1[1:] == pytest.approx(
        q0[1:]
    )


def test_relative_target_rejects_bad_joint_index():
    with pytest.raises(FR3PTPContractError):
        relative_target(
            safe_midpoint(),
            joint_index=7,
            delta_rad=0.005,
        )


def test_build_goal_uses_conservative_velocity():
    q = safe_midpoint()

    goal = build_goal(
        q,
        maximum_velocity_rad_s=0.020,
        goal_tolerance=0.001,
    )

    assert list(
        goal.goal_joint_configuration
    ) == pytest.approx(q)

    assert list(
        goal.maximum_joint_velocities
    ) == pytest.approx(
        [0.020] * 7
    )

    assert goal.goal_tolerance == pytest.approx(
        0.001
    )


def test_build_goal_rejects_velocity_above_project_limit():
    with pytest.raises(FR3PTPContractError):
        build_goal(
            safe_midpoint(),
            maximum_velocity_rad_s=0.021,
        )


def test_discover_unique_ptp_action():
    graph = [
        (
            "/fr3/ptp_motion",
            [PTP_ACTION_TYPE],
        )
    ]

    assert (
        discover_unique_ptp_action(graph)
        == "/fr3/ptp_motion"
    )


def test_discover_ptp_action_rejects_none():
    with pytest.raises(FR3PTPContractError):
        discover_unique_ptp_action([])


def test_discover_ptp_action_rejects_multiple():
    graph = [
        ("/a", [PTP_ACTION_TYPE]),
        ("/b", [PTP_ACTION_TYPE]),
    ]

    with pytest.raises(FR3PTPContractError):
        discover_unique_ptp_action(graph)
