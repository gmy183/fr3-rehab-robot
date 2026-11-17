import pytest

from rehab_robot_control.xhand_motion_contract import (
    ActiveGains,
    XHandMotionGateError,
    build_verified_sdk_command,
    load_active_gains,
    load_profile,
    plan_relative_motion,
    require_verified_gains,
    validate_active_joints,
    validate_positions,
)
from rehab_robot_control.xhand_state_mapper import (
    JOINT_NAMES,
)


def verified_candidate():
    return ActiveGains(
        mode=3,
        kp=225,
        ki=0,
        kd=0,
        verified=True,
    )


def test_current_candidate_values_are_recorded_but_gate_closed():
    profile = load_profile()
    gains = load_active_gains(profile)

    assert gains.mode == 3
    assert gains.kp == 225
    assert gains.ki == 0
    assert gains.kd == 0

    # Combined-motion production gate is intentionally
    # still closed at this stage.
    assert gains.verified is False


def test_current_candidate_is_rejected_by_formal_gate():
    gains = load_active_gains()

    with pytest.raises(XHandMotionGateError):
        require_verified_gains(gains)


def test_position_vector_requires_12_joints():
    result = validate_positions(
        [0.0] * 12
    )

    assert len(result) == 12

    with pytest.raises(XHandMotionGateError):
        validate_positions(
            [0.0] * 11
        )


def test_unknown_active_joint_is_rejected():
    with pytest.raises(XHandMotionGateError):
        validate_active_joints(
            ["not_a_joint"]
        )


def test_fixed_or_excluded_joint_is_rejected():
    profile = load_profile()

    forbidden = (
        list(profile["fixed_joints"])
        + list(profile["excluded_joints"])
    )

    assert forbidden

    with pytest.raises(XHandMotionGateError):
        validate_active_joints(
            [forbidden[0]],
            profile=profile,
        )


def test_index_joint1_relative_plan():
    current = [0.0] * 12

    plan = plan_relative_motion(
        current,
        active_joints=["index_joint1"],
        delta_rad=0.005,
        effort_limit_ma=150,
    )

    index = JOINT_NAMES.index(
        "index_joint1"
    )

    assert (
        plan["target_positions"][index]
        == pytest.approx(0.005)
    )

    for i in range(12):
        if i != index:
            assert (
                plan["target_positions"][i]
                == pytest.approx(0.0)
            )

    assert plan["effort_limit_ma"] == 150


def test_first_stage_delta_cannot_exceed_0p005():
    with pytest.raises(XHandMotionGateError):
        plan_relative_motion(
            [0.0] * 12,
            active_joints=["index_joint1"],
            delta_rad=0.0051,
            effort_limit_ma=150,
        )


def test_effort_limit_cannot_exceed_220():
    with pytest.raises(XHandMotionGateError):
        plan_relative_motion(
            [0.0] * 12,
            active_joints=["index_joint1"],
            delta_rad=0.005,
            effort_limit_ma=221,
        )


def test_verified_command_activates_only_requested_joint():
    plan = plan_relative_motion(
        [0.0] * 12,
        active_joints=["index_joint1"],
        delta_rad=0.005,
        effort_limit_ma=150,
    )

    command = build_verified_sdk_command(
        plan,
        gains=verified_candidate(),
    )

    target_index = JOINT_NAMES.index(
        "index_joint1"
    )

    assert len(
        command.finger_command
    ) == 12

    for i, finger in enumerate(
        command.finger_command
    ):
        if i == target_index:
            assert int(finger.mode) == 3
            assert int(finger.kp) == 225
            assert int(finger.ki) == 0
            assert int(finger.kd) == 0
            assert int(finger.tor_max) == 150
            assert float(
                finger.position
            ) == pytest.approx(0.005)

        else:
            assert int(finger.mode) == 0
            assert int(finger.kp) == 0
            assert int(finger.ki) == 0
            assert int(finger.kd) == 0
            assert int(finger.tor_max) == 0
            assert float(
                finger.position
            ) == pytest.approx(0.0)
