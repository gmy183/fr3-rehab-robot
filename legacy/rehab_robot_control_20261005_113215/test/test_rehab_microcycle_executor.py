import math

from rehab_robot_control.empty_grasp_executor import (
    load_configs,
)

from rehab_robot_control.rehab_microcycle_executor import (
    FR3_DELTA_RAD,
    FR3_VELOCITY_RAD_S,
    build_cycle_targets,
    validate_cycle_contract,
)


Q0 = (
    0.03,
    -0.65,
    0.00,
    -2.54,
    -0.02,
    1.56,
    1.58,
)


def close(a, b):
    return math.isclose(
        float(a),
        float(b),
        rel_tol=0.0,
        abs_tol=1e-12,
    )


def test_cycle_outbound_is_exactly_joint1_plus_0p005():
    q0, q_out, _ = build_cycle_targets(Q0)

    assert close(
        q_out[0] - q0[0],
        0.005,
    )

    for i in range(1, 7):
        assert close(
            q_out[i],
            q0[i],
        )


def test_cycle_return_target_is_original_q0():
    q0, _, q_return = build_cycle_targets(Q0)

    assert q_return == q0


def test_cycle_motion_constants_are_bounded():
    assert close(
        FR3_DELTA_RAD,
        0.005,
    )

    assert (
        0.0
        < FR3_VELOCITY_RAD_S
        <= 0.020
    )


def test_current_micro_contract_is_accepted():
    profile, sequence, micro = load_configs()

    active = validate_cycle_contract(
        profile,
        sequence,
        micro,
    )

    assert len(active) == 9
    assert int(
        micro["effort_limit_ma"]
    ) == 150

    assert close(
        micro["refresh_hz"],
        20.0,
    )


def test_fault_auto_return_and_recovery_remain_disabled():
    _, sequence, _ = load_configs()

    assert (
        sequence["fr3"][
            "automatic_recovery_on_fault"
        ]
        is False
    )

    assert (
        sequence["fr3"][
            "automatic_return_on_fault"
        ]
        is False
    )
