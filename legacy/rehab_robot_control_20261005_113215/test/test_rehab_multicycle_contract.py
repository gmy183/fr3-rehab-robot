import pytest

from rehab_robot_control.rehab_multicycle_contract import (
    RehabMultiCycleContractError,
    assert_cycle_start,
    build_multicycle_plan,
    validate_multicycle_parameters,
)


Q0 = (
    0.04,
    -0.65,
    0.00,
    -2.54,
    -0.02,
    1.56,
    1.58,
)


def test_cycle_count_1():
    cycles, _ = validate_multicycle_parameters(
        1,
        0.5,
    )

    assert cycles == 1


def test_cycle_count_3():
    cycles, _ = validate_multicycle_parameters(
        3,
        0.5,
    )

    assert cycles == 3


def test_cycle_count_5():
    cycles, _ = validate_multicycle_parameters(
        5,
        0.5,
    )

    assert cycles == 5


def test_invalid_cycle_count_is_rejected():
    with pytest.raises(
        RehabMultiCycleContractError
    ):
        validate_multicycle_parameters(
            2,
            0.5,
        )


def test_invalid_hold_is_rejected():
    with pytest.raises(
        RehabMultiCycleContractError
    ):
        validate_multicycle_parameters(
            3,
            5.1,
        )


def test_outbound_is_only_joint1_plus_0p005():
    plan = build_multicycle_plan(
        Q0,
        cycles=3,
        hold_s=0.5,
    )

    q0 = plan["q0"]
    q_out = plan["q_out"]

    assert abs(
        (q_out[0] - q0[0])
        - 0.005
    ) < 1e-12

    for i in range(1, 7):
        assert abs(
            q_out[i] - q0[i]
        ) < 1e-12


def test_all_cycles_reuse_original_run_q0():
    plan = build_multicycle_plan(
        Q0,
        cycles=5,
        hold_s=0.5,
    )

    assert len(
        plan["stages"]
    ) == 5

    for stage in plan["stages"]:
        assert (
            stage["expected_start"]
            == plan["q0"]
        )

        assert (
            stage["return_target"]
            == plan["q0"]
        )


def test_cycle_start_guard():
    good = list(Q0)
    good[0] += 0.005

    error = assert_cycle_start(
        good,
        Q0,
    )

    assert abs(
        error - 0.005
    ) < 1e-12

    bad = list(Q0)
    bad[0] += 0.011

    with pytest.raises(
        RehabMultiCycleContractError
    ):
        assert_cycle_start(
            bad,
            Q0,
        )
