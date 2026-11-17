import pytest

from rehab_robot_control.rehab_multicycle_contract import (
    build_multicycle_plan,
)

from rehab_robot_control.rehab_multicycle_hardware import (
    RehabMultiCycleHardwareError,
    run_cycle_sequence,
    validate_real_request,
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


def test_real_request_accepts_exact_3_cycles():
    cycles, hold = validate_real_request(
        3,
        0.5,
    )

    assert cycles == 3
    assert hold == 0.5


def test_real_request_rejects_5_cycles_for_first_validation():
    with pytest.raises(
        RehabMultiCycleHardwareError
    ):
        validate_real_request(
            5,
            0.5,
        )


def test_real_request_rejects_wrong_hold():
    with pytest.raises(
        RehabMultiCycleHardwareError
    ):
        validate_real_request(
            3,
            1.0,
        )


def test_three_cycle_sequence_runs_exactly_three():
    plan = build_multicycle_plan(
        Q0,
        cycles=3,
        hold_s=0.5,
    )

    called = []

    completed = run_cycle_sequence(
        plan,
        lambda stage: called.append(
            stage["cycle"]
        ),
    )

    assert completed == 3
    assert called == [1, 2, 3]


def test_fault_in_cycle_two_prevents_cycle_three():
    plan = build_multicycle_plan(
        Q0,
        cycles=3,
        hold_s=0.5,
    )

    called = []

    def run(stage):
        called.append(
            stage["cycle"]
        )

        if stage["cycle"] == 2:
            raise RuntimeError(
                "synthetic cycle fault"
            )

    with pytest.raises(
        RuntimeError,
        match="synthetic cycle fault",
    ):
        run_cycle_sequence(
            plan,
            run,
        )

    assert called == [1, 2]
